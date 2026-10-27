// Standby of NestBox_v2O - see standby.h.
#include "standby.h"
#include <WiFi.h>
#include <esp_sleep.h>
#include <driver/adc.h>
#include <esp32/ulp.h>
#include <soc/rtc.h>
#include <soc/sens_reg.h>
#include "canbus1.h"
#include "txqueue.h"
#include "ota.h"
#include "laststep.h"

float read12V();                      // telemetry.cpp
void telemetryGpsStartNext(char mode);
bool telemetryEngineOn();
void telemetryRecordNow();
bool telemetryRecordDone();
bool telemetryPollFix();
void telemetryCellOff();
void logNetEvent(const char* text);
void prepareRestart(const char* why);  // main.cpp

#define ADC_12V_FACTOR_SB 195.7f      // = telemetry.cpp ADC_12V_FACTOR (raw / factor = volts)
#define FALSE_WAKES_RAISE 3           // after this many unconfirmed ULP wakes in a row: threshold up
#define FALSE_WAKE_RAISE_V 0.3f

// RTC slow memory, start reserved for the ULP (CONFIG_ULP_COPROC_RESERVE_MEM 512 B = 128 words)
// p62 (27.10.2026): counters of one sleep, read at the wake - where does the ULP wake fail?
enum { ULP_THR = 0, ULP_LAST = 1, ULP_RUNS = 2, ULP_MIN = 3, ULP_MAX = 4, ULP_OVER = 5, ULP_NRDY = 6,
       ULP_WAKES = 7, ULP_PROG = 10 };

RTC_DATA_ATTR static time_t s_nextReport;     // time() of the next parked report (time() runs on in deep sleep)
RTC_DATA_ATTR static uint8_t s_falseWakes;    // ULP wakes not confirmed by the 5 s reading, in a row
RTC_DATA_ATTR static uint16_t s_thrRaise;     // raw x4 added to the ULP threshold after false wakes
RTC_DATA_ATTR static uint8_t s_ulpTest;       // SLEEP_ULP test: a wake always boots (no sleep again)
RTC_DATA_ATTR static uint8_t s_ulpArmed;      // p62: the ULP watched this sleep (its counters are valid)

static char s_wake[48] = "-";
static char s_ulpStats[56];                   // p62: the last sleep's ULP counters ("" = no ULP sleep)
static bool s_parked;
static uint32_t s_otaWaitSince, s_reportAt, s_recAt;
static bool s_reportFix, s_reportRecDone;
static volatile uint32_t s_testS, s_testAt;   // SLEEP_TEST / SLEEP_ULP from the web
static volatile bool s_testUlp;
static void standbyTestNow(uint32_t seconds, bool ulp);
static bool ulpMeasure(unsigned* sum, uint32_t* raw);

bool standbyParkedBoot() { return s_parked; }
const char* standbyWakeName() { return s_wake; }
const char* standbyUlpStats() { return s_ulpStats; }

static uint16_t rawX4(float volts) { return (uint16_t)(volts * ADC_12V_FACTOR_SB * 4.0f + 0.5f); }

// ULP: 4 ADC1 ch0 samples summed; sum >= threshold -> wake the chip, else halt until the next period.
// p55 (27.10.2026, as ESP-IDF 4.4 examples/system/ulp_fsm/ulp_adc): WAKE only when the RTC
// controller is ready (RTC_CNTL_RDY_FOR_WAKEUP, else WAKE has no effect - ulp_instruction_set.rst),
// not ready = halt and try at the next period; after WAKE the ULP timer stops (ulp_run starts it again)
// The ULP program (p65, 27.10.2026): every run first forces the SAR ADC power on (SENS_SAR_MEAS_WAIT2_REG
// FORCE_XPD_SAR = 3, sens_reg.h) - without it the ADC instruction hung in deep sleep (car 27.10.2026:
// no ADC = 901 runs in 180 s, ADC without the force = 0 runs; with it: woke at the engine start, low
// 7674 = 9.8 V crank dip). Then 4 ADC1 ch0 samples summed; sum >= threshold and the RTC ready for the
// wake (RTC_CNTL_RDY_FOR_WAKEUP, p55) -> WAKE + ULP timer off; else halt until the next period.
// Counters of one sleep (p62) go to FleetNest at the wake ("ULP n lo hi ov nr wk").
static bool ulpStart(uint16_t thrX4)
{
  static ulp_insn_t prog[96];
  size_t n = 0;
#define ADD(...) do { const ulp_insn_t t_[] = {__VA_ARGS__}; \
    for (size_t i_ = 0; i_ < sizeof(t_) / sizeof(t_[0]) && n < 96; i_++) prog[n++] = t_[i_]; } while (0)
#define INC(word) ADD(I_MOVI(R2, word), I_LD(R1, R2, 0), I_ADDI(R1, R1, 1), I_ST(R1, R2, 0))
  ADD(I_WR_REG(SENS_SAR_MEAS_WAIT2_REG, SENS_FORCE_XPD_SAR_S, SENS_FORCE_XPD_SAR_S + 1, SENS_FORCE_XPD_SAR_PU));
  INC(ULP_RUNS);
  ADD(I_MOVI(R3, 0),
      I_ADC(R1, 0, 0), I_ADDR(R3, R3, R1),
      I_ADC(R1, 0, 0), I_ADDR(R3, R3, R1),
      I_ADC(R1, 0, 0), I_ADDR(R3, R3, R1),
      I_ADC(R1, 0, 0), I_ADDR(R3, R3, R1),
      I_MOVI(R2, ULP_LAST), I_ST(R3, R2, 0),
      I_MOVI(R2, ULP_MIN), I_LD(R0, R2, 0),
      I_SUBR(R0, R3, R0),                  // sum - min: overflow = a new minimum
      M_BXF(10), M_BX(11),
      M_LABEL(10), I_ST(R3, R2, 0),
      M_LABEL(11),
      I_MOVI(R2, ULP_MAX), I_LD(R0, R2, 0),
      I_SUBR(R0, R0, R3),                  // max - sum: overflow = a new maximum
      M_BXF(12), M_BX(13),
      M_LABEL(12), I_ST(R3, R2, 0),
      M_LABEL(13),
      I_MOVI(R2, ULP_THR), I_LD(R0, R2, 0),
      I_SUBR(R0, R3, R0),                  // sum - threshold: overflow = below
      M_BXF(1));
  INC(ULP_OVER);
  ADD(I_RD_REG(RTC_CNTL_LOW_POWER_ST_REG, RTC_CNTL_RDY_FOR_WAKEUP_S, RTC_CNTL_RDY_FOR_WAKEUP_S),
      I_ANDI(R0, R0, 1),
      M_BXZ(2));                           // not ready for the wake: next period
  INC(ULP_WAKES);
  ADD(I_WAKE(),
      I_END(),                             // ULP timer off - the woken chip takes over
      M_LABEL(1),
      I_HALT(),
      M_LABEL(2));
  INC(ULP_NRDY);
  ADD(I_HALT());
#undef ADD
#undef INC
  if (n >= 96) return false;
  RTC_SLOW_MEM[ULP_THR] = thrX4;
  RTC_SLOW_MEM[ULP_LAST] = 0;
  RTC_SLOW_MEM[ULP_RUNS] = 0; RTC_SLOW_MEM[ULP_MIN] = 0xFFFF; RTC_SLOW_MEM[ULP_MAX] = 0;
  RTC_SLOW_MEM[ULP_OVER] = 0; RTC_SLOW_MEM[ULP_NRDY] = 0; RTC_SLOW_MEM[ULP_WAKES] = 0;
  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_11);   // as analogRead (Arduino default 11 dB)
  adc1_ulp_enable();
  if (ulp_process_macros_and_load(ULP_PROG, prog, &n) != ESP_OK) return false;
  ulp_set_wakeup_period(0, STANDBY_ULP_PERIOD_US);
  if (ulp_run(ULP_PROG) != ESP_OK) return false;
  return esp_sleep_enable_ulp_wakeup() == ESP_OK;
}

// deep sleep until the 12 V rises (ULP) or the parked report is due (timer); no return
static void sleepNow(bool ulp, uint32_t forceSeconds = 0)
{
  time_t now = time(nullptr);
  uint32_t secs = forceSeconds;
  if (!secs) {
    long left = (long)(s_nextReport - now);
    secs = left < 10 ? 10 : (uint32_t)left;
  }
  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
  s_ulpArmed = 0;
  if (ulp && !ulpStart(rawX4(ENGINE_OFF_VOLTAGE) + s_thrRaise)) Serial.println("[STANDBY] ULP start failed - timer only");
  else if (ulp) s_ulpArmed = 1;
  esp_sleep_enable_timer_wakeup((uint64_t)secs * 1000000ULL);
  lastStep("deep sleep %lus", (unsigned long)secs);
  Serial.printf("[STANDBY] deep sleep %lu s%s\n", (unsigned long)secs, ulp ? " + ULP 12 V" : "");
  Serial.flush();
  esp_deep_sleep_disable_rom_logging();
  esp_deep_sleep_start();
}

void standbyEarly()
{
  if (esp_reset_reason() != ESP_RST_DEEPSLEEP) return;
  esp_sleep_wakeup_cause_t c = esp_sleep_get_wakeup_cause();
  unsigned ulpSum = RTC_SLOW_MEM[ULP_LAST] & 0xFFFF;   // the ULP's last 4-sample sum
  if (s_ulpArmed)                     // runs, min/max sum, sums >= threshold, not ready, WAKE issued
    snprintf(s_ulpStats, sizeof(s_ulpStats), "ULP n=%u lo=%u hi=%u ov=%u nr=%u wk=%u",
             (unsigned)(RTC_SLOW_MEM[ULP_RUNS] & 0xFFFF), (unsigned)(RTC_SLOW_MEM[ULP_MIN] & 0xFFFF),
             (unsigned)(RTC_SLOW_MEM[ULP_MAX] & 0xFFFF), (unsigned)(RTC_SLOW_MEM[ULP_OVER] & 0xFFFF),
             (unsigned)(RTC_SLOW_MEM[ULP_NRDY] & 0xFFFF), (unsigned)(RTC_SLOW_MEM[ULP_WAKES] & 0xFFFF));
  s_ulpArmed = 0;
  if (s_ulpTest) {                    // SLEEP_ULP: report and boot, whatever woke it
    s_ulpTest = 0;
    snprintf(s_wake, sizeof(s_wake), "ULP_TEST %s ulp=%u v=%.2f", c == ESP_SLEEP_WAKEUP_ULP ? "ULP" :
             c == ESP_SLEEP_WAKEUP_TIMER ? "TIMER" : "OTHER", ulpSum, read12V());
    return;
  }
  if (c == ESP_SLEEP_WAKEUP_ULP) {
    // the second reading (v1F: two readings >= ENGINE_OFF_VOLTAGE 5 s apart); the ULP gave the first
    float v1 = read12V();
    bool ok = v1 >= ENGINE_OFF_VOLTAGE;
    if (ok) { delay(5000); ok = read12V() >= ENGINE_OFF_VOLTAGE; }
    if (!ok) {
      if (++s_falseWakes >= FALSE_WAKES_RAISE && s_thrRaise < rawX4(2.0f))
        s_thrRaise += rawX4(FALSE_WAKE_RAISE_V);   // ULP and analogRead differ: the ULP asks higher
      sleepNow(true);
    }
    s_falseWakes = 0;
    s_thrRaise = 0;
    snprintf(s_wake, sizeof(s_wake), "VOLTAGE ulp=%u", ulpSum);
  } else if (c == ESP_SLEEP_WAKEUP_TIMER) {
    if ((long)(s_nextReport - time(nullptr)) > 10) {   // a SLEEP_TEST wake, not the report
      snprintf(s_wake, sizeof(s_wake), "TEST");
      return;
    }
    s_parked = true;
    snprintf(s_wake, sizeof(s_wake), "REPORT");
  }
}

// CAN1 silent for ms (no frame since boot counts from the boot)
static uint32_t canQuietMs()
{
  Can1Stats s = can1Stats();
  return millis() - (s.lastFrameMs ? s.lastFrameMs : 0);
}

static void goStandby()
{
  char m[96];
  float v = read12V();
  s_nextReport = time(nullptr) + STANDBY_REPORT_S;
  snprintf(m, sizeof(m), "STANDBY v=%.2f quiet=%lus wake=%s next_report=%ds", v,
           (unsigned long)(canQuietMs() / 1000), s_wake, STANDBY_REPORT_S);
  logNetEvent(m);
  telemetryCellOff();
  prepareRestart("STANDBY");          // sniff file closed, queue saved, SD log synced
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  sleepNow(true);
}

// p59: the tests from FleetNest (cell or WiFi) - CMD=ULP_READ | STANDBY? | SLEEP_ULP=<10..600>;
// run from loop() like the web ones, the answer = one PID 0x399 line "CMD <command> ..." (<= 60 chars:
// CStorage::log byte buffer). The server resends until it sees the answer or the BOOT wake=ULP_TEST
// line, so a SLEEP_ULP within 5 min after a ULP_TEST wake is not done again.
static volatile uint8_t s_remote;      // 1 ULP_READ, 2 STANDBY?

void standbyRemote(const char* cmd)
{
  if (!strcmp(cmd, "ULP_READ")) { s_remote = 1; return; }
  if (!strcmp(cmd, "STANDBY?")) { s_remote = 2; return; }
  if (!strncmp(cmd, "GPS_TEST=", 9)) {   // p66: timer sleep <s>, then GNSS start N / H / C, TTFF reported
    char* e = nullptr;
    unsigned long sec = strtoul(cmd + 9, &e, 10);
    char mode = (e && *e == ',') ? e[1] : 'N';
    char m[64];
    if (sec < 30 || sec > 3600 || (mode != 'N' && mode != 'H' && mode != 'C'))
      snprintf(m, sizeof(m), "CMD %s ERR 30..3600 s, N|H|C", cmd);
    else if (!strcmp(s_wake, "TEST") && millis() < 300000UL)
      snprintf(m, sizeof(m), "CMD %s not again, woke %lus ago", cmd, (unsigned long)(millis() / 1000));
    else {
      telemetryGpsStartNext(mode);
      standbyTest(sec, false);
      snprintf(m, sizeof(m), "CMD %s sleep in ~2 s", cmd);
    }
    statusLine(m);
    return;
  }
  if (strncmp(cmd, "SLEEP_ULP=", 10)) return;
  unsigned long sec = strtoul(cmd + 10, nullptr, 10);
  char m[64];
  if (sec < 10 || sec > 600) snprintf(m, sizeof(m), "CMD %s ERR 10..600 s", cmd);
  else if (!strncmp(s_wake, "ULP_TEST", 8) && millis() < 300000UL)
    snprintf(m, sizeof(m), "CMD %s not again, woke %lus ago", cmd, (unsigned long)(millis() / 1000));
  else {
    standbyTest(sec, true);
    snprintf(m, sizeof(m), "CMD %s sleep in ~2 s", cmd);
  }
  statusLine(m);
}

static void remoteTick()
{
  uint8_t r = s_remote;
  if (!r) return;
  s_remote = 0;
  char m[64];
  if (r == 1) {
    unsigned sum;
    uint32_t raw;
    if (!ulpMeasure(&sum, &raw)) snprintf(m, sizeof(m), "CMD ULP_READ ERR ULP start");
    else if (sum == 0xFFFF) snprintf(m, sizeof(m), "CMD ULP_READ ulp=none ard=%.2fV(%lu)",
                                     raw / 4.0f / ADC_12V_FACTOR_SB, (unsigned long)raw);
    else snprintf(m, sizeof(m), "CMD ULP_READ ulp=%.2fV(%u) ard=%.2fV(%lu)", sum / 4.0f / ADC_12V_FACTOR_SB,
                  sum, raw / 4.0f / ADC_12V_FACTOR_SB, (unsigned long)raw);
  } else {
    snprintf(m, sizeof(m), "CMD STANDBY? v=%.2f eng=%d wake=%s", read12V(), telemetryEngineOn() ? 1 : 0, s_wake);
  }
  statusLine(m);
}

void standbyTick()
{
  remoteTick();
  if (s_testS && millis() - s_testAt > 2000) standbyTestNow(s_testS, s_testUlp);   // the HTTP answer went out first
  if (!STANDBY_AUTO) return;          // 21.10.2026: the ULP did not wake at the engine start - off until tested
  static uint32_t lastCheck;
  if (millis() - lastCheck < 1000) return;
  lastCheck = millis();
  float v = read12V();
  if (telemetryEngineOn()) s_parked = false;   // driven off during the parked report: a normal boot now
  if (v < 7.0f || telemetryEngineOn() || v >= ENGINE_OFF_VOLTAGE) return;   // bench / engine / charging
  uint32_t need = s_parked ? 10000UL : STANDBY_QUIET_MS;   // the parked report: the car is asleep anyway
  if (canQuietMs() < need) return;
  if (s_parked) {
    uint32_t up = millis() / 1000;
    if (!s_reportFix && up < STANDBY_REPORT_GPS_S) {
      static uint32_t lastPoll;
      if (millis() - lastPoll >= 5000) { lastPoll = millis(); s_reportFix = telemetryPollFix(); }
      if (!s_reportFix) return;
    }
    if (!s_reportAt) {
      s_reportAt = millis();
      telemetryRecordNow();           // the parked record (position if there is a fix, 12 V)
      return;
    }
    if (!s_reportRecDone) {
      if (!telemetryRecordDone()) return;
      s_reportRecDone = true;
      s_recAt = millis();
    }
    if (txqPending() && millis() - s_recAt < STANDBY_REPORT_SEND_S * 1000UL) return;
  }
  if (otaBusy()) {
    if (!s_otaWaitSince) s_otaWaitSince = millis();
    if (millis() - s_otaWaitSince < STANDBY_OTA_WAIT_MS) return;
  }
  goStandby();
}

void standbyTest(uint32_t seconds, bool ulp) { s_testUlp = ulp; s_testS = seconds; s_testAt = millis(); }   // web: runs from loop()

static void standbyTestNow(uint32_t seconds, bool ulp)
{
  char m[64];
  snprintf(m, sizeof(m), "STANDBY TEST %lus %s", (unsigned long)seconds, ulp ? "ULP + timer" : "(timer only)");
  logNetEvent(m);
  if (s_nextReport < time(nullptr) + (time_t)seconds + 60) s_nextReport = time(nullptr) + STANDBY_REPORT_S;
  telemetryCellOff();
  prepareRestart("SLEEP_TEST");
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  s_ulpTest = ulp ? 1 : 0;
  sleepNow(ulp, seconds);
}

// ULP_READ: the ULP measures while the chip is awake (threshold never reached = no wake), 1 s;
// its 4-sample sum next to analogRead - the same input and scale? (21.10.2026)
static bool ulpMeasure(unsigned* sum, uint32_t* raw)   // false = the ULP did not start
{
  RTC_SLOW_MEM[ULP_LAST] = 0xFFFF;
  if (!ulpStart(0xFFFF)) return false;
  delay(1100);
  ulp_timer_stop();
  delay(250);                          // a run in progress ends
  *sum = RTC_SLOW_MEM[ULP_LAST] & 0xFFFF;
  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ULP);
  *raw = 0;
  for (int i = 0; i < 4; i++) *raw += analogRead(36);  // adc1_get_raw: the ADC back from the ULP
  return true;
}

int standbyUlpRead(char* buf, int cap)
{
  unsigned sum;
  uint32_t raw;
  if (!ulpMeasure(&sum, &raw)) return snprintf(buf, cap, "ERR ULP start");
  return snprintf(buf, cap, "ulp_sum4=%u%s ulp_v=%.2f arduino_sum4=%lu v=%.2f thr_sum4=%u",
                  sum, sum == 0xFFFF ? " (ULP wrote nothing)" : "", sum / 4.0f / ADC_12V_FACTOR_SB,
                  (unsigned long)raw, raw / 4.0f / ADC_12V_FACTOR_SB, (unsigned)rawX4(ENGINE_OFF_VOLTAGE));
}

int standbyStatus(char* buf, int cap)
{
  return snprintf(buf, cap, "wake=%s parked=%d v=%.2f engine=%d can_quiet_s=%lu need_s=%lu ota_busy=%d "
                  "next_report_in=%lds false_wakes=%u thr_raise=%u ulp_last=%u",
                  s_wake, s_parked ? 1 : 0, read12V(), telemetryEngineOn() ? 1 : 0,
                  (unsigned long)(canQuietMs() / 1000),
                  (unsigned long)((s_parked ? 10000UL : STANDBY_QUIET_MS) / 1000), otaBusy() ? 1 : 0,
                  s_nextReport ? (long)(s_nextReport - time(nullptr)) : -1L, (unsigned)s_falseWakes,
                  (unsigned)s_thrRaise, (unsigned)(RTC_SLOW_MEM[ULP_LAST] & 0xFFFF));
}
