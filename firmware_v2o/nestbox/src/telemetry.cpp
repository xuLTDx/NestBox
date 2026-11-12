// Records to FleetNest - NestBox_v2O step 2a (WiFi). The path of NestBox_v1F:
// every record goes to the SD file /DATA/<n>.CSV (SD writer, confirmed) and
// into the delivery queue (txqueue.cpp, copied); the net task sends the queue
// in packets and drops a record only after the server's ACK (stored).
// Identity of a record = (boot, PID 0 = ms since boot), as v1F.
//
// Step 2b: the SIM7600 modem (cellmodem.cpp) as the fallback link and its
// GNSS in every record - OFF until CELL=1 (/api/control), not tested yet.
// Step B: engine START/STOP from the car's rpm (K0 0x208) as v1F, a record
// every second while the engine runs.
// Not yet (later steps):
// and standby (step 3), car values (step 4). Until then a record every
// REC_S seconds (default 60, /api/control REC_S=) carries the clock time
// (system clock from NTP / the server's login reply, as v1F does for engine
// event records - addTimeOf), the 12 V level, the chip temperature, the WiFi
// signal, and the SD fingerprint (PID 0x398) every 10 min.
#include "errreport.h"
#include <Arduino.h>
#include <WiFi.h>
#include <Preferences.h>
#include <nvs.h>
#include "store.h"
#include "netlog.h"                  // PG101: nbout = Serial + /api/serial
#include "txqueue.h"
#include "sddigest.h"
#include "sdwriter.h"
#include "teleclient.h"
#include "sniff.h"
#include "cellmodem.h"
#include "cardecode.h"
#include "canbus1.h"
#include "ota.h"
#include "otaverify.h"
#include "standby.h"
#include "laststep.h"

const char* bootResetInfo(char* buf, int cap);   // main.cpp
#include "nbread.h"
#include "nbapn.h"

#define PID_SD_DIGEST  0x398
#define SD_DIGEST_MS   600000UL
#define PID_SNIFF_LEFT 0x393     // sniff minutes left (v1F: the server sees the sniff state)
// car values (cardecode.cpp); PID numbers as v1F / FleetNest apps/telemetry/pids.py where they exist
#define PID_ODOMETER   0x1A6     // km (v1F: OBD 0xA6 | 0x100)
#define PID_FUEL_CAND  0x391     // fuel candidate, 0.1 L (v1F)
#define PID_FUEL_RAW   0x392     // fuel, raw byte (v1F)
#define PID_CAN_FRAMES 0x389     // CAN frames received (v1F: CAN - prijate ramce) - here: CAN1 since the last record
#define PID_UREA_KM    0x39A     // NEW: AdBlue range before start inhibit, km (candidate)
#define PID_UREA_RAW   0x39B     // NEW: 0x5A8 raw, 8 bytes
#define PID_RPM        0x10C     // rpm (v1F: OBD 0x0C | 0x100)
#define PID_AUTO_READ  0x39F     // NEW: automatic diagnostic read (K0), "<item>;<bus>;<result>;<nrc>;<response bytes>"
#define PID_ENG_STATE  0x39C     // NEW: K0 0x348 engine running state (1 off, 3 running, 4 stop-start ...)
#define PID_COOLANT    0x105     // p81: K0 0x488 coolant C (v1F: OBD 0x05 | 0x100)
#define PID_SPEED_CAN  0x10D     // p81: K0 0x38D speed km/h x 100 (v1F: OBD 0x0D | 0x100 - here x 100)
#define PID_DOORS      0x3A0     // p81: K0 0x412 open doors bits: 1 driver 2 passenger 4 RL 8 RR 16 boot
#define PID_FUEL_BSI   0x3A1     // p81: READ BSI D8C3 fuel, 0.01 L
#define PID_UREA_MEAS  0x3A2     // p81: READ CMM D558 urea measured, raw byte (scale from a refill)
// 0x3A3 (p81 D517 "ml") no longer sent - PG-p10: the 1-byte x 62.5 decoding was wrong, see 0x3A6
#define PID_UREA_D517  0x3A6     // PG-p10: READ CMM D517 2 bytes raw (AdBlue remaining candidate)
#define PID_FUEL_CALC  0x3A7     // PG-p10: READ BSI D8C4 fuel calculated, 0.01 L
#define PID_FUEL_PCT   0x3A8     // PG-p10: READ BSI D8C5 fuel displayed, %
#define PID_REFUEL_VOL 0x3A9     // PG-p12: K0 0x432 byte 4 added fuel volume, raw
#define PID_REFUEL_CNT 0x3AA     // PG-p12: K0 0x432 byte 6 bits 2-0 refuel counter
#define PID_CREW       0x3A4     // p83: bit0 driver belt, bit1 passenger belt, bit2 passenger seated
#define PID_SPEED_DISP 0x3A5     // p83: speed as on the cluster (BSI D8F9), km/h
#define PID_CREW_CAN   0x3AB     // PG55/PG57: K0 0x572 byte 0, byte 4, 0x4F2 byte 3 raw (belt / seat candidates), in the record after each change
#define PID_ENGINE_EVENT      0x380   // v1F: 1 = START, 2 = STOP
#define PID_ENGINE_EVENT_TIME 0x381   // v1F: unix time of the event
#define CAR_FRESH_MS   5000
// engine (v1F engineTick rules): START = rpm > 0 within 3 s; STOP = the engine ECU silent
// (no 0x208 for ENGINE_SILENT_MS = ignition off), timed at the last rpm > 0; rpm 0 with the
// ECU on the bus (stop-start at a light) is NOT a stop
#define ENGINE_SILENT_MS 10000
#define DRIVE_REC_MS     1000     // a record every second while the engine runs (v1F)
#define ADC_12V_PIN    36        // ADC1 channel 0 (OVMS esp32adc)
#define ADC_12V_FACTOR 195.7f    // raw / factor = volts (OVMS default system.adc factor12v; this unit NOT calibrated)

extern "C" uint8_t temprature_sens_read();
extern const char FW_BUILD_STR[];

nvs_handle_t nvs;                // txqueue.cpp uses it (v1F: the same "storage" namespace)
static DataLog s_log;
static TeleClientUDP s_client;
static WifiLink s_wifi;
static CellLink s_cell;
static volatile bool s_cellOn;               // NVS CELL (default 0 until tested on 12 V)
static volatile bool s_cellUp;
static GnssFix s_fix;                        // last valid reading - read and written only through fixGet / fixSet
// PG-p19 (3.11.2026): the record task never waits for the modem. why: makeRecord() asked the modem itself
// (AT+CSQ every record, the AT GNSS query when no NMEA) and waited for its lock without a limit; a pull-OTA
// over the modem held that lock up to 5 s per read (the AT+CCH read, gone since NAVRH 9) - Peugeot 3.11 boot 149/151/152:
// 4-5 s without records exactly during OTA-PULL (not on SD either). As v1F (telelogger.ino 3413: the RSSI
// read in the network task, 2548: the record only takes the variable). The record now takes the NMEA fix
// (memory, CellLink::gnssPushed) or the last values cellTask read from the modem (s_fix, s_csq).
static portMUX_TYPE s_fixMux = portMUX_INITIALIZER_UNLOCKED;   // s_fix is written by the record, cell and loop tasks
static GnssFix fixGet() { portENTER_CRITICAL(&s_fixMux); GnssFix f = s_fix; portEXIT_CRITICAL(&s_fixMux); return f; }
static void fixSet(const GnssFix& f) { portENTER_CRITICAL(&s_fixMux); s_fix = f; portEXIT_CRITICAL(&s_fixMux); }
static volatile int s_csq;                   // dBm from cellTask (0 = unknown)
static volatile uint32_t s_csqAt;            // millis() of that reading
#define CSQ_EVERY_MS 10000UL                 // cellTask reads it this often
#define CSQ_FRESH_MS 30000UL                 // older = not put into the record
// p66: GNSS start mode for the boot after a GPS_TEST sleep (one-shot), time to the first fix reported once
RTC_DATA_ATTR static char s_gpsNext;
static bool s_ttffSent;
void telemetryGpsStartNext(char mode) { s_gpsNext = mode; }
static void ttffCheck()
{
  GnssFix fx = fixGet();
  if (s_ttffSent || !fx.valid || !s_cell.gpsStartedAt()) return;
  s_ttffSent = true;
  char m[57];
  // PG150: a fix older than the GNSS start = ttff 0 + "pre". Why: Peugeot 27.10. boot 82 sent "GPS ttff=4294967s
  // boot=4s" - fx.at was before gpsStartedAt() and the unsigned difference wrapped. startGnss() sets the start time
  // only after up to 2 s + 2 s of commands; a GNSS still running from before (OVMS simcom_7600.cpp: it may) sends
  // NMEA with a fix meanwhile (NEOVERENE that this was the case in boot 82, the numbers fit).
  bool pre = (int32_t)(fx.at - s_cell.gpsStartedAt()) < 0;
  snprintf(m, sizeof(m), "GPS ttff=%lus%s boot=%lus mode=%c ok=%d sats=%u",
           pre ? 0UL : (unsigned long)((fx.at - s_cell.gpsStartedAt()) / 1000), pre ? " pre" : "",
           (unsigned long)(fx.at / 1000), s_cell.gpsMode(), s_cell.gpsStartOk() ? 1 : 0, (unsigned)fx.sats);
  statusLine(m);
}
static char s_apn[40] = "internet";          // the CELL_APN in force (NVS), home operator 4ka
static volatile uint32_t s_recS = 60;
static volatile bool s_otaReq;

void logNetEvent(const char* text) { s_log.logEvent(text); }

// PG149: the modem start result as ERR/OK (errreport.h). The "CELL <status> mux=..." line above says the same in its
// own words, but FleetNest e-mails the Admin only on a line starting "ERR CELL noanswer" (handoff 11.11.2026).
static void cellStartErr(bool up, const char* st)
{
  static const struct { const char* status; const char* what; } k[] = {
    {"no answer", "noanswer"}, {"CMUX failed", "cmux"}, {"SIM not ready", "sim"}, {"not registered", "notreg"},
    {"PPP failed", "ppp"}};
  for (auto& e : k) {
    if (up) errOk("CELL", e.what);
    else if (!strncmp(st, e.status, strlen(e.status))) errReport("CELL", e.what, strncmp(st, "PPP", 3) ? nullptr : st + 11);
  }
  if (s_cell.gpsStartedAt()) {               // the GNSS start command was sent (after CMUX)
    if (s_cell.gpsStartOk()) errOk("GNSS", "start");
    else errReport("GNSS", "start");
  }
}

// PG151: GNSS errors (agreed 11.11.2026, Vojtech + FleetNest handoff).
// - "ERR GNSS nonmea": the modem runs, no NMEA on port 1 for 10 s (gnssPushed() = -1) -> "OK GNSS nmea".
// - "ERR GNSS nofix <min>min drives=<n>": 2 drives in a row without a single fix, each >= GNSS_NOFIX_MOVE_MS moving by
//   the CAN speed -> "OK GNSS fix" at the next fix. Why not "no fix for N min": an underground garage, a parking house
//   or a tunnel is no fault (Vojtech 11.11.); two whole drives without a fix are - antenna / module. Without a CAN
//   speed (no car profile) nothing counts: the CAN speed is a required value for every car (notes PROJEKT_STAV 7).
//   FleetNest ttff 1.-11.11.: Peugeot max 96 s, U9K4M3G6 max 201 s - far under 10 min.
#define GNSS_NOFIX_MOVE_MS (10UL * 60UL * 1000UL)
#define GNSS_NOFIX_DRIVES  2
RTC_DATA_ATTR static uint8_t s_noFixDrives;   // drives in a row without a fix - kept over the deep sleep between them
RTC_DATA_ATTR static uint32_t s_noFixMin;     // their moving minutes
static uint32_t s_driveMoveMs, s_driveTickAt;
static bool s_driveFix;

static void gnssErrTick(const CarValues& cv, bool engineOn, bool started, bool stopped)
{
  uint32_t ms = millis();
  GnssFix fx = fixGet();
  bool fix = fx.valid && ms - fx.at < 5000;
  if (fix) errOk("GNSS", "nofix", "fix");
  if (started) { s_driveMoveMs = 0; s_driveFix = false; s_driveTickAt = ms; }
  if (engineOn) {
    if (fix) s_driveFix = true;
    bool moving = cv.speedAt && ms - cv.speedAt < 3000 && cv.speedKmh >= 3.0f;
    if (moving && s_driveTickAt) s_driveMoveMs += ms - s_driveTickAt;
    s_driveTickAt = ms;
  }
  if (stopped && s_driveMoveMs >= GNSS_NOFIX_MOVE_MS) {   // a shorter drive changes nothing
    if (s_driveFix) { s_noFixDrives = 0; s_noFixMin = 0; }
    else {
      if (s_noFixDrives < 255) s_noFixDrives++;
      s_noFixMin += s_driveMoveMs / 60000;
      if (s_noFixDrives >= GNSS_NOFIX_DRIVES) {
        char d[32];
        snprintf(d, sizeof(d), "%lumin drives=%u", (unsigned long)s_noFixMin, (unsigned)s_noFixDrives);
        errReport("GNSS", "nofix", d);
      }
    }
  }
  static uint32_t lastNmea;
  if (s_cellUp && ms - lastNmea >= 10000) {
    lastNmea = ms;
    GnssFix g;
    int r = s_cell.gnssPushed(&g);
    if (r < 0) errReport("GNSS", "nonmea");
    else if (r == 1 || s_cell.nmeaSeen()) errOk("GNSS", "nonmea", "nmea");
  }
}
static void loadApn()
{
  char a[40] = "";
  size_t l = sizeof(a);
  if (nvs_get_str(nvs, "CELL_APN", a, &l) == ESP_OK && a[0]) snprintf(s_apn, sizeof(s_apn), "%s", a);
  else snprintf(s_apn, sizeof(s_apn), "internet");
}
void telemetryApnCommand(const char* arg) { nbApnCommand(nvs, arg, "internet"); }
// NAVRH 9: CMUX always (PPP needs it) - CELL_MUX=0 is not possible any more, the stored value is ignored
bool telemetryMuxOn() { return s_cell.muxOn(); }
void telemetrySetCell(bool on) { s_cellOn = on; nvs_set_u8(nvs, "CELL", on ? 1 : 0); nvs_commit(nvs); }
bool telemetryCell() { return s_cellOn; }
void telemetrySetVehicle(const char* p) { carSetProfile(p); nvs_set_str(nvs, "VEH", carProfile()); nvs_commit(nvs); }
void telemetrySetApn(const char* apn) { nvs_set_str(nvs, "CELL_APN", apn); nvs_commit(nvs); loadApn(); s_cell.setApn(s_apn); }
void otaReadyRequested() { s_otaReq = true; otaRequest(); }   // server OTA_READY -> pull-OTA (ota.cpp)
bool telemetryLogin() { return s_client.login; }
uint32_t telemetryRecS() { return s_recS; }
void telemetrySetRecS(uint32_t s) { s_recS = s < 10 ? 10 : s; nvs_set_u32(nvs, "REC_S", s_recS); nvs_commit(nvs); }
uint32_t telemetryFileId() { return s_log.id(); }
int telemetryStatus(char* buf, int size)
{
  int n = snprintf(buf, size, "devid=%s login=%d link=%s cell=%d/%s mux=%d/%lu/%lu fix=%d sats=%u tx=%lu txB=%lu rxB=%lu file=%lu rec_s=%lu ota_req=%d ",
                   devid, s_client.login ? 1 : 0, s_client.link ? s_client.link->name() : "-", s_cellOn ? 1 : 0,
                   s_cell.status(), s_cell.muxOn() ? 1 : 0, (unsigned long)s_cell.muxFrames(),
                   (unsigned long)s_cell.muxErrors(), fixGet().valid ? 1 : 0, (unsigned)fixGet().sats, (unsigned long)s_client.txCount,
                   (unsigned long)s_client.txBytes, (unsigned long)s_client.rxBytes, (unsigned long)s_log.id(),
                   (unsigned long)s_recS, s_otaReq ? 1 : 0);
  txqStatus(buf + n, size - n);
  return strlen(buf);
}

float read12V()
{
  uint32_t raw = 0;
  for (int i = 0; i < 5; i++) raw += analogRead(ADC_12V_PIN);
  return raw / 5.0f / ADC_12V_FACTOR;
}

// one value into both the SD file ("<pid>,<v>") and the wire record ("<pid>:<v>")
struct Rec {
  CStorageRAM wire;
  char buf[512];
  void begin(uint32_t ts) { wire.init(buf, sizeof(buf)); wire.timestamp(ts); s_log.timestamp(ts); }
  void u32(uint16_t pid, uint32_t v) { wire.log(pid, &v, 1); s_log.log(pid, &v, 1); }
  void i32(uint16_t pid, int32_t v) { wire.log(pid, &v, 1); s_log.log(pid, &v, 1); }
  void u32n(uint16_t pid, uint32_t* v, uint8_t n) { wire.log(pid, v, n); s_log.log(pid, v, n); }
  void f(uint16_t pid, float v, const char* fmt) { wire.log(pid, &v, 1, fmt); s_log.log(pid, &v, 1, fmt); }
  void push(uint32_t ts)
  {
    int len = wire.length();
    if (len && buf[len - 1] == ',') len--;
    if (!txqPush(ts, buf, len)) nbout.println("[TXQ] record not queued");
    sdwDataMark(ts);             // confirmed to the queue once the card has these lines
  }
};

static volatile bool s_engineOn;
bool telemetryEngineOn() { return s_engineOn; }
static volatile bool s_recNow;                // standby.cpp: the parked report's record now
static volatile bool s_standby;               // standby.cpp: going to sleep - the modem stays off
void telemetryRecordNow() { s_recNow = true; }
bool telemetryRecordDone() { return !s_recNow; }
// standby.cpp, parked report: one GNSS reading now (true = a fix)
bool telemetryPollFix()
{
  GnssFix g;
  if (!s_cellOn || !s_cell.gnss(&g)) return false;   // the loop task (standby), not the record task
  fixSet(g);
  ttffCheck();
  return true;
}
// standby.cpp: modem (and its GNSS) off before the deep sleep; the cell task does not start it again
void telemetryCellOff()
{
  s_standby = true;
  if (s_cellOn) { s_cellUp = false; s_cell.powerOff(); logNetEvent("CELL OFF standby"); }
}
CellLink* telemetryCellLink() { return s_cellUp && s_cell.up() ? &s_cell : nullptr; }   // pull-OTA: PPP has an IP

// ---- automatic diagnostic reads (Peugeot K0, agreed 21.10.2026 "k") ------------------------
// At every engine START and then every 10 min while the engine runs, one UDS 22 read after
// another (1 s apart), only reading services (nbread.h). DIDs from PSA-RE (prototux/PSA-RE
// diag/: CMM DCM62 0x6A8->0x688, BSI2010_EV 0x752->0x652). The CMM answered Launch on CAN1
// (OBD 6/14, sniff 21.10.2026); on which bus the BSI answers is not known: CAN2 (OBD 3/8)
// first, then CAN1 - the bus that answered is kept for the boot. Raw response into the next
// record (PID 0x39F) and an SD event line; FleetNest stores it, decoding comes later.
#define AUTO_READ_EVERY_MS 60000UL     // p83: every minute (crew / cluster speed), was 10 min
#define AUTO_READ_GAP_MS   1000UL
struct AutoRead { uint32_t tx, rx; uint8_t did[6]; uint8_t buses[2]; };   // p83: up to 3 DIDs per request
// p81 (28.10.2026): BSI and the CMM (Delphi DCM6.2) both answered on CAN2 = OBD 3/8 (READ 28.10)
static const AutoRead K0_READS[] = {
  {0x752, 0x652, {0xD9, 0x97, 0xD9, 0x98, 0xD9, 0x9F}, {2, 1}},   // p83 BSI: belts driver / passenger, passenger seated
  {0x752, 0x652, {0xD8, 0xF9}, {2, 1}},   // p83 BSI: speed as on the cluster
  {0x752, 0x652, {0xD8, 0xC3}, {2, 1}},   // BSI: fuel measured x 0.01 L (verified 28.10)
  {0x6A8, 0x688, {0xD5, 0x58}, {2, 1}},   // CMM: urea volume measured (raw byte, scale at a refill)
  {0x6A8, 0x688, {0xD5, 0x17}, {2, 1}},   // CMM: urea remaining estimated, 2 bytes raw (PG-p10, unit NEOVERENÉ)
  {0x6A8, 0x688, {0xD5, 0x13}, {2, 1}},   // CMM: km before the urea refill (raw)
  {0x6A8, 0x688, {0xD4, 0x15}, {2, 1}},   // CMM: coolant (raw, the DB's -40 not plausible 28.10)
  {0x752, 0x652, {0xDB, 0xED}, {2, 1}},   // BSI: km to the next service
  {0x752, 0x652, {0xDC, 0x5F}, {2, 1}},   // BSI: months to the next service
  {0x6A8, 0x688, {0xD5, 0x1F}, {2, 1}},   // CMM: engine oil dilution
  {0x6A8, 0x688, {0xD4, 0x29}, {2, 1}},   // CMM: oil pressure
  // CVM camera: speed sign read - PG-p13 (3.11.2026): never answered on either bus (187x no reply), not read
  {0x74A, 0x64A, {0xD4, 0x09}, {0, 0}},
  // PG-p5 (2.11.2026): all fuel / urea candidates of the Diagbox DB, raw only (PID 0x39F + SD), to find
  // the one that follows the gauge and the receipts. Appended: items 0-11 keep their numbers.
  // BSI2010_EV: D8C4 fuel calculated, D8C5 fuel displayed, D8E1 range calculated
  {0x752, 0x652, {0xD8, 0xC4, 0xD8, 0xC5, 0xD8, 0xE1}, {2, 1}},
  // BSI2010_EV: D8C2 gauge sender impedance, D8C7 consumption instant, D8C8 consumption trip 1 average
  {0x752, 0x652, {0xD8, 0xC2, 0xD8, 0xC7, 0xD8, 0xC8}, {2, 1}},
  {0x75F, 0x65F, {0xD7, 0x03}, {2, 1}},   // cluster: fuel gauge needle (D703 changed in the 29.10 scan)
  {0x75F, 0x65F, {0xD7, 0x02}, {2, 1}},   // cluster: D702 (DB: rev counter needle; fell in the 29.10 scan)
  {0x6A8, 0x688, {0xD5, 0x5A}, {2, 1}},   // CMM DCM62: fuel added at the last refill (DB x0.5 L)
  {0x6A8, 0x688, {0xD5, 0x7E}, {2, 1}},   // CMM DCM62: diesel additive (DPF) remaining, ml (DB)
  {0x6A8, 0x688, {0xD5, 0x6A}, {2, 1}},   // CMM DCM62: fuel level minimum (flag)
  {0x6A8, 0x688, {0xD5, 0x68}, {2, 1}},   // CMM DCM62: urea injected, x0.015625 mg/stroke (DB)
  // CMM DCM62: urea tank temperature - PG-p13: always NRC 0x31 (not supported, 134x), not read
  {0x6A8, 0x688, {0xD5, 0x6D}, {0, 0}},
  // PG-p11 (3.11.2026): BSI2010_EV D8C6 fuel flap state (DB: 0 open, 1 closed); read only while the
  // engine runs, so a refuel with the engine off is mostly not seen - kept as raw data only
  {0x752, 0x652, {0xD8, 0xC6}, {2, 1}},
};
#define K0_READS_N (sizeof(K0_READS) / sizeof(K0_READS[0]))
static uint8_t s_autoBus[K0_READS_N];        // the bus that answered (0 = not known yet)
#define AUTO_Q 8
static uint8_t s_autoQ[AUTO_Q][5 + NB_READ_RECORD_MAX];
static int s_autoQLen[AUTO_Q];
static uint8_t s_autoHead, s_autoCount;
static portMUX_TYPE s_autoMux = portMUX_INITIALIZER_UNLOCKED;

static void autoReadPush(uint8_t item, uint8_t bus, int res, uint8_t nrc, const uint8_t* resp, int len)
{
  portENTER_CRITICAL(&s_autoMux);
  uint8_t slot = (s_autoHead + s_autoCount) % AUTO_Q;
  if (s_autoCount == AUTO_Q) s_autoHead = (s_autoHead + 1) % AUTO_Q; else s_autoCount++;
  uint8_t* q = s_autoQ[slot];
  q[0] = item; q[1] = bus; q[2] = (uint8_t)res; q[3] = nrc;
  int n = len < NB_READ_RECORD_MAX ? len : NB_READ_RECORD_MAX;
  if (n > 0) memcpy(q + 4, resp, n);
  s_autoQLen[slot] = 4 + (n > 0 ? n : 0);
  portEXIT_CRITICAL(&s_autoMux);
}

static int autoReadForRecord(uint8_t* out, int cap)
{
  int n = 0;
  portENTER_CRITICAL(&s_autoMux);
  if (s_autoCount) {
    n = s_autoQLen[s_autoHead] < cap ? s_autoQLen[s_autoHead] : cap;
    memcpy(out, s_autoQ[s_autoHead], n);
    s_autoHead = (s_autoHead + 1) % AUTO_Q;
    s_autoCount--;
  }
  portEXIT_CRITICAL(&s_autoMux);
  return n;
}

// loop() (the bus owner, as readTick): one read per call when due
static void autoReadTick()
{
  static bool wasOn;
  static int next = -1;
  static uint32_t lastRound, lastItem;
  if (strcmp(carProfile(), "K0")) return;
  bool on = s_engineOn;
  if (on && !wasOn) next = 0;                                   // START
  wasOn = on;
  if (on && next < 0 && lastRound && millis() - lastRound >= AUTO_READ_EVERY_MS) next = 0;
  if (next < 0 || millis() - lastItem < AUTO_READ_GAP_MS) return;
  const AutoRead& a = K0_READS[next];
  if (!a.buses[0]) {                    // PG-p13: buses {0, 0} = item switched off, its number stays
    if (++next >= (int)K0_READS_N) { next = -1; lastRound = millis(); }
    return;
  }
  NbRead r = {};
  r.txId = a.tx; r.rxId = a.rx;
  r.req[0] = 0x22; r.reqLen = 1;
  for (int k = 0; k < 6 && a.did[k]; k += 2) { r.req[r.reqLen++] = a.did[k]; r.req[r.reqLen++] = a.did[k + 1]; }
  static uint8_t resp[NB_READ_MAX];
  int len = 0, res = NBR_NO_REPLY;
  uint8_t nrc = 0;
  for (int k = 0; k < 2; k++) {
    uint8_t bus = s_autoBus[next] ? (k ? 0 : s_autoBus[next]) : a.buses[k];
    if (!bus) break;
    r.bus = bus;
    lastStep("auto read %d bus %u", next, bus);
    res = busExchange(&r, resp, sizeof(resp), &len, &nrc);
    if (res == NBR_OK || res == NBR_NRC) { s_autoBus[next] = bus; break; }
  }
  if (res == NBR_OK) carFromRead(a.tx, resp, len);      // p81: decoded values into the records
  autoReadPush((uint8_t)next, r.bus, res, nrc, resp, len);
  char line[300];
  nbReadLine(line, sizeof(line), 1000 + next, &r, res, nrc, resp, len);
  s_log.logEvent(line);
  lastItem = millis();
  if (++next >= (int)K0_READS_N) { next = -1; lastRound = millis(); }
}

// event: 0 = a regular record, 1 = START, 2 = STOP (timed at `when`, the record's own time)
static void makeRecord(int event = 0, time_t when = 0)
{
  static Rec r;
  static uint32_t lastDigest;
  uint32_t ts = millis();
  r.begin(ts);
  if (event) {
    r.u32(PID_ENGINE_EVENT, event);
    if (when > 1735689600) {
      r.u32(PID_ENGINE_EVENT_TIME, (uint32_t)when);
      struct tm u;
      gmtime_r(&when, &u);
      r.u32(0x10, (uint32_t)u.tm_hour * 1000000 + u.tm_min * 10000 + u.tm_sec * 100);
      r.u32(0x11, (uint32_t)u.tm_mday * 10000 + (u.tm_mon + 1) * 100 + (u.tm_year % 100));
    }
  }
  GnssFix g;
  if (s_cellOn && s_cell.gnssPushed(&g) == 1) { fixSet(g); ttffCheck(); }   // PG-p19: memory only, no AT
  GnssFix fx = fixGet();         // the NMEA fix above, or the AT reading of cellTask
  bool fresh = fx.valid && millis() - fx.at < 5000;
  if (fresh && event) {          // an event record carries the event's time (above), the position only
    r.f(PID_GPS_LATITUDE, fx.lat, "%f");
    r.f(PID_GPS_LONGITUDE, fx.lng, "%f");
    if (fx.sats) r.u32(PID_GPS_SAT_COUNT, fx.sats);
  } else if (fresh) {                   // as v1F processGPS(): the GNSS time also sets the clock
    struct tm t = {};
    t.tm_mday = fx.date / 10000; t.tm_mon = (fx.date / 100) % 100 - 1; t.tm_year = fx.date % 100 + 100;
    t.tm_hour = fx.time / 1000000; t.tm_min = (fx.time / 10000) % 100; t.tm_sec = (fx.time / 100) % 100;
    time_t gnow = mktime(&t), now0 = time(nullptr);
    if (gnow > 1735689600 && (now0 > gnow + 2 || now0 < gnow - 2)) { struct timeval tv = {gnow, 0}; settimeofday(&tv, nullptr); }
    r.u32(0x10, fx.time);
    r.u32(0x11, fx.date);
    r.f(PID_GPS_LATITUDE, fx.lat, "%f");
    r.f(PID_GPS_LONGITUDE, fx.lng, "%f");
    r.f(PID_GPS_ALTITUDE, fx.alt, "%.1f");
    r.f(PID_GPS_SPEED, fx.kph, "%.1f");
    r.u32(PID_GPS_HEADING, fx.heading);
    if (fx.sats) r.u32(PID_GPS_SAT_COUNT, fx.sats);
    if (fx.hdop > 0) r.u32(PID_GPS_HDOP, (uint32_t)(fx.hdop + 0.5f));
  }
  time_t now = time(nullptr);
  if (!fresh && !event && now > 1735689600) {   // no fix: date and time from the clock as v1F addTimeOf()
    struct tm u;
    gmtime_r(&now, &u);
    r.u32(0x10, (uint32_t)u.tm_hour * 1000000 + u.tm_min * 10000 + u.tm_sec * 100);
    r.u32(0x11, (uint32_t)u.tm_mday * 10000 + (u.tm_mon + 1) * 100 + (u.tm_year % 100));
  }
  r.f(PID_BATTERY_VOLTAGE, read12V() * 100, "%.0f");    // v1F: 0.01 V units (FleetNest: /100 V)
  r.i32(PID_DEVICE_TEMP, (int32_t)((temprature_sens_read() - 32) / 1.8f));
  if (WiFi.status() == WL_CONNECTED) r.i32(PID_CSQ, WiFi.RSSI());
  else if (s_cellUp && s_csq && millis() - s_csqAt < CSQ_FRESH_MS) r.i32(PID_CSQ, s_csq);   // PG-p19: cellTask's reading
  if (sniffActive()) r.u32(PID_SNIFF_LEFT, sniffMinutesLeft());
  static uint32_t lastFrames;
  Can1Stats cs = can1Stats();
  r.u32(PID_CAN_FRAMES, cs.frames - lastFrames);
  lastFrames = cs.frames;
  CarValues cv = carValues();
  uint32_t ms = millis();
  if (cv.odoAt && ms - cv.odoAt < CAR_FRESH_MS) r.u32(PID_ODOMETER, cv.odoKm);
  if (cv.rpmAt && ms - cv.rpmAt < CAR_FRESH_MS) r.u32(PID_RPM, (uint32_t)(cv.rpm + 0.5f));
  if (cv.engAt && ms - cv.engAt < CAR_FRESH_MS) r.u32(PID_ENG_STATE, cv.engState);
  if (cv.fuelAt && ms - cv.fuelAt < CAR_FRESH_MS) {
    r.u32(PID_FUEL_CAND, (uint32_t)(cv.fuelL * 10 + 0.5f));
    r.u32(PID_FUEL_RAW, cv.fuelRaw);
  }
  // p81: doors / coolant / speed from CAN1, fuel / urea from the automatic reads (every 10 min: kept 11 min)
  if (cv.doorsAt && ms - cv.doorsAt < CAR_FRESH_MS) r.u32(PID_DOORS, cv.doors);
  if (cv.coolAt && ms - cv.coolAt < CAR_FRESH_MS) r.i32(PID_COOLANT, cv.coolC);
  if (cv.speedAt && ms - cv.speedAt < CAR_FRESH_MS) r.u32(PID_SPEED_CAN, (uint32_t)(cv.speedKmh * 100 + 0.5f));
  if (cv.fuelBsiAt && ms - cv.fuelBsiAt < 660000UL) r.u32(PID_FUEL_BSI, cv.fuelBsiCl);
  if (cv.ureaMeasAt && ms - cv.ureaMeasAt < 660000UL) r.u32(PID_UREA_MEAS, cv.ureaMeasRaw);
  if (cv.ureaEstAt && ms - cv.ureaEstAt < 90000UL) r.u32(PID_UREA_D517, cv.ureaEstRaw);
  if (cv.fuelCalcAt && ms - cv.fuelCalcAt < 90000UL) {
    r.u32(PID_FUEL_CALC, cv.fuelCalcCl);
    r.u32(PID_FUEL_PCT, cv.fuelDispPct);
  }
  if (cv.refuelAt && ms - cv.refuelAt < CAR_FRESH_MS) {
    r.u32(PID_REFUEL_VOL, cv.refuelVol);
    r.u32(PID_REFUEL_CNT, cv.refuelCnt);
  }
  // PG47 (8.11.2026, FleetNest FN62): once per read - before, the value was repeated in every record up to 90 s
  // (boot 177: "99 km/h" on the cluster speed for ~37 s while the car braked from 100 to 49)
  static uint32_t crewSent, dispSent, c572Sent;
  if (cv.crewAt && cv.crewAt != crewSent) { crewSent = cv.crewAt; r.u32(PID_CREW, cv.crew); }
  if (cv.speedDispAt && cv.speedDispAt != dispSent) { dispSent = cv.speedDispAt; r.u32(PID_SPEED_DISP, cv.speedDisp); }
  if (cv.c572Chg != c572Sent) {                  // PG55: each change of the 0x572 belt candidate once
    c572Sent = cv.c572Chg;
    uint32_t v[3] = {cv.c572b0, cv.c572b4, cv.c4f2b3};
    r.u32n(PID_CREW_CAN, v, 3);
  }
  if (cv.ureaAt && ms - cv.ureaAt < CAR_FRESH_MS) {
    r.u32(PID_UREA_KM, cv.ureaKm);
    uint8_t* u = cv.urea;
    r.wire.log(PID_UREA_RAW, u, 8);
    s_log.log(PID_UREA_RAW, u, 8);
  }
  uint8_t ar[5 + NB_READ_RECORD_MAX];   // automatic read result (PID 0x39F), one per record
  int arn = autoReadForRecord(ar, sizeof(ar));
  if (arn) { r.wire.log(PID_AUTO_READ, ar, arn); s_log.log(PID_AUTO_READ, ar, arn); }
  uint8_t rd[3 + NB_READ_RECORD_MAX];   // READ result (nbread.h, PID 0x39D)
  int rn = nbReadForRecord(rd, sizeof(rd));
  if (rn) { r.wire.log(PID_READ_RESULT, rd, rn); s_log.log(PID_READ_RESULT, rd, rn); }
  uint8_t ap[64];                  // APN trial outcome (nbapn.h), once
  int an = nbApnTakeStatus(ap, sizeof(ap));
  if (an) { r.wire.log(PID_APN_STATUS, ap, an); s_log.log(PID_APN_STATUS, ap, an); }
  uint8_t ota[64];                 // one pending OTA step line per record (v1F PID 0x399)
  int on = otaTakeStatus(ota, sizeof(ota));
  if (on) { r.wire.log(0x399, ota, on); s_log.log(0x399, ota, on); }
  uint32_t v[6];
  if (txqSdDigestClosed(v)) {
    r.u32n(PID_SD_DIGEST, v, 6);
  } else if ((!lastDigest || millis() - lastDigest >= SD_DIGEST_MS) && txqSdDigest(v)) {
    r.u32n(PID_SD_DIGEST, v, 6);
    lastDigest = millis();
  }
  r.push(ts);
}

// PG-p11 + PG-p12 (3.11.2026): the refuel candidates side by side, compared later with the receipts.
// The level at the last engine STOP is kept (0x612 raw, the last BSI D8C3 / D8C4 reads); when the
// 0x432 refuel counter changes: one status line (+ SD) with the counter, 0x432 byte 4 and the levels
// before / now. The first counter seen after boot only arms it (no line). '-' = not known this boot.
// Sensors cap at 70.0 L, a full tank is ~74.6 L (receipts 28.10 / 2.11) - the comparison shows which fits.
static void refuelTick(const CarValues& cv)
{
  static bool wasOn, armed;
  static uint8_t lastCnt;
  static int stop612 = -1, stopC3 = -1, stopC4 = -1;
  if (wasOn && !s_engineOn) {                  // STOP: the level before a refuel
    stop612 = cv.fuelAt ? cv.fuelRaw : -1;
    stopC3 = cv.fuelBsiAt ? cv.fuelBsiCl : -1;
    stopC4 = cv.fuelCalcAt ? cv.fuelCalcCl : -1;
  }
  wasOn = s_engineOn;
  if (!cv.refuelAt) return;
  if (!armed) { armed = true; lastCnt = cv.refuelCnt; return; }
  if (cv.refuelCnt == lastCnt) return;
  char b612[12], bc3[12], bc4[12], n612[12];   // 11 = "-2147483648" + 0 (ESP-IDF -Wformat-truncation, 5.11.2026)
  if (stop612 >= 0) snprintf(b612, sizeof(b612), "%d", stop612); else strcpy(b612, "-");
  if (stopC3 >= 0) snprintf(bc3, sizeof(bc3), "%d", stopC3); else strcpy(bc3, "-");
  if (stopC4 >= 0) snprintf(bc4, sizeof(bc4), "%d", stopC4); else strcpy(bc4, "-");
  if (cv.fuelAt && millis() - cv.fuelAt < CAR_FRESH_MS) snprintf(n612, sizeof(n612), "%u", cv.fuelRaw); else strcpy(n612, "-");
  char m[57];
  snprintf(m, sizeof(m), "REFUEL cnt %u>%u b4 %u 612 %s>%s C3 %s C4 %s",
           lastCnt, cv.refuelCnt, cv.refuelVol, b612, n612, bc3, bc4);
  lastCnt = cv.refuelCnt;
  statusLine(m);
  logNetEvent(m);
}

static void engineTick()
{
  CarValues cv = carValues();
  uint32_t ms = millis();
  time_t now = time(nullptr);
  bool started = false, stopped = false;
  if (!s_engineOn) {
    if (cv.rpmPosAt && ms - cv.rpmPosAt < 3000) {
      s_engineOn = true;
      started = true;
      logNetEvent("ENGINE START");
      makeRecord(1, now);
    }
  } else if (!cv.rpmAt || ms - cv.rpmAt > ENGINE_SILENT_MS) {
    s_engineOn = false;
    stopped = true;
    time_t when = now - (time_t)((ms - cv.rpmPosAt) / 1000);   // the last rpm > 0
    logNetEvent("ENGINE STOP");
    makeRecord(2, when);
  }
  gnssErrTick(cv, s_engineOn, started, stopped);   // PG151
  refuelTick(cv);
}

// p67 (27.10.2026, as v1F telelogger.ino 2874-2882 DATA_INTERVAL_TABLE / STATIONARY_TIME_TABLE):
// moving = the engine runs (K0 rpm, or the 12 V >= ENGINE_OFF_VOLTAGE = charging) or GPS >= 3 km/h;
// a record every 1 s while moving, then 2 s (still 10-60 s), 5 s (60-180 s), REC_S after 180 s
static uint32_t recordEveryMs()
{
  static uint32_t lastMove, lastCheck;
  static bool moving;
  uint32_t ms = millis();
  if (!lastCheck || ms - lastCheck >= 1000) {
    lastCheck = ms;
    GnssFix fx = fixGet();
    bool gpsMove = fx.valid && ms - fx.at < 5000 && fx.kph >= 3.0f;
    moving = s_engineOn || read12V() >= ENGINE_OFF_VOLTAGE || gpsMove;
    if (moving || !lastMove) lastMove = ms;
  }
  uint32_t still = (ms - lastMove) / 1000;
  if (still < 10) return 1000;
  if (still < 60) return 2000;
  if (still < 180) return 5000;
  return s_recS * 1000UL;
}

static void recordTask(void*)
{
  uint32_t last = 0;
  for (;;) {
    engineTick();
    uint32_t every = recordEveryMs();
    if (!last || millis() - last >= every || s_recNow) {
      last = millis();
      makeRecord();
      s_recNow = false;
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

// PG-p1 (2.11.2026): (b) data session not back within 15 min -> modem restart; NAVRH 9: also 3 failed PPP
// recoveries in a row (OVMS s_netloss_retry_limit = 3, ovms_cellular.cpp:55, 721-751); (c, PG-p4) on the modem
// and no server answer (ACK / reply / ping) for 15 min -> network check (asked by netTask, done here):
// internet works = the server's side, no restart; internet down = restart, again at the earliest in 60 min
#define CELL_NET_RESTART_MS 900000UL
#define CELL_ACK_RESTART_MS 900000UL
#define CELL_ACK_RESTART_GAP_MS 3600000UL
#define CELL_PPP_RETRY_MS 30000UL      // OVMS gsmpppos.cpp:175-179: PPP reconnect after 30 s
#define CELL_PPP_RETRIES 3
static volatile bool s_cellRestartReq, s_netCheckReq;
static char s_cellRestartWhy[40];

// PG-p4: one line - where does the link break? NAVRH 9: the modem's own AT+CPING / AT+CDNSGIP are gone with
// the AT data path; the same questions through lwIP over PPP: DNS of google.com and of our server, a TCP
// connection to 8.8.8.8:53 and 1.1.1.1:53 (TCP instead of ICMP ping: lwIP has no ping in our code)
static bool tcpOk(const char* ip, uint16_t port)
{
  NetworkClient c;
  bool ok = c.connect(ip, port, 5000);
  c.stop();
  return ok;
}

static bool netCheck()
{
  s_cell.makeDefault();                          // the check goes over PPP
  bool t8 = tcpOk("8.8.8.8", 53);
  bool t1 = tcpOk("1.1.1.1", 53);
  IPAddress a;
  bool dns = WiFi.hostByName("google.com", a);
  bool sdns = WiFi.hostByName(SERVER_HOST, a);
  char m[57];
  snprintf(m, sizeof(m), "NETCHK tcp8=%d tcp1=%d dns=%s srv_dns=%s ppp=%d", t8 ? 1 : 0, t1 ? 1 : 0,
           dns ? "ok" : "err", sdns ? "ok" : "err", s_cell.up() ? 1 : 0);
  statusLine(m);
  if (s_cell.udpDiag) s_cell.udpDiag(m);           // also over HTTPS (OTA server log) when that works
  return t8 || t1 || dns;
}

// ---- PG-p8 (2.11.2026): SMS channel - works with the registration only, no data session needed ----------
// Settings (FleetNest command or web): SMS_ALLOW=<num>[,<num>[,<num>]] (the first = the car's owner) and
// optional SMS_PIN=<4-8 chars> (a code in the text, not the SIM PIN); while SMS_ALLOW is empty no SMS command
// is accepted. SMS from an allowed number ("<PIN> " in front when a PIN is set): "STAV" | "CELL RESTART" |
// "OTA" | "SMS <number> <text>" (the box sends <text> to <number>; SMS from that number in the next 15 min
// are forwarded to the one who asked - e.g. 900 "DATA MR STAV" of 4ka). FleetNest / web: SMS=<number>,<text>
// ('_' = space; answers into a status line). Every received SMS: status line + SD, then deleted from the
// SIM. A wrong PIN gets no answer (no cost, nothing to guess from).
// Data outage: the modem answers, registered, no server answer for 30 min -> one SMS to the owner (the first
// SMS_ALLOW number), at most once per 24 h.
#define SMS_POLL_MS 60000UL
#define SMS_FWD_MS  900000UL
#define SMS_OUTAGE_MS 1800000UL
static char s_smsAllow[64], s_smsPin[12];
static char s_smsReqNum[24], s_smsReqText[120];
static volatile bool s_smsReq;
static char s_fwdFrom[24], s_fwdTo[24];       // forward SMS from s_fwdFrom to s_fwdTo until s_fwdUntil
static uint32_t s_fwdUntil;

static void smsLoad()
{
  size_t l = sizeof(s_smsAllow);
  if (nvs_get_str(nvs, "SMS_ALLOW", s_smsAllow, &l) != ESP_OK) s_smsAllow[0] = 0;
  l = sizeof(s_smsPin);
  if (nvs_get_str(nvs, "SMS_PIN", s_smsPin, &l) != ESP_OK) s_smsPin[0] = 0;
}

static bool smsNumberOk(const char* n)        // digits, a leading '+', 3-20 chars
{
  int len = strlen(n);
  if (len < 3 || len > 20) return false;
  for (int i = 0; i < len; i++) if (!isdigit((unsigned char)n[i]) && !(i == 0 && n[i] == '+')) return false;
  return true;
}

// the same number in +421... / 0... form: the last 9 digits (short numbers as they are)
static bool smsSameNumber(const char* a, const char* b)
{
  int la = strlen(a), lb = strlen(b);
  if (la < 9 || lb < 9) return !strcmp(a, b);
  return !strcmp(a + la - 9, b + lb - 9);
}

static bool smsAllowed(const char* from)
{
  if (!s_smsAllow[0]) return false;
  char list[64];
  snprintf(list, sizeof(list), "%s", s_smsAllow);
  for (char* t = strtok(list, ","); t; t = strtok(nullptr, ",")) if (smsSameNumber(t, from)) return true;
  return false;
}

// SMS_ALLOW= / SMS_PIN= / SMS= from FleetNest or the web; the answer text into out
void telemetrySmsSetting(const char* cmd, char* out, int cap)
{
  if (!strncmp(cmd, "SMS_ALLOW=", 10)) {
    char list[64];
    snprintf(list, sizeof(list), "%s", cmd + 10);
    char chk[64];
    snprintf(chk, sizeof(chk), "%s", list);
    int n = 0;
    bool ok = true;
    for (char* t = strtok(chk, ","); t; t = strtok(nullptr, ",")) { n++; ok = ok && smsNumberOk(t); }
    if (!ok || n > 3) { snprintf(out, cap, "CMD SMS_ALLOW ERR numbers"); return; }
    nvs_set_str(nvs, "SMS_ALLOW", list); nvs_commit(nvs);
    smsLoad();
    snprintf(out, cap, "CMD SMS_ALLOW OK n=%d", n);
  } else if (!strncmp(cmd, "SMS_PIN=", 8)) {
    const char* p = cmd + 8;
    int len = strlen(p);
    bool ok = len == 0 || (len >= 4 && len <= 8);
    for (int i = 0; ok && i < len; i++) ok = isalnum((unsigned char)p[i]);
    if (!ok) { snprintf(out, cap, "CMD SMS_PIN ERR 4-8 a-z 0-9"); return; }
    nvs_set_str(nvs, "SMS_PIN", p); nvs_commit(nvs);
    smsLoad();
    snprintf(out, cap, len ? "CMD SMS_PIN OK" : "CMD SMS_PIN OFF");   // never echoed
  } else if (!strncmp(cmd, "SMS=", 4)) {
    const char* c = strchr(cmd + 4, ',');
    char num[24];
    snprintf(num, sizeof(num), "%.*s", c ? (int)(c - cmd - 4) : 0, cmd + 4);
    if (!c || !smsNumberOk(num) || !c[1]) { snprintf(out, cap, "CMD SMS ERR"); return; }
    char text[sizeof(s_smsReqText)];
    snprintf(text, sizeof(text), "%s", c + 1);
    for (char* p = text; *p; p++) if (*p == '_') *p = ' ';
    // PG-p9: the server may resend a command (lost UDP) - the same number + text within 10 min = sent once
    static char lastNum[24], lastText[sizeof(s_smsReqText)];
    static uint32_t lastAt;
    if (lastAt && millis() - lastAt < 600000UL && !strcmp(lastNum, num) && !strcmp(lastText, text)) {
      snprintf(out, cap, "CMD SMS queued (repeat ignored)");
      return;
    }
    if (s_smsReq) { snprintf(out, cap, "CMD SMS ERR busy"); return; }
    snprintf(lastNum, sizeof(lastNum), "%s", num);
    snprintf(lastText, sizeof(lastText), "%s", text);
    lastAt = millis();
    snprintf(s_smsReqNum, sizeof(s_smsReqNum), "%s", num);
    snprintf(s_smsReqText, sizeof(s_smsReqText), "%s", text);
    s_fwdTo[0] = 0;                              // the answer goes into a status line, not forwarded
    snprintf(s_fwdFrom, sizeof(s_fwdFrom), "%s", num);
    s_smsReq = true;
    snprintf(out, cap, "CMD SMS queued");
  }
}

static void smsStatusLine(const char* tag, const char* from, const char* text)
{
  char full[200];
  snprintf(full, sizeof(full), "%s %s: %s", tag, from, text);
  logNetEvent(full);                             // the whole text on SD
  char m[57];
  snprintf(m, sizeof(m), "%s", full);            // FleetNest: up to 3 lines of 56
  statusLine(m);
  int n = strlen(full);
  for (int at = 56; at < n && at < 168; at += 56) { snprintf(m, sizeof(m), "%s", full + at); statusLine(m); }
}

static void smsSendLogged(const char* num, const char* text)
{
  bool ok = s_cell.smsSend(num, text);
  char m[57];
  snprintf(m, sizeof(m), "SMS OUT %s %s", num, ok ? "ok" : "ERR");
  statusLine(m);
  logNetEvent(m);
}

// one received SMS: a command from an allowed number, a forwarded answer, or only logged
static void smsHandle(const char* from, const char* text)
{
  if (s_fwdUntil && millis() < s_fwdUntil && smsSameNumber(from, s_fwdFrom)) {
    smsStatusLine("SMS IN", from, text);
    if (s_fwdTo[0]) {
      char f[160];
      snprintf(f, sizeof(f), "%s: %s", from, text);
      smsSendLogged(s_fwdTo, f);
    }
    return;
  }
  bool allowed = smsAllowed(from);
  int pl = strlen(s_smsPin);
  if (pl) pl++;                                  // "<PIN> "
  if (!allowed || (pl && (strncmp(text, s_smsPin, pl - 1) || text[pl - 1] != ' '))) {
    smsStatusLine(allowed ? "SMS IN badpin" : "SMS IN", from, allowed ? "-" : text);   // a wrong PIN is not echoed
    return;
  }
  const char* c = text + pl;
  char m[57];
  snprintf(m, sizeof(m), "SMS CMD %s: %.30s", from, c);
  statusLine(m);
  logNetEvent(m);
  char reply[160];
  if (!strncasecmp(c, "STAV", 4)) {
    char net[40];
    s_cell.netInfo(net, sizeof(net));
    snprintf(reply, sizeof(reply), "%s fw %s %s rssi=%d data=%s q=%lu gps=%.5f,%.5f %s", devid, FW_BUILD_STR, net,
             s_cell.rssi(), s_cell.up() ? "up" : "DOWN", (unsigned long)txqPending(), fixGet().lat, fixGet().lng,
             s_engineOn ? "engine" : "off");
  } else if (!strncasecmp(c, "CELL RESTART", 12)) {
    strcpy(s_cellRestartWhy, "SMS");
    s_cellRestartReq = true;
    snprintf(reply, sizeof(reply), "%s CELL RESTART ok", devid);
  } else if (!strncasecmp(c, "OTA", 3)) {
    otaRequest();
    snprintf(reply, sizeof(reply), "%s OTA check ok", devid);
  } else if (!strncasecmp(c, "SMS ", 4)) {
    char num[24];
    const char* sp = strchr(c + 4, ' ');
    snprintf(num, sizeof(num), "%.*s", sp ? (int)(sp - c - 4) : 0, c + 4);
    if (!sp || !smsNumberOk(num) || !sp[1]) {
      snprintf(reply, sizeof(reply), "%s SMS ERR: <PIN> SMS <cislo> <text>", devid);
    } else {
      smsSendLogged(num, sp + 1);
      snprintf(s_fwdFrom, sizeof(s_fwdFrom), "%s", num);
      snprintf(s_fwdTo, sizeof(s_fwdTo), "%s", from);
      s_fwdUntil = millis() + SMS_FWD_MS;
      snprintf(reply, sizeof(reply), "%s SMS to %s sent, answers forwarded 15 min", devid, num);
    }
  } else {
    snprintf(reply, sizeof(reply), "%s ? STAV | CELL RESTART | OTA | SMS <cislo> <text>", devid);
  }
  smsSendLogged(from, reply);
}

// cellTask: a queued SMS= and the SIM's received SMS (every minute; the modem answers AT)
static void smsTick()
{
  static uint32_t lastPoll;
  // NAVRH 9: no otaBusy() any more - the OTA download goes over PPP (port 2), SMS over port 4
  if (!s_cellOn || s_standby || !s_cell.alive_()) return;
  if (s_smsReq) {
    smsSendLogged(s_smsReqNum, s_smsReqText);
    s_fwdUntil = millis() + SMS_FWD_MS;
    s_smsReq = false;
    lastPoll = 0;
  }
  if (lastPoll && millis() - lastPoll < SMS_POLL_MS) return;
  lastPoll = millis();
  char from[24], text[170];
  for (int i = 0; i < 5 && s_cell.smsTake(from, sizeof(from), text, sizeof(text)); i++) smsHandle(from, text);
  if (s_fwdUntil && millis() >= s_fwdUntil) s_fwdUntil = 0;
  // data outage -> the owner
  static uint32_t aliveSince, lastAlert;
  if (!aliveSince) aliveSince = millis();
  uint32_t ref = (int32_t)(s_client.lastSyncTime - aliveSince) > 0 ? s_client.lastSyncTime : aliveSince;
  if (s_smsAllow[0] && millis() - ref > SMS_OUTAGE_MS && (!lastAlert || millis() - lastAlert > 86400000UL)) {
    char net[40];
    s_cell.netInfo(net, sizeof(net));
    if (strstr(net, " home") || strstr(net, " roam")) {   // SMS needs the registration
      lastAlert = millis();
      char owner[24], m[160];
      snprintf(owner, sizeof(owner), "%.*s", (int)strcspn(s_smsAllow, ","), s_smsAllow);
      snprintf(m, sizeof(m), "%s BEZ DAT %lu min: %s rssi=%d data=%s q=%lu gps=%.5f,%.5f fw %s", devid,
               (unsigned long)((millis() - ref) / 60000), net, s_cell.rssi(), s_cell.up() ? "up" : "DOWN",
               (unsigned long)txqPending(), fixGet().lat, fixGet().lng, FW_BUILD_STR);
      statusLine("SMS ALERT no data 30 min");
      smsSendLogged(owner, m);
    }
  }
}

// the modem: started when CELL=1, again every 5 min after a failure
static void cellTask(void*)
{
  uint32_t lastTry = 0;
  uint32_t lostAt = 0, lastReopen = 0, tries = 0, ackRestartAt = 0;
  for (;;) {
    if (s_cellOn && !s_cellUp && !s_standby && (!lastTry || millis() - lastTry > 300000UL)) {
      lastTry = millis();
      logNetEvent("CELL START");
      s_cell.setApn(nbApnPick(nvs, s_apn));   // a server APN= is tried first (nbapn.h)
      s_cellUp = s_cell.start();
      // NAVRH 9: "PPP failed" (registered, no data session) = the APN's fault, as "NETOPEN failed" before
      if (!s_cellUp && !strncmp(s_cell.status(), "PPP failed", 10)) { nbApnAfterLogin(nvs, false); loadApn(); }
      char m[120];
      snprintf(m, sizeof(m), "CELL %s apn_profile=%s", s_cell.status(), s_cell.storedApn());
      logNetEvent(m);
      snprintf(m, sizeof(m), "CELL %.24s mux=%d gps=%c/%d st=%d/%us", s_cell.status(), s_cell.muxOn() ? 1 : 0,
               s_cell.gpsMode(), s_cell.gpsStartOk() ? 1 : 0, s_cell.startStep(),
               (unsigned)(s_cell.startMs() / 1000));   // p79: power key step / seconds
      statusLine(m);                             // p69: visible in FleetNest (Stav zariadenia)
      cellStartErr(s_cellUp, s_cell.status());   // PG149
    }
    if (!s_cellOn && s_cellUp) { s_cellUp = false; s_cell.stop(); logNetEvent("CELL OFF"); }
    if (s_cellUp && s_cell.muxDead()) {           // p69: no good CMUX frame for 3 min (OVMS) -> restart
      s_cellUp = false;
      logNetEvent("CELL CMUX dead - restart");
      statusLine("CELL CMUX dead - restart");
      s_cell.stop();
      lastTry = 0;
    }
    // NAVRH 9 (as OVMS NetMode -> NetLoss): PPP lost its IP (lwIP status, +PPPD: DISCONNECTED, NO CARRIER) or
    // the registration is gone -> close PPP, AT+CGATT=0, registered? -> dial again; every 30 s, 3 failures in a
    // row -> modem power cycle; and as PG-p1 (b): no data session for 15 min -> restart as well
    static uint32_t regAt;
    if (s_cellUp && !s_cell.pppLost() && s_cell.up() && millis() - regAt >= 30000UL) {   // OVMS polls every 30 s
      regAt = millis();
      if (!s_cell.registered() && !lostAt) {
        statusLine("CELL REG LOST");
        lostAt = millis();
        lastReopen = 0;
        tries = 0;
      }
    }
    if (s_cellUp && (s_cell.pppLost() || lostAt)) {
      char m[57];
      if (!lostAt) {
        lostAt = millis();
        lastReopen = 0;
        tries = 0;
        snprintf(m, sizeof(m), "CELL PPP LOST err=%d", s_cell.pppErr());
        statusLine(m);
      }
      if (!lastReopen || millis() - lastReopen >= CELL_PPP_RETRY_MS) {
        lastReopen = millis();
        tries++;
        if (s_cell.netRecover()) {
          snprintf(m, sizeof(m), "CELL PPP OK after %lus tries=%lu ip=%s", (unsigned long)((millis() - lostAt) / 1000),
                   (unsigned long)tries, s_cell.ip());
          statusLine(m);
          lostAt = 0;
          errOk("CELL", "ppp");               // PG149: a failed start before, the session is back
        } else if (tries >= CELL_PPP_RETRIES) {
          snprintf(s_cellRestartWhy, sizeof(s_cellRestartWhy), "PPP %lu tries", (unsigned long)tries);
          s_cellRestartReq = true;
        }
      }
      if (lostAt && millis() - lostAt > CELL_NET_RESTART_MS) {   // PG-p1 (b)
        strcpy(s_cellRestartWhy, "no data session 15 min");
        s_cellRestartReq = true;
      }
    }
    // PG-p4 (c): the server did not answer for 15 min - where is the problem?
    if (ackRestartAt && (int32_t)(s_client.lastSyncTime - ackRestartAt) > 0) ackRestartAt = 0;   // answers again
    if (s_netCheckReq) {
      s_netCheckReq = false;
      if (s_cellUp && s_cell.up()) {
        if (netCheck()) {
          statusLine("SERVER NO ANSWER internet ok");
        } else if (!ackRestartAt || millis() - ackRestartAt >= CELL_ACK_RESTART_GAP_MS) {
          ackRestartAt = millis();
          strcpy(s_cellRestartWhy, "no internet");
          s_cellRestartReq = true;
        } else {
          statusLine("NO INTERNET restart waits (60 min)");
        }
      }
    }
    if (s_cellUp && s_cellRestartReq) {
      char m[57];
      snprintf(m, sizeof(m), "CELL RESTART %s", s_cellRestartWhy);
      statusLine(m);
      logNetEvent(m);
      s_cellUp = false;
      s_cell.stop();
      lastTry = 0;                                   // started again right away (then every 5 min as before)
      lostAt = 0;
      s_cellRestartReq = false;
    } else if (!s_cellUp) {
      s_cellRestartReq = false;                      // nothing to restart
    }
    // PG-p19 (3.11.2026): the modem values the record takes - the record never asks the modem itself
    // (see s_fix). A busy modem (OTA download, CMUX) only delays these readings here, never a record.
    if (s_cellOn && !s_standby) {
      GnssFix g;
      if (s_cell.gnssPushed(&g) < 0 && s_cell.gnss(&g)) { fixSet(g); ttffCheck(); }   // no NMEA: the AT query
    }
    static uint32_t lastCsq;
    if (s_cellUp && !s_standby && (!lastCsq || millis() - lastCsq >= CSQ_EVERY_MS)) {
      lastCsq = millis();
      int q = s_cell.rssi();
      if (q) { s_csq = q; s_csqAt = millis(); }
    }
    smsTick();                                       // PG-p8
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

// the net task (v1F telemetry(): delivery part)
static void netTask(void*)
{
  static char pkt[TXQ_PACKET_BUF];
  s_client.reset();
  NetLink* cur = nullptr;
  uint32_t linkSince = 0;
  for (;;) {
    // WiFi first, the modem when WiFi is not there (v1F: cellular fallback)
    NetLink* want = s_wifi.up() ? (NetLink*)&s_wifi : (s_cellUp && s_cell.up() ? (NetLink*)&s_cell : nullptr);
    if (want != cur) {
      if (cur) cur->close();
      s_client.login = false;
      cur = want;
      if (cur == &s_cell) s_cell.makeDefault();   // NAVRH 9: lwIP route + DNS to PPP (WiFi's DHCP had them)
      if (cur == &s_wifi) WifiLink::makeDefault();   // PG96: and back to WiFi's (netlink.cpp)
      linkSince = millis();
      s_client.setLink(cur);
      char m[40];
      snprintf(m, sizeof(m), "NET LINK %s", cur ? cur->name() : "none");
      nbout.println(m);
      logNetEvent(m);
      lastStep("%s", m);
    }
    if (!cur) {
      vTaskDelay(pdMS_TO_TICKS(1000));
      continue;
    }
    if (cur == &s_wifi) WifiLink::restoreDns();   // PG96: PPP coming up overwrites the DNS (netlink.cpp)
    // PG-p1 (c): on the modem, session not reported lost, but no server answer for 15 min -> modem restart
    if (cur == &s_cell && s_cell.up()) {
      uint32_t ref = s_client.lastSyncTime;
      if ((int32_t)(linkSince - ref) > 0) ref = linkSince;   // the last answer on WiFi does not count
      if (millis() - ref > CELL_ACK_RESTART_MS) {
        s_netCheckReq = true;                        // PG-p4: cellTask checks the network, decides
        linkSince = millis();                        // the next check in 15 min (if still no answer)
      }
    }
    if (!s_client.login) {
      bool ok = s_client.connect();
      if (cur == &s_cell) { nbApnAfterLogin(nvs, ok); loadApn(); }   // a trial APN: kept on login, dropped after 2 failures
      if (!ok) { vTaskDelay(pdMS_TO_TICKS(10000)); continue; }
      nbout.printf("[NET] logged in as %s\n", devid);
      otaVerifyLoggedIn();                       // PG147: a new build is confirmed here
    }
    int n = txqNextPacket(pkt, sizeof(pkt), devid);
    if (n > 0) {
      if (!s_client.transmit(pkt, n)) {
        txqSendFailed();
        s_client.connect(true);
      }
    }
    for (int i = 0; i < 4; i++) s_client.inbound();
    txqSave(false);
    // nothing heard for 2 min (no ACK, no reply): log in again (v1F "Poor connection")
    if (millis() - s_client.lastSyncTime > 120000UL) {
      if (!s_client.ping()) s_client.login = false;
    }
    if (n <= 0) vTaskDelay(pdMS_TO_TICKS(50));
  }
}

void telemetryEarly() { txqEarlyInit(); }      // first thing in setup(): the PSRAM reserve

void telemetryBegin(uint32_t bootId, bool sdOk)
{
  if (s_gpsNext == 'H' || s_gpsNext == 'C' || s_gpsNext == 'N') s_cell.setGpsStart(s_gpsNext);   // p66, one boot
  s_gpsNext = 0;
  if (nvs_open("storage", NVS_READWRITE, &nvs) != ESP_OK) nbout.println("[NVS] storage not open");
  uint32_t r = 60;
  if (nvs_get_u32(nvs, "REC_S", &r) == ESP_OK && r >= 10) s_recS = r;
  char veh[8] = "K0";
  size_t vl = sizeof(veh);
  nvs_get_str(nvs, "VEH", veh, &vl);
  carSetProfile(veh);
  uint8_t c = 0;
  nvs_get_u8(nvs, "CELL", &c);
  s_cellOn = c == 1;
  // APN of the home operator (4ka, as v1F CELL_APN "internet"); it stays the same in roaming -
  // only the visited network changes, the modem picks it. CELL_APN= overrides (stored).
  loadApn();
  smsLoad();                                     // PG-p8
  s_cell.setApn(s_apn);
  s_cell.begin();
  genDeviceID(devid);
  nbout.printf("[TEL] device ID %s\n", devid);
  txqBegin(bootId);
  if (sdOk) {
    uint32_t last = 0;
    nvs_get_u32(nvs, "DATA_ID", &last);
    uint32_t id = s_log.begin(last);
    if (id) { nvs_set_u32(nvs, "DATA_ID", id); nvs_commit(nvs); }
    txqSetFile(id);
    // PG-p14 (3.11.2026, as v1F telelogger.ino 2056-2061): the boot number in every file - the delivery
    // queue's SD replay (RESEND) needs it to rebuild the records' identity; without it every record was
    // skipped (Peugeot file 137 never refilled)
    char bid[24];
    snprintf(bid, sizeof(bid), "BOOTID=%lu", (unsigned long)bootId);
    s_log.logEvent(bid);
    char ev[160], ri[100];
    snprintf(ev, sizeof(ev), "BOOT %lu ID=%s %s", (unsigned long)bootId, devid, bootResetInfo(ri, sizeof(ri)));
    s_log.logEvent(ev);
  } else {
    txqSetFile(0);
  }
  xTaskCreatePinnedToCore(recordTask, "record", 4096, nullptr, 2, nullptr, 1);
  xTaskCreatePinnedToCore(netTask, "net", 8192, nullptr, 2, nullptr, 1);
  xTaskCreatePinnedToCore(cellTask, "cell", 6144, nullptr, 2, nullptr, 1);
}

void telemetryBeforeRestart()
{
  char st[220];
  txqStatus(st, sizeof(st));
  s_log.logEvent(st);
  s_log.sync(3000);
  txqSave(true);
}

// READ command (nbread.h): run a waiting request on its bus; called from loop()
// ---- p84 (28.10.2026): which ECUs the car has - every K0 diagnostic address (Diagbox DB, 52)
// gets 22 F0FE, on silence 22 F080; an answer or a refusal (NRC) = present. Web: PROBE=<bus>, PROBE?
static const uint16_t K0_ADDR[][2] = {{0x590,0x58F}, {0x5B0,0x5AF}, {0x5CA,0x5AA}, {0x6A1,0x681}, {0x6A2,0x682}, {0x6A6,0x686}, {0x6A8,0x688}, {0x6A9,0x689}, {0x6AA,0x68A}, {0x6AC,0x68C}, {0x6AD,0x68D}, {0x6AF,0x68F}, {0x6B4,0x694}, {0x6B5,0x695}, {0x6B6,0x696}, {0x6BA,0x69A}, {0x6BC,0x69C}, {0x6BD,0x69D}, {0x6C4,0x604}, {0x6C8,0x628}, {0x6E5,0x625}, {0x6E6,0x626}, {0x730,0x710}, {0x731,0x711}, {0x732,0x712}, {0x736,0x716}, {0x737,0x717}, {0x73C,0x71C}, {0x742,0x642}, {0x743,0x643}, {0x744,0x644}, {0x747,0x647}, {0x748,0x648}, {0x74A,0x64A}, {0x74D,0x64D}, {0x74F,0x64F}, {0x752,0x652}, {0x755,0x655}, {0x756,0x656}, {0x75D,0x65D}, {0x75F,0x65F}, {0x760,0x660}, {0x764,0x664}, {0x765,0x665}, {0x766,0x666}, {0x76C,0x66C}, {0x76D,0x66D}, {0x770,0x670}, {0x778,0x678}, {0x77B,0x67B}, {0x77C,0x67C}, {0x77F,0x67F}};
#define K0_ADDR_N (sizeof(K0_ADDR) / sizeof(K0_ADDR[0]))
static volatile int s_probeBus, s_probeIdx = -1;
static char s_probeOut[1400];
static int s_probeLen, s_probeFound;
void probeStart(int bus) { s_probeLen = 0; s_probeOut[0] = 0; s_probeFound = 0; s_probeBus = bus; s_probeIdx = 0; }
int probeStatus(char* buf, int cap)
{
  return snprintf(buf, cap, "%s %d/%d found=%d: %s", s_probeIdx < 0 ? "done" : "running",
                  s_probeIdx < 0 ? (int)K0_ADDR_N : s_probeIdx, (int)K0_ADDR_N, s_probeFound, s_probeOut);
}
static bool probeTick()
{
  if (s_probeIdx < 0) return false;
  if (s_probeIdx >= (int)K0_ADDR_N) {
    s_probeIdx = -1;
    char m[56];
    snprintf(m, sizeof(m), "PROBE done bus=%d found=%d", s_probeBus, s_probeFound);
    statusLine(m);
    s_log.logEvent(s_probeOut);
    return true;
  }
  NbRead r = {};
  r.bus = s_probeBus; r.txId = K0_ADDR[s_probeIdx][0]; r.rxId = K0_ADDR[s_probeIdx][1];
  r.req[0] = 0x22; r.req[1] = 0xF0; r.req[2] = 0xFE; r.reqLen = 3;
  static uint8_t resp[NB_READ_MAX];
  int len = 0; uint8_t nrc = 0;
  int res = busExchange(&r, resp, sizeof(resp), &len, &nrc);
  if (res != NBR_OK && res != NBR_NRC) { r.req[2] = 0x80; res = busExchange(&r, resp, sizeof(resp), &len, &nrc); }
  if ((res == NBR_OK || res == NBR_NRC) && s_probeLen < (int)sizeof(s_probeOut) - 16) {
    s_probeFound++;
    s_probeLen += snprintf(s_probeOut + s_probeLen, sizeof(s_probeOut) - s_probeLen, "%X%s ",
                           (unsigned)r.txId, res == NBR_OK ? "" : "n");   // "n" = answered with a refusal
  }
  s_probeIdx++;
  return true;
}

// ---- p86 (28.10.2026): DID scan of one ECU - 22 <DID> for every DID of a range; every positive
// answer as an SD event line "SCAN <tx> <DID> <answer hex>" (download: /api/events/0), counts at the end
// as a FleetNest line. Web: SCAN=<bus>,<tx hex>,<rx hex>,<from hex>,<to hex>; SCAN? ; SCAN=0 stops.
// Only reading (service 22). To find what an ECU knows that the Diagbox DB does not list (speed sign).
// Loop mode (user 28.10: "nebude bežať stále?"): SCAN_AUTO=<same args> is kept in NVS; at every engine
// START the scan starts and repeats until the engine stops - the first pass logs every answer, later
// passes only the DIDs whose answer changed (a CRC per DID in PSRAM), so a sign change shows with its time.
static volatile int32_t s_scanCur = -1;
static uint32_t s_scanFrom, s_scanTo, s_scanTx, s_scanRx, s_scanOk, s_scanNrc, s_scanNone, s_scanPass;
static uint8_t s_scanBus;
static bool s_scanLoop;
static uint32_t* s_scanCrc;                   // one CRC per DID of the range (0 = no answer yet)
static char s_scanAuto[40];
static uint32_t crcOf(const uint8_t* d, int n) { uint32_t c = 2166136261u; for (int i = 0; i < n; i++) c = (c ^ d[i]) * 16777619u; return c | 1; }
static bool scanParse(const char* a, unsigned* bus, unsigned* tx, unsigned* rx, unsigned* from, unsigned* to)
{
  return sscanf(a, "%u,%x,%x,%x,%x", bus, tx, rx, from, to) == 5 && *bus >= 1 && *bus <= 3 && *from <= *to && *to <= 0xFFFF;
}
static bool scanBegin(const char* a, bool loop)
{
  unsigned bus, tx, rx, from, to;
  if (!scanParse(a, &bus, &tx, &rx, &from, &to)) return false;
  s_scanBus = bus; s_scanTx = tx; s_scanRx = rx; s_scanFrom = from; s_scanTo = to; s_scanLoop = loop;
  s_scanOk = s_scanNrc = s_scanNone = 0; s_scanPass = 1;
  if (s_scanCrc) { free(s_scanCrc); s_scanCrc = nullptr; }
  s_scanCrc = (uint32_t*)ps_calloc(to - from + 1, sizeof(uint32_t));
  char m[80];
  snprintf(m, sizeof(m), "SCAN start bus=%u %X->%X %04X-%04X%s", bus, tx, rx, from, to, loop ? " loop" : "");
  s_log.logEvent(m);
  s_scanCur = from;
  return true;
}
bool scanStart(const char* a) { return scanBegin(a, false); }
bool scanSetAuto(const char* a)                // "" = off
{
  unsigned b, t, r, f, e;
  if (a[0] && !scanParse(a, &b, &t, &r, &f, &e)) return false;
  snprintf(s_scanAuto, sizeof(s_scanAuto), "%s", a);
  nvs_set_str(nvs, "SCAN_AUTO", s_scanAuto); nvs_commit(nvs);
  return true;
}
static void scanAutoTick()                     // from autoReadTick's START/STOP view
{
  static bool was;
  static bool loaded;
  if (!loaded) { size_t l = sizeof(s_scanAuto); nvs_get_str(nvs, "SCAN_AUTO", s_scanAuto, &l); loaded = true; }
  bool on = s_engineOn;
  if (on && !was && s_scanAuto[0]) scanBegin(s_scanAuto, true);
  if (!on && was && s_scanLoop) { s_scanCur = -1; s_log.logEvent("SCAN loop stop (engine off)"); }
  was = on;
}
void scanStop() { s_scanCur = -1; }
int scanStatus(char* buf, int cap)
{
  return snprintf(buf, cap, "%s%s pass=%u auto='%s' %X cur=%04X to=%04X ok=%u nrc=%u none=%u", s_scanCur < 0 ? "idle" : "running",
                  s_scanLoop ? " loop" : "", (unsigned)s_scanPass, s_scanAuto,
                  (unsigned)s_scanTx, (unsigned)(s_scanCur < 0 ? 0 : s_scanCur), (unsigned)s_scanTo,
                  (unsigned)s_scanOk, (unsigned)s_scanNrc, (unsigned)s_scanNone);
}
static bool scanTick()
{
  if (s_scanCur < 0) return false;
  static uint32_t lastReq;                   // at most 20 requests/s: the BSI forwards each one to the
  if (millis() - lastReq < 50) return true;  // slower body bus of the ECU (user 28.10: bus load)
  lastReq = millis();
  if ((uint32_t)s_scanCur > s_scanTo && s_scanLoop) {        // next pass
    char m[64];
    snprintf(m, sizeof(m), "SCAN pass %u done ok=%u nrc=%u none=%u", (unsigned)s_scanPass,
             (unsigned)s_scanOk, (unsigned)s_scanNrc, (unsigned)s_scanNone);
    s_log.logEvent(m);
    if (s_scanPass == 1) statusLine(m);
    s_scanPass++; s_scanOk = s_scanNrc = s_scanNone = 0;
    s_scanCur = s_scanFrom;
    return true;
  }
  if ((uint32_t)s_scanCur > s_scanTo) {
    s_scanCur = -1;
    char m[56];
    snprintf(m, sizeof(m), "SCAN done %X ok=%u nrc=%u none=%u", (unsigned)s_scanTx,
             (unsigned)s_scanOk, (unsigned)s_scanNrc, (unsigned)s_scanNone);
    statusLine(m);
    s_log.logEvent(m);
    return true;
  }
  NbRead r = {};
  r.bus = s_scanBus; r.txId = s_scanTx; r.rxId = s_scanRx;
  r.req[0] = 0x22; r.req[1] = (uint8_t)(s_scanCur >> 8); r.req[2] = (uint8_t)s_scanCur; r.reqLen = 3;
  static uint8_t resp[NB_READ_MAX];
  int len = 0; uint8_t nrc = 0;
  int res = busExchange(&r, resp, sizeof(resp), &len, &nrc);
  if (res == NBR_OK) {
    s_scanOk++;
    uint32_t* slot = s_scanCrc ? &s_scanCrc[s_scanCur - s_scanFrom] : nullptr;
    uint32_t c = crcOf(resp, len);
    if (slot && *slot == c) { s_scanCur++; return true; }  // unchanged since the last pass: not logged
    if (slot) *slot = c;
    char line[300];
    int n = snprintf(line, sizeof(line), "SCAN %X %04X ", (unsigned)s_scanTx, (unsigned)s_scanCur);
    for (int i = 0; i < len && n < (int)sizeof(line) - 3; i++) n += snprintf(line + n, sizeof(line) - n, "%02X", resp[i]);
    s_log.logEvent(line);
  } else if (res == NBR_NRC) s_scanNrc++;
  else s_scanNone++;
  s_scanCur++;
  return true;
}

void readTick()
{
  scanAutoTick();                           // p86 loop: start at engine START, stop at STOP
  if (probeTick()) return;                  // p84: the probe owns the bus while it runs
  if (scanTick()) return;                   // p86
  NbRead r;
  uint32_t seq;
  if (!nbReadTake(&r, &seq)) { autoReadTick(); return; }
  static uint8_t resp[NB_READ_MAX];
  int len = 0;
  uint8_t nrc = 0;
  int res = busExchange(&r, resp, sizeof(resp), &len, &nrc);
  nbReadSetResult(seq, res, nrc, resp, len);
  char line[300];
  nbReadLine(line, sizeof(line), seq, &r, res, nrc, resp, len);
  nbout.println(line);
  s_log.logEvent(line);
}

// /api/live (web page): what the box sees now
int telemetryLive(char* buf, int size)
{
  CarValues cv = carValues();
  GnssFix fx = fixGet();
  uint32_t ms = millis();
  auto age = [&](uint32_t at) -> long { return at ? (long)(ms - at) : -1L; };
  return snprintf(buf, size,
      "{\"v\":%.2f,\"engine\":%d,\"veh\":\"%s\",\"rpm\":%.0f,\"rpm_age\":%ld,\"eng_state\":%u,\"odo\":%lu,\"odo_age\":%ld,"
      "\"fuel\":%.1f,\"fuel_age\":%ld,\"urea_km\":%lu,\"urea_age\":%ld,"
      "\"gps\":{\"fix\":%d,\"lat\":%.6f,\"lng\":%.6f,\"kph\":%.1f,\"sats\":%u,\"age\":%ld},"
      "\"net\":{\"link\":\"%s\",\"login\":%d,\"cell\":\"%s\",\"tx\":%lu,\"bytes\":%lu},"
      "\"can1\":{\"frames\":%lu,\"kbit\":%lu,\"found\":%d,\"last_age\":%ld},\"file\":%lu,\"pending\":%lu}",
      read12V(), s_engineOn ? 1 : 0, carProfile(), cv.rpm, age(cv.rpmAt), cv.engState, (unsigned long)cv.odoKm, age(cv.odoAt),
      cv.fuelL, age(cv.fuelAt), (unsigned long)cv.ureaKm, age(cv.ureaAt),
      fx.valid ? 1 : 0, fx.lat, fx.lng, fx.kph, (unsigned)fx.sats, age(fx.at),
      s_client.link ? s_client.link->name() : "-", s_client.login ? 1 : 0, s_cell.status(),
      (unsigned long)s_client.txCount, (unsigned long)s_client.txBytes,
      (unsigned long)can1Stats().frames, (unsigned long)can1Stats().kbit, can1Stats().speedFound ? 1 : 0,
      age(can1Stats().lastFrameMs), (unsigned long)s_log.id(), (unsigned long)txqPending());
}

int telemetryModemAt(const char* cmd, char* out, int cap) { return s_cell.raw(cmd, 5000, out, cap); }
