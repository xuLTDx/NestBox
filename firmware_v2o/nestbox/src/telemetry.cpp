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
#include <Arduino.h>
#include <WiFi.h>
#include <Preferences.h>
#include <nvs.h>
#include "store.h"
#include "txqueue.h"
#include "sddigest.h"
#include "sdwriter.h"
#include "teleclient.h"
#include "sniff.h"
#include "cellmodem.h"
#include "cardecode.h"
#include "canbus1.h"
#include "ota.h"
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
#define PID_UREA_EST   0x3A3     // p81: READ CMM D517 urea estimated, ml
#define PID_CREW       0x3A4     // p83: bit0 driver belt, bit1 passenger belt, bit2 passenger seated
#define PID_SPEED_DISP 0x3A5     // p83: speed as on the cluster (BSI D8F9), km/h
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

nvs_handle_t nvs;                // txqueue.cpp uses it (v1F: the same "storage" namespace)
static DataLog s_log;
static TeleClientUDP s_client;
static WifiLink s_wifi;
static CellLink s_cell;
static volatile bool s_cellOn;               // NVS CELL (default 0 until tested on 12 V)
static volatile bool s_cellUp;
static GnssFix s_fix;                        // last valid reading
// p66: GNSS start mode for the boot after a GPS_TEST sleep (one-shot), time to the first fix reported once
RTC_DATA_ATTR static char s_gpsNext;
static bool s_ttffSent;
void telemetryGpsStartNext(char mode) { s_gpsNext = mode; }
static void ttffCheck()
{
  if (s_ttffSent || !s_fix.valid || !s_cell.gpsStartedAt()) return;
  s_ttffSent = true;
  char m[57];
  snprintf(m, sizeof(m), "GPS ttff=%lus boot=%lus mode=%c ok=%d sats=%u",
           (unsigned long)((s_fix.at - s_cell.gpsStartedAt()) / 1000), (unsigned long)(s_fix.at / 1000),
           s_cell.gpsMode(), s_cell.gpsStartOk() ? 1 : 0, (unsigned)s_fix.sats);
  statusLine(m);
}
static char s_apn[40] = "internet";          // the CELL_APN in force (NVS), home operator 4ka
static volatile uint32_t s_recS = 60;
static volatile bool s_otaReq;

void logNetEvent(const char* text) { s_log.logEvent(text); }
static void loadApn()
{
  char a[40] = "";
  size_t l = sizeof(a);
  if (nvs_get_str(nvs, "CELL_APN", a, &l) == ESP_OK && a[0]) snprintf(s_apn, sizeof(s_apn), "%s", a);
  else snprintf(s_apn, sizeof(s_apn), "internet");
}
void telemetryApnCommand(const char* arg) { nbApnCommand(nvs, arg, "internet"); }
void telemetrySetMux(bool on) { nvs_set_u8(nvs, "CELL_MUX", on ? 1 : 0); nvs_commit(nvs); s_cell.setMuxWanted(on); }
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
                   (unsigned long)s_cell.muxErrors(), s_fix.valid ? 1 : 0, (unsigned)s_fix.sats, (unsigned long)s_client.txCount,
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
    if (!txqPush(ts, buf, len)) Serial.println("[TXQ] record not queued");
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
  if (!s_cellOn || !s_cell.gnss(&g)) return false;
  s_fix = g;
  ttffCheck();
  return true;
}
// standby.cpp: modem (and its GNSS) off before the deep sleep; the cell task does not start it again
void telemetryCellOff()
{
  s_standby = true;
  if (s_cellOn) { s_cellUp = false; s_cell.powerOff(); logNetEvent("CELL OFF standby"); }
}
CellLink* telemetryCellLink() { return s_cellUp ? &s_cell : nullptr; }   // pull-OTA over cellular

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
  {0x6A8, 0x688, {0xD5, 0x17}, {2, 1}},   // CMM: urea volume estimated, byte 5 x 0.0625 L (DB)
  {0x6A8, 0x688, {0xD5, 0x13}, {2, 1}},   // CMM: km before the urea refill (raw)
  {0x6A8, 0x688, {0xD4, 0x15}, {2, 1}},   // CMM: coolant (raw, the DB's -40 not plausible 28.10)
  {0x752, 0x652, {0xDB, 0xED}, {2, 1}},   // BSI: km to the next service
  {0x752, 0x652, {0xDC, 0x5F}, {2, 1}},   // BSI: months to the next service
  {0x6A8, 0x688, {0xD5, 0x1F}, {2, 1}},   // CMM: engine oil dilution
  {0x6A8, 0x688, {0xD4, 0x29}, {2, 1}},   // CMM: oil pressure
  {0x74A, 0x64A, {0xD4, 0x09}, {1, 2}},   // CVM camera: speed sign read (no answer on CAN2 28.10)
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
  if (s_cellOn && s_cell.gnss(&g)) { s_fix = g; ttffCheck(); }
  bool fresh = s_fix.valid && millis() - s_fix.at < 5000;
  if (fresh && event) {          // an event record carries the event's time (above), the position only
    r.f(PID_GPS_LATITUDE, s_fix.lat, "%f");
    r.f(PID_GPS_LONGITUDE, s_fix.lng, "%f");
    if (s_fix.sats) r.u32(PID_GPS_SAT_COUNT, s_fix.sats);
  } else if (fresh) {                   // as v1F processGPS(): the GNSS time also sets the clock
    struct tm t = {};
    t.tm_mday = s_fix.date / 10000; t.tm_mon = (s_fix.date / 100) % 100 - 1; t.tm_year = s_fix.date % 100 + 100;
    t.tm_hour = s_fix.time / 1000000; t.tm_min = (s_fix.time / 10000) % 100; t.tm_sec = (s_fix.time / 100) % 100;
    time_t gnow = mktime(&t), now0 = time(nullptr);
    if (gnow > 1735689600 && (now0 > gnow + 2 || now0 < gnow - 2)) { struct timeval tv = {gnow, 0}; settimeofday(&tv, nullptr); }
    r.u32(0x10, s_fix.time);
    r.u32(0x11, s_fix.date);
    r.f(PID_GPS_LATITUDE, s_fix.lat, "%f");
    r.f(PID_GPS_LONGITUDE, s_fix.lng, "%f");
    r.f(PID_GPS_ALTITUDE, s_fix.alt, "%.1f");
    r.f(PID_GPS_SPEED, s_fix.kph, "%.1f");
    r.u32(PID_GPS_HEADING, s_fix.heading);
    if (s_fix.sats) r.u32(PID_GPS_SAT_COUNT, s_fix.sats);
    if (s_fix.hdop > 0) r.u32(PID_GPS_HDOP, (uint32_t)(s_fix.hdop + 0.5f));
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
  else if (s_cellUp) { int q = s_cell.rssi(); if (q) r.i32(PID_CSQ, q); }
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
  if (cv.ureaEstAt && ms - cv.ureaEstAt < 660000UL) r.u32(PID_UREA_EST, cv.ureaEstMl);
  if (cv.crewAt && ms - cv.crewAt < 90000UL) r.u32(PID_CREW, cv.crew);
  if (cv.speedDispAt && ms - cv.speedDispAt < 90000UL) r.u32(PID_SPEED_DISP, cv.speedDisp);
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

static void engineTick()
{
  CarValues cv = carValues();
  uint32_t ms = millis();
  time_t now = time(nullptr);
  if (!s_engineOn) {
    if (cv.rpmPosAt && ms - cv.rpmPosAt < 3000) {
      s_engineOn = true;
      logNetEvent("ENGINE START");
      makeRecord(1, now);
    }
  } else if (!cv.rpmAt || ms - cv.rpmAt > ENGINE_SILENT_MS) {
    s_engineOn = false;
    time_t when = now - (time_t)((ms - cv.rpmPosAt) / 1000);   // the last rpm > 0
    logNetEvent("ENGINE STOP");
    makeRecord(2, when);
  }
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
    bool gpsMove = s_fix.valid && ms - s_fix.at < 5000 && s_fix.kph >= 3.0f;
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

// the modem: started when CELL=1, again every 5 min after a failure
static void cellTask(void*)
{
  uint32_t lastTry = 0;
  for (;;) {
    if (s_cellOn && !s_cellUp && !s_standby && (!lastTry || millis() - lastTry > 300000UL)) {
      lastTry = millis();
      logNetEvent("CELL START");
      s_cell.setApn(nbApnPick(nvs, s_apn));   // a server APN= is tried first (nbapn.h)
      s_cellUp = s_cell.start();
      // the data connection failed after registration (NETOPEN) = the APN's fault: counts for a trial
      if (!s_cellUp && !strcmp(s_cell.status(), "NETOPEN failed")) { nbApnAfterLogin(nvs, false); loadApn(); }
      char m[120];
      snprintf(m, sizeof(m), "CELL %s apn_profile=%s", s_cell.status(), s_cell.storedApn());
      logNetEvent(m);
      snprintf(m, sizeof(m), "CELL %.24s mux=%d gps=%c/%d st=%d/%us", s_cell.status(), s_cell.muxOn() ? 1 : 0,
               s_cell.gpsMode(), s_cell.gpsStartOk() ? 1 : 0, s_cell.startStep(),
               (unsigned)(s_cell.startMs() / 1000));   // p79: power key step / seconds
      statusLine(m);                             // p69: visible in FleetNest (Stav zariadenia)
    }
    if (!s_cellOn && s_cellUp) { s_cellUp = false; s_cell.stop(); logNetEvent("CELL OFF"); }
    if (s_cellUp && s_cell.muxDead()) {           // p69: no good CMUX frame for 3 min (OVMS) -> restart
      s_cellUp = false;
      logNetEvent("CELL CMUX dead - restart");
      statusLine("CELL CMUX dead - restart");
      s_cell.stop();
      lastTry = 0;
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

// the net task (v1F telemetry(): delivery part)
static void netTask(void*)
{
  static char pkt[TXQ_PACKET_BUF];
  s_client.reset();
  NetLink* cur = nullptr;
  for (;;) {
    // WiFi first, the modem when WiFi is not there (v1F: cellular fallback)
    NetLink* want = s_wifi.up() ? (NetLink*)&s_wifi : (s_cellUp ? (NetLink*)&s_cell : nullptr);
    if (want != cur) {
      if (cur) cur->close();
      s_client.login = false;
      cur = want;
      s_client.setLink(cur);
      char m[40];
      snprintf(m, sizeof(m), "NET LINK %s", cur ? cur->name() : "none");
      Serial.println(m);
      logNetEvent(m);
      lastStep("%s", m);
    }
    if (!cur) {
      vTaskDelay(pdMS_TO_TICKS(1000));
      continue;
    }
    if (!s_client.login) {
      bool ok = s_client.connect();
      if (cur == &s_cell) { nbApnAfterLogin(nvs, ok); loadApn(); }   // a trial APN: kept on login, dropped after 2 failures
      if (!ok) { vTaskDelay(pdMS_TO_TICKS(10000)); continue; }
      Serial.printf("[NET] logged in as %s\n", devid);
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
  if (nvs_open("storage", NVS_READWRITE, &nvs) != ESP_OK) Serial.println("[NVS] storage not open");
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
  s_cell.setApn(s_apn);
  uint8_t mux = 1;                               // p69: CMUX on unless CELL_MUX=0 (local web)
  nvs_get_u8(nvs, "CELL_MUX", &mux);
  s_cell.setMuxWanted(mux != 0);
  s_cell.begin();
  genDeviceID(devid);
  Serial.printf("[TEL] device ID %s\n", devid);
  txqBegin(bootId);
  if (sdOk) {
    uint32_t last = 0;
    nvs_get_u32(nvs, "DATA_ID", &last);
    uint32_t id = s_log.begin(last);
    if (id) { nvs_set_u32(nvs, "DATA_ID", id); nvs_commit(nvs); }
    txqSetFile(id);
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
static volatile int32_t s_scanCur = -1;
static uint32_t s_scanTo, s_scanTx, s_scanRx, s_scanOk, s_scanNrc, s_scanNone;
static uint8_t s_scanBus;
bool scanStart(const char* a)
{
  unsigned bus, tx, rx, from, to;
  if (sscanf(a, "%u,%x,%x,%x,%x", &bus, &tx, &rx, &from, &to) != 5 || bus < 1 || bus > 3 || from > to || to > 0xFFFF) return false;
  s_scanBus = bus; s_scanTx = tx; s_scanRx = rx; s_scanTo = to;
  s_scanOk = s_scanNrc = s_scanNone = 0;
  char m[80];
  snprintf(m, sizeof(m), "SCAN start bus=%u %X->%X %04X-%04X", bus, tx, rx, from, to);
  s_log.logEvent(m);
  s_scanCur = from;
  return true;
}
void scanStop() { s_scanCur = -1; }
int scanStatus(char* buf, int cap)
{
  return snprintf(buf, cap, "%s %X cur=%04X to=%04X ok=%u nrc=%u none=%u", s_scanCur < 0 ? "idle" : "running",
                  (unsigned)s_scanTx, (unsigned)(s_scanCur < 0 ? 0 : s_scanCur), (unsigned)s_scanTo,
                  (unsigned)s_scanOk, (unsigned)s_scanNrc, (unsigned)s_scanNone);
}
static bool scanTick()
{
  if (s_scanCur < 0) return false;
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
  Serial.println(line);
  s_log.logEvent(line);
}

// /api/live (web page): what the box sees now
int telemetryLive(char* buf, int size)
{
  CarValues cv = carValues();
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
      s_fix.valid ? 1 : 0, s_fix.lat, s_fix.lng, s_fix.kph, (unsigned)s_fix.sats, age(s_fix.at),
      s_client.link ? s_client.link->name() : "-", s_client.login ? 1 : 0, s_cell.status(),
      (unsigned long)s_client.txCount, (unsigned long)s_client.txBytes,
      (unsigned long)can1Stats().frames, (unsigned long)can1Stats().kbit, can1Stats().speedFound ? 1 : 0,
      age(can1Stats().lastFrameMs), (unsigned long)s_log.id(), (unsigned long)txqPending());
}

int telemetryModemAt(const char* cmd, char* out, int cap) { return s_cell.raw(cmd, 5000, out, cap); }
