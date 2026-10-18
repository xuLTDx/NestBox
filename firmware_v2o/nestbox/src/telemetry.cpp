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
#include "nbread.h"

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
#define PID_ENG_STATE  0x39C     // NEW: K0 0x348 engine running state (1 off, 3 running, 4 stop-start ...)
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
static volatile uint32_t s_recS = 60;
static volatile bool s_otaReq;

void logNetEvent(const char* text) { s_log.logEvent(text); }
void telemetrySetCell(bool on) { s_cellOn = on; nvs_set_u8(nvs, "CELL", on ? 1 : 0); nvs_commit(nvs); }
bool telemetryCell() { return s_cellOn; }
void telemetrySetVehicle(const char* p) { carSetProfile(p); nvs_set_str(nvs, "VEH", carProfile()); nvs_commit(nvs); }
void telemetrySetApn(const char* apn) { s_cell.setApn(apn); nvs_set_str(nvs, "CELL_APN", apn); nvs_commit(nvs); }
void otaReadyRequested() { s_otaReq = true; otaRequest(); }   // server OTA_READY -> pull-OTA (ota.cpp)
bool telemetryLogin() { return s_client.login; }
uint32_t telemetryRecS() { return s_recS; }
void telemetrySetRecS(uint32_t s) { s_recS = s < 10 ? 10 : s; nvs_set_u32(nvs, "REC_S", s_recS); nvs_commit(nvs); }
uint32_t telemetryFileId() { return s_log.id(); }
int telemetryStatus(char* buf, int size)
{
  int n = snprintf(buf, size, "devid=%s login=%d link=%s cell=%d/%s fix=%d sats=%u tx=%lu txB=%lu rxB=%lu file=%lu rec_s=%lu ota_req=%d ",
                   devid, s_client.login ? 1 : 0, s_client.link ? s_client.link->name() : "-", s_cellOn ? 1 : 0,
                   s_cell.status(), s_fix.valid ? 1 : 0, (unsigned)s_fix.sats, (unsigned long)s_client.txCount,
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
  if (s_cellOn && s_cell.gnss(&g)) s_fix = g;
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
  if (cv.ureaAt && ms - cv.ureaAt < CAR_FRESH_MS) {
    r.u32(PID_UREA_KM, cv.ureaKm);
    uint8_t* u = cv.urea;
    r.wire.log(PID_UREA_RAW, u, 8);
    s_log.log(PID_UREA_RAW, u, 8);
  }
  uint8_t rd[3 + NB_READ_RECORD_MAX];   // READ result (nbread.h, PID 0x39D)
  int rn = nbReadForRecord(rd, sizeof(rd));
  if (rn) { r.wire.log(PID_READ_RESULT, rd, rn); s_log.log(PID_READ_RESULT, rd, rn); }
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

static void recordTask(void*)
{
  uint32_t last = 0;
  for (;;) {
    engineTick();
    uint32_t every = s_engineOn ? DRIVE_REC_MS : s_recS * 1000UL;
    if (!last || millis() - last >= every) {
      last = millis();
      makeRecord();
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

// the modem: started when CELL=1, again every 5 min after a failure
static void cellTask(void*)
{
  uint32_t lastTry = 0;
  for (;;) {
    if (s_cellOn && !s_cellUp && (!lastTry || millis() - lastTry > 300000UL)) {
      lastTry = millis();
      logNetEvent("CELL START");
      s_cellUp = s_cell.start();
      char m[120];
      snprintf(m, sizeof(m), "CELL %s apn_profile=%s", s_cell.status(), s_cell.storedApn());
      logNetEvent(m);
    }
    if (!s_cellOn && s_cellUp) { s_cellUp = false; s_cell.stop(); logNetEvent("CELL OFF"); }
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
    }
    if (!cur) {
      vTaskDelay(pdMS_TO_TICKS(1000));
      continue;
    }
    if (!s_client.login) {
      if (!s_client.connect()) { vTaskDelay(pdMS_TO_TICKS(10000)); continue; }
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
  char apn[40] = "internet";
  size_t al = sizeof(apn);
  nvs_get_str(nvs, "CELL_APN", apn, &al);
  s_cell.setApn(apn);
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
    char ev[64];
    snprintf(ev, sizeof(ev), "BOOT %lu ID=%s", (unsigned long)bootId, devid);
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
void readTick()
{
  NbRead r;
  uint32_t seq;
  if (!nbReadTake(&r, &seq)) return;
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
