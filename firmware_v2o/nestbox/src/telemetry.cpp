// Records to FleetNest - NestBox_v2O step 2a (WiFi). The path of NestBox_v1F:
// every record goes to the SD file /DATA/<n>.CSV (SD writer, confirmed) and
// into the delivery queue (txqueue.cpp, copied); the net task sends the queue
// in packets and drops a record only after the server's ACK (stored).
// Identity of a record = (boot, PID 0 = ms since boot), as v1F.
//
// Not yet (later steps): GNSS position (modem, step 2b), engine START/STOP
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

#define PID_SD_DIGEST  0x398
#define SD_DIGEST_MS   600000UL
#define PID_SNIFF_LEFT 0x393     // sniff minutes left (v1F: the server sees the sniff state)
#define ADC_12V_PIN    36        // ADC1 channel 0 (OVMS esp32adc)
#define ADC_12V_FACTOR 195.7f    // raw / factor = volts (OVMS default system.adc factor12v; this unit NOT calibrated)

extern "C" uint8_t temprature_sens_read();

nvs_handle_t nvs;                // txqueue.cpp uses it (v1F: the same "storage" namespace)
static DataLog s_log;
static TeleClientUDP s_client;
static WifiLink s_wifi;
static volatile uint32_t s_recS = 60;
static volatile bool s_otaReq;

void logNetEvent(const char* text) { s_log.logEvent(text); }
void otaReadyRequested() { s_otaReq = true; }   // pull-OTA (v1F) not ported yet: logged only
bool telemetryLogin() { return s_client.login; }
uint32_t telemetryRecS() { return s_recS; }
void telemetrySetRecS(uint32_t s) { s_recS = s < 10 ? 10 : s; nvs_set_u32(nvs, "REC_S", s_recS); nvs_commit(nvs); }
uint32_t telemetryFileId() { return s_log.id(); }
int telemetryStatus(char* buf, int size)
{
  int n = snprintf(buf, size, "devid=%s login=%d link=%s tx=%lu txB=%lu rxB=%lu file=%lu rec_s=%lu ota_req=%d ",
                   devid, s_client.login ? 1 : 0, s_wifi.up() ? "WIFI" : "-", (unsigned long)s_client.txCount,
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

static void makeRecord()
{
  static Rec r;
  static uint32_t lastDigest;
  uint32_t ts = millis();
  r.begin(ts);
  time_t now = time(nullptr);
  if (now > 1735689600) {        // a valid clock: date and time as v1F addTimeOf()
    struct tm u;
    gmtime_r(&now, &u);
    r.u32(0x10, (uint32_t)u.tm_hour * 1000000 + u.tm_min * 10000 + u.tm_sec * 100);
    r.u32(0x11, (uint32_t)u.tm_mday * 10000 + (u.tm_mon + 1) * 100 + (u.tm_year % 100));
  }
  r.f(PID_BATTERY_VOLTAGE, read12V() * 100, "%.0f");    // v1F: 0.01 V units (FleetNest: /100 V)
  r.i32(PID_DEVICE_TEMP, (int32_t)((temprature_sens_read() - 32) / 1.8f));
  if (WiFi.status() == WL_CONNECTED) r.i32(PID_CSQ, WiFi.RSSI());
  if (sniffActive()) r.u32(PID_SNIFF_LEFT, sniffMinutesLeft());
  uint32_t v[6];
  if (txqSdDigestClosed(v)) {
    r.u32n(PID_SD_DIGEST, v, 6);
  } else if ((!lastDigest || millis() - lastDigest >= SD_DIGEST_MS) && txqSdDigest(v)) {
    r.u32n(PID_SD_DIGEST, v, 6);
    lastDigest = millis();
  }
  r.push(ts);
}

static void recordTask(void*)
{
  uint32_t last = 0;
  for (;;) {
    if (!last || millis() - last >= s_recS * 1000UL) {
      last = millis();
      makeRecord();
    }
    vTaskDelay(pdMS_TO_TICKS(200));
  }
}

// the net task (v1F telemetry(): delivery part)
static void netTask(void*)
{
  static char pkt[TXQ_PACKET_BUF];
  s_client.setLink(&s_wifi);
  s_client.reset();
  bool wasUp = false;
  for (;;) {
    bool up = s_wifi.up();
    if (!up) {
      if (wasUp) { s_client.login = false; s_wifi.close(); Serial.println("[NET] WiFi down"); }
      wasUp = false;
      vTaskDelay(pdMS_TO_TICKS(1000));
      continue;
    }
    if (!wasUp || !s_client.login) {
      wasUp = true;
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
}

void telemetryBeforeRestart()
{
  char st[220];
  txqStatus(st, sizeof(st));
  s_log.logEvent(st);
  s_log.sync(3000);
  txqSave(true);
}
