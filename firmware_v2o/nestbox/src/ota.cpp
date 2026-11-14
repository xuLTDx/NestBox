// see ota.h
#include "crash.h"
#include <esp_core_dump.h>
#include <esp_flash.h>
#include "ota.h"
#include "laststep.h"
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <esp_ota_ops.h>
#include <esp_partition.h>
#include "otaverify.h"
#include <nvs.h>
#include <mbedtls/sha256.h>
#include "nb_sd.h"
#include "store.h"
#include "cellmodem.h"
#include "netlog.h"
#include <errno.h>

extern const char FW_BUILD_STR[];
#include "wifiplaces.h"
#include "standby.h"
extern nvs_handle_t nvs;
void logNetEvent(const char* text);
void prepareRestart(const char* why);
void telemetryCellOff();
bool telemetryEngineOn();
bool telemetryWifiForData();                 // PG164: WiFi joined and with internet (telemetry.cpp)
class CellLink;
CellLink* telemetryCellLink();
float read12V();

// Flash only with the engine off and a battery that is not weak (same rule as v1F,
// Vojtech 2026-10-19: >= 11.5 V before the flash); under 7 V = bench (USB), not a car.
#define OTA_FLASH_MIN_VOLTAGE 11.5f

static bool flashAllowed()
{
  if (telemetryEngineOn()) return false;
  const float v = read12V();
  return v < 7.0f || v >= OTA_FLASH_MIN_VOLTAGE;
}

// the marker: in this firmware's image, never in a v1F one
__attribute__((used)) const char NESTBOX_IMAGE_MARK[] = "NESTBOX_V2O_IMAGE_MARK_5f1d2c";

#define OTA_PENDING_PATH "/ota_fw.bin"
#define OTA_META_PATH    "/ota_meta.txt"
#define OTA_RESUME_PATH  "/ota_resume.txt"
#define OTA_MIN_SIZE     65536U
#define OTA_CHUNK        4096
#define OTA_TIMEOUT_MS   15000
#define OTA_PERIODIC_MS  (6UL * 3600UL * 1000UL)
#define OTA_VARIANT      "NestBox_v2O"

static char s_token[68], s_host[128] = "ultd.mooo.com";
static uint16_t s_port = 6001;               // freematics-ota LISTEN_PORT (v1F default)
static volatile bool s_req;
static volatile bool s_busy;                 // a check / download runs (standby waits)
bool otaBusy() { return s_busy; }
static uint8_t s_retries;                    // p74: a broken download is tried again after 60 s,
static uint32_t s_retryAt;                   // resumed from the partial file, at most 10 times
static volatile bool s_staged;               // a verified image waits in the other partition for the engine to stop
static uint8_t s_buf[OTA_CHUNK];

// -------------------------------------------------------------- status ---
#define OTA_STATUS_Q 24      // p42: several OTA lines in a row (was for the CCH diagnostics)
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;
static char s_q[OTA_STATUS_Q][64];
static uint8_t s_head, s_count;
static char s_last[64] = "-";

static void otaLog(const char* text)
{
  {
    char m[120];                              // PG99/PG98: on /api/serial with the internal + DMA memory at that moment
    snprintf(m, sizeof(m), "%s | heap=%u dma_max=%u", text, (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
             (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_DMA));
    netlogLine(m);
  }
  logNetEvent(text);
  nbout.println(text);
  portENTER_CRITICAL(&s_mux);
  uint8_t slot = (s_head + s_count) % OTA_STATUS_Q;
  if (s_count == OTA_STATUS_Q) s_head = (s_head + 1) % OTA_STATUS_Q;
  else s_count++;
  strncpy(s_q[slot], text, sizeof(s_q[0]) - 1);
  s_q[slot][sizeof(s_q[0]) - 1] = 0;
  strncpy(s_last, text, sizeof(s_last) - 1);
  portEXIT_CRITICAL(&s_mux);
}

// <= 56 chars: as byte codes ("101;" = 4 chars each) the record line stays in CStorage::log's 256 B
void statusLine(const char* text)
{
  char t[57];
  snprintf(t, sizeof(t), "%s", text);
  otaLog(t);
}

int otaTakeStatus(uint8_t* out, int cap)
{
  int n = 0;
  portENTER_CRITICAL(&s_mux);
  if (s_count) {
    n = strnlen(s_q[s_head], cap);
    memcpy(out, s_q[s_head], n);
    s_head = (s_head + 1) % OTA_STATUS_Q;
    s_count--;
  }
  portEXIT_CRITICAL(&s_mux);
  return n;
}

int otaStatus(char* buf, int size)
{
  return snprintf(buf, size, "token=%d host=%s:%u staged=%d last=%s", s_token[0] ? 1 : 0, s_host, (unsigned)s_port,
                  s_staged ? 1 : 0, s_last);
}

const char* otaTokenForHmac() { return s_token; }   // PG214 (telemetry.cpp telemetrySmsCfgReport): the HMAC key only
void telemetrySmsCfgReport(int what);
bool otaSet(const char* key, const char* value)
{
  if (!strcmp(key, "OTA_TOKEN")) {
    snprintf(s_token, sizeof(s_token), "%s", value); nvs_set_str(nvs, "OTA_TOKEN", s_token);
    nvs_commit(nvs);
    telemetrySmsCfgReport(2);                  // PG214: the PIN's fingerprint is keyed by the token
    return true;
  }
  else if (!strcmp(key, "OTA_HOST")) { snprintf(s_host, sizeof(s_host), "%s", value); nvs_set_str(nvs, "OTA_HOST", s_host); }
  else if (!strcmp(key, "OTA_PORT")) { s_port = (uint16_t)atoi(value); nvs_set_u16(nvs, "OTA_PORT", s_port); }
  else return false;
  nvs_commit(nvs);
  return true;
}

void otaRequest() { s_req = true; }

// -------------------------------------------------- p71 UDP diagnostics ---
// The cellular UDP link's answers (cellmodem udpDiag): to SD, and over HTTPS to the OTA server
// as GET .../diag?l=<lines> - the server logs every request line (a 404 is fine), so they are
// readable while the records themselves do not get through.
#define UDP_DIAG_Q 16
static char s_d[UDP_DIAG_Q][120];
static uint8_t s_dHead, s_dCount;
static uint32_t s_dAt;

static void udpDiagNote(const char* text)
{
  logNetEvent(text);
  nbout.println(text);
  portENTER_CRITICAL(&s_mux);
  uint8_t slot = (s_dHead + s_dCount) % UDP_DIAG_Q;
  if (s_dCount == UDP_DIAG_Q) s_dHead = (s_dHead + 1) % UDP_DIAG_Q;
  else s_dCount++;
  strncpy(s_d[slot], text, sizeof(s_d[0]) - 1);
  s_d[slot][sizeof(s_d[0]) - 1] = 0;
  portEXIT_CRITICAL(&s_mux);
}

// ---------------------------------------------------------------- HTTP ---
// One HTTPS connection (NetworkClientSecure = mbedTLS in the ESP32) over WiFi or over the modem's PPP -
// lwIP takes the default route (NAVRH 9, 2026-11-05). Before: the modem's own TLS (AT+CCH*) - 4.11.2026
// boot 157 / 164 a 1024-byte block of the download was lost on it (ERR=SHA256), now TCP resends itself.
struct OtaConn {
  virtual bool connect(const char* host, uint16_t port) = 0;
  virtual void write(const char* s) = 0;
  virtual void writeRaw(const uint8_t* d, int n) = 0;   // PG49: a POST body
  virtual int read(uint8_t* buf, int cap, unsigned int timeoutMs) = 0;   // >0, 0 closed, -1 timeout
  virtual void stop() = 0;
  virtual ~OtaConn() {}
  // buffered line reader on top of read()
  uint8_t rb[1024];
  int rn = 0, rp = 0;
  int getc(unsigned int timeoutMs) {
    if (rp >= rn) { rn = read(rb, sizeof(rb), timeoutMs); rp = 0; if (rn <= 0) { rn = 0; return -1; } }
    return rb[rp++];
  }
  String line(unsigned int timeoutMs) {
    String s;
    for (int c; (c = getc(timeoutMs)) >= 0 && c != '\n';) s += (char)c;
    return s;
  }
  int body(uint8_t* buf, int cap, unsigned int timeoutMs) {   // what the line reader buffered first
    if (rp < rn) { int n = rn - rp < cap ? rn - rp : cap; memcpy(buf, rb + rp, n); rp += n; return n; }
    return read(buf, cap, timeoutMs);
  }
};

struct WifiConn : OtaConn {
  WiFiClientSecure c;
  bool connect(const char* host, uint16_t port) override {
    c.setInsecure();
    c.setTimeout(OTA_TIMEOUT_MS / 1000);
    return c.connect(host, port);
  }
  void write(const char* s) override { c.print(s); }
  void writeRaw(const uint8_t* d, int n) override { c.write(d, n); }
  int read(uint8_t* buf, int cap, unsigned int timeoutMs) override {
    uint32_t t = millis();
    while (!c.available() && c.connected() && millis() - t < timeoutMs) delay(2);
    if (!c.available()) return c.connected() ? -1 : 0;
    return c.read(buf, cap);
  }
  void stop() override { c.stop(); }
};

// The connection lives outside the ota task's stack, as v1F's global otaWifiClient
// (telelogger.ino): on the stack (2 x 1 KB line buffer + the TLS client) the second TLS
// handshake overflowed the 8 KB stack - "Stack canary watchpoint triggered (ota)", serial 21.10.2026.
static WifiConn s_wc;                        // WiFi or PPP: the same TLS client

// one GET; returns the status code (-1 = no connection), *len = Content-Length (-1 unknown).
// The body is then read with c.body().
static String s_lastStatus;                  // the answer's first line (p42 diagnostics)

static int httpGet(OtaConn& c, const char* path, size_t rangeFrom, long* len)
{
  c.rn = c.rp = 0;
  if (!c.connect(s_host, s_port)) return -1;
  char req[900];
  int n = snprintf(req, sizeof(req), "GET %s HTTP/1.1\r\nHost: %s\r\nConnection: close\r\n", path, s_host);
  if (rangeFrom) n += snprintf(req + n, sizeof(req) - n, "Range: bytes=%u-\r\n", (unsigned)rangeFrom);
  snprintf(req + n, sizeof(req) - n, "\r\n");
  c.write(req);
  String status = c.line(OTA_TIMEOUT_MS);
  s_lastStatus = status.substring(0, 40);
  int code = status.length() > 12 ? status.substring(9, 12).toInt() : 0;
  *len = -1;
  for (;;) {
    String h = c.line(OTA_TIMEOUT_MS);
    if (h.length() <= 1) break;             // empty line (just "\r") = end of the headers
    if (h.startsWith("Content-Length:") || h.startsWith("content-length:")) *len = h.substring(15).toInt();
  }
  return code;
}

static String readAll(OtaConn& c)
{
  String s;
  uint8_t b[256];
  for (int n; (n = c.body(b, sizeof(b), OTA_TIMEOUT_MS)) > 0;) s.concat((const char*)b, n);
  return s;
}

static String urlBuild()
{
  String s;
  for (const char* p = FW_BUILD_STR; *p; p++) s += *p == ' ' ? String("%20") : String(*p);
  return s;
}

// PG205 + PG208 (wifiplaces.h): the WiFi networks of the company's places (name, password, position), from the OTA
// server. The answer is never logged (passwords).
// When (Vojtech 14.11.2026: "nejde o ota ale o zoznam WiFi kazde 2 hod, ota nech zostane ako je"): on its own, not
// tied to the OTA check - every PLACES_EVERY_S while the box is up, counted from NVS (survives sleep), first 60 s
// after a boot (otaTask's start delay). Why 2 h: FleetNest sends at most 16 networks, the nearest to the box's last
// position first (FN165) - a car that drove far needs the list of where it is now. Not in the 3-hour parked report
// boot (the car has not moved; 8 fetches a day over the modem for nothing). A requested OTA check (OTA_READY / SMS
// OTA) fetches it at once. After a failed try the next one PLACES_RETRY_MS later (the list it has stays).
#define PLACES_EVERY_S 7200
#define PLACES_RETRY_MS 600000UL
static void placesFetch(bool force, bool clock, time_t now);
static void placesSync(bool force)
{
  if (!s_token[0]) return;
  time_t now = time(nullptr);
  uint32_t last = 0;
  nvs_get_u32(nvs, "WIFI_PL_T", &last);
  bool clock = now > 1735689600;
  static uint32_t triedAt;
  if (!force) {
    if (standbyParkedBoot()) return;
    if (last && (!clock || (uint32_t)now - last < PLACES_EVERY_S)) return;
    if (triedAt && millis() - triedAt < PLACES_RETRY_MS) return;
  }
  triedAt = millis() | 1;
  bool wasBusy = s_busy;                      // the box does not fall asleep in the middle of the fetch
  s_busy = true;
  placesFetch(force, clock, now);
  s_busy = wasBusy;
}

static void placesFetch(bool force, bool clock, time_t now)
{
  CellLink* cell = telemetryWifiForData() ? nullptr : telemetryCellLink();
  if (!telemetryWifiForData() && !cell) return;
  char path[200];
  snprintf(path, sizeof(path), "/api/nestbox/ota/%s/wifi.txt", s_token);
  if (cell) cell->makeDefault();
  else WifiLink::makeDefault();
  OtaConn& c = s_wc;
  long len = 0;
  int code = httpGet(c, path, 0, &len);
  if (code != 200) {
    char m[48];
    snprintf(m, sizeof(m), "WIFI PLACES ERR http=%d", code);
    otaLog(m);
    c.stop();
    return;
  }
  String body = readAll(c);
  c.stop();
  bool changed = false;
  int total = 0, usable = 0;
  if (!wifiPlacesStore(body, &changed, &total, &usable)) { otaLog("WIFI PLACES ERR bad list"); return; }
  if (clock) { nvs_set_u32(nvs, "WIFI_PL_T", (uint32_t)now); nvs_commit(nvs); }
  if (changed || force) {
    char m[56];
    snprintf(m, sizeof(m), "WIFI PLACES %d of %d stored%s", usable, total, usable ? "" : " - built-in networks");
    otaLog(m);
  }
}

// -------------------------------------------------------------- check ---
// PG191 (14.11.2026, Vojtech "191 ano"): the download goes straight into the other app partition - no SD card
// needed. Why: an update must not depend on the card (Peugeot 12.11.: SD state unknown, OTA went through SD;
// U9K4M3G6 9.11.: pull-OTA failed because its SD died 72-80 s after boot). The running image is never touched;
// the box switches to the new one only after the partition was read back and its sha256 equals the server's,
// with the engine off, and the PG147 rule (login in 15 min or back) stays as it was.
// NVS (namespace of telemetry.cpp): the target of the download in progress (size + sha256), how far it got
// (saved every OTAD_SAVE bytes - a resume after a restart or a power loss continues from there) and "verified,
// waits for the engine to stop".
#define KEY_D_SIZE "OTAD_SIZE"
#define KEY_D_SHA  "OTAD_SHA"
#define KEY_D_OFF  "OTAD_OFF"
#define KEY_D_OK   "OTAD_OK"
#define OTAD_SAVE  0x10000U                   // 64 KB: 21 NVS writes per 1.3 MB image
#define OTAD_SECTOR 4096U

static void dlClear()
{
  nvs_erase_key(nvs, KEY_D_SIZE);
  nvs_erase_key(nvs, KEY_D_SHA);
  nvs_erase_key(nvs, KEY_D_OFF);
  nvs_erase_key(nvs, KEY_D_OK);
  nvs_commit(nvs);
}

// the partition read back: sha256 of its first <size> bytes equals <sha>, and the NestBox_v2O marker is in it
// (*mark). Chunks overlap by the marker length, so a marker across two chunks is found.
static bool partVerify(const esp_partition_t* part, size_t size, const char* sha, bool* mark)
{
  const int ml = strlen(NESTBOX_IMAGE_MARK);
  static uint8_t win[OTA_CHUNK + 40];
  int keep = 0;
  bool found = false;
  mbedtls_sha256_context ctx;
  mbedtls_sha256_init(&ctx);
  mbedtls_sha256_starts(&ctx, 0);
  size_t done = 0;
  while (done < size) {
    int n = size - done < OTA_CHUNK ? size - done : OTA_CHUNK;
    if (esp_partition_read(part, done, win + keep, n) != ESP_OK) break;
    mbedtls_sha256_update(&ctx, win + keep, n);
    int tot = keep + n;
    if (!found)
      for (int i = 0; i + ml <= tot; i++)
        if (win[i] == (uint8_t)NESTBOX_IMAGE_MARK[0] && !memcmp(win + i, NESTBOX_IMAGE_MARK, ml)) { found = true; break; }
    keep = ml - 1 < tot ? ml - 1 : tot;
    memmove(win, win + tot - keep, keep);
    done += n;
    if ((done & 0xFFFF) == 0) vTaskDelay(1);
  }
  uint8_t dig[32];
  mbedtls_sha256_finish(&ctx, dig);
  mbedtls_sha256_free(&ctx);
  if (mark) *mark = found;
  if (done != size) return false;
  char hex[65];
  for (int i = 0; i < 32; i++) sprintf(hex + 2 * i, "%02x", dig[i]);
  return strcasecmp(hex, sha) == 0;
}

static void check()
{
  if (!s_token[0]) { otaLog("OTA-PULL STOP no token"); return; }
  CellLink* cell = telemetryWifiForData() ? nullptr : telemetryCellLink();   // PG164: not a WiFi without internet
  if (!telemetryWifiForData() && !cell) { otaLog("OTA-PULL STOP no wifi, no cell"); return; }
  if (s_staged) { otaLog("OTA-PULL STAGED waits for engine off"); return; }
  // PG147: an unconfirmed build does not fetch the next one - the next restart would throw it away anyway
  if (otaVerifyPending()) { otaLog("OTA-PULL WAIT build not confirmed (no login yet)"); return; }
  char path[300];
  snprintf(path, sizeof(path), "/api/nestbox/ota/%s/meta.json?build=%s&variant=%s", s_token,
           urlBuild().c_str(), OTA_VARIANT);
  if (cell) cell->makeDefault();              // NAVRH 9: the download goes over PPP
  else WifiLink::makeDefault();               // PG96: WiFi's route + DNS (PPP may have overwritten the DNS)
  OtaConn& c = s_wc;
  otaLog(cell ? "OTA-PULL CHECK cell" : "OTA-PULL CHECK wifi");
  long len = 0;
  int code = httpGet(c, path, 0, &len);
  if (code != 200) {
    char m[48];
    snprintf(m, sizeof(m), "OTA-PULL ERR=META_HTTP%d", code);
    otaLog(m);
    char st[64];
    snprintf(st, sizeof(st), "OTA-PULL STATUS '%s' len=%u", s_lastStatus.c_str(), s_lastStatus.length());
    otaLog(st);
    c.stop();
    return;
  }
  String body = readAll(c);
  c.stop();
  if (body.indexOf("\"available\": true") < 0 && body.indexOf("\"available\":true") < 0) {
    otaLog("OTA-PULL NO_UPDATE");
    s_retries = 0;
    return;
  }
  int si = body.indexOf("\"size\":");
  size_t size = si >= 0 ? (size_t)body.substring(si + 7).toInt() : 0;
  int hi = body.indexOf("\"sha256\":");
  String sha;
  if (hi >= 0) {
    int a = body.indexOf('"', hi + 9);
    int b = a >= 0 ? body.indexOf('"', a + 1) : -1;
    if (a >= 0 && b > a) sha = body.substring(a + 1, b);
  }
  if (size < OTA_MIN_SIZE || sha.length() != 64) { otaLog("OTA-PULL ERR=META_FIELDS"); return; }
  // PG147: this image went back to the previous build once (crash / no login in 15 min) - never again
  // (each try = download + flash + 15 min awake on a parked car); a new build from the server has a new sha256
  if (otaVerifyRejected(sha.c_str())) { otaLog("OTA-PULL SKIP rejected sha256 (went back once)"); s_retries = 0; return; }
  char m[64];
  snprintf(m, sizeof(m), "OTA-PULL START SIZE=%u", (unsigned)size);
  otaLog(m);

  const esp_partition_t* part = esp_ota_get_next_update_partition(nullptr);
  if (!part || size > part->size) { otaLog("OTA-PULL ERR=PARTITION"); return; }
  // resume: the bytes already in the partition belong to this target when size and sha256 match the NVS note
  size_t from = 0;
  {
    uint32_t ds = 0, off = 0;
    char dsha[65] = "";
    size_t l = sizeof(dsha);
    if (nvs_get_u32(nvs, KEY_D_SIZE, &ds) == ESP_OK && nvs_get_str(nvs, KEY_D_SHA, dsha, &l) == ESP_OK &&
        ds == size && sha.equalsIgnoreCase(dsha) && nvs_get_u32(nvs, KEY_D_OFF, &off) == ESP_OK &&
        off > 0 && off < size && off % OTAD_SAVE == 0)
      from = off;
  }
  if (!from) {
    dlClear();
    nvs_set_u32(nvs, KEY_D_SIZE, (uint32_t)size);
    nvs_set_str(nvs, KEY_D_SHA, sha.c_str());
    nvs_set_u32(nvs, KEY_D_OFF, 0);
    nvs_commit(nvs);
  } else {
    snprintf(m, sizeof(m), "OTA-PULL RESUME from %u", (unsigned)from);
    otaLog(m);
  }
  snprintf(path, sizeof(path), "/api/nestbox/ota/%s/firmware.bin", s_token);
  lastStep("OTA GET firmware from %u", (unsigned)from);
  code = httpGet(c, path, from, &len);
  size_t written = from;
  uint32_t t0 = millis();
  if (code == 200 && from) written = 0;       // no Range support: start over
  bool flashErr = false;
  if (code == 200 || code == 206) {
    while (written < size) {
      int want = size - written < OTA_CHUNK ? size - written : OTA_CHUNK;
      int n = c.body(s_buf, want, OTA_TIMEOUT_MS);
      if (n <= 0) {                           // PG99: which side stopped - the network ...
        char d[64];
        snprintf(d, sizeof(d), "OTA download: network read %d at %u", n, (unsigned)written);
        netlogLine(d);
        break;
      }
      // every 4 KB sector is erased right before its first byte is written (one sector at a time: the flash is
      // busy for tens of ms, not for the seconds a whole-partition erase takes)
      esp_err_t e = ESP_OK;
      for (size_t sec = (written + OTAD_SECTOR - 1) & ~(size_t)(OTAD_SECTOR - 1); sec < written + n && e == ESP_OK;
           sec += OTAD_SECTOR)
        e = esp_partition_erase_range(part, sec, OTAD_SECTOR);
      if (e == ESP_OK) e = esp_partition_write(part, written, s_buf, n);
      if (e != ESP_OK) {                      // ... or the flash
        char d[80];
        snprintf(d, sizeof(d), "OTA download: flash write %d bytes failed err=0x%x at %u", n, (unsigned)e, (unsigned)written);
        netlogLine(d);
        flashErr = true;
        break;
      }
      size_t before = written;
      written += n;
      if (written / OTAD_SAVE != before / OTAD_SAVE) {   // the resume point, a multiple of 64 KB
        nvs_set_u32(nvs, KEY_D_OFF, (uint32_t)(written / OTAD_SAVE * OTAD_SAVE));
        nvs_commit(nvs);
      }
      lastStep("OTA download %u/%u", (unsigned)written, (unsigned)size);
    }
  }
  c.stop();
  lastStep("OTA download end %u/%u", (unsigned)written, (unsigned)size);
  if (written != size) {
    snprintf(m, sizeof(m), "OTA-PULL ERR=%s %u/%u http=%d in %ums", flashErr ? "FLASH_WR" : "RECV", (unsigned)written,
             (unsigned)size, code, (unsigned)(millis() - t0));
    otaLog(m);                                // the partial image stays: the next check resumes
    if (s_retries < 10) { s_retries++; s_retryAt = millis(); }   // p74
    return;
  }
  bool mark = false;
  if (!partVerify(part, size, sha.c_str(), &mark)) {
    otaLog("OTA-PULL ERR=SHA256");
    dlClear();
    return;
  }
  if (!mark) {
    otaLog("OTA-PULL ERR=NOT_V2O_IMAGE (no NestBox_v2O marker) - not flashed");
    dlClear();
    return;
  }
  nvs_set_u32(nvs, KEY_D_OFF, (uint32_t)size);
  nvs_set_u8(nvs, KEY_D_OK, 1);
  nvs_commit(nvs);
  snprintf(path, sizeof(path), "/api/nestbox/ota/%s/ota_confirm?size=%u&sha256=%.16s", s_token,
           (unsigned)size, sha.c_str());
  if (httpGet(c, path, 0, &len) == 200) otaLog("OTA-PULL CONFIRMED");
  c.stop();
  s_staged = true;
  otaLog("OTA-PULL STAGED (flash when the engine is off)");
  s_retries = 0;
}

// the verified image in the other app partition becomes the boot image, then restart. The partition is read
// back once more: between the download and the engine stopping there may be hours, a restart or a local
// /api/ota push into the same partition.
static void flashStaged()
{
  uint32_t ds = 0;
  char fsha[65] = "";
  size_t l = sizeof(fsha);
  nvs_get_u32(nvs, KEY_D_SIZE, &ds);
  nvs_get_str(nvs, KEY_D_SHA, fsha, &l);
  size_t size = ds;
  const esp_partition_t* part = esp_ota_get_next_update_partition(nullptr);
  char m[48];
  snprintf(m, sizeof(m), "OTA-PULL FLASH START SIZE=%u", (unsigned)size);
  otaLog(m);
  bool mark = false;
  if (!part || size < OTA_MIN_SIZE || !partVerify(part, size, fsha, &mark) || !mark ||
      esp_ota_set_boot_partition(part) != ESP_OK) {
    otaLog("OTA-PULL ERR=FLASH");
    s_staged = false;
    dlClear();
    return;
  }
  dlClear();
  otaLog("OTA-PULL FLASH OK - restart");
  otaVerifyFlashed(fsha);                     // PG147
  // p79: the modem off first, as before the sleep (AT+CPOF; fully off after 25-26 s per the SIM7600G
  // Hardware Design table 9) - the new firmware then finds it off and its start() step 1 powers it on
  telemetryCellOff();
  delay(27000);
  prepareRestart("OTA_FLASHED");
  delay(1500);
  esp_restart();
}

// p71: the waiting lines, as many as fit into one GET (encoded, '~' between lines)
static void udpDiagUpload(CellLink* cell)
{
  static char path[700];
  int n = snprintf(path, sizeof(path), "/api/nestbox/ota/%s/diag?l=", s_token);
  int sent = 0;
  for (;;) {
    char line[120];
    portENTER_CRITICAL(&s_mux);
    bool have = sent < s_dCount;
    if (have) strcpy(line, s_d[(s_dHead + sent) % UDP_DIAG_Q]);
    portEXIT_CRITICAL(&s_mux);
    if (!have || n + (int)strlen(line) * 3 + 2 >= (int)sizeof(path)) break;
    if (sent) path[n++] = '~';
    for (const char* p = line; *p; p++) {
      char ch = *p;
      if (isalnum((unsigned char)ch) || ch == '-' || ch == '_' || ch == '.') path[n++] = ch;
      else n += snprintf(path + n, sizeof(path) - n, "%%%02X", (unsigned char)ch);
    }
    path[n] = 0;
    sent++;
  }
  if (!sent) return;
  if (cell) cell->makeDefault();
  else WifiLink::makeDefault();               // PG96
  long len = 0;
  int code = httpGet(s_wc, path, 0, &len);
  s_wc.stop();
  if (code < 0) return;                        // no connection: the lines wait for the next try
  portENTER_CRITICAL(&s_mux);
  s_dHead = (s_dHead + sent) % UDP_DIAG_Q;
  s_dCount -= sent;
  portEXIT_CRITICAL(&s_mux);
}

// ---- PG49 (2026-11-08): one SD data file to the OTA server over HTTPS (WiFi or the modem's PPP) ----------
// Why: Peugeot 7.11.2026, 34 km without records, the box ~1500 km away - its SD files could only be read on the
// local web (/api/log/<n>). FleetNest command LOG=<file>[,<from byte>] (0 = the file written now); the box POSTs
// the file in pieces to .../log?file=<n>&off=<byte>&size=<bytes>, the server appends and answers "next=<byte>"
// (what it has) - after a break the box goes on from there, as the OTA download (p74). The size is taken at the
// start: the file being written is sent up to that point. Status lines: "LOG <n> start size=<b> from=<b>",
// "LOG <n> done <b>", "LOG <n> ERR ...".
#define LOG_CHUNK    8192            // one POST
#define LOG_GAP_MS   2000UL          // between two POSTs - the upload never holds the line long (records first)
#define LOG_RETRY_MS 60000UL         // after an error (no answer, HTTP error), as the OTA retry
#define LOG_TRIES    10
uint32_t telemetryFileId();          // telemetry.cpp: the file written now
static volatile bool s_logReq;
static uint32_t s_logFile, s_logOff, s_logSize, s_logAt, s_logWait;
static uint8_t s_logTries;
static uint8_t* s_logBuf;            // LOG_CHUNK in PSRAM, taken at the first LOG

bool otaLogRequest(uint32_t file, uint32_t from)
{
  if (s_logReq) return false;
  s_logFile = file ? file : telemetryFileId();
  s_logOff = from;
  s_logSize = 0;
  s_logTries = 0;
  s_logAt = 0;
  s_logWait = 0;
  s_logReq = true;
  return true;
}

static int httpPost(OtaConn& c, const char* path, const uint8_t* body, int n, String* answer)
{
  c.rn = c.rp = 0;
  if (!c.connect(s_host, s_port)) return -1;
  char req[400];
  snprintf(req, sizeof(req), "POST %s HTTP/1.1\r\nHost: %s\r\nContent-Type: text/plain\r\nContent-Length: %d\r\n"
           "Connection: close\r\n\r\n", path, s_host, n);
  c.write(req);
  c.writeRaw(body, n);
  String status = c.line(OTA_TIMEOUT_MS);
  int code = status.length() > 12 ? status.substring(9, 12).toInt() : 0;
  for (;;) {
    String h = c.line(OTA_TIMEOUT_MS);
    if (h.length() <= 1) break;
  }
  if (answer) *answer = readAll(c);
  return code;
}

static void logEnd(const char* m) { otaLog(m); s_logReq = false; }

static void logTick()
{
  if (!s_logReq || (s_logAt && millis() - s_logAt < s_logWait)) return;
  CellLink* cell = telemetryWifiForData() ? nullptr : telemetryCellLink();   // PG164
  if (!telemetryWifiForData() && !cell) return;   // no network: wait (no try counted)
  char m[64];
  if (!s_logBuf) s_logBuf = (uint8_t*)ps_malloc(LOG_CHUNK);
  if (!s_logBuf) { snprintf(m, sizeof(m), "LOG %u ERR no memory", (unsigned)s_logFile); logEnd(m); return; }
  char path[200];
  if (s_logFile >= 2000000UL) snprintf(path, sizeof(path), "/DEBUG/C%u.LOG", (unsigned)(s_logFile - 2000000UL));   // PG175
  else if (s_logFile >= 1000000UL) snprintf(path, sizeof(path), "/DEBUG/D%u.LOG", (unsigned)(s_logFile - 1000000UL));
  else snprintf(path, sizeof(path), "/DATA/%u.CSV", (unsigned)s_logFile);
  // PG198: file 3000000+<boot> = the core dump in the flash partition "coredump" (ESP-IDF wrote it at the panic),
  // read straight from flash - no SD involved
  bool core = s_logFile >= 3000000UL;
  size_t cdAddr = 0, cdSize = 0;
  bool opened = false;
  int n = -1;
  if (core) {
    opened = esp_core_dump_image_get(&cdAddr, &cdSize) == ESP_OK && cdSize > 0;
    if (opened) {
      if (!s_logSize) s_logSize = cdSize;
      if (s_logOff < s_logSize) {
        uint32_t want = s_logSize - s_logOff < LOG_CHUNK ? s_logSize - s_logOff : LOG_CHUNK;
        n = esp_flash_read(NULL, s_logBuf, cdAddr + s_logOff, want) == ESP_OK ? (int)want : -1;
      } else {
        n = 0;
      }
    }
  }
  sdLock();
  File f = core ? File() : SD.open(path, FILE_READ);
  if (!core) opened = (bool)f;         // a File is false again after close()
  if (!core && opened) {
    if (!s_logSize) s_logSize = f.size();
    if (s_logOff < s_logSize && f.seek(s_logOff)) {
      uint32_t want = s_logSize - s_logOff < LOG_CHUNK ? s_logSize - s_logOff : LOG_CHUNK;
      n = f.read(s_logBuf, want);
    } else {
      n = 0;
    }
    f.close();
  }
  sdUnlock();
  if (!opened) { snprintf(m, sizeof(m), "LOG %u ERR no file", (unsigned)s_logFile); logEnd(m); return; }
  if (!s_logAt) {
    snprintf(m, sizeof(m), "LOG %u start size=%u from=%u", (unsigned)s_logFile, (unsigned)s_logSize, (unsigned)s_logOff);
    otaLog(m);
  }
  s_logAt = millis();
  if (s_logOff >= s_logSize) {
    if (core) crashCoreDumpSent();             // PG198: the server has it whole - erased, not offered again
    snprintf(m, sizeof(m), "LOG %u done %u", (unsigned)s_logFile, (unsigned)s_logSize);
    logEnd(m);
    return;
  }
  if (n <= 0) { snprintf(m, sizeof(m), "LOG %u ERR SD read @%u", (unsigned)s_logFile, (unsigned)s_logOff); logEnd(m); return; }
  if (cell) cell->makeDefault();
  else WifiLink::makeDefault();               // PG96
  snprintf(path, sizeof(path), "/api/nestbox/ota/%s/log?file=%u&off=%u&size=%u", s_token, (unsigned)s_logFile,
           (unsigned)s_logOff, (unsigned)s_logSize);
  String answer;
  s_busy = true;
  int code = httpPost(s_wc, path, s_logBuf, n, &answer);
  s_wc.stop();
  s_busy = false;
  int k = answer.indexOf("next=");
  if (code == 200 && k >= 0) {
    uint32_t next = (uint32_t)answer.substring(k + 5).toInt();   // what the server has (normally off + n)
    s_logOff = next;
    s_logTries = 0;
    s_logWait = LOG_GAP_MS;
    return;
  }
  s_logTries++;
  s_logWait = LOG_RETRY_MS;
  snprintf(m, sizeof(m), "LOG %u ERR http=%d @%u try %u/%u", (unsigned)s_logFile, code, (unsigned)s_logOff,
           (unsigned)s_logTries, (unsigned)LOG_TRIES);
  if (s_logTries >= LOG_TRIES) logEnd(m); else otaLog(m);
}

static void otaTask(void*)
{
  uint32_t last = 0;
  vTaskDelay(pdMS_TO_TICKS(60000));          // after the boot: network and clock first
  for (;;) {
    if (s_staged && flashAllowed() && !otaVerifyPending()) flashStaged();   // PG147: not over an unconfirmed build
    if (s_retryAt && millis() - s_retryAt > 60000) {   // p74: resume the broken download
      s_retryAt = 0;
      char m[40];
      snprintf(m, sizeof(m), "OTA-PULL RETRY %u/10", (unsigned)s_retries);
      otaLog(m);
      s_req = true;
    }
    if (s_req || !last || millis() - last > OTA_PERIODIC_MS) {
      bool req = s_req;
      s_req = false;
      last = millis();
      if (req || s_token[0]) {
        otaLog(req ? "OTA-PULL REQ" : "OTA-PULL CHECK periodic");
        s_busy = true;
        check();
        if (req && !s_retryAt) placesSync(true);   // PG205: a requested check = fetch the WiFi list now
        s_busy = false;
      }
    }
    if (!s_retryAt && !s_staged) placesSync(false);   // PG205: the WiFi list on its own clock; not during a firmware download
    if (!s_staged || !flashAllowed()) logTick();   // PG49
    CellLink* cell = telemetryCellLink();     // p71
    if (cell && !cell->udpDiag) cell->udpDiag = udpDiagNote;
    if (cell && s_dCount && s_token[0] && !telemetryWifiForData() && millis() - s_dAt > 20000) {
      s_dAt = millis();
      s_busy = true;
      udpDiagUpload(cell);
      s_busy = false;
    }
    vTaskDelay(pdMS_TO_TICKS(2000));
  }
}

void otaBegin()
{
  otaVerifyBegin();                           // PG147: first, the 15 min limit starts as early as possible
  size_t l = sizeof(s_token);
  nvs_get_str(nvs, "OTA_TOKEN", s_token, &l);
  l = sizeof(s_host);
  nvs_get_str(nvs, "OTA_HOST", s_host, &l);
  uint16_t p = 0;
  if (nvs_get_u16(nvs, "OTA_PORT", &p) == ESP_OK && p) s_port = p;
  // a verified image left in the other partition by the last boot (download done, engine was running)
  uint8_t ok = 0;
  s_staged = nvs_get_u8(nvs, KEY_D_OK, &ok) == ESP_OK && ok == 1;
  // PG191: the files of the SD path (builds up to Nov 14 2026 10:22:20) are not used any more
  sdLock();
  if (SD.exists(OTA_PENDING_PATH)) SD.remove(OTA_PENDING_PATH);
  if (SD.exists(OTA_META_PATH)) SD.remove(OTA_META_PATH);
  if (SD.exists(OTA_RESUME_PATH)) SD.remove(OTA_RESUME_PATH);
  sdUnlock();
  xTaskCreatePinnedToCore(otaTask, "ota", 8192, nullptr, 1, nullptr, 1);
}
