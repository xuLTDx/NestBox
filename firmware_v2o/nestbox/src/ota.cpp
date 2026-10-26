// see ota.h
#include "ota.h"
#include "laststep.h"
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <Update.h>
#include <nvs.h>
#include <mbedtls/sha256.h>
#include "nb_sd.h"
#include "store.h"
#include "cellmodem.h"

extern const char FW_BUILD_STR[];
extern nvs_handle_t nvs;
void logNetEvent(const char* text);
void prepareRestart(const char* why);
bool telemetryEngineOn();
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
static volatile bool s_staged;               // a verified image waits on SD for the engine to stop
static uint8_t s_buf[OTA_CHUNK];

// -------------------------------------------------------------- status ---
#define OTA_STATUS_Q 24      // p42: CCH diagnostics - several lines in a row
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;
static char s_q[OTA_STATUS_Q][64];
static uint8_t s_head, s_count;
static char s_last[64] = "-";

static void otaLog(const char* text)
{
  logNetEvent(text);
  Serial.println(text);
  portENTER_CRITICAL(&s_mux);
  uint8_t slot = (s_head + s_count) % OTA_STATUS_Q;
  if (s_count == OTA_STATUS_Q) s_head = (s_head + 1) % OTA_STATUS_Q;
  else s_count++;
  strncpy(s_q[slot], text, sizeof(s_q[0]) - 1);
  s_q[slot][sizeof(s_q[0]) - 1] = 0;
  strncpy(s_last, text, sizeof(s_last) - 1);
  portEXIT_CRITICAL(&s_mux);
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

bool otaSet(const char* key, const char* value)
{
  if (!strcmp(key, "OTA_TOKEN")) { snprintf(s_token, sizeof(s_token), "%s", value); nvs_set_str(nvs, "OTA_TOKEN", s_token); }
  else if (!strcmp(key, "OTA_HOST")) { snprintf(s_host, sizeof(s_host), "%s", value); nvs_set_str(nvs, "OTA_HOST", s_host); }
  else if (!strcmp(key, "OTA_PORT")) { s_port = (uint16_t)atoi(value); nvs_set_u16(nvs, "OTA_PORT", s_port); }
  else return false;
  nvs_commit(nvs);
  return true;
}

void otaRequest() { s_req = true; }

// ---------------------------------------------------------------- HTTP ---
// One HTTPS connection: WiFi (WiFiClientSecure) or the modem's own TLS (AT+CCH*, 2026-10-20 -
// over cellular when there is no WiFi; NOT TESTED: the data link did not come up on USB power).
struct OtaConn {
  virtual bool connect(const char* host, uint16_t port) = 0;
  virtual void write(const char* s) = 0;
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
  int read(uint8_t* buf, int cap, unsigned int timeoutMs) override {
    uint32_t t = millis();
    while (!c.available() && c.connected() && millis() - t < timeoutMs) delay(2);
    if (!c.available()) return c.connected() ? -1 : 0;
    return c.read(buf, cap);
  }
  void stop() override { c.stop(); }
};

struct CellConn : OtaConn {
  CellLink* m;
  explicit CellConn(CellLink* l) : m(l) {}
  bool connect(const char* host, uint16_t port) override { return m->tlsOpen(host, port); }
  void write(const char* s) override { m->tlsSend(s, strlen(s)); }
  int read(uint8_t* buf, int cap, unsigned int timeoutMs) override {
    return m->tlsRead(buf, cap > 1024 ? 1024 : cap, timeoutMs);
  }
  void stop() override { m->tlsClose(); }
};

// Both connections live outside the ota task's stack, as v1F's global otaWifiClient
// (telelogger.ino): on the stack (2 x 1 KB line buffer + the TLS client) the second TLS
// handshake overflowed the 8 KB stack - "Stack canary watchpoint triggered (ota)", serial 21.10.2026.
static WifiConn s_wc;
static CellConn s_cc(nullptr);

// one GET; returns the status code (-1 = no connection), *len = Content-Length (-1 unknown).
// The body is then read with c.body().
static String s_lastStatus;                  // the answer's first line (p42 diagnostics)

static int httpGet(OtaConn& c, const char* path, size_t rangeFrom, long* len)
{
  c.rn = c.rp = 0;
  if (!c.connect(s_host, s_port)) return -1;
  char req[400];
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

// -------------------------------------------------------------- check ---
static bool fileHasMark(const char* path)
{
  const int ml = strlen(NESTBOX_IMAGE_MARK);
  sdLock();
  File f = SD.open(path, FILE_READ);
  sdUnlock();
  if (!f) return false;
  bool found = false;
  // read in chunks with an overlap of the marker length, so a marker across two chunks is found
  static uint8_t win[OTA_CHUNK + 64];
  int keep = 0;
  for (;;) {
    sdLock();
    int n = f.read(win + keep, OTA_CHUNK);
    sdUnlock();
    if (n <= 0) break;
    int tot = keep + n;
    for (int i = 0; i + ml <= tot; i++) {
      if (win[i] == (uint8_t)NESTBOX_IMAGE_MARK[0] && !memcmp(win + i, NESTBOX_IMAGE_MARK, ml)) { found = true; break; }
    }
    if (found) break;
    keep = ml - 1 < tot ? ml - 1 : tot;
    memmove(win, win + tot - keep, keep);
  }
  sdLock();
  f.close();
  sdUnlock();
  return found;
}

static void check()
{
  if (!s_token[0]) { otaLog("OTA-PULL STOP no token"); return; }
  CellLink* cell = WiFi.status() == WL_CONNECTED ? nullptr : telemetryCellLink();
  if (WiFi.status() != WL_CONNECTED && !cell) { otaLog("OTA-PULL STOP no wifi, no cell"); return; }
  if (s_staged) { otaLog("OTA-PULL STAGED waits for engine off"); return; }
  char path[300];
  snprintf(path, sizeof(path), "/api/nestbox/ota/%s/meta.json?build=%s&variant=%s", s_token,
           urlBuild().c_str(), OTA_VARIANT);
  s_cc.m = cell;
  if (cell) cell->diag = otaLog;              // p42: the modem's answers into the record
  OtaConn& c = cell ? (OtaConn&)s_cc : (OtaConn&)s_wc;
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
  char m[64];
  snprintf(m, sizeof(m), "OTA-PULL START SIZE=%u", (unsigned)size);
  otaLog(m);

  // resume: the partial file belongs to this target when size and sha256 match /ota_resume.txt
  size_t from = 0;
  sdLock();
  if (SD.exists(OTA_RESUME_PATH) && SD.exists(OTA_PENDING_PATH)) {
    File rf = SD.open(OTA_RESUME_PATH, FILE_READ);
    String sl = rf.readStringUntil('\n'), hl = rf.readStringUntil('\n');
    rf.close();
    hl.trim();
    if ((size_t)sl.toInt() == size && hl == sha) {
      File pf = SD.open(OTA_PENDING_PATH, FILE_READ);
      size_t ex = pf.size();
      pf.close();
      if (ex > 0 && ex < size) from = ex;
    }
  }
  if (SD.exists(OTA_META_PATH)) SD.remove(OTA_META_PATH);
  if (!from) {
    SD.remove(OTA_PENDING_PATH);
    File rf = SD.open(OTA_RESUME_PATH, FILE_WRITE);
    if (rf) { rf.printf("%u\n%s\n", (unsigned)size, sha.c_str()); rf.close(); }
  }
  File out = SD.open(OTA_PENDING_PATH, from ? FILE_APPEND : FILE_WRITE);
  sdUnlock();
  if (!out) { otaLog("OTA-PULL ERR=SD_OPEN"); return; }
  mbedtls_sha256_context ctx;
  mbedtls_sha256_init(&ctx);
  mbedtls_sha256_starts_ret(&ctx, 0);
  if (from) {                                 // the bytes already on SD
    sdLock();
    File rf = SD.open(OTA_PENDING_PATH, FILE_READ);
    int n;
    while (rf && (n = rf.read(s_buf, OTA_CHUNK)) > 0) mbedtls_sha256_update_ret(&ctx, s_buf, n);
    if (rf) rf.close();
    sdUnlock();
    snprintf(m, sizeof(m), "OTA-PULL RESUME from %u", (unsigned)from);
    otaLog(m);
  }
  snprintf(path, sizeof(path), "/api/nestbox/ota/%s/firmware.bin", s_token);
  lastStep("OTA GET firmware from %u", (unsigned)from);
  code = httpGet(c, path, from, &len);
  size_t written = from;
  uint32_t t0 = millis();
  if (code == 200 && from) {                  // no Range support: start over
    sdLock(); out.close(); out = SD.open(OTA_PENDING_PATH, FILE_WRITE); sdUnlock();
    mbedtls_sha256_free(&ctx); mbedtls_sha256_init(&ctx); mbedtls_sha256_starts_ret(&ctx, 0);
    written = 0;
  }
  if (code == 200 || code == 206) {
    while (written < size) {
      int want = size - written < OTA_CHUNK ? size - written : OTA_CHUNK;
      int n = c.body(s_buf, want, OTA_TIMEOUT_MS);
      if (n <= 0) break;
      sdLock();
      bool ok = out.write(s_buf, n) == (size_t)n;
      sdUnlock();
      if (!ok) break;
      mbedtls_sha256_update_ret(&ctx, s_buf, n);
      written += n;
      lastStep("OTA download %u/%u", (unsigned)written, (unsigned)size);
    }
  }
  c.stop();
  sdLock(); out.close(); sdUnlock();
  lastStep("OTA download end %u/%u", (unsigned)written, (unsigned)size);
  if (written != size) {
    mbedtls_sha256_free(&ctx);
    snprintf(m, sizeof(m), "OTA-PULL ERR=RECV %u/%u http=%d in %ums", (unsigned)written, (unsigned)size, code,
             (unsigned)(millis() - t0));
    otaLog(m);                                // the partial file stays: the next check resumes
    return;
  }
  uint8_t dig[32];
  mbedtls_sha256_finish_ret(&ctx, dig);
  mbedtls_sha256_free(&ctx);
  char hex[65];
  for (int i = 0; i < 32; i++) sprintf(hex + 2 * i, "%02x", dig[i]);
  if (sha.equalsIgnoreCase(hex) == false) {
    otaLog("OTA-PULL ERR=SHA256");
    sdLock(); SD.remove(OTA_PENDING_PATH); SD.remove(OTA_RESUME_PATH); sdUnlock();
    return;
  }
  if (!fileHasMark(OTA_PENDING_PATH)) {
    otaLog("OTA-PULL ERR=NOT_V2O_IMAGE (no NestBox_v2O marker) - not flashed");
    sdLock(); SD.remove(OTA_PENDING_PATH); SD.remove(OTA_RESUME_PATH); sdUnlock();
    return;
  }
  sdLock();
  File mf = SD.open(OTA_META_PATH, FILE_WRITE);
  if (mf) { mf.printf("%u\n", (unsigned)size); mf.close(); }
  sdUnlock();
  snprintf(path, sizeof(path), "/api/nestbox/ota/%s/ota_confirm?size=%u&sha256=%.16s", s_token,
           (unsigned)size, hex);
  if (httpGet(c, path, 0, &len) == 200) otaLog("OTA-PULL CONFIRMED");
  c.stop();
  s_staged = true;
  otaLog("OTA-PULL STAGED (flash when the engine is off)");
}

// flash from SD (v1F performPullOtaFlash), then restart
static void flashStaged()
{
  sdLock();
  File mf = SD.open(OTA_META_PATH, FILE_READ);
  size_t size = mf ? (size_t)mf.readStringUntil('\n').toInt() : 0;
  if (mf) mf.close();
  File f = SD.open(OTA_PENDING_PATH, FILE_READ);
  size_t actual = f ? f.size() : 0;
  sdUnlock();
  if (!f || size < OTA_MIN_SIZE || actual != size) {
    otaLog("OTA-PULL ERR=SD_SIZE");
    sdLock(); if (f) f.close(); SD.remove(OTA_PENDING_PATH); SD.remove(OTA_META_PATH); sdUnlock();
    s_staged = false;
    return;
  }
  char m[48];
  snprintf(m, sizeof(m), "OTA-PULL FLASH START SIZE=%u", (unsigned)size);
  otaLog(m);
  if (!Update.begin(size)) { otaLog("OTA-PULL ERR=UPD_BEGIN"); sdLock(); f.close(); sdUnlock(); return; }
  size_t done = 0;
  while (done < size) {
    sdLock();
    int n = f.read(s_buf, size - done < OTA_CHUNK ? size - done : OTA_CHUNK);
    sdUnlock();
    if (n <= 0 || Update.write(s_buf, n) != (size_t)n) break;
    done += n;
    lastStep("OTA flash %u/%u", (unsigned)done, (unsigned)size);
  }
  sdLock(); f.close(); sdUnlock();
  if (done != size || !Update.end()) {
    Update.abort();
    otaLog("OTA-PULL ERR=FLASH");
    s_staged = false;
    sdLock(); SD.remove(OTA_PENDING_PATH); SD.remove(OTA_META_PATH); sdUnlock();
    return;
  }
  sdLock(); SD.remove(OTA_PENDING_PATH); SD.remove(OTA_META_PATH); SD.remove(OTA_RESUME_PATH); sdUnlock();
  otaLog("OTA-PULL FLASH OK - restart");
  prepareRestart("OTA_FLASHED");
  delay(1500);
  esp_restart();
}

static void otaTask(void*)
{
  uint32_t last = 0;
  vTaskDelay(pdMS_TO_TICKS(60000));          // after the boot: network and clock first
  for (;;) {
    if (s_staged && flashAllowed()) flashStaged();
    if (s_req || !last || millis() - last > OTA_PERIODIC_MS) {
      bool req = s_req;
      s_req = false;
      last = millis();
      if (req || s_token[0]) {
        otaLog(req ? "OTA-PULL REQ" : "OTA-PULL CHECK periodic");
        s_busy = true;
        check();
        s_busy = false;
      }
    }
    vTaskDelay(pdMS_TO_TICKS(2000));
  }
}

void otaBegin()
{
  size_t l = sizeof(s_token);
  nvs_get_str(nvs, "OTA_TOKEN", s_token, &l);
  l = sizeof(s_host);
  nvs_get_str(nvs, "OTA_HOST", s_host, &l);
  uint16_t p = 0;
  if (nvs_get_u16(nvs, "OTA_PORT", &p) == ESP_OK && p) s_port = p;
  // a verified image left on SD by the last boot (download done, not flashed yet)
  sdLock();
  s_staged = SD.exists(OTA_META_PATH) && SD.exists(OTA_PENDING_PATH);
  sdUnlock();
  xTaskCreatePinnedToCore(otaTask, "ota", 8192, nullptr, 1, nullptr, 1);
}
