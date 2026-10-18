// Local HTTP API - NestBox_v2O step 1. The handlers are NestBox_v1F's
// (firmware_v5/telelogger/dataserver.cpp: /api/control, /api/ota, /api/sniff),
// the same MiniWeb server (libraries/httpd); SD through SD_MMC instead of SD.
//   /                           status page (page.h)
//   /api/info                   device, firmware, uptime, SD, sniff, buses (JSON)
//   /api/control?cmd=...        SNIFF=1[,<min>] | SNIFF=0 | SNIFF? | SNIFF_VEH=<tag> |
//                               SNIFF_BUSES=<mask 1-7> | SNIFF_PURGE | SDW? | RESTART |
//                               TEL? | REC_S=<s> | CELL=0|1 | CELL_APN=<apn> | VEH=K0 | CAR? |
//                               OTA? | OTA_CHECK | OTA_TOKEN= | OTA_HOST= | OTA_PORT= |
//                               READ=<bus>,<tx hex>,<rx hex>,<request hex>
//   /api/ota                    raw firmware POST (v1F handlerOTA)
//   /api/sniff, /api/sniff/live?id=&mask=, /api/sniff/list, /api/sniff/<n>[?from=&to=]
#include <Arduino.h>
#include <WiFi.h>
#include <SD_MMC.h>
#include <Update.h>
#include <esp_timer.h>
#include <lwip/sockets.h>
#include <httpd.h>
#include "sniff.h"
#include "sdwriter.h"
#include "page.h"
#include "cardecode.h"
#include "ota.h"
#include "nbread.h"

extern const char FW_BUILD_STR[];
uint32_t bootId();
void sdLock();
void sdUnlock();
void prepareRestart(const char* why);
int telemetryStatus(char* buf, int size);
void telemetrySetRecS(uint32_t s);
uint32_t telemetryRecS();
bool telemetryLogin();
extern char devid[12];
void telemetrySetCell(bool on);
bool telemetryCell();
void telemetrySetApn(const char* apn);
void telemetrySetVehicle(const char* p);

static HttpParam httpParam;

static int handlerInfo(UrlHandlerParam* param)
{
  char* buf = param->pucBuffer;
  int bufsize = param->bufSize;
  sdLock();
  uint64_t tot = SD_MMC.totalBytes(), used = SD_MMC.usedBytes();
  sdUnlock();
  int n = snprintf(buf, bufsize,
      "{\"device\":\"NestBox_v2O\",\"id\":\"%s\",\"login\":%d,\"mac\":\"%s\",\"fw\":\"%s\",\"boot\":%lu,\"uptime_s\":%lu,"
      "\"reset\":%d,\"heap\":%u,\"psram\":%u,\"sd_mb\":%llu,\"sd_used_mb\":%llu,\"wifi\":\"%s\",\"rssi\":%d,\"sniff\":",
      devid, telemetryLogin() ? 1 : 0, WiFi.macAddress().c_str(), FW_BUILD_STR, (unsigned long)bootId(), (unsigned long)(millis() / 1000),
      (int)esp_reset_reason(), (unsigned)ESP.getFreeHeap(), (unsigned)ESP.getFreePsram(),
      (unsigned long long)(tot >> 20), (unsigned long long)(used >> 20), WiFi.SSID().c_str(), (int)WiFi.RSSI());
  n += sniffStatus(buf + n, bufsize - n - 2);
  n += snprintf(buf + n, bufsize - n, "}");
  param->contentLength = n;
  param->contentType = HTTPFILETYPE_JSON;
  return FLAG_DATA_RAW;
}

static void restart_cb(void*) { esp_restart(); }

static void restartSoon(const char* why)
{
  static esp_timer_handle_t t = nullptr;
  if (!t) {
    esp_timer_create_args_t a = {};
    a.callback = restart_cb;
    a.dispatch_method = ESP_TIMER_TASK;
    a.name = "restart";
    esp_timer_create(&a, &t);
  } else {
    esp_timer_stop(t);
  }
  prepareRestart(why);
  esp_timer_start_once(t, 1500000);
}

static int handlerControl(UrlHandlerParam* param)
{
  char* buf = param->pucBuffer;
  int bufsize = param->bufSize;
  const char* cmd = mwGetVarValue(param->pxVars, "cmd", "");
  int n = 0;
  if (!*cmd) {
    n = snprintf(buf, bufsize, "ERR");
  } else if (!strncmp(cmd, "SNIFF=", 6)) {          // SNIFF=1[,<min>] | SNIFF=0
    const char* c = strchr(cmd, ',');
    sniffRequest(cmd[6] == '1', c ? (uint16_t)atoi(c + 1) : 0);
    n = snprintf(buf, bufsize, "OK");
  } else if (!strcmp(cmd, "SNIFF?")) {
    n = sniffStatus(buf, bufsize);
  } else if (!strncmp(cmd, "SNIFF_VEH=", 10)) {
    n = snprintf(buf, bufsize, sniffSetVehicle(cmd + 10) ? "OK" : "ERR");
  } else if (!strncmp(cmd, "SNIFF_BUSES=", 12)) {   // bit 0 = CAN1, 1 = CAN2, 2 = CAN3; next start
    int m = atoi(cmd + 12);
    if (m < 1 || m > 7) n = snprintf(buf, bufsize, "ERR 1-7");
    else { sniffSetBuses((uint8_t)m); n = snprintf(buf, bufsize, "OK buses=%d (from the next start)", m); }
  } else if (!strcmp(cmd, "SNIFF_PURGE")) {
    n = snprintf(buf, bufsize, "removed %d", sniffPurgeOld(time(nullptr)));
#ifdef TEST_SNIFF_LOAD
  } else if (!strncmp(cmd, "SNIFF_CHECK", 11)) {
    n = sniffCheck(cmd[11] == '=' ? strtoul(cmd + 12, nullptr, 10) : 0, buf, bufsize);
  } else if (!strncmp(cmd, "SNIFF_LIMIT=", 12)) {
    sniffSetSdLimit(atoi(cmd + 12));
    n = snprintf(buf, bufsize, "OK");
#endif
  } else if (!strcmp(cmd, "TEL?")) {               // FleetNest link + delivery queue
    n = telemetryStatus(buf, bufsize);
  } else if (!strncmp(cmd, "REC_S=", 6)) {          // seconds between records (>= 10)
    telemetrySetRecS(strtoul(cmd + 6, nullptr, 10));
    n = snprintf(buf, bufsize, "OK rec_s=%lu", (unsigned long)telemetryRecS());
  } else if (!strncmp(cmd, "READ=", 5)) {           // one read request into the car (nbread.h)
    uint32_t seq = 0;
    int r = nbReadSubmit(cmd + 5, &seq);
    n = snprintf(buf, bufsize, r ? "ERR %d (#%u; 10 syntax, 11 not a reading service)" : "OK #%u - result in the next record (PID 0x39D) and the SD log",
                 r ? r : (int)seq, (unsigned)seq);
  } else if (!strcmp(cmd, "OTA?")) {
    n = otaStatus(buf, bufsize);
  } else if (!strcmp(cmd, "OTA_CHECK")) {           // pull-OTA check now (as the server's OTA_READY)
    otaRequest();
    n = snprintf(buf, bufsize, "OK");
  } else if (!strncmp(cmd, "OTA_TOKEN=", 10) || !strncmp(cmd, "OTA_HOST=", 9) || !strncmp(cmd, "OTA_PORT=", 9)) {
    char key[12];
    const char* eq = strchr(cmd, '=');
    snprintf(key, sizeof(key), "%.*s", (int)(eq - cmd), cmd);
    n = snprintf(buf, bufsize, otaSet(key, eq + 1) ? "OK %s set" : "ERR", key);
  } else if (!strncmp(cmd, "VEH=", 4)) {            // car profile for the CAN1 values: K0 or empty
    telemetrySetVehicle(cmd + 4);
    n = snprintf(buf, bufsize, "OK veh=%s", carProfile());
  } else if (!strncmp(cmd, "CAR?", 4)) {
    CarValues v = carValues();
    uint32_t ms = millis();
    n = snprintf(buf, bufsize, "veh=%s odo=%lu km (%ld ms ago) fuel=%.1f L raw=%u (%ld ms ago) urea=%lu km raw=%02X%02X%02X%02X%02X%02X%02X%02X (%ld ms ago)",
                 carProfile(), (unsigned long)v.odoKm, v.odoAt ? (long)(ms - v.odoAt) : -1L, v.fuelL, v.fuelRaw,
                 v.fuelAt ? (long)(ms - v.fuelAt) : -1L, (unsigned long)v.ureaKm, v.urea[0], v.urea[1], v.urea[2],
                 v.urea[3], v.urea[4], v.urea[5], v.urea[6], v.urea[7], v.ureaAt ? (long)(ms - v.ureaAt) : -1L);
  } else if (!strncmp(cmd, "CELL=", 5)) {           // modem on/off (stored)
    telemetrySetCell(cmd[5] == '1');
    n = snprintf(buf, bufsize, "OK cell=%d", telemetryCell() ? 1 : 0);
  } else if (!strncmp(cmd, "CELL_APN=", 9)) {       // empty = the network's default
    telemetrySetApn(cmd + 9);
    n = snprintf(buf, bufsize, "OK apn=%s (next modem start)", cmd + 9);
  } else if (!strcmp(cmd, "SDW?")) {
    n = sdwStatus(buf, bufsize);
  } else if (!strcmp(cmd, "RESTART")) {
    restartSoon("HTTP_RESTART");
    n = snprintf(buf, bufsize, "OK");
  } else {
    n = snprintf(buf, bufsize, "ERR unknown");
  }
  param->contentLength = n;
  param->contentType = HTTPFILETYPE_TEXT;
  return FLAG_DATA_RAW;
}

// ------------------------------------------------------------------ OTA ---
// NestBox_v1F handlerOTA: raw binary POST, streamed from the socket into the
// other app partition; Update.end() checks the image, then a restart.
#define OTA_RECV_BUF_SIZE 4096

static int otaErr(UrlHandlerParam* param, const char* fmt, unsigned a = 0, unsigned b = 0)
{
  param->contentLength = snprintf(param->pucBuffer, param->bufSize, fmt, a, b);
  param->contentType = HTTPFILETYPE_TEXT;
  return FLAG_DATA_RAW;
}

static int handlerOTA(UrlHandlerParam* param)
{
  const char* cl = strcasestr(param->hs->buffer, "Content-Length:");
  size_t fw_size = 0;
  if (cl) {
    cl += 15;
    while (*cl == ' ') cl++;
    fw_size = (size_t)atol(cl);
  }
  if (fw_size == 0) return otaErr(param, "ERR: missing Content-Length");
  Serial.printf("[OTA] Starting update: %u bytes\n", (unsigned)fw_size);
  if (!Update.begin(fw_size)) {
    param->contentLength = snprintf(param->pucBuffer, param->bufSize, "ERR: Update.begin failed: %s", Update.errorString());
    param->contentType = HTTPFILETYPE_TEXT;
    return FLAG_DATA_RAW;
  }
  size_t written = 0;
  if (param->pucPayload && param->payloadSize > 0) {
    size_t first = (param->payloadSize < fw_size) ? (size_t)param->payloadSize : fw_size;
    size_t w = Update.write((uint8_t*)param->pucPayload, first);
    if (w != first) { Update.abort(); return otaErr(param, "ERR: write error at offset 0 (%u/%u written)", w, first); }
    written = w;
  }
  uint8_t* rb = (uint8_t*)malloc(OTA_RECV_BUF_SIZE);
  if (!rb) { Update.abort(); return otaErr(param, "ERR: out of memory for recv buffer"); }
  int sock = param->hs->socket;
  while (written < fw_size) {
    size_t remaining = fw_size - written;
    int to_read = (int)(remaining < OTA_RECV_BUF_SIZE ? remaining : OTA_RECV_BUF_SIZE);
    fd_set rfds;
    struct timeval tv;
    FD_ZERO(&rfds);
    FD_SET(sock, &rfds);
    tv.tv_sec = 30;
    tv.tv_usec = 0;
    if (select(sock + 1, &rfds, NULL, NULL, &tv) <= 0) {
      Update.abort(); free(rb);
      return otaErr(param, "ERR: recv timeout at offset %u (remaining %u)", written, remaining);
    }
    int n = recv(sock, rb, to_read, 0);
    if (n <= 0) {
      Update.abort(); free(rb);
      return otaErr(param, "ERR: recv failed at offset %u (remaining %u)", written, remaining);
    }
    if (Update.write(rb, (size_t)n) != (size_t)n) {
      Update.abort(); free(rb);
      return otaErr(param, "ERR: flash write error at offset %u", written);
    }
    written += (size_t)n;
  }
  free(rb);
  Serial.printf("[OTA] Upload complete: %u bytes\n", (unsigned)written);
  if (!Update.end()) {
    param->contentLength = snprintf(param->pucBuffer, param->bufSize, "ERR: Update.end failed: %s", Update.errorString());
    param->contentType = HTTPFILETYPE_TEXT;
    return FLAG_DATA_RAW;
  }
  Serial.println("[OTA] Flash successful, rebooting in 1.5 s");
  restartSoon("HTTP_OTA");
  strcpy(param->pucBuffer, "OK");
  param->contentLength = 2;
  param->contentType = HTTPFILETYPE_TEXT;
  return FLAG_DATA_RAW;
}

// ---------------------------------------------------------------- sniff ---
// v1F handlerSniff, unchanged except SD_MMC
class SniffFileContext {
public:
  File file;
  uint32_t toMs = 0;
  uint32_t dataPos = 0;
  bool headerSent = true;
  bool done = false;
};

static uint32_t sniffMsAt(File& f, uint32_t pos, uint32_t* at)
{
  char line[96];
  f.seek(pos);
  if (pos) f.readBytesUntil('\n', line, sizeof(line));
  for (;;) {
    uint32_t p = f.position();
    int n = f.available() ? f.readBytesUntil('\n', line, sizeof(line) - 1) : -1;
    if (n < 0) return 0xFFFFFFFF;
    if (n == 0 || line[0] == '#') continue;
    line[n] = 0;
    *at = p;
    return strtoul(line, nullptr, 10);
  }
}

static int handlerSniff(UrlHandlerParam* param)
{
  char* buf = param->pucBuffer;
  int bufsize = param->bufSize;
  SniffFileContext* ctx = (SniffFileContext*)param->hs->ptr;
  if (ctx) {
    if (!param->pucBuffer) {
      sdLock();
      ctx->file.close();
      sdUnlock();
      delete ctx;
      param->hs->ptr = 0;
      return 0;
    }
  } else {
    const char* r = param->pucRequest;
    if (*r == '/') r++;
    if (!*r || *r == '?') {
      param->contentLength = sniffStatus(buf, bufsize);
      param->contentType = HTTPFILETYPE_JSON;
      return FLAG_DATA_RAW;
    }
    if (!strncmp(r, "live", 4)) {
      param->contentLength = sniffLive(buf, bufsize, mwGetVarValueHex(param->pxVars, "id", 0),
                                       mwGetVarValueHex(param->pxVars, "mask", 0));
      param->contentType = HTTPFILETYPE_TEXT;
      return FLAG_DATA_RAW;
    }
    if (!strncmp(r, "list", 4)) {
      int n = snprintf(buf, bufsize, "[");
      sdLock();
      File root = SD_MMC.open("/SNIFF");
      File f;
      while (root && n < bufsize - 64 && (f = root.openNextFile())) {
        const char* fn = f.name();
        const char* p = strrchr(fn, '/');
        if (p) fn = p + 1;
        if (fn[0] == 'S') n += snprintf(buf + n, bufsize - n, "{\"n\":%u,\"size\":%u},", (unsigned)atoi(fn + 1), (unsigned)f.size());
        f.close();
      }
      if (root) root.close();
      sdUnlock();
      if (buf[n - 1] == ',') n--;
      n += snprintf(buf + n, bufsize - n, "]");
      param->contentLength = n;
      param->contentType = HTTPFILETYPE_JSON;
      return FLAG_DATA_RAW;
    }
    char path[32];
    snprintf(path, sizeof(path), "/SNIFF/S%u.LOG", (unsigned)atoi(r));
    ctx = new SniffFileContext;
    sdLock();
    ctx->file = SD_MMC.open(path, FILE_READ);
    sdUnlock();
    if (!ctx->file) {
      delete ctx;
      param->contentLength = snprintf(buf, bufsize, "%s not found", path);
      param->contentType = HTTPFILETYPE_TEXT;
      return FLAG_DATA_RAW;
    }
    param->hs->ptr = (void*)ctx;
    uint32_t fromMs = mwGetVarValueInt(param->pxVars, "from", 0);
    ctx->toMs = mwGetVarValueInt(param->pxVars, "to", 0);
    if (fromMs) {
      sdLock();
      uint32_t lo = 0, hi = ctx->file.size(), at = 0;
      while (hi - lo > 2048) {
        uint32_t mid = lo + (hi - lo) / 2;
        uint32_t ms = sniffMsAt(ctx->file, mid, &at);
        if (ms < fromMs) lo = mid; else hi = mid;
      }
      uint32_t ms = sniffMsAt(ctx->file, lo, &at);
      while (ms != 0xFFFFFFFF && ms < fromMs) {
        uint32_t next = 0;
        ms = sniffMsAt(ctx->file, at + 1, &next);
        at = next;
      }
      ctx->dataPos = ms == 0xFFFFFFFF ? ctx->file.size() : at;
      ctx->file.seek(0);
      sdUnlock();
      ctx->headerSent = false;
    }
  }
  if (ctx->done) return 0;
  sdLock();
  int n = 0;
  if (!ctx->headerSent) {
    char line[200];
    while (ctx->file.available() && n < bufsize - (int)sizeof(line)) {
      int l = ctx->file.readBytesUntil('\n', line, sizeof(line) - 1);
      if (l <= 0 || line[0] != '#') break;
      memcpy(buf + n, line, l);
      n += l;
      buf[n++] = '\n';
    }
    ctx->headerSent = true;
    ctx->file.seek(ctx->dataPos);
  } else if (ctx->file.available()) {
    n = ctx->file.readBytes(buf, bufsize);
    if (ctx->toMs && n > 0) {
      int end = n;
      while (end > 0 && buf[end - 1] != '\n') end--;
      if (end > 0 && end < n) ctx->file.seek(ctx->file.position() - (n - end));
      if (end > 0) n = end;
      for (int i = 0; i < n;) {
        if (buf[i] != '#' && strtoul(buf + i, nullptr, 10) > ctx->toMs) { n = i; ctx->done = true; break; }
        char* nl = (char*)memchr(buf + i, '\n', n - i);
        if (!nl) break;
        i = nl - buf + 1;
      }
    }
  }
  sdUnlock();
  if (!n) return 0;
  param->contentLength = n;
  param->contentType = HTTPFILETYPE_TEXT;
  return FLAG_DATA_STREAM;
}

static int handlerPage(UrlHandlerParam* param)
{
  param->pucBuffer = (char*)PAGE_HTML;
  param->contentLength = sizeof(PAGE_HTML) - 1;
  param->contentType = HTTPFILETYPE_HTML;
  return FLAG_DATA_RAW;
}

// ---------------------------------------------------------- /DATA files ---
// v1F dataserver.cpp handlerLogList / handlerLogFile / handlerLogEvents (SD_MMC here):
// /api/list = the /DATA files, /api/log/<n> = one file (0 = the current), /api/events/<n> = its
// "FE," event lines only
uint32_t telemetryFileId();
struct LogDataContext { File file; };

static int handlerLogFile(UrlHandlerParam* param)
{
  LogDataContext* ctx = (LogDataContext*)param->hs->ptr;
  param->contentType = HTTPFILETYPE_TEXT;
  if (ctx) {
    if (!param->pucBuffer) {
      sdLock(); ctx->file.close(); sdUnlock();
      delete ctx;
      param->hs->ptr = 0;
      return 0;
    }
  } else {
    int id = param->pucRequest[0] == '/' ? atoi(param->pucRequest + 1) : 0;
    sprintf(param->pucBuffer, "/DATA/%u.CSV", id == 0 ? (unsigned)telemetryFileId() : (unsigned)id);
    ctx = new LogDataContext;
    sdLock(); ctx->file = SD_MMC.open(param->pucBuffer, FILE_READ); sdUnlock();
    if (!ctx->file) {
      strcat(param->pucBuffer, " not found");
      param->contentLength = strlen(param->pucBuffer);
      delete ctx;
      return FLAG_DATA_RAW;
    }
    param->hs->ptr = (void*)ctx;
  }
  sdLock();
  bool more = ctx->file.available();
  if (more) param->contentLength = ctx->file.readBytes(param->pucBuffer, param->bufSize);
  sdUnlock();
  if (!more) return 0;
  return FLAG_DATA_STREAM;
}

static int handlerLogList(UrlHandlerParam* param)
{
  char* buf = param->pucBuffer;
  int bufsize = param->bufSize;
  uint32_t fileid = telemetryFileId();
  sdLock();
  File root = SD_MMC.open("/DATA");
  int n = snprintf(buf, bufsize, "[");
  if (root) {
    const int reserve = 64;               // v1F 2026-09-12: never run past the buffer
    File file;
    while (n < bufsize - reserve && (file = root.openNextFile())) {
      const char* fn = file.name();
      const char* p = strrchr(fn, '/');
      if (p) fn = p + 1;
      unsigned id = atoi(fn), size = file.size();
      if (id) {
        n += snprintf(buf + n, bufsize - n, "{\"id\":%u,\"size\":%u%s},", id, size, id == fileid ? ",\"active\":true" : "");
      }
      file.close();
    }
    if (n > 0 && buf[n - 1] == ',') n--;
    root.close();
  }
  sdUnlock();
  n += snprintf(buf + n, bufsize - n, "]");
  param->contentType = HTTPFILETYPE_JSON;
  param->contentLength = n;
  return FLAG_DATA_RAW;
}

static int handlerLogEvents(UrlHandlerParam* param)
{
  int id = param->pucRequest[0] == '/' ? atoi(param->pucRequest + 1) : 0;
  char path[24];
  sprintf(path, "/DATA/%u.CSV", id == 0 ? (unsigned)telemetryFileId() : (unsigned)id);
  sdLock();
  File file = SD_MMC.open(path, FILE_READ);
  if (!file) {
    sdUnlock();
    param->contentLength = snprintf(param->pucBuffer, param->bufSize, "%s not found", path);
    param->contentType = HTTPFILETYPE_TEXT;
    return FLAG_DATA_RAW;
  }
  int n = 0, len = 0;
  char line[160];
  const int reserve = 8;
  char block[256];                        // v1F 2026-09-14: block reads, yield - a byte at a time hung the httpd
  int blockLen = 0, blockPos = 0;
  uint32_t iterations = 0;
  for (;;) {
    if (blockPos >= blockLen) {
      blockLen = file.readBytes(block, sizeof(block));
      blockPos = 0;
      if (blockLen <= 0) {
        line[len] = 0;
        if (len >= 3 && !strncmp(line, "FE,", 3) && n < param->bufSize - reserve)
          n += snprintf(param->pucBuffer + n, param->bufSize - n, "%s\n", line + 3);
        break;
      }
    }
    char c = block[blockPos++];
    if ((++iterations & 0x3FF) == 0) yield();
    if (c == '\n') {
      line[len] = 0;
      if (len >= 3 && !strncmp(line, "FE,", 3) && n < param->bufSize - reserve)
        n += snprintf(param->pucBuffer + n, param->bufSize - n, "%s\n", line + 3);
      len = 0;
      if (n >= param->bufSize - reserve) break;
    } else if (len < (int)sizeof(line) - 1) {
      line[len++] = c;
    }
  }
  file.close();
  sdUnlock();
  param->contentLength = n;
  param->contentType = HTTPFILETYPE_TEXT;
  return FLAG_DATA_RAW;
}

int telemetryLive(char* buf, int size);
static int handlerLive(UrlHandlerParam* param)
{
  param->contentLength = telemetryLive(param->pucBuffer, param->bufSize);
  param->contentType = HTTPFILETYPE_JSON;
  return FLAG_DATA_RAW;
}

static UrlHandler urlHandlerList[] = {
  {"", handlerPage},           // empty prefix = only "/" (httpd.c _mwCheckUrlHandlers)
  {"api/info", handlerInfo},
  {"api/live", handlerLive},
  {"api/list", handlerLogList},
  {"api/log", handlerLogFile},
  {"api/events", handlerLogEvents},
  {"api/control", handlerControl},
  {"api/ota", handlerOTA},
  {"api/sniff", handlerSniff},
  {0}
};

bool webStart()
{
  mwInitParam(&httpParam, 80, "/spiffs");
  httpParam.pxUrlHandler = urlHandlerList;
  httpParam.maxClients = 2;
  if (mwServerStart(&httpParam)) return false;
  return true;
}

void webProcess(int timeoutMs) { mwHttpLoop(&httpParam, timeoutMs); }
