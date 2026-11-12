// Debug log to SD - see dbglog.h (PG175, 12.11.2026)
#include "dbglog.h"
#include <Arduino.h>
#include <nvs.h>
#include "nb_sd.h"
#include "netlog.h"
#include "wdt.h"

extern nvs_handle_t nvs;                     // telemetry.cpp
void sdLock();
void sdUnlock();
void statusLine(const char* text);           // ota.cpp

#define DBG_DIR      "/DEBUG"
#define DBG_EVERY_MS 2000UL                  // copy period. Why 2 s: the 64 KB ring holds minutes of log at normal load
#define DBG_CHUNK    4096                    // bytes per SD write
#define DBG_KEEP_S   (7L * 24L * 3600L)      // 7 days (agreed 12.11.2026)

static volatile bool s_on;
static bool s_sdOk;
static uint32_t s_boot, s_pos;
static char* s_buf;

static bool appendFile(const char* path, const char* data, size_t n)
{
  sdLock();
  if (!SD.exists(DBG_DIR)) SD.mkdir(DBG_DIR);
  File f = SD.open(path, FILE_APPEND);
  bool ok = f && f.write((const uint8_t*)data, n) == n;
  if (f) f.close();
  sdUnlock();
  return ok;
}

static void dbgTask(void*)
{
  char path[32];
  char* prev = nullptr;
  uint32_t prevLen = 0;
  // the boot before crashed: its last 64 KB, also with DEBUG off. A software restart counts only after the own
  // watchdog (an OTA / RESTART / SMS restart is no crash)
  esp_reset_reason_t r = esp_reset_reason();
  bool crash = r != ESP_RST_SW || wdtWasRestart();
  if (s_sdOk && netlogTakePrev(&prev, &prevLen) && crash) {
    snprintf(path, sizeof(path), DBG_DIR "/C%lu.LOG", (unsigned long)(s_boot - 1));
    bool ok = appendFile(path, prev, prevLen);
    free(prev);
    char m[56];
    snprintf(m, sizeof(m), "DBG crash log C%lu %lu B%s", (unsigned long)(s_boot - 1), (unsigned long)prevLen,
             ok ? "" : " ERR SD");
    statusLine(m);
  } else if (prev) {
    free(prev);
  }
  snprintf(path, sizeof(path), DBG_DIR "/D%lu.LOG", (unsigned long)s_boot);
  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(DBG_EVERY_MS));
    if (!s_on || !s_sdOk) { s_pos = netlogHead(); continue; }   // off: start from "now" when switched on
    uint32_t lost = 0;
    int n;
    while ((n = netlogCopy(&s_pos, s_buf, DBG_CHUNK, &lost)) > 0 || lost) {
      if (lost) {
        char t[48];
        int k = snprintf(t, sizeof(t), "\n[DBG] lost %lu bytes\n", (unsigned long)lost);
        appendFile(path, t, k);
        lost = 0;
      }
      if (n > 0 && !appendFile(path, s_buf, n)) break;   // card error: tried again in 2 s (the ring may lose bytes)
      if (!s_on) break;
    }
  }
}

void dbglogBegin(uint32_t bootId, bool sdOk)
{
  s_boot = bootId;
  s_sdOk = sdOk;
  uint8_t v = 0;
  nvs_get_u8(nvs, "DEBUG", &v);
  s_on = v == 1;
  s_pos = netlogHead();
  s_buf = (char*)ps_malloc(DBG_CHUNK);
  if (!s_buf) return;
  // priority 1 (as the ota task), core 1: below the record / net / cell tasks; not in the watchdog (wdt.h) - a slow
  // card only delays the copy
  xTaskCreatePinnedToCore(dbgTask, "dbglog", 4096, nullptr, 1, nullptr, 1);
}

bool dbglogSet(bool on)
{
  s_on = on;
  bool ok = nvs_set_u8(nvs, "DEBUG", on ? 1 : 0) == ESP_OK && nvs_commit(nvs) == ESP_OK;
  return ok;
}

bool dbglogOn() { return s_on; }

void dbglogPurge(time_t now)
{
  if (!s_sdOk || now < 1735689600) return;   // no valid clock: file times cannot be compared
  sdLock();
  File dir = SD.open(DBG_DIR);
  char names[16][32];
  int nn = 0;
  if (dir) {
    for (File f = dir.openNextFile(); f && nn < 16; f = dir.openNextFile()) {
      time_t t = f.getLastWrite();
      if (!f.isDirectory() && t > 0 && now - t > DBG_KEEP_S)
        snprintf(names[nn++], sizeof(names[0]), DBG_DIR "/%s", f.name());
      f.close();
    }
    dir.close();
  }
  for (int i = 0; i < nn; i++) SD.remove(names[i]);   // after the listing (no removal while iterating)
  sdUnlock();
}
