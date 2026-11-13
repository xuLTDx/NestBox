// Crash capture - see crash.h (PG175, 12.11.2026)
#include "crash.h"
#include <Arduino.h>
#include <esp_attr.h>
#include <esp_system.h>
#include "errreport.h"
#include <esp_core_dump.h>
#include <nvs.h>

extern nvs_handle_t nvs;                     // telemetry.cpp

extern "C" void esp_psram_extram_writeback_cache(void);   // as txqueue.cpp persist()
void statusLine(const char* text);                          // ota.cpp
void logNetEvent(const char* text);                         // telemetry.cpp: SD event line

#define CRASH_MAGIC 0xC4A5B007u
#define CRASH_BT    8                // return addresses kept. Why 8: 2 status lines of <= 56 chars (4 addresses each)

struct CrashInfo {
  uint32_t magic;
  uint32_t core;
  uint32_t pc;                       // the faulting instruction (panic_info_t.addr)
  uint32_t bt[CRASH_BT];             // backtrace PCs from the exception frame
  uint8_t nbt;
  char reason[32];
  char task[16];
};
RTC_NOINIT_ATTR static CrashInfo s_crash;

static IRAM_ATTR void copyStr(char* dst, int cap, const char* src)
{
  int i = 0;
  if (src) for (; i < cap - 1 && src[i]; i++) dst[i] = src[i];
  dst[i] = 0;
}

// Arduino 3.3.11 already wraps esp_panic_handler (esp32-hal-misc.c 457-462) and gives its backtrace to the handler
// set with set_arduino_panic_handler() (same walk as ESP-IDF panic_arch.c) - our own --wrap collided with it at link
// time. Runs in the panic context: no locks, no heap, no flash writes - only RTC memory and the PSRAM write-back.
static IRAM_ATTR void onPanic(arduino_panic_info_t* info, void*)
{
  s_crash.core = (uint32_t)info->core;
  s_crash.pc = (uint32_t)info->pc;
  copyStr(s_crash.reason, sizeof(s_crash.reason), info->reason);
  TaskHandle_t t = xTaskGetCurrentTaskHandleForCore(info->core);
  copyStr(s_crash.task, sizeof(s_crash.task), t ? pcTaskGetName(t) : "-");
  s_crash.nbt = 0;
  for (unsigned i = 0; i < info->backtrace_len && s_crash.nbt < CRASH_BT; i++) s_crash.bt[s_crash.nbt++] = info->backtrace[i];
  s_crash.magic = CRASH_MAGIC;
  esp_psram_extram_writeback_cache();
}

void crashBegin() { set_arduino_panic_handler(onPanic, nullptr); }

void crashReportLast()
{
  if (s_crash.magic != CRASH_MAGIC) return;
  s_crash.magic = 0;
  esp_reset_reason_t r = esp_reset_reason();
  if (r == ESP_RST_POWERON || r == ESP_RST_DEEPSLEEP || r == ESP_RST_BROWNOUT) return;   // not this boot's cause
  char m[64];
  snprintf(m, sizeof(m), "%.24s %.12s", s_crash.reason, s_crash.task);
  errReport("SYS", "panic", m);
  // the addresses: 4 per status line ("PANIC bt " + 4 x 11 chars = 53 <= 56, ota.cpp statusLine)
  char full[200];
  int fn = snprintf(full, sizeof(full), "PANIC core=%u reason=%s task=%s pc=0x%08lx bt", (unsigned)s_crash.core,
                    s_crash.reason, s_crash.task, (unsigned long)s_crash.pc);
  for (int i = 0; i < s_crash.nbt && i < CRASH_BT; i += 4) {
    int n = snprintf(m, sizeof(m), "PANIC bt");
    for (int k = i; k < i + 4 && k < s_crash.nbt; k++) {
      n += snprintf(m + n, sizeof(m) - n, " %08lx", (unsigned long)s_crash.bt[k]);
      if (fn < (int)sizeof(full) - 12) fn += snprintf(full + fn, sizeof(full) - fn, " 0x%08lx", (unsigned long)s_crash.bt[k]);
    }
    statusLine(m);
  }
  logNetEvent(full);
}

// PG198 - see crash.h. NVS "CD_BOOT" = the boot that crashed (the dump stays until uploaded, over later boots)
void crashCoreDumpReport(uint32_t bootId)
{
  size_t addr = 0, size = 0;
  if (esp_core_dump_image_get(&addr, &size) != ESP_OK || !size) return;   // none / damaged / no partition
  esp_reset_reason_t r = esp_reset_reason();
  bool crashedNow = r == ESP_RST_PANIC || r == ESP_RST_TASK_WDT || r == ESP_RST_INT_WDT || r == ESP_RST_WDT;
  uint32_t cdBoot = 0;
  if (crashedNow || nvs_get_u32(nvs, "CD_BOOT", &cdBoot) != ESP_OK || !cdBoot) {
    cdBoot = bootId - 1;
    nvs_set_u32(nvs, "CD_BOOT", cdBoot);
    nvs_commit(nvs);
  }
  char m[56];
  snprintf(m, sizeof(m), "COREDUMP %u boot=%lu", (unsigned)size, (unsigned long)cdBoot);
  statusLine(m);
  logNetEvent(m);
}

void crashCoreDumpSent()
{
  esp_core_dump_image_erase();
  nvs_erase_key(nvs, "CD_BOOT");
  nvs_commit(nvs);
  statusLine("COREDUMP sent - erased");
}
