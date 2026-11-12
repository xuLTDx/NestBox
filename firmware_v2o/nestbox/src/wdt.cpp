// Watchdog of the tasks on core 1 - see wdt.h (PG155, 12.11.2026)
#include "wdt.h"
#include <Arduino.h>
#include <esp_timer.h>
#include <esp_attr.h>
#include <esp_system.h>
#include <string.h>
#include "errreport.h"
#include "laststep.h"

static const char* const NAMES[WDT_COUNT] = {"record", "net", "cell", "loop"};

// every 5 s. Why: the limits are minutes; 5 s more on top does not matter, a short period costs nothing (core 0)
#define WDT_CHECK_US (5ULL * 1000000ULL)

struct Slot {
  TaskHandle_t task;
  uint32_t limitMs;
  volatile uint32_t fedAt;
  const char* volatile step;
};
static Slot s_slot[WDT_COUNT];

// the stopped task of the last restart; RTC memory not initialised at boot = survives esp_restart() (as laststep.h),
// not a power-on (the magic is then random)
struct WdtLast {
  uint32_t magic;
  char task[8];
  char step[24];
  uint32_t sec;
};
#define WDT_MAGIC 0x57D7A5C1u
RTC_NOINIT_ATTR static WdtLast s_last;

static void check(void*)
{
  uint32_t now = millis();
  for (int i = 0; i < WDT_COUNT; i++) {
    Slot& s = s_slot[i];
    if (!s.task || !s.limitMs) continue;
    uint32_t idle = now - s.fedAt;
    if (idle < s.limitMs) continue;
    const char* st = s.step ? s.step : "-";
    snprintf(s_last.task, sizeof(s_last.task), "%s", NAMES[i]);
    snprintf(s_last.step, sizeof(s_last.step), "%s", st);
    s_last.sec = idle / 1000;
    s_last.magic = WDT_MAGIC;
    lastStep("wdt %s %s %lus", NAMES[i], st, (unsigned long)(idle / 1000));   // also in the BOOT line (step=)
    Serial.printf("[WDT] task %s stopped %lu s at '%s' - restart\n", NAMES[i], (unsigned long)(idle / 1000), st);
    Serial.flush();
    esp_restart();
  }
}

void wdtBegin()
{
  enableCore1WDT();                  // part 1 (wdt.h): the idle task of core 1, 5 s, panic = restart
  esp_timer_create_args_t a = {};
  a.callback = check;
  a.dispatch_method = ESP_TIMER_TASK;
  a.name = "wdt";
  esp_timer_handle_t t;
  if (esp_timer_create(&a, &t) == ESP_OK) esp_timer_start_periodic(t, WDT_CHECK_US);
}

void wdtRegister(WdtTask t, uint32_t limitMs)
{
  s_slot[t].fedAt = millis();
  s_slot[t].step = "start";
  s_slot[t].limitMs = limitMs;
  s_slot[t].task = xTaskGetCurrentTaskHandle();
}

void wdtFeed(WdtTask t, const char* step)
{
  s_slot[t].step = step;
  s_slot[t].fedAt = millis();
}

void wdtFeedSelf()
{
  TaskHandle_t me = xTaskGetCurrentTaskHandle();
  for (int i = 0; i < WDT_COUNT; i++)
    if (s_slot[i].task == me) { s_slot[i].fedAt = millis(); return; }
}

static volatile int s_test = -1;     // WdtTask to stop, WDT_COUNT = hog, -1 = none

bool wdtTest(const char* what)
{
  if (!strcmp(what, "hog")) { s_test = WDT_COUNT; return true; }
  for (int i = 0; i < WDT_COUNT; i++)
    if (!strcmp(what, NAMES[i])) { s_test = i; return true; }
  return false;
}

void wdtTestPoint(WdtTask t)
{
  if (s_test == (int)t) {
    Serial.printf("[WDT] TEST: task %s blocks now\n", NAMES[t]);
    for (;;) vTaskDelay(pdMS_TO_TICKS(1000));   // waits for good, core 1 stays free = only the own check sees it
  }
  if (s_test == WDT_COUNT && t == WDT_LOOP) {
    Serial.println("[WDT] TEST: loop() keeps core 1 busy now");
    for (volatile uint32_t i = 0;; i++) {}      // no yield: the idle task of core 1 never runs
  }
}

void wdtReportLast()
{
  if (s_last.magic != WDT_MAGIC) return;
  s_last.magic = 0;
  if (esp_reset_reason() != ESP_RST_SW) return;   // a power-on / other reset since: not this restart any more
  char d[40];
  snprintf(d, sizeof(d), "%s %s %lus", s_last.task, s_last.step, (unsigned long)s_last.sec);
  errReport("SYS", "wdt", d);
}
