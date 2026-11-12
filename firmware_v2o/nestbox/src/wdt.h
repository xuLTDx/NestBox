// Watchdog of the tasks on core 1 (PG155, 12.11.2026).
// Why: Peugeot VXQH2SY2 boot 225, 11.11.2026 19:33:32 -> 12.11. 06:31:09 (local time): no record, no status line,
// no SMS for 11 h without a restart (packet 1614 -> 1615, the same boot), then it ran on as if nothing happened; the
// same again 12.11. from 07:16. The record, net, cell, OTA and loop tasks all run on core 1; the task watchdog of
// Arduino 2.0.14 watches only the idle task of core 0 (tools/sdk/esp32/sdkconfig: CHECK_IDLE_TASK_CPU1 not set) and
// this firmware never used it - a stop on core 1 never ends in a restart and leaves no trace. a3 (Arduino 3.3.11 /
// ESP-IDF 5.5, port 12.11.2026): the same - libs esp32/sdkconfig "CONFIG_ESP_TASK_WDT_CHECK_IDLE_TASK_CPU1 is not
// set", enableCore1WDT() = esp32-hal-misc.c 172.
// Two parts:
//  1. enableCore1WDT(): the idle task of core 1 too (5 s, panic -> restart, reason TASK_WDT in the BOOT line) - catches
//     a task that keeps core 1 busy without a break.
//  2. Own check of the tasks that wait (a lock, a queue, a driver): each task feeds its slot from its loop and from
//     every modem wait (CellLink::waitFor, wdtFeedSelf); a slot not fed for its limit -> the task, its step and the
//     seconds into RTC memory (survives the software restart), then esp_restart(). The next boot sends
//     "ERR SYS wdt <task> <step> <s>s" (errreport.h; FleetNest area SYS, device_errors.py).
//  The check runs in the esp_timer task (core 0, ESP-IDF 4.4.6 esp_timer.c 475 PRO_CPU_NUM;
//  a3: libs sdkconfig CONFIG_ESP_TIMER_TASK_AFFINITY_CPU0=y): a stop on core 1 does not
//  stop it. No prepareRestart() there: it takes the SD lock, which the stuck task may hold; the delivery queue saves
//  itself on esp_restart() (txqueue.cpp shutdown handler).
#pragma once
#include <stdint.h>

enum WdtTask { WDT_RECORD, WDT_NET, WDT_CELL, WDT_LOOP, WDT_COUNT };

void wdtBegin();                                    // setup(), early: core-1 idle WDT + the check timer
void wdtRegister(WdtTask t, uint32_t limitMs);      // the task itself, at its start (its handle is kept)
void wdtFeed(WdtTask t, const char* step);          // step: a string literal (only the pointer is kept)
void wdtFeedSelf();                                 // from wait loops: feeds the calling task's slot, if it has one
void wdtReportLast();                               // after telemetryBegin(): "ERR SYS wdt ..." of the last restart
// bench test (web WDT_TEST=, NESTBOX_DIAG builds): record|net|cell|loop = that task blocks for good at its next
// wdtTestPoint() (the own check must restart), hog = loop() keeps core 1 busy (the core-1 idle WDT must restart)
bool wdtTest(const char* what);
void wdtTestPoint(WdtTask t);
bool wdtWasRestart();                               // PG175: this boot follows an own-watchdog restart
