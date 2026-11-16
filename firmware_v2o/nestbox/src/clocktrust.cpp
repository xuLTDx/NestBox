// PG259 - see clocktrust.h for the why.
#include "clocktrust.h"
#include <Arduino.h>
#include <sys/time.h>
#include <esp_system.h>
#include <esp_sntp.h>

#define CLOCK_VALID_T 1735689600            // 2025-01-01: anything earlier = the clock was never set

static volatile bool s_trusted = true;      // power-on / restart: only the > 2025 check counts (as before)
static volatile bool s_afterSleep;
static time_t s_wakeClock;                  // the clock right after the wake (RTC, fast)
static uint32_t s_wakeMs;
static volatile bool s_lineReady;
static volatile long s_drift;
static const char* volatile s_src;

static void markTrusted(time_t truth, const char* src)
{
  if (s_trusted) return;
  // what the RTC clock would show now minus the truth = the drift of the sleep
  s_drift = (long)(s_wakeClock + (time_t)((millis() - s_wakeMs) / 1000) - truth);
  s_src = src;
  s_trusted = true;
  s_lineReady = true;
}

static void onNtp(struct timeval* tv) { markTrusted(tv->tv_sec, "ntp"); }

void clockTrustBegin()
{
  s_afterSleep = esp_reset_reason() == ESP_RST_DEEPSLEEP;
  if (s_afterSleep) {
    s_wakeClock = time(nullptr);
    s_wakeMs = millis();
    s_trusted = false;
  }
  sntp_set_time_sync_notification_cb(onNtp);
}

bool clockTrusted() { return s_trusted && time(nullptr) > CLOCK_VALID_T; }

void clockSetFrom(time_t t, const char* src)
{
  if (t <= CLOCK_VALID_T) return;
  time_t now = time(nullptr);
  markTrusted(t, src);                      // before the set: the drift is measured against the RTC run
  if (now <= CLOCK_VALID_T || labs((long)(now - t)) > 2) {
    struct timeval tv = {t, 0};
    settimeofday(&tv, nullptr);
  }
}

bool clockTakeLine(char* out, int cap)
{
  if (!s_lineReady) return false;
  s_lineReady = false;
  snprintf(out, cap, "CLOCK %s after sleep %+lds", s_src ? s_src : "?", s_drift);
  return true;
}
