// The last step before a reset (21.10.2026: two silent restarts during the OTA download, the
// reason only on the serial port). The reset reason of the chip and the last marked step are
// written into the BOOT event of the SD log. The step lives in RTC memory that is not
// initialised at boot: it survives a software restart, a panic and a watchdog, not a power-on
// (the reason then says POWERON and the step is empty).
#pragma once
#include <Arduino.h>
#include <esp_system.h>
#include <esp_attr.h>

struct LastStep {
  uint32_t magic;
  char text[60];
};
#define LASTSTEP_MAGIC 0x15A57E90u

extern LastStep g_lastStep;   // laststep definition in main.cpp (RTC_NOINIT_ATTR)

// mark the step now running; cheap (RTC memory), may be called per chunk
inline void lastStep(const char* fmt, ...)
{
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(g_lastStep.text, sizeof(g_lastStep.text), fmt, ap);
  va_end(ap);
  g_lastStep.magic = LASTSTEP_MAGIC;
}

inline const char* resetReasonName(esp_reset_reason_t r)
{
  switch (r) {
    case ESP_RST_POWERON:   return "POWERON";
    case ESP_RST_EXT:       return "EXT";
    case ESP_RST_SW:        return "SW";
    case ESP_RST_PANIC:     return "PANIC";
    case ESP_RST_INT_WDT:   return "INT_WDT";
    case ESP_RST_TASK_WDT:  return "TASK_WDT";
    case ESP_RST_WDT:       return "WDT";
    case ESP_RST_DEEPSLEEP: return "DEEPSLEEP";
    case ESP_RST_BROWNOUT:  return "BROWNOUT";
    case ESP_RST_SDIO:      return "SDIO";
    default:                return "UNKNOWN";
  }
}
