// see install.h (PG233)
#include "install.h"
#include "wifiplaces.h"
#include <WiFi.h>
#include <nvs.h>
#include <time.h>

extern nvs_handle_t nvs;                       // telemetry.cpp ("storage")
void statusLine(const char* text);             // ota.h
bool telemetryLogin();
const char* telemetryLinkName();               // telemetry.cpp: "WIFI", "CELL" or "-"
void webInstallEnded();                        // web.cpp: the full API again

#define INST_DAYS_S (7UL * 24UL * 3600UL)     // c) 7 days after the first login (Vojtech 18.11.2026)
#define CLOCK_OK 1735689600                    // a time before 2025 = the clock is not set yet

static char s_pw[64];
static bool s_active;

void instBegin()
{
  size_t l = sizeof(s_pw);
  if (nvs_get_str(nvs, "INST_WIFI_PW", s_pw, &l) != ESP_OK) s_pw[0] = 0;
  uint8_t done = 0;
  nvs_get_u8(nvs, "INST_DONE", &done);
  s_active = !done && strlen(s_pw) >= 8;       // WPA2: at least 8 characters
  if (s_active) statusLine("INST active - own WiFi with the install password, mini page only");
}

bool instActive() { return s_active; }
const char* instPassword() { return s_active ? s_pw : ""; }

void instEnd(const char* why)
{
  if (!s_active) return;
  s_active = false;
  nvs_set_u8(nvs, "INST_DONE", 1);
  nvs_commit(nvs);
  webInstallEnded();
  char ln[56];
  snprintf(ln, sizeof(ln), "INST done (%s)", why);
  statusLine(ln);
}

void instTick()
{
  if (!s_active) return;
  static uint32_t last;
  if (millis() - last < 2000) return;
  last = millis();
  time_t now = time(nullptr);
  if (telemetryLogin() && now > CLOCK_OK) {
    uint32_t t0 = 0;
    if (nvs_get_u32(nvs, "INST_T0", &t0) != ESP_OK || !t0) {   // the first login: c) counts from here
      nvs_set_u32(nvs, "INST_T0", (uint32_t)now);
      nvs_commit(nvs);
    } else if ((uint32_t)now - t0 >= INST_DAYS_S) {
      instEnd("7 days");
      return;
    }
  }
  // a) the chosen network joined and logged in over it
  WifiPlace u;
  if (wifiUserGet(&u) && WiFi.status() == WL_CONNECTED && WiFi.SSID() == u.ssid && telemetryLogin() &&
      !strcmp(telemetryLinkName(), "WIFI"))
    instEnd("wifi");
  memset(u.pass, 0, sizeof(u.pass));
}
