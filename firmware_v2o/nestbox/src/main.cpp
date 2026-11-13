// NestBox_v2O - main. Step 1 of ../NAVRH.md: hardware (MAX7317, SD over
// SDMMC, CAN1 TWAI + CAN2/CAN3 MCP2515), sniff on all buses, WiFi, local HTTP
// API with OTA. Step 2a: records to FleetNest over WiFi (telemetry.cpp).
#include "part.h"
#include "crash.h"
#include "dbglog.h"
#include "wdt.h"
// PG155: limit of loop() (wdt.h). Why 3 min: one pass does web requests (an upload feeds, web.cpp), the CAN READ /
// probe (one ECU per pass), the standby check; the sleep itself never returns. Bench master 12.11.2026: loop() was
// never restarted during the tests and the OTA over the modem (that runs in the ota task).
#define WDT_LOOP_MS (180UL * 1000UL)
#include "errreport.h"
#include <Arduino.h>
#include <WiFi.h>
#include <SD_MMC.h>
#include <Preferences.h>
#include "driver/gpio.h"
#include "hw.h"
#include "spibus.h"
#include "max7317.h"
#include "sdwriter.h"
#include "sniff.h"
#include "canbus1.h"
#include "ota.h"
#include "laststep.h"
#include "netlog.h"
void sdPinsLine(const char* when);            // sdwriter.cpp (PG102)
#include "netlink.h"                           // PG96: WifiLink::saveDns
#include "standby.h"

extern const char FW_BUILD_STR[] = __DATE__ " " __TIME__;

bool webStart();
void webProcess(int timeoutMs);

// WiFi: the two networks of NestBox_v1F (config.h); passwords from the build
// secrets (wifi_secrets.py -> WIFI_PASSWORD / WIFI_PASSWORD2), never in the repo
#ifndef WIFI_PASSWORD
#define WIFI_PASSWORD ""
#endif
#ifndef WIFI_PASSWORD2
#define WIFI_PASSWORD2 ""
#endif
static const char* WIFI_NETS[][2] = {{"NTLIRIS", WIFI_PASSWORD}, {"uLTD_ext", WIFI_PASSWORD2}};
#define WIFI_JOIN_MS 15000
#define WIFI_RETRY_MS 30000

static uint32_t s_bootId;
static bool s_sdOk;

RTC_NOINIT_ATTR LastStep g_lastStep;          // laststep.h
static char s_prevStep[sizeof(g_lastStep.text)];
// the BOOT event's tail: reset reason + the step marked before the reset
const char* bootResetInfo(char* buf, int cap)
{
  snprintf(buf, cap, "reset=%s wake=%s step=%s", resetReasonName(esp_reset_reason()), standbyWakeName(),
           s_prevStep[0] ? s_prevStep : "-");
  return buf;
}

uint32_t bootId() { return s_bootId; }

// telemetry.cpp
void telemetryEarly();
void telemetryBegin(uint32_t bootId, bool sdOk);
void telemetryBeforeRestart();
void readTick();

// before every deliberate restart: the sniff file ends properly (v1F prepareRestart)
void prepareRestart(const char* why)
{
  nbout.printf("[RESTART] %s\n", why);
  sniffCloseForRestart();
  telemetryBeforeRestart();
}

static bool sdMount()
{
  // CMD and D0 need pull-ups; the board has them, the internal ones help (OVMS ovms_peripherals.cpp)
  gpio_set_pull_mode((gpio_num_t)HW_SD_CMD, GPIO_PULLUP_ONLY);
  gpio_set_pull_mode((gpio_num_t)HW_SD_D0, GPIO_PULLUP_ONLY);
  // 1-bit, NEVER format: a card that does not mount stays as it is (OVMS formats it - not here)
  for (int i = 0; i < 3; i++) {
    if (SD_MMC.begin(SDW_MOUNT, true, false, SDMMC_FREQ_DEFAULT, 5)) return true;
    nbout.println("[SD] mount failed, retry");
    SD_MMC.end();
    delay(300);
  }
  return false;
}

static int s_wifiNet = -1;
static uint32_t s_wifiTry;

// K12 (NAVRH 8, 27.10.2026, only when AP=1 from FleetNest, default off): no known network for 2 min -> the box's own access point
// "NestBox-<ID>" (password = the web password, K13; none set = no AP), the same page on 192.168.4.1;
// a known network found again -> joined, the AP switched off
#define WIFI_AP_AFTER_MS 120000UL
extern char devid[12];
const char* webPassword();
bool webApAllowed();
static uint32_t s_knownAt;
static bool s_apOn;

static void apOff()
{
  if (!s_apOn) return;
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_STA);
  s_apOn = false;
  statusLine("WIFI AP off");
}

// PG65 (9.11.2026): WIFI=0 / WIFI=1 from FleetNest. Why: on the bench the box sits in range of NTLIRIS, the
// telemetry takes WiFi whenever it is up (telemetry.cpp "want = s_wifi.up() ? ..."), so the modem path (PPP,
// OTA over cell) can only be tested by switching the network off - the user does not want that. Kept in RAM
// only: every restart / deep-sleep wake starts with WiFi on, so a box can never stay cut off from WiFi OTA.
// Refused while the modem is off (CELL=0): WiFi off + cell off = no way to send WIFI=1 until a restart.
// The command arrives in the UDP task (teleclient.cpp), applied here in loop() where the WiFi calls live.
static volatile int8_t s_wifiReq = -1;         // -1 nothing, 0 off, 1 on
static bool s_wifiOff;
bool telemetryCell();                          // telemetry.cpp
bool wifiRequest(bool on)
{
  if (!on && !telemetryCell()) return false;
  s_wifiReq = on ? 1 : 0;
  return true;
}

static void wifiTick()
{
  int8_t req = s_wifiReq;
  if (req >= 0) {
    s_wifiReq = -1;
    if (req == 0 && !s_wifiOff) {
      WiFi.disconnect(true);
      WiFi.mode(WIFI_OFF);
      s_apOn = false;
      s_wifiOff = true;
      statusLine("WIFI off (WIFI=0, until WIFI=1 or restart)");
    } else if (req == 1 && s_wifiOff) {
      s_wifiOff = false;
      WiFi.mode(WIFI_STA);
      s_wifiTry = 0;                           // scan right now, not after WIFI_RETRY_MS
      s_knownAt = millis();
      statusLine("WIFI on (WIFI=1)");
    }
  }
  if (s_wifiOff) return;
  if (WiFi.status() == WL_CONNECTED) { s_knownAt = millis(); apOff(); return; }
  if (s_apOn && !webApAllowed()) apOff();
  if (!s_apOn && webApAllowed() && millis() - s_knownAt > WIFI_AP_AFTER_MS && strlen(webPassword()) >= 8) {
    char ssid[24];
    snprintf(ssid, sizeof(ssid), "NestBox-%s", devid);
    WiFi.mode(WIFI_AP_STA);
    s_apOn = WiFi.softAP(ssid, webPassword());
    char ln[56];
    snprintf(ln, sizeof(ln), "WIFI AP %s %s %s", ssid, s_apOn ? "on" : "ERR", WiFi.softAPIP().toString().c_str());
    statusLine(ln);
  }
  if (s_wifiTry && millis() - s_wifiTry < WIFI_RETRY_MS) return;
  s_wifiTry = millis();
  // the network in range: scan, then join the strongest known one
  int n = WiFi.scanNetworks();
  int best = -1, bestRssi = -1000;
  for (int i = 0; i < n; i++) {
    for (int k = 0; k < 2; k++) {
      if (WiFi.SSID(i) == WIFI_NETS[k][0] && WiFi.RSSI(i) > bestRssi) { best = k; bestRssi = WiFi.RSSI(i); }
    }
  }
  WiFi.scanDelete();
  if (best < 0) {
    nbout.printf("[WIFI] no known network in range (%d seen)\n", n);
    return;
  }
  s_knownAt = millis();
  s_wifiNet = best;
  nbout.printf("[WIFI] joining %s (%d dBm)\n", WIFI_NETS[best][0], bestRssi);
  WiFi.begin(WIFI_NETS[best][0], WIFI_NETS[best][1]);
  for (uint32_t t = millis(); millis() - t < WIFI_JOIN_MS && WiFi.status() != WL_CONNECTED;) delay(100);
  if (WiFi.status() == WL_CONNECTED) {
    WifiLink::saveDns();                      // PG96: DHCP has just set WiFi's DNS - kept (netlink.cpp)
    nbout.printf("[WIFI] %s IP %s\n", WIFI_NETS[best][0], WiFi.localIP().toString().c_str());
    char ln[64];                              // p58: the box's address on a phone hotspot, seen in FleetNest
    snprintf(ln, sizeof(ln), "WIFI %s IP %s", WIFI_NETS[best][0], WiFi.localIP().toString().c_str());
    statusLine(ln);
    configTime(0, 0, "pool.ntp.org");        // the sniff file's clock line
  } else {
    nbout.println("[WIFI] join timeout");
  }
}

void setup()
{
  telemetryEarly();                           // first: the delivery queue's PSRAM reserve (txqueue.h)
  if (g_lastStep.magic == LASTSTEP_MAGIC && esp_reset_reason() != ESP_RST_POWERON) {
    memcpy(s_prevStep, g_lastStep.text, sizeof(s_prevStep));
    s_prevStep[sizeof(s_prevStep) - 1] = 0;
  }
  lastStep("boot");
  standbyEarly();                             // a deep-sleep wake: confirmed, or asleep again here
  Serial.begin(115200);
  netlogBegin();                              // PG99: ESP-IDF log copied into RAM for /api/serial
  delay(100);
  nbout.printf("\nNestBox_v2O %s reset=%d step=%s\n", FW_BUILD_STR, (int)esp_reset_reason(), s_prevStep);
  wdtBegin();                                 // PG155: core-1 idle WDT + the task check (wdt.h)
  crashBegin();                               // PG175: panic reason + backtrace into RTC memory (crash.h)

  Preferences p;
  p.begin("nestbox", false);
  s_bootId = p.getUInt("boot", 0) + 1;
  p.putUInt("boot", s_bootId);
  p.end();
  nbout.printf("boot %lu, PSRAM %u free\n", (unsigned long)s_bootId, (unsigned)ESP.getFreePsram());

  spiBusBegin();
  mx7317Begin();
  mx7317Out(MX_CAN1_RS, 1);                   // CAN1 transmitter off from the start

  s_sdOk = sdMount();
  sdPinsLine(s_sdOk ? "after-mount" : "mount-failed");   // PG102 (sdwriter.cpp)
  if (s_sdOk) {
    nbout.printf("[SD] %llu MB, %llu MB used\n", SD_MMC.totalBytes() >> 20, SD_MMC.usedBytes() >> 20);
    sdwStart();
  } else {
    nbout.println("[SD] NO CARD - sniff cannot start");
  }

  can1Begin();                                // CAN1 listens all the time (car values; sniff takes its frames)
  sniffInit();
  telemetryBegin(s_bootId, s_sdOk);
  // PG152 (11.11.2026): a box that boots without a working card sent nothing to the server - only "[SD] NO CARD" on
  // the serial log; ERR SD write/dead come only from writes to a card that mounted. "ERR SD dead" prefix = the same
  // e-mail to the Admin in FleetNest as a card that died while running. After telemetryBegin: logNetEvent is set up.
  if (!s_sdOk) errReport("SD", "dead", "nocard");
  wdtReportLast();                            // PG155: "ERR SYS wdt <task> <step> <s>s" of the restart before this boot
  crashReportLast();                          // PG175: "ERR SYS panic ..." + "PANIC bt ..." of a panic before this boot
  partBootReport();                           // PG177: "PART OK coredump ..." once after the table change
  crashCoreDumpReport(s_bootId);              // PG198: "COREDUMP <bytes> boot=<n>" while a core dump waits in flash
  dbglogBegin(s_bootId, s_sdOk);              // PG175: debug log to SD (DEBUG=1), the crash log of the boot before
  otaBegin();                                // pull-OTA (needs the NVS handle opened by telemetryBegin)
#ifdef TEST_OTA_CRASH
  // PG147 bench only: this build crashes 20 s after start, before any server login - the bootloader must boot
  // the previous build and that one must report 'OTA ROLLBACK crash-or-reset' (otaverify.cpp; bench 11.11.2026: OK)
  nbout.println("[TEST] TEST_OTA_CRASH: abort in 20 s");
  delay(20000);
  abort();
#endif
  {
    char ln[120] = "BOOT ";                   // p58: reset reason + wake to FleetNest (the BOOT event is SD only)
    bootResetInfo(ln + 5, sizeof(ln) - 5);
    statusLine(ln);
    if (standbyUlpStats()[0] && esp_reset_reason() == ESP_RST_DEEPSLEEP) statusLine(standbyUlpStats());   // p62
  }

  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  wifiTick();
  webStart();
  wdtRegister(WDT_LOOP, WDT_LOOP_MS);         // PG155: setup() runs in the loop task - from here on it is watched
}

void loop()
{
  wdtTestPoint(WDT_LOOP);
  wdtFeed(WDT_LOOP, "loop");
  webProcess(20);
  can1Tick();
  readTick();                                 // READ command: the request waiting for the bus
  int ev = sniffTick();
  if (ev == SNIFF_EV_STARTED) nbout.println("[SNIFF] started");
  wifiTick();
  standbyTick();
  errTick();                                  // PG149: error summaries + SD / CAN1 counters to the server
  partTick();                                 // PG177: the one-time partition table change when armed (part.h)
  static uint32_t lastPurge;
  if (millis() - lastPurge > 3600000UL || !lastPurge) {   // hourly, once the clock is set
    if (time(nullptr) > 1735689600) { lastPurge = millis(); sniffPurgeOld(time(nullptr)); dbglogPurge(time(nullptr)); }
  }
}
