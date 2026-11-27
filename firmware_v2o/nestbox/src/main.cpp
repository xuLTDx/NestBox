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
#include "clocktrust.h"                // PG259
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
#include "wifiplaces.h"                        // PG205
#include "install.h"                           // PG233

extern const char FW_BUILD_STR[] = __DATE__ " " __TIME__;

bool webStart();
void webProcess(int timeoutMs);

// WiFi: only the networks of the list from FleetNest (wifiplaces.h). PG230 (14.11.2026, Vojtech "230 ano"): the two
// networks built into the firmware (NTLIRIS, uLTD_ext of NestBox_v1F, passwords from the build secrets) are gone -
// they meant nothing for other companies, and the password was readable from every build (no flash encryption,
// PROJEKT_STAV 6/8). Both passwords are in FleetNest (checked 14.11.2026 against the build secrets, equal).
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
void telemetrySmsCfgReport(int what);
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

// PG274 (18.11.2026): mbedTLS (HTTPS: OTA, WiFi list, LOG upload) takes its memory from PSRAM, internal RAM only if
// PSRAM is out. Why: the Arduino 3.3.11 libraries are built with CONFIG_MBEDTLS_INTERNAL_MEM_ALLOC=y (sdkconfig), so
// each HTTPS connection borrowed ~45 KB of the ~80 KB free internal RAM (2Z 18.11. 18:12:31: 82 -> 37 KB, largest
// block 2 KB) and failed once something else held 5-15 KB more (first web request -4.3 KB for good, the own WiFi in
// install mode: 2Z 'META_HTTP-1' at <= 74 KB, J0 and 2Z fine at >= 78 KB) - while ~2.9 MB of PSRAM lay unused.
// OVMS3 on the same board does the same by config (support/sdkconfig.default.hw31: CONFIG_MBEDTLS_EXTERNAL_MEM_ALLOC=y).
// Done at run time (mbedtls_platform_set_calloc_free, MBEDTLS_PLATFORM_MEMORY is on in esp_config.h:130) - changing the
// sdkconfig would need the libraries rebuilt. heap_caps_free() frees from either heap, so blocks taken before the
// switch are freed correctly.
#include <mbedtls/platform.h>
static void* tlsCalloc(size_t n, size_t size)
{
  void* p = heap_caps_calloc(n, size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  return p ? p : heap_caps_calloc(n, size, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
}
static void tlsMemToPsram() { mbedtls_platform_set_calloc_free(tlsCalloc, heap_caps_free); }

static void apOff()
{
  if (!s_apOn) return;
  dbgMem("AP off before");                    // PG273 (after: the "WIFI AP off" line)
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
bool telemetryEngineOn();
bool telemetryPosition(float* lat, float* lng);
bool telemetryLastPosition(float* lat, float* lng);   // PG232
bool telemetryWifiMarked();                    // PG239
bool telemetryWifiSkip(const uint8_t* bssid);
// PG239 (14.11.2026, Vojtech "239 ano"): a joined WiFi marked "no internet" is left for another network of the list
// in range. Why: bench U9K2M3J0 14.11. 20:41, MAC blocked: the router keeps the station joined and only drops its
// traffic, so the box stayed on NTLIRIS and never tried the phone hotspot Pocof7ultra in range. Every WIFI_ALT_MS
// while marked: one scan; a listed network allowed here, not the marked access point -> leave and join it. The
// marked access point is skipped by every scan for WIFI_BAD_MS (then tried again, PG164).
#define WIFI_ALT_MS 60000UL
static uint32_t s_altTry;
// PG241 (14.11.2026, Vojtech "ano"): a network that failed to join twice in a row is skipped for WIFI_SKIP_MS, so the
// next network of the list gets its turn. Why: bench U9K2M3J0 14.11. 20:57-21:05, MAC blocked: every 30 s the
// strongest listed network NTLIRIS (-66 dBm) was tried and refused ("join timeout reason=8"); the phone hotspot of
// the list was never tried. After WIFI_SKIP_MS the network is tried again.
#define WIFI_SKIP_MS (5UL * 60UL * 1000UL)
#define WIFI_SKIP_N  4
struct WifiFail { char ssid[33]; uint8_t fails; uint32_t until; };
static WifiFail s_wfail[WIFI_SKIP_N];
static WifiFail* wifiFail(const char* ssid, bool make)
{
  WifiFail* freeSlot = nullptr;
  for (auto& f : s_wfail) {
    if (f.ssid[0] && !strcmp(f.ssid, ssid)) return &f;
    if (!f.ssid[0] && !freeSlot) freeSlot = &f;
  }
  if (!make) return nullptr;
  if (!freeSlot) freeSlot = &s_wfail[0];
  memset(freeSlot, 0, sizeof(*freeSlot));
  snprintf(freeSlot->ssid, sizeof(freeSlot->ssid), "%s", ssid);
  return freeSlot;
}
static bool wifiSkipped(const char* ssid)
{
  WifiFail* f = wifiFail(ssid, false);
  if (!f || !f->until) return false;
  if ((int32_t)(millis() - f->until) >= 0) { f->until = 0; return false; }
  return true;
}

// PG205 + PG208 (13.-14.11.2026, wifiplaces.h - why there): which networks may be searched right now.
// Agreed with Vojtech 13.11.2026:
//  - inside the radius of a place with a WiFi name: that place's network only;
//  - away from all of them: nothing;
//  - no position now (garage, just woken): the last position the box had decides (PG232, kept over sleep and power
//    loss); all networks of the list only for a box that never had a position - a car under the depot's roof had
//    its fix when it drove in;
//  - no list: nothing (PG230, 14.11.2026) - the box gets the list over the modem 60 s after the boot, keeps it in
//    NVS; a box without modem reaches only its own AP NestBox-<id> (web password) and USB;
//  - a phone hotspot has no address: all networks for WIFI_START_MS after every engine start (hotspot on, start the
//    engine - works where the box has neither data nor SMS) and for WIFI_CMD_MS after WIFI=1 (FleetNest or SMS).
// A network already joined is kept until it is lost (a hotspot leaving with the car).
#define WIFI_START_MS (3UL * 60UL * 1000UL)
#define WIFI_CMD_MS   (30UL * 60UL * 1000UL)
static uint32_t s_wifiCmdAt, s_wifiStartAt;    // millis() | 1, 0 = not running
// the networks that may be searched now: 0 none, 1 only the places holding the position, 2 all
// PG219 (14.11.2026): why a join failed. Bench U9K2M3J0 boot 36: 40 min of "[WIFI] join timeout" every 30 s with
// NTLIRIS at -65 dBm and nothing saying why (the router had the MAC blocked - known only after asking Vojtech).
// The driver's last disconnect reason (wifi_err_reason_t: 2 auth expired, 15 4-way handshake timeout = wrong
// password, 201 no AP found, 202 auth fail, ...) goes into the debug log and once into "ERR WIFI join <net> r=<n>";
// repeats are counted, the next join that works sends "ERR WIFI join x<n>" + "OK WIFI join" (errreport.h).
static volatile uint8_t s_wifiReason;         // 0 = the driver reported none during this join
static void wifiReasonEvent(arduino_event_id_t, arduino_event_info_t info)
{
  s_wifiReason = info.wifi_sta_disconnected.reason;
}

static int wifiSearch(const char** why, float* lat, float* lng)
{
  static bool wasEngine;
  bool eng = telemetryEngineOn();
  if (eng && !wasEngine) s_wifiStartAt = millis() | 1;
  wasEngine = eng;
  // signed: "millis() | 1" is 1 ms in the future for an even millis() - unsigned, the difference was 0xFFFFFFFF and the
  // 30 min ended at once (bench U9K2M3J0 14.11.2026 boot 53: WIFI=1 at 56 s, "search on (cmd)" never came, the listed
  // hotspot Pocof7ultra in range never tried; the same trap as sdwriter.cpp 2026-10-12)
  if (s_wifiCmdAt && (int32_t)(millis() - s_wifiCmdAt) >= (int32_t)WIFI_CMD_MS) s_wifiCmdAt = 0;
  if (s_wifiStartAt && (int32_t)(millis() - s_wifiStartAt) >= (int32_t)WIFI_START_MS) s_wifiStartAt = 0;
  if (!wifiPlacesCount()) { *why = "nolist"; return 0; }   // PG230: no list = no network known, also after WIFI=1
  if (instActive()) { *why = "install"; return 2; }        // PG233: being set up - every known network, anywhere
  if (s_wifiCmdAt) { *why = "cmd"; return 2; }
  if (s_wifiStartAt) { *why = "start"; return 2; }
  if (!telemetryPosition(lat, lng) && !telemetryLastPosition(lat, lng)) { *why = "neverfix"; return 2; }   // PG232
  WifiPlace p;
  for (int i = 0; wifiPlaceGet(i, &p); i++)
    if (wifiPlaceHolds(p, *lat, *lng)) { *why = "place"; return 1; }
  *why = "away";
  return 0;
}
bool wifiRequest(bool on)
{
  if (!on && !telemetryCell()) return false;
  s_wifiReq = on ? 1 : 0;
  return true;
}

// PG248 (14.11.2026, Vojtech "248 ano, preco ma blokovat hlavnu slucku navyse ak WiFi je raz za cas"): the scan and
// the join do not wait in loop() any more. Why: scanNetworks() (2-3 s) and the join wait (up to WIFI_JOIN_MS 15 s)
// stopped loop() - the web, READ requests, the standby check and the error summaries - up to ~18 s every 30 s where
// a listed network refused the box (bench U9K2M3J0 14.11.2026, MAC blocked). Now: the scan runs in the driver
// (scanNetworks(true)), the join is started and checked on the next passes; loop() goes on meanwhile.
enum { WS_IDLE, WS_SCAN, WS_JOIN };
static uint8_t s_ws;
static uint32_t s_wsAt;
static int s_wsNets;
static float s_wsLat, s_wsLng;
static bool s_wsAlt;
static WifiPlace s_wsNet;                      // the network being joined (with its password until the end)
#define WIFI_SCAN_MAX_MS 10000UL
static void wifiScanDone(int n);
static void wifiJoinEnd(bool ok);
static bool s_joinSeen;                        // PG255: this connection was reported (IP line, OK WIFI join)

// PG233 (install.h): what the mini page shows - the networks of the last scan and how the join of the chosen one went
#define PAGE_NETS 20
struct PageNet { char ssid[33]; int8_t rssi; bool open; };
static PageNet s_pageNets[PAGE_NETS];
static int s_pageN = -1;                       // -1 = no scan yet
static uint32_t s_pageAt;
static volatile bool s_pageScanReq;
static bool s_wsPage;                          // the running scan was asked for by the page
static bool s_preferUser;                      // the network was just chosen: join it even when another is stronger
static char s_pjSsid[33];                      // the last join: network, state 0 none / 1 joining / 2 ok / 3 failed, reason
static uint8_t s_pjState, s_pjReason;

static void pageCopy(int n)
{
  int k = 0;
  for (int i = 0; i < n && k < PAGE_NETS; i++) {
    String s = WiFi.SSID(i);
    if (!s.length()) continue;                 // a hidden network cannot be chosen by name
    bool dup = false;
    for (int j = 0; j < k && !dup; j++) dup = s == s_pageNets[j].ssid;
    if (dup) continue;                         // one line per name (the strongest comes first from the driver)
    snprintf(s_pageNets[k].ssid, sizeof(s_pageNets[k].ssid), "%s", s.c_str());
    s_pageNets[k].rssi = (int8_t)WiFi.RSSI(i);
    s_pageNets[k].open = WiFi.encryptionType(i) == WIFI_AUTH_OPEN;
    k++;
  }
  s_pageN = k;
  s_pageAt = millis();
}

static int jsonStr(char* out, int cap, const char* s)   // a JSON string, quotes and control characters escaped
{
  int n = 0;
  if (cap < 3) return 0;
  out[n++] = '"';
  for (; *s && n < cap - 3; s++) {
    unsigned char c = (unsigned char)*s;
    if (c == '"' || c == '\\') { out[n++] = '\\'; out[n++] = c; }
    else if (c < 32) out[n++] = ' ';
    else out[n++] = c;
  }
  out[n++] = '"';
  out[n] = 0;
  return n;
}

void wifiPageScan() { s_pageScanReq = true; }  // web.cpp (the loop task, as wifiTick)

// web.cpp: a network chosen on the page - stored and joined next (a joined other network is left for it)
bool wifiPageSave(const char* ssid, const char* pass)
{
  if (!wifiUserSet(ssid, pass)) return false;
  s_preferUser = true;
  s_wifiTry = 0;
  s_pjState = 0;
  char ln[64];
  snprintf(ln, sizeof(ln), "WIFI %.32s chosen on the page", ssid);   // the name only, never the password
  statusLine(ln);
  if (WiFi.status() == WL_CONNECTED && WiFi.SSID() != ssid) WiFi.disconnect();
  return true;
}

int wifiPageJson(char* out, int cap)
{
  int n = snprintf(out, cap, "\"scan_s\":%ld,\"nets\":[", s_pageN < 0 ? -1L : (long)((millis() - s_pageAt) / 1000));
  for (int i = 0; i < s_pageN && n < cap - 80; i++) {
    n += snprintf(out + n, cap - n, "%s{\"s\":", i ? "," : "");
    n += jsonStr(out + n, cap - n, s_pageNets[i].ssid);
    n += snprintf(out + n, cap - n, ",\"r\":%d,\"o\":%d}", s_pageNets[i].rssi, s_pageNets[i].open ? 1 : 0);
  }
  static const char* const st[] = {"", "joining", "ok", "failed"};
  n += snprintf(out + n, cap - n, "],\"join\":{\"ssid\":");
  n += jsonStr(out + n, cap - n, s_pjSsid);
  n += snprintf(out + n, cap - n, ",\"st\":\"%s\",\"r\":%u},\"wifi\":{\"ssid\":", st[s_pjState & 3], (unsigned)s_pjReason);
  bool up = WiFi.status() == WL_CONNECTED;
  n += jsonStr(out + n, cap - n, up ? WiFi.SSID().c_str() : "");
  n += snprintf(out + n, cap - n, ",\"ip\":\"%s\",\"rssi\":%d},\"chosen\":",
                up ? WiFi.localIP().toString().c_str() : "", up ? (int)WiFi.RSSI() : 0);
  WifiPlace u;
  bool has = wifiUserGet(&u);
  memset(u.pass, 0, sizeof(u.pass));
  n += jsonStr(out + n, cap - n, has ? u.ssid : "");
  return n;
}

// PG255: a network was joined - by wifiJoinEnd() or by the driver itself (wifiTick)
static void wifiJoined(const char* ssid)
{
  s_joinSeen = true;
  WifiLink::saveDns();                        // PG96: DHCP has just set WiFi's DNS - kept (netlink.cpp)
  nbout.printf("[WIFI] %s IP %s\n", ssid, WiFi.localIP().toString().c_str());
  char ln[64];                                // p58: the box's address on a phone hotspot, seen in FleetNest
  snprintf(ln, sizeof(ln), "WIFI %.32s IP %s", ssid, WiFi.localIP().toString().c_str());
  statusLine(ln);
  configTime(0, 0, "pool.ntp.org");          // the sniff file's clock line
  errOk("WIFI", "join");
  WifiFail* f = wifiFail(ssid, false);        // PG241
  if (f) memset(f, 0, sizeof(*f));
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
    if (req == 1) {                            // PG205: and all networks are searched, wherever the box is
      s_wifiCmdAt = millis() | 1;
      s_wifiTry = 0;
      statusLine("WIFI search 30 min (WIFI=1)");
    }
  }
  if (s_wifiOff) { if (s_ws == WS_SCAN) WiFi.scanDelete(); s_ws = WS_IDLE; return; }
  // PG233: install mode - the own WiFi from the start and kept while joined (install.h)
  if (instActive() && !s_apOn) {
    char ssid[24];
    snprintf(ssid, sizeof(ssid), "NestBox-%s", devid);
    dbgMem("AP on before");                   // PG273 (after: the "WIFI AP ... on" line)
    WiFi.mode(WIFI_AP_STA);
    s_apOn = WiFi.softAP(ssid, instPassword());
    char ln[64];
    snprintf(ln, sizeof(ln), "WIFI AP %s %s %s (install)", ssid, s_apOn ? "on" : "ERR", WiFi.softAPIP().toString().c_str());
    statusLine(ln);
  }
  if (s_ws == WS_SCAN) {                       // PG248
    int n = WiFi.scanComplete();
    if (n == WIFI_SCAN_RUNNING && millis() - s_wsAt < WIFI_SCAN_MAX_MS) return;
    s_ws = WS_IDLE;
    bool page = s_wsPage;
    s_wsPage = false;
    if (n < 0) { WiFi.scanDelete(); if (page) s_pageN = 0; return; }
    pageCopy(n);                               // PG233: every scan refreshes the page's list
    if (page && !s_wsNets) { WiFi.scanDelete(); return; }   // asked for by the page only: nothing to join
    wifiScanDone(n);
    return;
  }
  if (s_ws == WS_JOIN) {                       // PG248
    if (WiFi.status() == WL_CONNECTED) wifiJoinEnd(true);
    else if (millis() - s_wsAt >= WIFI_JOIN_MS) wifiJoinEnd(false);
    return;
  }
  if (s_pageScanReq) {                         // PG233: the page's "search" - in the driver, as every scan (PG248)
    s_pageScanReq = false;
    if (WiFi.scanNetworks(true) != WIFI_SCAN_FAILED) {
      s_ws = WS_SCAN; s_wsAt = millis(); s_wsPage = true; s_wsNets = 0;
      return;
    }
    nbout.printf("[WIFI] page scan refused (status %d)\n", (int)WiFi.status());
  }
  bool alt = false;                            // PG239: looking for another network while joined to a marked one
  if (WiFi.status() != WL_CONNECTED) s_joinSeen = false;
  if (WiFi.status() == WL_CONNECTED) {
    static uint32_t learnAt;                   // PG233: the chosen network gets its place (200 m) on the first fix
    if (millis() - learnAt > 10000) {
      learnAt = millis();
      WifiPlace u;
      float la, ln;
      if (wifiUserGet(&u) && WiFi.SSID() == u.ssid && telemetryPosition(&la, &ln) && wifiUserLearn(la, ln)) {
        char m[72];
        snprintf(m, sizeof(m), "WIFI %.32s place kept (%.0f m)", u.ssid, WIFI_USER_RADIUS_M);
        statusLine(m);
      }
      memset(u.pass, 0, sizeof(u.pass));
    }
    // PG255 (15.11.2026, Vojtech "pg255 ano"): a join the WiFi driver made by itself (setAutoReconnect) is handled
    // like ours. Why: bench U9K2M3J0 boot 57, 12:04:19 - MAC unblocked, the driver rejoined NTLIRIS while our 5 min
    // skip (PG241) still ran; no "WIFI <net> IP" line, no "OK WIFI join", the fail count stayed and at 12:11:01
    // "ERR WIFI join x2" reached FleetNest with the box on the network.
    if (!s_joinSeen) wifiJoined(WiFi.SSID().c_str());
    s_knownAt = millis();
    if (!instActive()) apOff();                // PG233: in install mode the page stays reachable
    if (!telemetryWifiMarked() || millis() - s_altTry < WIFI_ALT_MS) return;
    s_altTry = millis();
    alt = true;
  }
  if (s_apOn && !webApAllowed() && !instActive()) apOff();
  if (!s_apOn && webApAllowed() && millis() - s_knownAt > WIFI_AP_AFTER_MS && strlen(webPassword()) >= 8) {
    char ssid[24];
    snprintf(ssid, sizeof(ssid), "NestBox-%s", devid);
    dbgMem("AP on before");                   // PG273
    WiFi.mode(WIFI_AP_STA);
    s_apOn = WiFi.softAP(ssid, webPassword());
    char ln[56];
    snprintf(ln, sizeof(ln), "WIFI AP %s %s %s", ssid, s_apOn ? "on" : "ERR", WiFi.softAPIP().toString().c_str());
    statusLine(ln);
  }
  // PG205: where no known network can be, nothing is searched
  static const char* lastWhy = "";
  static bool idle;
  const char* why = "";
  float lat = 0, lng = 0;
  int nets = wifiSearch(&why, &lat, &lng);
  if (why != lastWhy) {                        // into the serial / debug log only (it may change often)
    nbout.printf("[WIFI] search %s (%s)\n", nets ? "on" : "off", why);
    lastWhy = why;
    if (nets) s_wifiTry = 0;                   // entering a place: scan now
  }
  if (!nets) {
    if (!alt && !idle) { WiFi.disconnect(); idle = true; }   // the station stops retrying the network it lost
    return;
  }
  idle = false;
  if (!alt && s_wifiTry && millis() - s_wifiTry < WIFI_RETRY_MS) return;
  s_wifiTry = millis();
  // PG248: the scan runs in the WiFi driver; wifiScanDone() picks the network when it is finished
  s_wsNets = nets; s_wsLat = lat; s_wsLng = lng; s_wsAlt = alt;
  // PG264 (17.11.2026, Vojtech "264 ano"): a refused scan stops the driver's own reconnect, the next try (30 s)
  // scans. Why: bench U9K2M3J0 boot 91, 11:39:42 - the hotspot Pocof7ultra was switched off, the last scan saw 0
  // networks, then 30+ min no scan and no join (D91.LOG), also after SMS WIFI at 11:51, NTLIRIS in range. The
  // Arduino STA reconnects to the lost network by itself (STA.cpp DoReconnect) and the driver refuses a scan while
  // it is connecting (esp_wifi.h: ESP_ERR_WIFI_STATE "wifi still connecting"). disconnect() = reason ASSOC_LEAVE,
  // which Arduino never reconnects. Not when alt: then the box is joined and the data may still use that network.
  static bool refusedSaid;
  if (WiFi.scanNetworks(true) == WIFI_SCAN_FAILED) {
    nbout.printf("[WIFI] scan refused (status %d)%s\n", (int)WiFi.status(), alt ? "" : " - driver reconnect stopped");
    if (!refusedSaid) { statusLine(alt ? "WIFI scan refused" : "WIFI scan refused - reconnect stopped"); refusedSaid = true; }
    if (!alt) WiFi.disconnect();
    return;
  }
  refusedSaid = false;
  s_ws = WS_SCAN;
  s_wsAt = millis();
}

// PG248: the scan is finished (n networks) - join the strongest network of the list allowed here (PG208)
static void wifiScanDone(int n)
{
  int nets = s_wsNets;
  float lat = s_wsLat, lng = s_wsLng;
  bool alt = s_wsAlt;
  static WifiPlace cand;
  WifiPlace& bestNet = s_wsNet;
  int best = -1, bestRssi = -1000, bestScore = -1000;
  char seen[160];                              // PG241: the listed networks in range, into the debug log
  int sl = 0;
  seen[0] = 0;
  char user[33] = "";                          // PG233: the network just chosen on the page wins over a stronger one
  if (s_preferUser) {
    WifiPlace u;
    if (wifiUserGet(&u)) snprintf(user, sizeof(user), "%s", u.ssid);
    memset(u.pass, 0, sizeof(u.pass));
  }
  for (int i = 0; i < n; i++) {
    String ssid = WiFi.SSID(i);
    bool hit = false;
    for (int k = 0; !hit && wifiPlaceGet(k, &cand); k++)
      // 27.11.2026 (Vojtech „2 áno“): at a place (nets 1) every known network may win, the strongest one - not only
      // the place's own. Why: J0 in the Toyota at the depot saw uLTD_ext -82..-88 dBm (sniff upload 46 kB/s, TLS
      // fails) and the phone hotspot Pocof7ultra of the list at -31..-55 dBm, but tried only uLTD_ext. Away from
      // every place (nets 0) the box still does not search at all.
      hit = ssid == cand.ssid && (nets == 2 || nets == 1);
    if (!hit) continue;
    bool skip = telemetryWifiSkip(WiFi.BSSID(i)) || wifiSkipped(cand.ssid);   // PG239 / PG241
    // PG257 (15.11.2026, Vojtech "257 ano"): never "another" network = the access point the box is on. Why: bench
    // U9K2M3J0 boot 58 13:44:02 - a new join is "no internet" until the PG204 check; the PG239 scan started then
    // and ended 2 s after the check passed: "WIFI NTLIRIS no internet - trying NTLIRIS", left and rejoined the same
    // access point, 1.7 s on the modem. Also the scan result is dropped when the mark is gone (below).
    if (alt && WiFi.status() == WL_CONNECTED && WiFi.BSSID() && !memcmp(WiFi.BSSID(i), WiFi.BSSID(), 6)) skip = true;
    if (sl < (int)sizeof(seen) - 48)
      sl += snprintf(seen + sl, sizeof(seen) - sl, " %s(%d)%s", cand.ssid, (int)WiFi.RSSI(i), skip ? "-skip" : "");
    int score = WiFi.RSSI(i) + (user[0] && ssid == user ? 200 : 0);   // PG233: the chosen network first
    if (skip || score <= bestScore) continue;
    best = i; bestRssi = WiFi.RSSI(i); bestScore = score; bestNet = cand;
  }
  if (sl) nbout.printf("[WIFI] listed in range:%s\n", seen);
  {                                            // PG243 (14.11.2026, Vojtech "pg243 ano"): every network the scan saw -
    char all[400];                             // the phone hotspot of the list was not among the listed ones (21:14-21:27)
    int al = 0;                                // and the box could not tell whether it saw it at all
    all[0] = 0;
    for (int i = 0; i < n && al < (int)sizeof(all) - 48; i++)
      al += snprintf(all + al, sizeof(all) - al, " %.24s(%d,ch%d)", WiFi.SSID(i).c_str(), (int)WiFi.RSSI(i), (int)WiFi.channel(i));
    nbout.printf("[WIFI] scan %d:%s\n", n, all);
  }
  WiFi.scanDelete();
  if (best < 0) {
    if (!alt) nbout.printf("[WIFI] no known network in range (%d seen)\n", n);
    return;
  }
  if (alt) {                                   // PG239
    if (WiFi.status() != WL_CONNECTED) alt = false;   // lost meanwhile: a plain join
    else if (!telemetryWifiMarked()) return;   // PG257: the internet check passed while the scan ran - stay
    else {
      char ln[57];
      snprintf(ln, sizeof(ln), "WIFI %.16s no internet - trying %.16s", WiFi.SSID().c_str(), bestNet.ssid);
      statusLine(ln);
      WiFi.disconnect();
    }
  }
  s_knownAt = millis();
  nbout.printf("[WIFI] joining %s (%d dBm)\n", bestNet.ssid, bestRssi);
  static bool reasonHooked;
  if (!reasonHooked) { WiFi.onEvent(wifiReasonEvent, ARDUINO_EVENT_WIFI_STA_DISCONNECTED); reasonHooked = true; }
  s_wifiReason = 0;
  snprintf(s_pjSsid, sizeof(s_pjSsid), "%s", bestNet.ssid);   // PG233: the page shows the join
  s_pjState = 1; s_pjReason = 0;
  WiFi.begin(bestNet.ssid, bestNet.pass);
  s_ws = WS_JOIN;                              // PG248: wifiJoinTick() watches it
  s_wsAt = millis();
}

// PG248: the join started by wifiScanDone(), checked on every pass of loop() until it worked or WIFI_JOIN_MS passed
static void wifiJoinEnd(bool ok)
{
  WifiPlace& bestNet = s_wsNet;
  s_ws = WS_IDLE;
  s_pjState = ok ? 2 : 3;                      // PG233
  s_pjReason = s_wifiReason;
  if (s_preferUser) {                          // the chosen network tried once; then the strongest as always
    WifiPlace u;
    if (wifiUserGet(&u) && !strcmp(u.ssid, bestNet.ssid)) s_preferUser = false;
    memset(u.pass, 0, sizeof(u.pass));
  }
  if (ok) {
    wifiJoined(bestNet.ssid);
  } else {
    int r = s_wifiReason;                     // PG219
    nbout.printf("[WIFI] join timeout reason=%d\n", r);
    char d[48];
    snprintf(d, sizeof(d), "%.24s r=%d", bestNet.ssid, r);
    errReport("WIFI", "join", d);
    WifiFail* f = wifiFail(bestNet.ssid, true);    // PG241
    if (++f->fails >= 2) {
      f->fails = 0;
      f->until = (millis() + WIFI_SKIP_MS) | 1;
      nbout.printf("[WIFI] %s skipped for %lu min (2 failed joins)\n", bestNet.ssid, WIFI_SKIP_MS / 60000UL);
      s_wifiTry = 0;                            // the next network at once
    }
  }
  memset(bestNet.pass, 0, sizeof(bestNet.pass));   // the password does not stay in RAM longer than the join
}

void setup()
{
  telemetryEarly();                           // first: the delivery queue's PSRAM reserve (txqueue.h)
  clockTrustBegin();                          // PG259: after a deep-sleep wake the clock waits for login / GNSS / NTP
  if (g_lastStep.magic == LASTSTEP_MAGIC && esp_reset_reason() != ESP_RST_POWERON) {
    memcpy(s_prevStep, g_lastStep.text, sizeof(s_prevStep));
    s_prevStep[sizeof(s_prevStep) - 1] = 0;
  }
  lastStep("boot");
  standbyEarly();                             // a deep-sleep wake: confirmed, or asleep again here
  Serial.begin(115200);
  netlogBegin();                              // PG99: ESP-IDF log copied into RAM for /api/serial
  tlsMemToPsram();                            // PG274: mbedTLS buffers from PSRAM (after netlogBegin: fixed PSRAM order)
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
  if (!s_sdOk) sdPinsLine("mount-failed");    // PG102 (sdwriter.cpp); an error: always. "after-mount": PG270 below
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
  if (s_sdOk && dbglogOn()) sdPinsLine("after-mount");   // PG270: a detail line, only with DEBUG=1 (dbglog.h)
  wifiPlacesBegin();                         // PG205: the stored place list (NVS)
  instBegin();                               // PG233: install mode (install.h)
  otaBegin();                                // pull-OTA (needs the NVS handle opened by telemetryBegin)
  telemetrySmsCfgReport(3);                  // PG214: the SMS settings after every boot / wake (needs the OTA token)
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
  instTick();                                 // PG233
  standbyTick();
  errTick();                                  // PG149: error summaries + SD / CAN1 counters to the server
  partTick();                                 // PG177: the one-time partition table change when armed (part.h)
  static uint32_t lastPurge;
  if (millis() - lastPurge > 3600000UL || !lastPurge) {   // hourly, once the clock is set
    if (time(nullptr) > 1735689600) { lastPurge = millis(); sniffPurgeOld(time(nullptr)); dbglogPurge(time(nullptr)); }
  }
}
