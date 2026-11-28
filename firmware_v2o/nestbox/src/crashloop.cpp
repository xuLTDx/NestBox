// PG358 crash-loop protection - behaviour and why: crashloop.h
#include "crashloop.h"
#include <esp_ota_ops.h>
#include <esp_partition.h>
#include <esp_rtc_time.h>
#include <esp_attr.h>
#include <nvs.h>
#include <string.h>
#include <time.h>
#include "clocktrust.h"
#include "otaverify.h"
#include "netlog.h"

extern nvs_handle_t nvs;                     // telemetry.cpp
void statusLine(const char* text);           // ota.cpp
bool wdtWasRestart();                        // wdt.cpp
const char* crashLastTask();                 // crash.cpp
bool wifiRequest(bool on);                   // main.cpp
bool telemetryWifiForData();                 // telemetry.cpp

#define CL_MAGIC     0xC1A55E01u
#define CL_N         5                       // crashes ...
#define CL_WINDOW_US (10ULL * 60ULL * 1000000ULL)   // ... within 10 min = a loop
#define CL_EARLY_N   2                       // 2 within 10 min: the OTA check at once (crashLoopEarlyOta)
#define CL_GOOD_S    (24UL * 3600UL)         // a build is "good" 24 h after its OTA VALID without a loop
static const uint32_t OFF_MIN[3] = {30, 120, 360};   // the off period by loop number (30 min, 2 h, 6 h)
enum { PART_NONE, PART_CELL, PART_WIFI, PART_SNIFF };
static const char* const PART_NAME[] = {"-", "modem", "wifi", "sniff+read"};

// RTC memory: survives software restarts, panics and deep sleep (not a power-on - then the magic is random)
struct ClState {
  uint32_t magic;
  uint64_t at[CL_N];                         // RTC time (us) of the last crash restarts, newest first
  uint8_t level;                             // loops so far (0..3), back to 0 after 24 h without one
  uint8_t part;                              // PART_*
  uint64_t offUntil;                         // RTC us, 0 = nothing off
  uint64_t lastLoop;                         // RTC us of the last loop
};
RTC_NOINIT_ATTR static ClState s_cl;

#define KEY_GOOD  "CL_GOOD"                  // label of the app partition holding a build marked good
#define KEY_RUN   "CL_RUN"                   // label the next two keys belong to
#define KEY_RSHA  "CL_RSHA"                  // sha256 of the running build's image (from the OTA download)
#define KEY_VALT  "CL_VALT"                  // unix time the running build was seen valid with a trusted clock

static bool isCrash(esp_reset_reason_t r)
{
  return r == ESP_RST_PANIC || r == ESP_RST_INT_WDT || r == ESP_RST_TASK_WDT || r == ESP_RST_WDT ||
         (r == ESP_RST_SW && wdtWasRestart());
}

static uint8_t whichPart(const char* task, const char* step)
{
  if (!strcmp(task, "cell") || strstr(step, "CELL") || strstr(step, "CMUX") || strstr(step, "PPP") || strstr(step, "cell"))
    return PART_CELL;
  if (!strncmp(step, "WIFI", 4) || !strcmp(task, "wifi")) return PART_WIFI;
  return PART_SNIFF;                         // not known: the networks stay, the sniff + reads go off
}

static uint32_t crashesWithin(uint64_t now)
{
  uint32_t n = 0;
  for (int i = 0; i < CL_N; i++)
    if (s_cl.at[i] && now - s_cl.at[i] < CL_WINDOW_US) n++;
  return n;
}

static bool tryRollback()
{
  const esp_partition_t* run = esp_ota_get_running_partition();
  const esp_partition_t* other = esp_ota_get_next_update_partition(nullptr);
  char good[17] = "", sha[65] = "";
  size_t l = sizeof(good);
  if (!run || !other || nvs_get_str(nvs, KEY_GOOD, good, &l) != ESP_OK || strcmp(good, other->label)) return false;
  l = sizeof(sha);
  if (nvs_get_str(nvs, KEY_RSHA, sha, &l) != ESP_OK || !sha[0]) return false;   // unknown image: would come again
  esp_app_desc_t d;
  if (esp_ota_get_partition_description(other, &d) != ESP_OK) return false;
  if (esp_ota_set_boot_partition(other) != ESP_OK) return false;
  nvs_set_str(nvs, "OTAV_REJ", sha);         // otaverify.cpp KEY_REJ_SHA: the pull-OTA skips this image
  nvs_erase_key(nvs, KEY_GOOD);              // the build we go to is the running one now; marked again after 24 h
  nvs_commit(nvs);
  char m[96];
  snprintf(m, sizeof(m), "CRASHLOOP rollback sha=%.12s -> %s %s %s", sha, other->label, d.date, d.time);
  statusLine(m);                             // in the PSRAM queue, which survives the restart
  nbout.println(m);
  delay(500);
  esp_restart();
  return true;
}

void crashLoopBegin(const char* prevStep)
{
  if (s_cl.magic != CL_MAGIC) { memset(&s_cl, 0, sizeof(s_cl)); s_cl.magic = CL_MAGIC; }
  uint64_t now = esp_rtc_get_time_us();
  // the running partition changed (OTA, rollback, USB): its sha / valid time start empty
  const esp_partition_t* run = esp_ota_get_running_partition();
  char lab[17] = "";
  size_t l = sizeof(lab);
  if (run && (nvs_get_str(nvs, KEY_RUN, lab, &l) != ESP_OK || strcmp(lab, run->label))) {
    nvs_set_str(nvs, KEY_RUN, run->label);
    nvs_erase_key(nvs, KEY_RSHA);
    nvs_erase_key(nvs, KEY_VALT);
    nvs_commit(nvs);
  }
  if (s_cl.lastLoop && now - s_cl.lastLoop > (uint64_t)CL_GOOD_S * 1000000ULL) s_cl.level = 0;
  esp_reset_reason_t r = esp_reset_reason();
  if (r == ESP_RST_POWERON) { memset(s_cl.at, 0, sizeof(s_cl.at)); return; }
  if (!isCrash(r)) return;
  memmove(&s_cl.at[1], &s_cl.at[0], sizeof(s_cl.at[0]) * (CL_N - 1));
  s_cl.at[0] = now;
  if (crashesWithin(now) < CL_N) return;
  // a crash loop
  memset(s_cl.at, 0, sizeof(s_cl.at));
  s_cl.lastLoop = now;
  nvs_erase_key(nvs, KEY_VALT);              // this build is not "good" - 24 h again from now
  nvs_commit(nvs);
  const char* task = crashLastTask();
  const char* step = prevStep && prevStep[0] ? prevStep : "-";
  if (s_cl.level == 0 && !otaVerifyPending() && tryRollback()) return;   // (restarts)
  uint8_t lv = s_cl.level < 3 ? s_cl.level : 2;
  if (s_cl.level < 3) s_cl.level++;
  s_cl.part = whichPart(task ? task : "-", step);
  s_cl.offUntil = now + (uint64_t)OFF_MIN[lv] * 60ULL * 1000000ULL;
  char m[96];
  snprintf(m, sizeof(m), "CRASHLOOP n=%d 10min last=%.12s/%.20s -> %s off %umin", CL_N, task ? task : "-", step,
           PART_NAME[s_cl.part], (unsigned)OFF_MIN[lv]);
  statusLine(m);
  nbout.println(m);
}

static bool partOff(uint8_t p)
{
  return s_cl.magic == CL_MAGIC && s_cl.part == p && s_cl.offUntil && esp_rtc_get_time_us() < s_cl.offUntil;
}

// the modem only while WiFi carries the data - without WiFi the modem off would leave the box unreachable
bool crashLoopCellOff() { return partOff(PART_CELL) && telemetryWifiForData(); }
bool crashLoopSniffOff() { return partOff(PART_SNIFF); }
bool crashLoopEarlyOta()
{
  return s_cl.magic == CL_MAGIC && (crashesWithin(esp_rtc_get_time_us()) >= CL_EARLY_N || (s_cl.offUntil && esp_rtc_get_time_us() < s_cl.offUntil));
}

void crashLoopTick()
{
  static uint32_t last;
  if (millis() - last < 5000) return;
  last = millis();
  // WiFi off as a part: through wifiRequest (main.cpp) - it refuses while the modem is off by setting (CELL=0)
  static bool wifiOffDone;
  bool wOff = partOff(PART_WIFI);
  if (wOff && !wifiOffDone) wifiOffDone = wifiRequest(false);
  if (!wOff && wifiOffDone) { wifiRequest(true); wifiOffDone = false; }
  if (s_cl.magic == CL_MAGIC && s_cl.offUntil && esp_rtc_get_time_us() >= s_cl.offUntil) {
    char m[48];
    snprintf(m, sizeof(m), "CRASHLOOP %s on again", PART_NAME[s_cl.part]);
    statusLine(m);
    s_cl.offUntil = 0;
    s_cl.part = PART_NONE;
  }
  // the 24 h "good" mark of the running build
  if (otaVerifyPending() || !clockTrusted()) return;
  const esp_partition_t* run = esp_ota_get_running_partition();
  if (!run) return;
  uint32_t valt = 0;
  time_t now = time(nullptr);
  if (nvs_get_u32(nvs, KEY_VALT, &valt) != ESP_OK) {
    nvs_set_u32(nvs, KEY_VALT, (uint32_t)now);
    nvs_commit(nvs);
    return;
  }
  if ((uint32_t)now - valt < CL_GOOD_S) return;
  char good[17] = "";
  size_t l = sizeof(good);
  if (nvs_get_str(nvs, KEY_GOOD, good, &l) == ESP_OK && !strcmp(good, run->label)) return;
  nvs_set_str(nvs, KEY_GOOD, run->label);
  nvs_commit(nvs);
  statusLine("CRASHLOOP build marked good (24 h)");
}

void crashLoopPartitionWritten(const char* label)
{
  char good[17] = "";
  size_t l = sizeof(good);
  if (label && nvs_get_str(nvs, KEY_GOOD, good, &l) == ESP_OK && !strcmp(good, label)) {
    nvs_erase_key(nvs, KEY_GOOD);            // the good build there is being overwritten
    nvs_commit(nvs);
  }
}

void crashLoopValid(const char* sha)
{
  const esp_partition_t* run = esp_ota_get_running_partition();
  if (!run) return;
  nvs_set_str(nvs, KEY_RUN, run->label);
  if (sha && sha[0]) nvs_set_str(nvs, KEY_RSHA, sha);
  nvs_erase_key(nvs, KEY_VALT);              // the 24 h count from now (crashLoopTick)
  nvs_commit(nvs);
}
