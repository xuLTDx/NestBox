// OTA verify - see otaverify.h (PG147, 11.11.2026)
#include "otaverify.h"
#include <Arduino.h>
#include <esp_ota_ops.h>
#include <esp_timer.h>
#include <esp_partition.h>
#include <nvs.h>
#include <string.h>

extern nvs_handle_t nvs;                     // telemetry.cpp
void statusLine(const char* text);           // ota.cpp
#include "laststep.h"

// 15 min for the first login after an update. Why: on the modem the first PPP + login can take minutes (cell
// OTA ~3 min, PG-p1 restart of the modem after 15 min without an answer is the outer limit); longer = more battery
// on a parked car, because the box does not go to standby while the build is unconfirmed (standby.cpp).
#define OTA_VERIFY_MS (15UL * 60UL * 1000UL)

// NVS (namespace of telemetry.cpp): partition label of the image just flashed + its sha256; the sha256 of an image
// that went back to the previous build (never flashed again - user 11.11.2026: no endless download/flash/rollback
// loop on a car without coverage, each loop = 15 min awake).
#define KEY_FL_PART "OTAV_PART"
#define KEY_FL_SHA  "OTAV_SHA"
#define KEY_REJ_SHA "OTAV_REJ"

static volatile bool s_pending;
static esp_timer_handle_t s_timer;

// Arduino 3 hook (esp32-hal-misc.c 285): true = initArduino() leaves the PENDING_VERIFY state alone
extern "C" bool verifyRollbackLater() { return true; }

#define KEY_WHY     "OTAV_WHY"

// runs in the previous build after the bootloader went back: the image written last is not the one running.
// Reported here (not before the restart) because the queue of the failed build may never reach the server.
static void reportRollback()
{
  char sha[65] = "", why[24] = "";
  size_t l = sizeof(sha);
  nvs_get_str(nvs, KEY_FL_SHA, sha, &l);
  l = sizeof(why);
  if (nvs_get_str(nvs, KEY_WHY, why, &l) != ESP_OK || !why[0]) strcpy(why, "crash-or-reset");
  if (sha[0]) nvs_set_str(nvs, KEY_REJ_SHA, sha);
  nvs_erase_key(nvs, KEY_FL_PART);
  nvs_erase_key(nvs, KEY_FL_SHA);
  nvs_erase_key(nvs, KEY_WHY);
  nvs_commit(nvs);
  char m[120];
  snprintf(m, sizeof(m), "OTA ROLLBACK %s sha=%.16s", why, sha[0] ? sha : "-");
  statusLine(m);
}

// runs in the esp_timer task, not in loop() or the telemetry task: a hang there does not stop it
static void deadline(void*)
{
  if (!s_pending) return;
  lastStep("ota verify: no login in 15 min");
  nvs_set_str(nvs, KEY_WHY, "no-login-15min");
  nvs_commit(nvs);
  statusLine("OTA VERIFY no login in 15 min - back");
  esp_ota_mark_app_invalid_rollback_and_reboot();   // ABORTED + restart into the previous build
}

void otaVerifyBegin()
{
  const esp_partition_t* run = esp_ota_get_running_partition();
  esp_ota_img_states_t st;
  bool pending = run && esp_ota_get_state_partition(run, &st) == ESP_OK && st == ESP_OTA_IMG_PENDING_VERIFY;

  char part[17] = "";
  size_t l = sizeof(part);
  bool flashed = nvs_get_str(nvs, KEY_FL_PART, part, &l) == ESP_OK && part[0];
  if (flashed && run && strcmp(part, run->label) != 0) {
    // not running the image written last. A rollback only if the bootloader / deadline() marked it ABORTED or
    // INVALID - else something else was installed after it (bench 11.11.2026: a3 -> /api/ota test firmware ->
    // a3 again reported "OTA ROLLBACK crash-or-reset" without any rollback)
    const esp_partition_t* fp = esp_partition_find_first(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_ANY, part);
    esp_ota_img_states_t fst;
    if (fp && esp_ota_get_state_partition(fp, &fst) == ESP_OK &&
        (fst == ESP_OTA_IMG_ABORTED || fst == ESP_OTA_IMG_INVALID)) {
      reportRollback();
    } else {
      nvs_erase_key(nvs, KEY_FL_PART);
      nvs_erase_key(nvs, KEY_FL_SHA);
      nvs_erase_key(nvs, KEY_WHY);
      nvs_commit(nvs);
    }
    flashed = false;
  }
  if (!pending) {
    if (flashed && run && strcmp(part, run->label) == 0) {   // an older build without this module marked it valid
      nvs_erase_key(nvs, KEY_FL_PART);
      nvs_erase_key(nvs, KEY_FL_SHA);
      nvs_erase_key(nvs, KEY_WHY);
      nvs_commit(nvs);
    }
    return;
  }
  s_pending = true;
  statusLine("OTA VERIFY pending (login in 15 min or back)");   // <= 56 chars (ota.cpp statusLine)
  esp_timer_create_args_t a = {};
  a.callback = deadline;
  a.dispatch_method = ESP_TIMER_TASK;
  a.name = "otaverify";
  if (esp_timer_create(&a, &s_timer) == ESP_OK) esp_timer_start_once(s_timer, (uint64_t)OTA_VERIFY_MS * 1000ULL);
}

void otaVerifyLoggedIn()
{
  if (!s_pending) return;
  s_pending = false;
  if (s_timer) esp_timer_stop(s_timer);
  esp_ota_mark_app_valid_cancel_rollback();
  nvs_erase_key(nvs, KEY_FL_PART);
  nvs_erase_key(nvs, KEY_FL_SHA);
  nvs_erase_key(nvs, KEY_WHY);
  nvs_commit(nvs);
  statusLine("OTA VALID (server login)");
}

bool otaVerifyPending() { return s_pending; }

void otaVerifyFlashed(const char* sha)
{
  const esp_partition_t* next = esp_ota_get_boot_partition();   // Update.end() has set it to the new image
  if (!next) return;
  nvs_set_str(nvs, KEY_FL_PART, next->label);
  nvs_set_str(nvs, KEY_FL_SHA, sha ? sha : "");
  nvs_commit(nvs);
}

bool otaVerifyRejected(const char* sha)
{
  char rej[65] = "";
  size_t l = sizeof(rej);
  return sha && sha[0] && nvs_get_str(nvs, KEY_REJ_SHA, rej, &l) == ESP_OK && strcasecmp(rej, sha) == 0;
}
