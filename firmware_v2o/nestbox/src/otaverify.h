// OTA verify (PG147, 11.11.2026): a new firmware counts as good only after its first server login.
// Why: Arduino 3 marks a new app valid at once in initArduino() (esp32-hal-misc.c 315-321, verifyOta() = true),
// so the bootloader rollback (CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE=1) only caught a crash before setup(); a build
// that crashes or cannot reach the server later stayed on a car box, reachable only over USB.
// ESP-IDF 5.5.5 bootloader (bootloader_utility.c 396-404, 446-451): a new app boots as PENDING_VERIFY; any reset
// before esp_ota_mark_app_valid_cancel_rollback() marks it ABORTED and the previous app boots.
#pragma once
#include <stdint.h>

void otaVerifyBegin();                       // setup(), after the NVS handle is open (telemetryBegin)
void otaVerifyLoggedIn();                    // telemetry: first successful server login of this boot
bool otaVerifyPending();                     // the running build still waits for its login
void otaVerifyFlashed(const char* sha);      // just before the restart into a new image (sha256 hex or "")
bool otaVerifyRejected(const char* sha);     // this image went back to the previous build once: do not flash it again
