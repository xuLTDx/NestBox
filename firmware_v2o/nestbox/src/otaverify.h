// OTA verify (PG147, ported from the a3 build 12.11.2026): a new firmware counts as good only after its first
// server login. Why: Arduino 2.0.14 marks a new app valid at once in initArduino() (framework-arduinoespressif32
// cores/esp32/esp32-hal-misc.c 222-238: verifyRollbackLater() = false, verifyOta() = true), so the bootloader rollback
// (CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE=y in tools/sdk/esp32/sdkconfig) only caught a crash before setup(); a build
// that crashes or cannot reach the server later stays on a car box, reachable only over USB (Peugeot 12.11.2026:
// abroad, 11 h frozen in the night - a bad update there could not be undone).
// ESP-IDF 4.4.6 bootloader (framework-espidf 4.4.6 components/bootloader_support/src/bootloader_utility.c 394-399,
// 344-352): a new app boots as PENDING_VERIFY; any reset before esp_ota_mark_app_valid_cancel_rollback() marks it
// ABORTED and the previous app boots. The Peugeot box got this bootloader over USB on 18.10.2026 (sha256 40608a7b...
// = .pio/build/ovms33/bootloader.bin).
#pragma once
#include <stdint.h>

void otaVerifyBegin();                       // setup(), after the NVS handle is open (telemetryBegin)
void otaVerifyLoggedIn();                    // telemetry: first successful server login of this boot
bool otaVerifyPending();                     // the running build still waits for its login
void otaVerifyFlashed(const char* sha);      // just before the restart into a new image (sha256 hex or "")
bool otaVerifyRejected(const char* sha);     // this image went back to the previous build once: do not flash it again
