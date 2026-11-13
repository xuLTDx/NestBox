// One-time change of the partition table over OTA (PG177, 12.11.2026): + "coredump" 64 KB at the end of the flash,
// "spiffs" 64 KB smaller; nvs, otadata, app0, app1 stay byte for byte the same (same offsets, same entries).
// Why: a panic dump was nowhere (a3 bench 12.11. 16:47) - CONFIG_ESP_COREDUMP_ENABLE_TO_FLASH=y is already in the
// Arduino 3.3.11 libs, only the partition is missing ("No core dump partition found!"); the Peugeot can get a new
// table only over OTA (USB = half the car taken apart, Vojtech 12.11.: the engine is kept running for the update).
// How (agreed 12.11.2026, PG177):
//  - armed by PART=CD (web /api/control; FleetNest after its validate() allows it), stored in NVS, PART=0 disarms;
//  - runs only when: armed, the 12 V >= PART_MIN_V without a break for PART_STABLE_MS (engine running / bench supply),
//    the delivery queue at most 5 live records (no backlog), no OTA running, the build confirmed (otaverify), and the table in flash is EXACTLY the
//    known old one (PART_OLD) - anything else: "PART ERR ..." and disarmed, nothing written;
//  - the write: ESP-IDF refuses writes to the table (CONFIG_SPI_FLASH_DANGEROUS_WRITE_ABORTS=y, esp_flash_api.c
//    CHECK_WRITE_ADDRESS -> chip->os_func->region_protected); for the one sector 0x8000 a copy of the os_func table
//    whose region_protected allows only that sector is used (the normal esp_flash erase/write with its cache handling),
//    then the original pointer is back;
//  - read back and compared; a difference -> the old table written back the same way ("PART ERR verify");
//  - the only unsafe moment: erase + write of one 4 KB sector (tens of ms) - a power loss or crash then = no boot;
//  - then disarmed, "PART CD written - restart", restart; the next boot finds the coredump partition -> "PART OK";
//    SPIFFS (standby records only, the queue was empty) is formatted at its next mount if it does not fit.
#pragma once

bool partArm(bool on);               // PART=CD / PART=0; true = stored
const char* partState();             // "old" | "new" | "unknown" + armed flag, for PART?
void partTick();                     // loop(): checks the conditions, does the change once
void partBootReport();               // setup(), after telemetryBegin(): "PART OK coredump 64 KB" once after the change
