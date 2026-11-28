// PG358 (28.11.2026, Vojtech "358 áno"): protection against a crash loop on a valid build.
// Why: J0 13.11.2026 08:24-08:39 - 14 restarts (PANIC / TASK_WDT "net") in 15 min, the box did nothing useful until
// the cause went away by itself; a new build that crashes is already rolled back (otaverify.cpp), a valid one was not.
// Behaviour (AKO_FUNGUJE.md 3.7):
//  1. 5 crash restarts (PANIC, TASK_WDT, INT_WDT, WDT, own watchdog) within 10 min = a crash loop.
//  2. The first loop on this build: back to the other app partition when it holds a build marked good (ran 24 h after
//     its OTA VALID without a loop) and the crashing build's sha256 is known (it is rejected, OTAV_REJ, so the pull-OTA
//     does not install it again until a newer one comes). Status line "CRASHLOOP rollback ...".
//  3. Otherwise one part is switched off for 30 min (then 2 h, then 6 h): the modem when the crash was in the modem
//     (and WiFi is there for the data), WiFi when it was in WiFi, else the sniff + reads into the car. NEVER both networks.
//     Records go on to SD all the time. Status line "CRASHLOOP n=5 10min last=<task>/<step> -> <part> off <min>min".
//  4. While crashes repeat (2 within 10 min) or a part is off, the OTA check runs as soon as a network is up (ota.cpp),
//     not after 60 s - a box that crashes within a minute of the start gets the fixed build.
#pragma once
#include <Arduino.h>

void crashLoopBegin(const char* prevStep);   // setup(), after telemetryBegin() + otaVerifyBegin()
void crashLoopTick();                       // loop(): the 24 h "good build" mark, the end of an off period
bool crashLoopEarlyOta();                   // ota.cpp: crashes repeating or a part off -> OTA check at once
bool crashLoopCellOff();                    // telemetry.cpp cellTask
bool crashLoopSniffOff();                   // sniff.cpp startNow, canbus1.cpp / sniff.cpp reads
void crashLoopPartitionWritten(const char* label);   // ota.cpp: a download starts writing this app partition
void crashLoopValid(const char* sha);       // otaverify.cpp: the running build logged in (sha of its image or "")
