// Debug log to SD, switched on/off (PG175, 12.11.2026).
// Why: Vojtech 27.10. ("Prečo nemáš debug kde ti to viazne") and 12.11. ("debug log ... musí byť, zapnúť vypnúť"):
// every field problem so far (CMUX 27.10, Peugeot 11 h stop 11.-12.11, a3 PANIC 12.11) had no log of the moment.
//  - DEBUG=1 / DEBUG=0 (FleetNest command, web /api/control), kept in NVS, default off: when on, everything of the
//    /api/serial ring (our lines + ESP-IDF log, netlog.cpp) is appended to /DEBUG/D<boot>.LOG every 2 s.
//    Not written synchronously from the print itself: v1F's Serial->SD mirror (16.9.2026) reset the box when /api/list
//    printed while iterating the card; here a separate task copies the ring, so a print never touches the card.
//    Bytes the ring overwrote before the copy (card slow / busy) are counted in the file ("[DBG] lost <n> bytes").
//  - always, also with DEBUG off: after a panic / watchdog restart the boot before's last 64 KB of the ring
//    (netlogTakePrev) -> /DEBUG/C<that boot>.LOG once, status line "DBG crash log C<boot> <bytes>".
//  - files in /DEBUG older than 7 days are deleted (hourly, with a valid clock).
//  - fetch: FleetNest LOG=D<boot> / LOG=C<boot> (ota.cpp, the PG49 upload; server file id 1000000+ / 2000000+boot).
#pragma once
#include <stdint.h>
#include <time.h>

void dbglogBegin(uint32_t bootId, bool sdOk);   // setup(), after the NVS handle is open (telemetryBegin)
bool dbglogSet(bool on);                        // stored in NVS; true = done
bool dbglogOn();
void dbglogPurge(time_t now);                   // loop(), hourly
