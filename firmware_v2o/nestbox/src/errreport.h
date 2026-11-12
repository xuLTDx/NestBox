// Errors to the server (PG149, 11.11.2026). Why: Vojtech 10.11. "VSETKY CHYBY HLASIT NA SERVER" - an SD error was
// written only to the card that was failing (sdwriter.cpp FE,SD_ERR), UDP / connect failures only to the SD log
// (teleclient.cpp), the CAN driver counters only on /api; FleetNest saw none of it (Peugeot 1.-11.11.: 5432 status
// lines, none about SD). Format agreed with FleetNest (notes/handoff/2026-11-11_..._errors-to-server.md):
//   "ERR <area> <what> [detail]"  - the first one at once
//   "ERR <area> <what> x<n>"      - the same error again: counted, one summary every ERR_SUMMARY_MS (if n > 0)
//   "OK <area> <what>"            - the first success after the error
// One PID 0x399 status line each (statusLine, <= 56 chars, ota.cpp); the record carries its time and goes through the
// delivery queue + SD, so an error without network arrives later with the time of its record.
// Lines that reach FleetNest already today (CELL CMUX dead, CELL REG LOST, CELL PPP LOST, SERVER NO ANSWER, NO INTERNET,
// OTA-PULL ERR=, OTA ROLLBACK, BOOT reset=, ULP ...) stay as they are - no second line for the same event.
#pragma once
#include <stdint.h>

// 10 min. Why: the status queue holds 24 lines and one goes into each record (ota.cpp / telemetry.cpp); Peugeot sent
// "CCH START ERR" 1548x in 10 days - one line each would fill the queue and the data. FleetNest counts x<n> as n errors.
#define ERR_SUMMARY_MS (10UL * 60UL * 1000UL)

void errReport(const char* area, const char* what, const char* detail = nullptr);   // any task
void errOk(const char* area, const char* what, const char* okWhat = nullptr);        // any task; nothing if no error
                                         // okWhat: the word after "OK <area>" (agreed: ERR GNSS nofix -> OK GNSS fix)
void errTick();                                                                      // loop(): summaries, polled counters
