// PG233 (18.11.2026, Vojtech "ano" to the design, "2 ano" to the code): install mode with the box's own WiFi and a
// mini page. Why: a new box after the USB upload (PG266 script, tools/novy_box.py) has the OTA token but no WiFi it
// may search - box U9K2652Z 17.11.2026: "search off (nolist)", the setup went over 2 Claude sessions; a customer's
// box must be set up from a phone without typing codes (Vojtech 18.11.). Design and decisions:
// /home/claude/notes/handoff/2026-11-18_nestbox-v2o_to_fleetnest_novy-box.md, section PG233.
//
// Install mode = NVS INST_WIFI_PW set (8-63 chars, one password for every box, written by the USB script) and
// INST_DONE not set. Then:
//  - the box's own WiFi "NestBox-<ID>" with that password right from the boot, kept while joined elsewhere;
//  - the web serves only the mini page (status, WiFi scan, choose network + password) - the rest of the API (AT,
//    OTA upload, files, commands) is closed, because the password is the same on every box;
//  - every listed network (server list + the chosen one) is searched wherever the box is.
// It ends with whatever comes first (Vojtech 18.11.): a) the chosen network joined and the box logged in to
// FleetNest over it; b) INST=0 from FleetNest (FN252: sent once the box is assigned to a car); c) 7 days after the
// first login. Then NVS INST_DONE=1, the own WiFi only with AP=1 as before (K12).
// Boxes without INST_WIFI_PW (all boxes set up before PG266) never enter install mode.
#pragma once
#include <Arduino.h>

void instBegin();                 // after telemetryBegin (NVS handle)
bool instActive();
const char* instPassword();       // "" when not in install mode
void instTick();                  // loop(): ends a) and c)
void instEnd(const char* why);    // b) and the others; idempotent
