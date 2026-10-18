// APN from the server with a safety net - NestBox_v1F and NestBox_v2O (2026-10-18).
//
//   server command  APN=<apn>   (APN=DEFAULT = the build's default, "internet" for 4ka)
//
// The new APN is only TRIED: it is kept as "APN_NEW" next to the working "CELL_APN" (NVS
// namespace "storage"). The next modem start uses it; when the box then logs in to the server
// over the cellular link, it becomes CELL_APN ("APN OK"). When the modem registered but the
// login failed NB_APN_TRIES times in a row, the trial is dropped and the old APN is used again
// ("APN REVERT") - a wrong APN cannot cut the box off for good. A failed registration (no
// signal) is not counted: that is not the APN's fault.
// Every outcome goes into the next record as text, PID 0x39E.
#pragma once
#include <Arduino.h>
#include <nvs.h>

#define PID_APN_STATUS 0x39E
#define NB_APN_TRIES 2

void nbApnCommand(nvs_handle_t nvs, const char* arg, const char* buildDefault);  // server / web
// the APN for this modem start: the trial one, else `current` (the box's own CELL_APN or default)
const char* nbApnPick(nvs_handle_t nvs, const char* current);
// after a modem start that REGISTERED: loginOk = logged in to the server over this link
void nbApnAfterLogin(nvs_handle_t nvs, bool loginOk);
int nbApnTakeStatus(uint8_t* out, int cap);   // the record's PID 0x39E text, 0 = nothing new
