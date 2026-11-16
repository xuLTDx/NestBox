// PG259 (16.11.2026, Vojtech "259 teraz"): is the system clock good enough for a record's time?
// Why: the RTC runs about 2-3 % fast in deep sleep. J0 16.11.: after 90 min of sleep the first 5 status lines
// carried 09:21:25-29 local, the server got them at 09:18:56 (149 s in the future); in the night 109-198 s after
// each 3 h report sleep. The clock looked valid (> 2025), so the records were stamped with it. After a weekend in a
// car it would be about an hour (computed, NEOVERENE). Now, after a deep-sleep wake, the clock is not trusted until
// the server login (TM=), GNSS or NTP sets it; records made meanwhile wait without a time and get it computed
// back from their ms stamp (txqueue.cpp untimedAction, the same path as a power-on without a clock).
#pragma once
#include <time.h>

bool clockTrusted();                        // a valid time (> 2025) and, after a deep-sleep wake, set since
void clockSetFrom(time_t t, const char* src);   // settimeofday(t) (when off > 2 s) + trusted; any task
void clockTrustBegin();                     // setup(): remembers the clock at the wake, registers the NTP callback
// "CLOCK <src> after sleep <+-s>s" once per wake (the drift that was corrected), "" if none; record task
bool clockTakeLine(char* out, int cap);
