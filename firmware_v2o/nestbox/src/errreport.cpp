// Errors to the server - see errreport.h (PG149, 11.11.2026)
#include "errreport.h"
#include <Arduino.h>
#include <string.h>
#include "canbus1.h"
#include "sdwriter.h"

void statusLine(const char* text);           // ota.cpp

// 16 different errors open at the same time. Why: the list agreed with FleetNest has ~15 <area> <what> pairs; a full
// table still sends the line (counted in s_full), only without the repeat summary.
#define ERR_SLOTS 16

struct ErrSlot {
  char key[24];                // "<area> <what>"
  bool open;                   // an error without an OK since
  uint32_t count;              // repeats since the last line
  uint32_t since;              // millis() of the last line
};
static ErrSlot s_slots[ERR_SLOTS];
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;
static uint32_t s_full;

static void makeKey(char* k, int cap, const char* area, const char* what) { snprintf(k, cap, "%s %s", area, what); }

static ErrSlot* find(const char* key)          // under s_mux
{
  for (int i = 0; i < ERR_SLOTS; i++)
    if (s_slots[i].key[0] && !strcmp(s_slots[i].key, key)) return &s_slots[i];
  return nullptr;
}

void errReport(const char* area, const char* what, const char* detail)
{
  char key[24];
  makeKey(key, sizeof(key), area, what);
  bool send = false;
  portENTER_CRITICAL(&s_mux);
  ErrSlot* s = find(key);
  if (!s)
    for (int i = 0; i < ERR_SLOTS && !s; i++)
      if (!s_slots[i].open) { s = &s_slots[i]; strcpy(s->key, key); }
  if (!s) { s_full++; send = true; }
  else if (!s->open) { s->open = true; s->count = 0; s->since = millis(); send = true; }
  else s->count++;
  portEXIT_CRITICAL(&s_mux);
  if (!send) return;
  char m[64];
  snprintf(m, sizeof(m), detail && detail[0] ? "ERR %s %s" : "ERR %s", key, detail);
  statusLine(m);
}

void errOk(const char* area, const char* what, const char* okWhat)
{
  char key[24];
  makeKey(key, sizeof(key), area, what);
  uint32_t n = 0;
  bool was = false;
  portENTER_CRITICAL(&s_mux);
  ErrSlot* s = find(key);
  if (s && s->open) { was = true; n = s->count; s->open = false; s->count = 0; }
  portEXIT_CRITICAL(&s_mux);
  if (!was) return;
  char m[64];
  if (n) { snprintf(m, sizeof(m), "ERR %s x%lu", key, (unsigned long)n); statusLine(m); }   // the repeats first
  if (okWhat) snprintf(m, sizeof(m), "OK %s %s", area, okWhat);
  else snprintf(m, sizeof(m), "OK %s", key);
  statusLine(m);
}

// counters that only grow: SD bytes not written, CAN1 driver counters (twai_get_status_info) - read every 10 s.
// A counter that went down (CAN1 driver restarted by the speed probe) starts a new base.
static void pollCounter(uint32_t now, uint32_t* base, const char* area, const char* what)
{
  if (now < *base) { *base = now; return; }
  if (now == *base) return;
  char d[16];
  snprintf(d, sizeof(d), "+%lu", (unsigned long)(now - *base));
  *base = now;
  errReport(area, what, d);
}

void errTick()
{
  static uint32_t lastPoll;
  if (millis() - lastPoll >= 10000) {
    lastPoll = millis();
    static uint32_t sdLost, canMissed, canOverrun, canBusErr;
    static bool first = true;
    uint32_t lost = sdwLostBytes();
    Can1Stats c = can1Stats();
    if (first) { first = false; sdLost = lost; canMissed = c.rxMissed; canOverrun = c.rxOverrun; canBusErr = c.busErr; }
    // after "ERR SD dead" every record is lost bytes - the dead line says it already
    if (!sdwDataDead()) pollCounter(lost, &sdLost, "SD", "lost");
    else sdLost = lost;
    // bus errors / losses while the speed probe tries a wrong bit rate are expected - only on the found speed
    if (c.running && c.speedFound) {
      pollCounter(c.rxMissed, &canMissed, "CAN1", "missed");
      pollCounter(c.rxOverrun, &canOverrun, "CAN1", "overrun");
      pollCounter(c.busErr, &canBusErr, "CAN1", "buserr");
    } else {
      canMissed = c.rxMissed; canOverrun = c.rxOverrun; canBusErr = c.busErr;
    }
  }
  // summaries of repeats (one per slot per call is enough: loop() runs many times a second)
  for (int i = 0; i < ERR_SLOTS; i++) {
    char m[64] = "";
    portENTER_CRITICAL(&s_mux);
    ErrSlot& s = s_slots[i];
    if (s.open && s.count && millis() - s.since >= ERR_SUMMARY_MS) {
      snprintf(m, sizeof(m), "ERR %s x%lu", s.key, (unsigned long)s.count);
      s.count = 0;
      s.since = millis();
    }
    portEXIT_CRITICAL(&s_mux);
    if (m[0]) statusLine(m);
  }
}
