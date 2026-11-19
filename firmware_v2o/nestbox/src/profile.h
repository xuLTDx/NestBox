// PG283 (19.11.2026, Vojtech "283 áno"): the car profile comes from the server instead of being written in the code.
// Why: what is read from a car (CAN rules, UDS reads, OBD questions, how the engine is known) was fixed in the firmware
// (cardecode.cpp K0, telemetry.cpp K0_READS) - every new item found on one K0 car needed a new firmware for all boxes.
// FleetNest keeps the profiles (FN260 / FN263, by VIN); the OTA server turns the box's profile into short TAB lines
// (GET /api/nestbox/ota/<token>/profile.txt, nestbox-ota 2.7.0), the box keeps them in /spiffs/profile.txt.
//
// Lines (TAB between fields, numbers decimal, lists comma separated, "-" = none):
//   V  <version 16 hex>  <code>                         first line; version "" = the car has no profile
//   M  can  <id>  <bytes>  <mult>                       engine runs while this value > 0 (K0: 0x208 x 0.125)
//   M  obd                                              engine from OBD 01 0C (as PG277)
//   E  <uds round s>                                    all UDS reads after the engine START and then every <s>
//   C  <bus> <id> <dlc min> <bytes> <shift> <mask> <mult> <add> <invalid> <raw 0|1> <pid> <field> <h|z>
//                                                       one CAN value: ((bytes big-endian >> shift) & mask) * mult + add;
//                                                       h = in every record while fresh, z = once after each change;
//                                                       field >= 0: the rules of one pid go together (u32n)
//   U  <number> <tx> <rx> <buses> <dids>                a UDS 22 read; buses "-" = switched off, the number stays
//   O  <tx> <rx> <buses> <obd pid> <pid> <every s>      an OBD 01 question while the engine runs, raw data bytes
//
// A new profile is stored at once but used from the next start of the box (after a sleep / restart) - swapping the
// tables while the CAN task decodes would need a lock on every frame; profiles change rarely.
// Without a stored profile the box works as before: VEH=K0 -> the fixed K0 code, VEH= empty -> OBD (PG277).
#pragma once
#include <Arduino.h>

#define PROF_CAN_MAX 48
#define PROF_UDS_MAX 32
#define PROF_OBD_MAX 16

struct ProfCan {
  uint32_t id, mask, inv;
  float mult, add;
  uint16_t pid;
  uint8_t bus, dlcMin, nb, bytes[8], shift;
  int8_t field;
  bool hasInv, raw, change;
};
struct ProfUds {
  uint32_t tx, rx;
  uint16_t did[3];
  uint8_t num, nDid, buses[2];
};
struct ProfObd {
  uint32_t tx, rx;
  uint16_t pid, everyS;
  uint8_t obdPid, buses[2];
};
struct Profile {
  char ver[17];
  char code[16];
  bool motorObd, motorCan;
  uint32_t motorId;
  uint8_t motorNb, motorBytes[4];
  float motorMult;
  uint16_t udsEveryS;
  uint8_t nCan, nUds, nObd;
  ProfCan can[PROF_CAN_MAX];
  ProfUds uds[PROF_UDS_MAX];
  ProfObd obd[PROF_OBD_MAX];
};

void profileBegin();                                  // setup(), after the file system (telemetryEarly)
const Profile* profileGet();                          // the profile in use this boot, or nullptr
const char* profileStoredVer();                       // the stored version ("" none) - asked for by ota.cpp
// ota.cpp: the body of profile.txt -> stored when its version differs; *changed = a new one was stored
bool profileStore(const String& body, bool* changed, char* summary, int cap);

// CAN1 task (cardecode.cpp): one frame of bus 1 against the profile's rules (and the engine rule)
void profileCanFrame(uint8_t bus, uint32_t id, uint8_t dlc, const uint8_t* d);
// record (telemetry.cpp): the latest value of rule i; false = never seen
struct ProfVal { uint32_t val, at, chg; uint8_t raw[8], rawN; };
bool profileCanValue(uint8_t i, ProfVal* v);
