// Car values from CAN1 broadcasts (listen only) - NestBox_v2O step A.
// Vehicle profile "K0" = PSA K0 platform (Peugeot Traveller, Opel Zafira Life,
// Toyota Proace Verso; github.com/prototux/PSA-RE cars/K0.yml, bus HS.IS on
// OBD 6/14 - verified: 75 of 76 IDs of S19.LOG 2026-10-12 are HS.IS):
//   0x552 BSI_ODOMETER_DATA  bytes 5-7 = km          VERIFIED (69722 = dashboard)
//   0x612 BSI_DATA_3         byte 4 x 0.5 = fuel L   NOT VERIFIED (S19: always 70.0 =
//                                                    the tank size - level or size?)
//   0x5A8 urea data          bytes 1-2 (15 bits) = AdBlue range before start
//                            inhibit, km             NOT VERIFIED (S19: 5000); the raw
//                            8 bytes are sent too (tank-volume field jumps 19/30/0)
//   0x208 ENGINE_SPEED       bytes 1-2 x 0.125 = rpm  (S19: 0, then ~900 idle after the start)
//   0x348 ENGINE_RUNNING     byte 6 bits 3-0: 1 off, 2 starting, 3 running, 4 stopped (stop-start),
//                            7 preparing (S19: 1, 2, 3, 7 - consistent with the engine start)
// Each value is kept with the time it came; a record takes only fresh ones.
#pragma once
#include <Arduino.h>

struct CarValues {
  uint32_t odoKm;   uint32_t odoAt;
  float fuelL;      uint8_t fuelRaw;    uint32_t fuelAt;
  uint32_t ureaKm;  uint8_t urea[8];    uint32_t ureaAt;
  float rpm;        uint32_t rpmAt;     uint32_t rpmPosAt;   // rpmPosAt: last rpm > 0
  uint8_t engState; uint32_t engAt;
};

void carSetProfile(const char* p);    // "K0" or "" (none)
const char* carProfile();
CarValues carValues();
