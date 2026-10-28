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
//   p81 (28.10.2026, PSA-RE buses/AEE2010.full/HS.IS, all NOT VERIFIED against the car yet):
//   0x412 BSI_CABIN_DATA_4  byte 7 bits 7-3 DOORS_STATUS: 1 driver, 2 passenger, 4 rear left,
//                           8 rear right, 16 boot (the same five READ BSI DA96/DA94/DA92/DA90/DA98
//                           showed open one by one, 28.10)
//   0x488 ENGINE_PERFORMANCE_DATA_2 byte 1 - 40 = coolant C (0xFF invalid)
//   0x38D VEHICLE_DYNAMICS_WHEELS   bytes 1-2 x 0.01 = speed km/h (0xFFFF invalid)
//   plus values from the automatic reads (telemetry.cpp K0_READS): BSI D8C3 fuel x 0.01 L
//   (= 44.3 L vs 0x612 44.5 L and BSI D8C4 44.71 L, 28.10), CMM D558 urea volume measured (raw
//   byte, 0xB9 on 28.10 - scale unknown: the refill will show), CMM D517 byte 5 x 0.0625 L
// Each value is kept with the time it came; a record takes only fresh ones.
#pragma once
#include <Arduino.h>

struct CarValues {
  uint32_t odoKm;   uint32_t odoAt;
  float fuelL;      uint8_t fuelRaw;    uint32_t fuelAt;
  uint32_t ureaKm;  uint8_t urea[8];    uint32_t ureaAt;
  float rpm;        uint32_t rpmAt;     uint32_t rpmPosAt;   // rpmPosAt: last rpm > 0
  uint8_t engState; uint32_t engAt;
  uint8_t doors;    uint32_t doorsAt;                       // p81 0x412
  int16_t coolC;    uint32_t coolAt;                        // p81 0x488
  float speedKmh;   uint32_t speedAt;                       // p81 0x38D
  uint16_t fuelBsiCl; uint32_t fuelBsiAt;                   // p81 READ BSI D8C3, 0.01 L
  uint8_t ureaMeasRaw; uint32_t ureaMeasAt;                 // p81 READ CMM D558 raw
  uint16_t ureaEstMl; uint32_t ureaEstAt;                   // p81 READ CMM D517 byte 5 x 62.5 ml
  uint8_t crew;     uint32_t crewAt;                        // p83 READ BSI D997/D998/D99F: bit0 driver belt,
                                                            // bit1 passenger belt, bit2 passenger seated (verified 28.10)
  uint8_t speedDisp; uint32_t speedDispAt;                  // p83 READ BSI D8F9 speed as on the cluster, km/h (verified 50/70)
};

void carSetProfile(const char* p);    // "K0" or "" (none)
const char* carProfile();
CarValues carValues();
// p81: a positive answer of an automatic read (whole answer from 0x62) - decoded when known
void carFromRead(uint32_t txId, const uint8_t* resp, int len);
