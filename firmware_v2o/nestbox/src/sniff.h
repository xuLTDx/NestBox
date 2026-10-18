// Sniff mode - NestBox_v2O. The v1F sniff (firmware_v5/telelogger/sniff.cpp,
// verified in the Peugeot 2026-10-12) for the three buses of OVMS v3:
//   CAN1 = ESP32 TWAI (OBD 6/14 on our cable), CAN2 / CAN3 = MCP2515.
// All listen only - nothing is ever transmitted (v2O has no SNIFF_SEND).
// One file for all buses, /SNIFF/S<n>.LOG, one line per frame:
//     <ms>.<us>,<bus 1-3>,<ID hex>,<s|x>,<dlc>,<data hex>
// (v1F has no bus column - the "# format:" header line says which it is).
// "# stats" lines every minute count, per bus, the frames and every loss
// (controller missed/overrun, MCP2515 buffer overflow, bus errors), and the
// frames lost before SD; sdwriter.cpp writes and confirms every block.
// Each bus finds its speed itself (500, 250, 125 kbit/s).
#pragma once
#include <Arduino.h>

#define SNIFF_DEFAULT_MIN 120
#define SNIFF_MAX_MIN     1440
#define SNIFF_KEEP_DAYS   60
#define SNIFF_SD_LIMIT_PCT 70
#define SNIFF_BUSES 3

bool sniffInit();                        // after SD + buses; true = was on before the restart
void sniffRequest(bool on, uint16_t minutes);
#define SNIFF_EV_STARTED 1
#define SNIFF_EV_STOPPED 2
#define SNIFF_EV_REFUSED 3
int sniffTick();                         // loop(): requests, speed probe, auto-off
void sniffStopNow();
bool sniffActive();
uint16_t sniffMinutesLeft();
uint32_t sniffFileNo();
void sniffCloseForRestart();
int sniffPurgeOld(time_t now);
void sniffSetSdLimit(int pct);
// which buses take part (bit 0 = CAN1 ...); default all present
void sniffSetBuses(uint8_t mask);
uint8_t sniffBuses();

// READ command exchange on bus 1-3 (nbread.h); returns NbReadResult
struct NbRead;
int busExchange(const NbRead* r, uint8_t* resp, int cap, int* respLen, uint8_t* nrc);

// sdwriter side
int sniffFill(char* buf, int cap, uint32_t* frames, bool* last);
void sniffConfirmed(uint32_t frames);
void sniffNoteSdError(const char* op, int err);
void sniffSdDead(uint32_t lostFrames);

// web
int sniffStatus(char* buf, int size);
int sniffLive(char* buf, int size, uint32_t id, uint32_t mask);
bool sniffSetVehicle(const char* tag);

#ifdef TEST_SNIFF_LOAD
int sniffCheck(uint32_t fileNo, char* out, int cap);
#endif
