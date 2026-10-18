// CAN1 (ESP32 TWAI, OBD 6/14) - always listening, NestBox_v2O step A.
// Owns the controller (sniff.cpp no longer starts/stops it): listen only, the
// transceiver's transmitter off (MAX7317 port 2 = 1), speed probe 500 / 250 /
// 125 kbit/s until frames come. Every frame goes to the car decoder
// (cardecode.cpp) and, while the sniff runs, to the sniff queue.
#pragma once
#include <Arduino.h>

struct Can1Stats {
  bool running;
  uint32_t kbit;
  bool speedFound;
  uint32_t frames;             // since boot
  uint32_t lastFrameMs;        // millis() of the last frame (0 = none yet)
  uint32_t rxMissed, rxOverrun, busErr;
};

typedef void (*Can1Fn)(int64_t us, uint32_t id, bool ext, uint8_t dlc, const uint8_t* data);

void can1Begin();
void can1Tick();               // loop(): speed probe
Can1Stats can1Stats();
void can1SetSniffHook(Can1Fn fn);   // null = sniff off
