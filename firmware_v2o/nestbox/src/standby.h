// Standby (sleep) of NestBox_v2O - agreed 21.10.2026 (B), rules of NestBox_v1F:
//  - sleep: engine off (no rpm, 12 V under ENGINE_OFF_VOLTAGE) and CAN1 silent for 5 min (the car
//    locked and asleep - the lock and the doors are still in the sniff); never on the bench (< 7 V);
//  - before: an OTA download waits (max 3 min), the sniff file and the delivery queue are closed
//    (prepareRestart), the modem with its GNSS and WiFi off; a staged firmware was flashed already
//    (ota.cpp flashes at engine off, >= 11.5 V);
//  - in deep sleep the ULP co-processor measures the 12 V (ADC1 ch0, GPIO36) every 200 ms and wakes
//    the chip at >= ENGINE_OFF_VOLTAGE; the chip then checks again 5 s later (two readings, v1F) -
//    lower = asleep again at once (OVMS ovms_boot.cpp does the same with a timer instead of the ULP);
//  - every 3 h a parked report (v1F STANDBY_REPORT_INTERVAL): modem on, GNSS fix max 150 s, one
//    record, delivered (max 60 s more), asleep again.
#pragma once
#include <Arduino.h>

#define ENGINE_OFF_VOLTAGE     12.8f      // V, v1F config.h: at or above = engine running
#define STANDBY_QUIET_MS       300000UL   // CAN1 silent this long -> sleep (5 min, user 21.10.2026)
#define STANDBY_REPORT_S       10800      // parked report every 3 h
#define STANDBY_REPORT_GPS_S   150        // max wait for a fix in the parked report
#define STANDBY_REPORT_SEND_S  60         // max wait for the delivery after the record
#define STANDBY_OTA_WAIT_MS    180000UL   // a running OTA download delays the sleep at most this
#define STANDBY_ULP_PERIOD_US  200000     // ULP measures every 200 ms
#ifndef STANDBY_AUTO
#define STANDBY_AUTO 0                    // automatic sleep: off since 21.10.2026 (ULP did not wake) until tested
#endif

void standbyEarly();          // first in setup(): a deep-sleep wake - confirm or sleep again (no return)
bool standbyParkedBoot();     // this boot is the 3-hour parked report
const char* standbyWakeName();// why this boot: "-", "VOLTAGE", "REPORT", "TEST"
void standbyTick();           // loop()
void standbyTest(uint32_t seconds, bool ulp);   // /api/control SLEEP_TEST=<s> (timer) / SLEEP_ULP=<s> (ULP + timer): sleep in ~2 s
int standbyUlpRead(char* buf, int cap);         // /api/control ULP_READ
int standbyStatus(char* buf, int cap);
