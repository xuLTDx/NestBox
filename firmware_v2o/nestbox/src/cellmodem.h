// SIM7600G modem of OVMS v3.3 - NestBox_v2O step 2b. NOT TESTED on the module
// yet: the v3.3 modem may not start on USB power (OVMS installation.rst), it
// needs the car's 12 V.
//
// Sources (read 2026-10-18):
//  - power key: OVMS3 components/simcom/src/simcom_powering.h - PWRKEY through
//    MAX7317 port 0, inverted by a transistor (port 1 = key pressed); SIM7600:
//    on = press >= 100 ms (OVMS uses 200), off = press >= 2500 ms; UART 115200
//    on GPIO 13 (RX) / 4 (TX), modem ready ~12 s after power-on (simcom_7600.cpp);
//  - AT sequence: NestBox_v1F libraries/FreematicsPlus/FreematicsNetwork.cpp,
//    the CELL_SIM7600 / shared SIMCom branches: CPSI? / CREG? registration,
//    CGSOCKCONT + CSOCKSETPN (APN), CIPMODE=0, NETOPEN, UDP with CIPRXGET=1
//    (buffer mode - the server's commands are not lost between polls, v1F
//    2026-09-23), CIPOPEN=0,"UDP",..., CIPSEND=0,<len>,"<ip>",<port>, CIPRXGET=2;
//  - GNSS: AT+CGPS=1,1 (v1F setGPS), position polled with AT+CGNSSINFO (v1F
//    used the CGPSINFO URC, which has no satellite count; FleetNest hides
//    points with < 4 satellites). The field list of CGNSSINFO differs between
//    firmware versions (an extra GALILEO count) - the parser finds the fields
//    by the N/S and E/W letters, not by position.
// One mutex around every AT exchange: the net task (UDP) and the record task
// (GNSS) share the UART.
#pragma once
#include <Arduino.h>
#include "netlink.h"

struct GnssFix {
  bool valid;
  float lat, lng, alt;         // degrees, m
  float kph;                   // km/h (modem gives knots)
  uint16_t heading;
  uint8_t sats;                // all systems
  float hdop;
  uint32_t date, time;         // DDMMYY, HHMMSSss (the protocol's PID 0x11 / 0x10 format)
  uint32_t at;                 // millis() of the reading
};

class CellLink : public NetLink {
public:
  const char* name() override { return "CELL"; }
  bool up() override { return m_netOpen; }
  bool open(const char* host, uint16_t port) override;
  bool send(const char* data, unsigned int len) override;
  int receive(char* buf, int cap, unsigned int timeoutMs) override;
  void close() override;

  void begin();                // UART + mutex; the modem task does the rest
  bool start();                // power on, SIM, registration, data - true = up (blocking, up to ~2 min)
  void stop();                 // network closed, modem powered off
  bool gnss(GnssFix* out);     // one AT+CGNSSINFO reading (modem on, network not needed)
  bool alive_() { return m_alive; }
  int rssi();                  // dBm, 0 = unknown
  const char* status() { return m_status; }
  const char* storedApn() { return m_cgdcont; }
  void setApn(const char* apn);
private:
  bool at(const char* cmd, unsigned int timeoutMs = 1000, const char* expect = nullptr);
  bool waitFor(const char* a, const char* b, unsigned int timeoutMs);
  bool alive();
  void pressKey(int ms);
  void lock();
  void unlock();
  char m_buf[1600];
  int m_len = 0;
  volatile bool m_netOpen = false;
  volatile bool m_alive = false;     // answers AT (GNSS can be read)
  bool m_sockOpen = false;
  char m_ip[40] = "";
  uint16_t m_port = 0;
  char m_apn[40] = "";
  char m_status[96] = "off";
  char m_cgdcont[80] = "";       // the modem's stored PDP profile (AT+CGDCONT?), for the log
  SemaphoreHandle_t m_mux = nullptr;
};
