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
//  - GNSS: AT+CGPS=1,1 (v1F setGPS), position polled with AT+CGNSSINFO, and
//    AT+CGPSINFO when that is empty (SIM7600G V2.0.2, Peugeot 21.10.2026) (v1F
//    used the CGPSINFO URC, which has no satellite count; FleetNest hides
//    points with < 4 satellites). The field list of CGNSSINFO differs between
//    firmware versions (an extra GALILEO count) - the parser finds the fields
//    by the N/S and E/W letters, not by position.
// One mutex around every AT exchange: the net task (UDP) and the record task
// (GNSS) share the UART.
//
// p69 (27.10.2026, NAVRH 8 K1-K3/K11, as OVMS3 components/ovms_cellular gsmmux.cpp + simcom_7600.cpp):
// after the power-on the modem is switched to CMUX (AT+CMUX=0;+CATR=6, 3GPP 27.010 basic mode, the
// frame code of OVMS gsmmux.cpp) - four virtual ports on the one UART: 1 = GNSS (NMEA pushed by the
// modem, AT+CGPSNMEA=258 + AT+CGPSINFOCFG=1,258 = RMC + GNS every second, manual V3.00 17.2.16),
// 2 = our AT data link (CIP UDP, CCH HTTPS, everything at() does), 3 = all URCs (+CATR=6, manual
// 3.2.11), 4 = spare. GNSS no longer costs an AT exchange on the data port; system time from RMC when
// there is no NTP. A CMUX that does not come up -> the modem is switched off and started the old way
// (one UART, GNSS polled) - never worse than before. No good frame for 180 s -> link down (OVMS).
#pragma once
#include <Arduino.h>
#include <freertos/stream_buffer.h>
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
  void powerOff();             // stop() under the lock (standby, from another task)
  bool gnss(GnssFix* out);     // one AT+CGNSSINFO reading, CGPSINFO when empty (modem on, network not needed)
  bool alive_() { return m_alive; }
  // p66: how the next start() switches GNSS on - 'N' AT+CGPS=1,1 (as before), 'H' AT+CGPSHOT, 'C' AT+CGPSCOLD
  // (SIM7500_SIM7600 GNSS Application Note V2.00 3.1: only with GNSS off - after power on it is)
  void setGpsStart(char mode) { m_gpsMode = mode; }
  char gpsMode() const { return m_gpsMode; }
  uint32_t gpsStartedAt() const { return m_gpsAt; }   // millis() of the GNSS start command, 0 = not yet
  bool gpsStartOk() const { return m_gpsOk; }
  int rssi();                  // dBm, 0 = unknown
  const char* status() { return m_status; }
  const char* storedApn() { return m_cgdcont; }
  // p69 CMUX
  void setMuxWanted(bool on) { m_muxWanted = on; }
  bool muxOn() const { return m_muxOn; }
  bool muxDead() const { return m_muxOn && millis() - m_lastFrame > 180000UL; }   // OVMS: 3 min
  uint32_t muxFrames() const { return m_rxFrames; }
  uint32_t muxErrors() const { return m_frameErr; }
  void muxLoop();              // the reader task (internal)
  void setApn(const char* apn);
  // diagnostics (local web API only): one AT command, the modem's raw answer into out
  int raw(const char* cmd, unsigned int timeoutMs, char* out, int cap);
  // HTTPS inside the modem (AT+CCH*, session 0, cache receive mode) - pull-OTA over cellular
  // (2026-10-20, from v1F CellHTTP; SIM7600 command set per the SIMCom SSL application note).
  // NOT TESTED on the module: the data link itself (NETOPEN) did not come up on USB power.
  bool tlsOpen(const char* host, uint16_t port);
  bool tlsSend(const char* data, int len);
  int tlsRead(uint8_t* buf, int cap, unsigned int timeoutMs);   // >0 bytes, 0 closed, -1 timeout
  void tlsClose();
  // HTTPS diagnostics (p42, 2026-10-26: OTA over cellular ended with META_HTTP0) - the modem's
  // AT+CCH* answers into the record (PID 0x399); set by ota.cpp, nullptr = none
  void (*diag)(const char*) = nullptr;
  // p71 (27.10.2026: no UDP records over cellular with CMUX, HTTPS fine) - the modem's AT+CIP*
  // answers (data port + port 3 URCs); set by ota.cpp, which sends them over HTTPS; nullptr = none
  void (*udpDiag)(const char*) = nullptr;
private:
  int m_udpOk = 0, m_udpErr = 0;
  void udpNote(const char* what, bool ok);
  // p69 CMUX state
  volatile bool m_muxOn = false;
  bool m_muxWanted = true;
  StreamBufferHandle_t m_rx2 = nullptr;        // port 2 (AT data link) bytes
  SemaphoreHandle_t m_txLock = nullptr;
  TaskHandle_t m_rdTask = nullptr;
  volatile uint8_t m_chOpen[5] = {0, 0, 0, 0, 0};
  volatile uint32_t m_lastFrame = 0, m_rxFrames = 0, m_frameErr = 0, m_rx2Lost = 0;
  uint8_t m_frame[2100];
  int m_fpos = 0, m_flen = 0, m_fipos = 0;
  bool m_fmore = false;
  char m_lines[5][256];
  int m_lineLen[5] = {0, 0, 0, 0, 0};
  int m_binLeft = 0;                           // raw bytes of a pushed +CCHRECV: DATA still to take
  char m_urc[1024];                            // port 3 lines (URCs) for waitFor
  int m_urcLen = 0;
  portMUX_TYPE m_spin = portMUX_INITIALIZER_UNLOCKED;
  volatile bool m_recvEvent = false, m_peerClosed = false, m_recvEnd = false;
  volatile int8_t m_gpsCmdResult = -1;         // port 1: 1 OK, 0 ERROR
  GnssFix m_nfix = {};
  volatile uint32_t m_rmcAt = 0, m_gnsAt = 0, m_nmeaAt = 0, m_timeSetAt = 0;
  int linkAvail();
  int linkRead();
  void linkWrite(const uint8_t* d, size_t n);
  void muxTx(int ch, const uint8_t* d, size_t n);
  void muxTxStr(int ch, const char* s) { muxTx(ch, (const uint8_t*)s, strlen(s)); }
  bool muxStart();
  void muxStop();
  void muxByte(uint8_t b);
  void muxFrame();
  void portLine(int ch, char* line);
  void nmeaLine(const char* line);
  void startGnss();
  int tlsReadMux(uint8_t* buf, int cap, unsigned int timeoutMs);
  int m_diagRecv = 0;
  int m_diagFlush = 0;                         // p75: "CCH FLUSHED n" lines sent
  bool m_moreData = false;   // the last AT+CCHRECV filled the buffer: read again without a new notice
  // data the modem pushed by itself (+CCHRECV: DATA,0,<n> without AT+CCHRECV - p43), not yet read
  uint8_t m_push[2048];
  int m_pushLen = 0, m_pushPos = 0;
  bool at(const char* cmd, unsigned int timeoutMs = 1000, const char* expect = nullptr);
  bool gnssLine(const char* cmd, const char* tag, char* line, int cap);
  bool waitFor(const char* a, const char* b, unsigned int timeoutMs);
  bool alive();
  void pressKey(int ms);
  void lock();
  void unlock();
  char m_buf[1600];
  int m_len = 0;
  volatile bool m_netOpen = false;
  volatile bool m_alive = false;     // answers AT (GNSS can be read)
  char m_gpsMode = 'N';
  uint32_t m_gpsAt = 0;
  bool m_gpsOk = false;
  bool m_sockOpen = false;
  char m_ip[40] = "";
  uint16_t m_port = 0;
  char m_apn[40] = "";
  char m_status[96] = "off";
  char m_cgdcont[80] = "";       // the modem's stored PDP profile (AT+CGDCONT?), for the log
  SemaphoreHandle_t m_mux = nullptr;
};
