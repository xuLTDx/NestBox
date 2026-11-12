// SIM7600G modem of OVMS v3.3 - NestBox_v2O.
//
// Sources (read 2026-10-18):
//  - power key: OVMS3 components/simcom/src/simcom_powering.h - PWRKEY through
//    MAX7317 port 0, inverted by a transistor (port 1 = key pressed); SIM7600:
//    on = press >= 100 ms (OVMS uses 200), off = press >= 2500 ms; UART 115200
//    on GPIO 13 (RX) / 4 (TX), modem ready ~12 s after power-on (simcom_7600.cpp);
//  - GNSS: AT+CGPS=1,1 (v1F setGPS); without NMEA the position is polled with AT+CGNSSINFO, and
//    AT+CGPSINFO when that is empty (SIM7600G V2.0.2, Peugeot 21.10.2026). The field list of CGNSSINFO
//    differs between firmware versions (an extra GALILEO count) - the parser finds the fields by the
//    N/S and E/W letters, not by position.
//
// p69 (27.10.2026, NAVRH 8 K1-K3/K11, as OVMS3 components/ovms_cellular gsmmux.cpp + simcom_7600.cpp):
// after the power-on the modem is switched to CMUX (AT+CMUX=0;+CATR=6, 3GPP 27.010 basic mode, the
// frame code of OVMS gsmmux.cpp) - four virtual ports on the one UART. System time from RMC when there
// is no NTP. No good frame for 180 s -> modem restart (OVMS).
//
// NAVRH 9 (PG42/PG43, 2026-11-05): the data = PPP on port 2, IP in the ESP32 (lwIP) as OVMS3
// (gsmpppos.cpp, ovms_cellular.cpp NetStart/NetMode/NetLoss). Why: until now UDP (AT+CIP*) and HTTPS
// (AT+CCH*) ran as AT commands on port 2 - 4.11.2026 boot 157 and boot 164 one 1024-byte block of the
// OTA download was lost each time (ERR=RECV 1024 short -> ERR=SHA256), the modem mixes the ports
// against its own manual (+CCHEVENT RECV EVENT came on port 2; SSL manual 2.3.2: an URC -> port 3 with
// +CATR=6), OVMS3 never uses CCH over CMUX. With PPP port 2 carries no AT text at all; TCP sends lost
// bytes again itself; UDP records keep the ACK queue. Ports (OVMS simcom_7600.h:47-51):
//   1 = GNSS: NMEA pushed by the modem (AT+CGPSNMEA=258, AT+CGPSINFOCFG=1,258) + the GNSS commands
//   2 = PPP only (AT+CGDCONT + AT+CGDATA="PPP",1 -> CONNECT -> lwIP pppos)
//   3 = URCs (+CATR=6, manual 4.11)
//   4 = every other AT command (registration, CSQ, SMS, AT= from the web)
// The modem never starts without CMUX any more (the old CIP/CCH path is gone - NAVRH 9: keeping it =
// a half solution again); a CMUX that does not come up = modem off, the next start() tries again.
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
  // NetLink: UDP through lwIP over PPP - the same code as WifiLink (netlink.cpp), only the interface differs
  const char* name() override { return "CELL"; }
  bool up() override { return m_pppUp; }
  bool open(const char* host, uint16_t port) override;
  bool send(const char* data, unsigned int len) override;
  int receive(char* buf, int cap, unsigned int timeoutMs) override;
  void close() override;

  void begin();                // UART, mutexes, the CMUX reader task; the modem task does the rest
  bool start();                // power on, CMUX, SIM, registration, PPP - true = IP (blocking, up to ~3 min)
  void stop();                 // PPP closed, modem powered off
  void powerOff();             // stop() under the lock (standby, from another task)
  bool gnss(GnssFix* out);     // NMEA fix, or one AT+CGNSSINFO / CGPSINFO reading (port 4) when NMEA is not there
  // PG-p19 (3.11.2026): the fix the modem pushes as NMEA (CMUX port 1) - memory only, never waits for the
  // modem. 1 = fix, 0 = NMEA running, no fix now, -1 = no NMEA (only the AT query of gnss() can read it).
  // why: the record task must never wait for the modem
  int gnssPushed(GnssFix* out);
  bool alive_() { return m_alive; }
  // p66: how the next start() switches GNSS on - 'N' AT+CGPS=1,1 (as before), 'H' AT+CGPSHOT, 'C' AT+CGPSCOLD
  // (SIM7500_SIM7600 GNSS Application Note V2.00 3.1: only with GNSS off - after power on it is)
  void setGpsStart(char mode) { m_gpsMode = mode; }
  char gpsMode() const { return m_gpsMode; }
  uint32_t gpsStartedAt() const { return m_gpsAt; }   // millis() of the GNSS start command, 0 = not yet
  bool gpsStartOk() const { return m_gpsOk; }
  bool nmeaSeen() const { return m_nmeaAt && millis() - m_nmeaAt < 10000; }   // PG151: NMEA on port 1 in the last 10 s
  int rssi();                  // dBm, 0 = unknown (AT+CSQ on port 4)
  const char* status() { return m_status; }
  const char* storedApn() { return m_cgdcont; }
  bool muxOn() const { return m_muxOn; }
  bool muxDead() const { return m_muxOn && millis() - m_lastFrame > 180000UL; }   // OVMS: 3 min
  uint32_t muxFrames() const { return m_rxFrames; }
  uint32_t muxErrors() const { return m_frameErr; }
  void muxLoop();              // the reader task (internal)
  void setApn(const char* apn);
  // diagnostics (local web API only): one AT command on port 4, the modem's raw answer into out
  int raw(const char* cmd, unsigned int timeoutMs, char* out, int cap);
  // p71 diagnostics sink (ota.cpp sends the lines over HTTPS to the OTA server log when UDP does not get through)
  void (*udpDiag)(const char*) = nullptr;
  // p79: how the last start() got an answer - 0 already on, 1 after 3000 ms, 2 after 3000 + 500 ms
  int startStep() const { return m_startStep; }
  uint32_t startMs() const { return m_startMs; }

  // NAVRH 9 - PPP state and recovery (OVMS ovms_cellular.cpp NetMode / NetLoss):
  //  - pppLost(): PPP had an IP and lost it (lwIP status != OK, +PPPD: DISCONNECTED or NO CARRIER on port 2)
  //    or the registration is gone; cellTask then calls netRecover()
  //  - netRecover(): as OVMS NetLoss -> NetWait -> NetStart: PPP closed, AT+CGATT=0 (port 4), registered?
  //    then the dial again; true = IP again. cellTask counts the failures: 3 in a row -> modem power cycle
  //    (OVMS s_netloss_retry_limit = 3, ovms_cellular.cpp:55, 721-751)
  bool pppLost() const { return m_pppLost; }
  int pppErr() const { return m_pppErr; }      // the last lwIP PPP status (PPPERR_*, 0 = none)
  bool netRecover();
  void makeDefault();          // PPP = lwIP's default route + its DNS servers (netTask: switch WiFi -> modem)
  const char* ip() const { return m_ipStr; }
  bool registered();           // AT+CREG? / AT+CEREG? home or roaming (port 4)
  // PG-p7 (2.11.2026, 4ka: roaming data on/off by SMS to 900): SMS in text mode (manual V3.00 4.2.x
  // AT+CMGF / AT+CMGS / AT+CMGL / AT+CMGD) - needs only the registration, not the data session
  bool smsSend(const char* num, const char* text);
  // the first stored SMS: sender and text (UCS2 hex decoded, ASCII only), then deleted; false = none
  bool smsTake(char* from, int fcap, char* text, int tcap);
  // "<operator> home|roam|reg=<n>" from AT+COPS? and AT+CREG? (PG-p8 STAV answer)
  void netInfo(char* out, int cap);

  // lwIP callbacks (internal, called from the tcpip task)
  uint32_t pppOutput(const uint8_t* data, uint32_t len);
  void pppStatus(int err);
private:
  int m_startStep = 0;
  uint32_t m_startMs = 0;
  // CMUX state
  volatile bool m_muxOn = false;
  StreamBufferHandle_t m_rx4 = nullptr;        // port 4 (AT commands) bytes
  SemaphoreHandle_t m_txLock = nullptr;
  TaskHandle_t m_rdTask = nullptr;
  volatile uint8_t m_chOpen[5] = {0, 0, 0, 0, 0};
  volatile uint32_t m_lastFrame = 0, m_rxFrames = 0, m_frameErr = 0, m_rx4Lost = 0;
  uint8_t m_frame[2100];
  int m_fpos = 0, m_flen = 0, m_fipos = 0;
  bool m_fmore = false;
  char m_lines[5][256];
  int m_lineLen[5] = {0, 0, 0, 0, 0};
  char m_urc[1024];                            // port 3 lines (URCs) for waitFor
  int m_urcLen = 0;
  portMUX_TYPE m_spin = portMUX_INITIALIZER_UNLOCKED;
  volatile int8_t m_gpsCmdResult = -1;         // port 1: 1 OK, 0 ERROR
  GnssFix m_nfix = {};
  volatile uint32_t m_rmcAt = 0, m_gnsAt = 0, m_nmeaAt = 0, m_timeSetAt = 0;
  // PPP (NAVRH 9)
  void* m_ppp = nullptr;                       // ppp_pcb* (lwIP), created once, reused for every dial
  volatile bool m_pppMode = false;             // port 2 bytes go to lwIP (after CONNECT) - else text lines
  volatile int8_t m_dial = 0;                  // port 2 answer to the dial: 0 waiting, 1 CONNECT, -1 refused
  volatile bool m_pppUp = false, m_pppLost = false;
  volatile int m_pppErr = 0, m_pppStatusSeen = 0;
  uint32_t m_dns[2] = {0, 0};                  // the PPP peer's DNS servers (ip4 addr, network order)
  char m_ipStr[20] = "";
  bool pppDial(uint32_t timeoutMs);            // port 2: dial, CONNECT, lwIP connect, wait for the IP
  void pppClose();
  int linkAvail();
  int linkRead();
  void linkWrite(const uint8_t* d, size_t n);
  void linkFlush();
  void muxTx(int ch, const uint8_t* d, size_t n);
  void muxTxStr(int ch, const char* s) { muxTx(ch, (const uint8_t*)s, strlen(s)); }
  bool muxStart();
  void muxStop();
  void muxByte(uint8_t b);
  void muxFrame();
  void portLine(int ch, char* line);
  void nmeaLine(const char* line);
  void startGnss();
  bool at(const char* cmd, unsigned int timeoutMs = 1000, const char* expect = nullptr);
  bool gnssLine(const char* cmd, const char* tag, char* line, int cap);
  bool waitFor(const char* a, const char* b, unsigned int timeoutMs);
  bool alive();
  void simInfoLine();                // PG68 a: status line "SIM ICCID=<n> NUM=<number>|-" (cellmodem.cpp)
  void pressKey(int ms);
  void lock();
  void unlock();
  char m_buf[1600];
  int m_len = 0;
  volatile bool m_alive = false;     // answers AT (GNSS can be read)
  char m_gpsMode = 'N';
  uint32_t m_gpsAt = 0;
  bool m_gpsOk = false;
  char m_apn[40] = "";
  char m_status[96] = "off";
  char m_cgdcont[80] = "";       // the modem's stored PDP profile (AT+CGDCONT?), for the log
  SemaphoreHandle_t m_mux = nullptr;
};
