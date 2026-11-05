// SIM7600G - see cellmodem.h (sources, NAVRH 9 PPP)
#include "cellmodem.h"
#include "max7317.h"
#include "hw.h"
#include <sys/time.h>
#include <time.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <lwip/netif.h>
#include <lwip/dns.h>
#include <lwip/ip_addr.h>
#include <netif/ppp/pppapi.h>
#include <netif/ppp/pppos.h>

static HardwareSerial s_uart(1);
static void muxReaderTask(void* p) { ((CellLink*)p)->muxLoop(); }
static struct netif s_pppNetif;              // the PPP interface in lwIP (one, reused for every dial)
static WiFiUDP s_udp;                        // NetworkUDP: lwIP socket, not tied to WiFi (Arduino 3 WiFiUdp.h:3)
static IPAddress s_srvIp;
static uint16_t s_srvPort;
static bool s_udpBegun;

void CellLink::lock() { if (m_mux) xSemaphoreTakeRecursive(m_mux, portMAX_DELAY); }
void CellLink::unlock() { if (m_mux) xSemaphoreGiveRecursive(m_mux); }

void CellLink::begin()
{
  if (m_mux) return;
  m_mux = xSemaphoreCreateRecursiveMutex();
  m_txLock = xSemaphoreCreateMutex();
  m_rx4 = xStreamBufferCreate(4096, 1);
  xTaskCreatePinnedToCore(muxReaderTask, "cmux", 4096, this, 6, &m_rdTask, 1);
  s_uart.setRxBufferSize(8192);
  // NAVRH 9: lwIP writes the PPP frames from its tcpip task (pppOutput) - with a TX buffer the UART write
  // returns at once instead of blocking that task for the whole frame (1500 B at 115200 = ~130 ms)
  s_uart.setTxBufferSize(4096);
  s_uart.begin(115200, SERIAL_8N1, HW_MODEM_RX, HW_MODEM_TX);
  mx7317Out(MX_MODEM_PWR, 0);          // key released (OVMS PwrKeyHigh = 0)
}

void CellLink::setApn(const char* apn)
{
  strncpy(m_apn, apn ? apn : "", sizeof(m_apn) - 1);
}

// read until a or b appears (or timeout); m_buf holds what came. The answer of a command on port 4;
// some answers come as URCs on port 3 (+CPIN, +CMTI ...) - those are in m_urc
bool CellLink::waitFor(const char* a, const char* b, unsigned int timeoutMs)
{
  uint32_t t = millis();
  do {
    while (linkAvail() && m_len < (int)sizeof(m_buf) - 1) {
      int c = linkRead();
      if (c < 0) break;
      m_buf[m_len++] = (char)c;
      m_buf[m_len] = 0;
    }
    if (a && strstr(m_buf, a)) return true;
    if (b && strstr(m_buf, b)) return false;
    if (m_muxOn && a) {
      bool hitA, hitB;
      portENTER_CRITICAL(&m_spin);
      hitA = strstr(m_urc, a) != nullptr;
      hitB = b && strstr(m_urc, b) != nullptr && strcmp(b, "\r\nERROR") != 0;
      portEXIT_CRITICAL(&m_spin);
      if (hitA && strcmp(a, "\r\nOK") != 0) return true;
      if (hitB) return false;
    }
    delay(2);
  } while (millis() - t < timeoutMs);
  return false;
}

bool CellLink::at(const char* cmd, unsigned int timeoutMs, const char* expect)
{
  lock();
  linkFlush();                          // stale bytes are not this command's answer (port 4: no data ever)
  m_len = 0;
  m_buf[0] = 0;
  portENTER_CRITICAL(&m_spin);
  m_urcLen = 0;
  m_urc[0] = 0;
  portEXIT_CRITICAL(&m_spin);
  if (cmd) linkWrite((const uint8_t*)cmd, strlen(cmd));
  bool ok = waitFor(expect ? expect : "\r\nOK", "\r\nERROR", timeoutMs);
  unlock();
  return ok;
}

bool CellLink::alive()
{
  for (int i = 0; i < 3; i++) if (at("AT\r", 300)) return true;
  return false;
}

void CellLink::pressKey(int ms)
{
  mx7317Out(MX_MODEM_PWR, 1);          // key pressed (inverted by the transistor)
  delay(ms);
  mx7317Out(MX_MODEM_PWR, 0);
}

bool CellLink::start()
{
  begin();
  m_pppUp = false;
  m_pppLost = false;
  m_pppErr = 0;
  m_ipStr[0] = 0;
  snprintf(m_status, sizeof(m_status), "power");
  m_startStep = 0;
  m_startMs = 0;
  if (m_muxOn) muxStop();              // a restart: the old CMUX session is gone with the power cycle
  if (!alive()) {
    // p79 (28.10.2026), times from SIM7600G Hardware Design V1.00 tables 8/9: PWRKEY >= 2.5 s powers a
    // running modem off - fully off (STATUS low) only after 25-26 s; 100-500 ms powers an off modem on,
    // UART ready after 13-14 s. The state at boot is unknown (after an OTA / crash restart it still runs,
    // in CMUX - p72's 500 ms press came during the power-off and was lost, 27.10 19:34/20:00/20:15).
    // 1) 3000 ms: an off modem comes on (answers within 30 s), a running one starts to power off;
    // 2) no answer = it was running and is off after those 30 s: 500 ms, answer within 25 s.
    uint32_t t0 = millis();
    while (s_uart.available()) s_uart.read();
    pressKey(3000);
    uint32_t t = millis();
    while (!alive() && millis() - t < 30000) delay(500);
    m_startStep = 1;
    if (!alive()) {
      while (s_uart.available()) s_uart.read();
      pressKey(500);
      t = millis();
      while (!alive() && millis() - t < 25000) delay(500);
      m_startStep = 2;
    }
    m_startMs = millis() - t0;
    if (!alive()) { snprintf(m_status, sizeof(m_status), "no answer after power key"); return false; }
  }
  at("ATE0\r");
  // NAVRH 9: CMUX or nothing - the old single-UART path (AT sockets, CCH) is gone. A modem that does
  // not take CMUX is switched off; cellTask starts it again (as OVMS: power cycle and retry).
  if (!muxStart()) {
    Serial.println("[CELL] CMUX failed - power off");
    muxStop();
    pressKey(3000);
    snprintf(m_status, sizeof(m_status), "CMUX failed");
    return false;
  }
  startGnss();
  m_alive = true;
  at("ATI\r", 1000);
  if (!strstr(m_buf, "7600")) Serial.printf("[CELL] ATI: %s\n", m_buf);   // not the expected model: logged, continue
  bool sim = false;
  for (int i = 0; i < 20 && !(sim = at("AT+CPIN?\r", 500, ": READY")); i++) delay(500);
  if (!sim) { snprintf(m_status, sizeof(m_status), "SIM not ready"); return false; }
  snprintf(m_status, sizeof(m_status), "registering");
  bool reg = false;
  for (uint32_t t = millis(); !reg && millis() - t < 90000; delay(1000)) reg = registered();
  if (!reg) { snprintf(m_status, sizeof(m_status), "not registered"); return false; }
  // what the modem itself has stored as APN (its own profile) - logged, not changed
  if (at("AT+CGDCONT?\r", 1000)) {
    char* p = strstr(m_buf, "+CGDCONT:");
    char* e = p ? strpbrk(p, "\r\n") : nullptr;
    if (p && e) { *e = 0; snprintf(m_cgdcont, sizeof(m_cgdcont), "%s", p + 9); }
    Serial.printf("[CELL] stored APN profile: %s\n", m_cgdcont);
  }
  // "PPP failed" = registered, but no data session: telemetry.cpp counts it for a trial APN (nbapn.h),
  // as "NETOPEN failed" before NAVRH 9
  if (!pppDial(30000)) { snprintf(m_status, sizeof(m_status), "PPP failed err=%d", m_pppErr); return false; }
  snprintf(m_status, sizeof(m_status), "online ip=%s", m_ipStr);
  Serial.printf("[CELL] online, PPP ip %s\n", m_ipStr);
  return true;
}

void CellLink::stop()
{
  pppClose();
  close();
  if (m_muxOn) muxTxStr(1, "AT+CGPS=0\r");
  bool off = at("AT+CPOF\r", 3000);
  muxStop();
  if (!off) pressKey(3000);            // v1F end(): AT+CPOF, else the long key press (3000 ms as OVMS, K11)
  m_alive = false;
  snprintf(m_status, sizeof(m_status), "off");
}

void CellLink::powerOff()
{
  if (!m_mux) return;
  lock();
  stop();
  unlock();
}

// ------------------------------------------------------------------ PPP (NAVRH 9) ---
// As OVMS3 gsmpppos.cpp / ovms_cellular.cpp: the dial on the data port, then lwIP's PPPoS client.

static u32_t pppOutCb(ppp_pcb*, const void* data, u32_t len, void* ctx)
{
  return ((CellLink*)ctx)->pppOutput((const uint8_t*)data, len);
}

static void pppStatusCb(ppp_pcb*, int err, void* ctx)
{
  ((CellLink*)ctx)->pppStatus(err);
}

// lwIP tcpip task: the PPP frame to the modem (OVMS GsmPPPOS_OutputCallback: mux->tx on the data channel)
uint32_t CellLink::pppOutput(const uint8_t* data, uint32_t len)
{
  if (!m_muxOn || !m_pppMode) return 0;
  muxTx(2, data, len);
  return len;
}

// lwIP tcpip task: PPPERR_NONE = we have an IP (OVMS GsmPPPOS_StatusCallback); only flags here - the status
// lines and the recovery run in cellTask (no SD / log calls from the tcpip task)
void CellLink::pppStatus(int err)
{
  m_pppErr = err;
  if (err == PPPERR_NONE) {
    if (ip_addr_isany(&s_pppNetif.ip_addr) || ip_addr_isany(&s_pppNetif.gw)) return;   // OVMS: ignored
    snprintf(m_ipStr, sizeof(m_ipStr), "%s", ipaddr_ntoa(&s_pppNetif.ip_addr));
    for (int i = 0; i < 2; i++) {      // usepeerdns: lwIP set the peer's DNS servers - kept for makeDefault()
      const ip_addr_t* d = dns_getserver(i);
      m_dns[i] = (d && IP_IS_V4(d)) ? ip_2_ip4(d)->addr : 0;
    }
    m_pppLost = false;
    m_pppUp = true;
  } else {
    bool was = m_pppUp;
    m_pppUp = false;
    if (was && err != PPPERR_USER) m_pppLost = true;   // not our own close
  }
  m_pppStatusSeen++;
}

bool CellLink::pppDial(uint32_t timeoutMs)
{
  lock();
  m_pppMode = false;
  m_dial = 0;
  m_lineLen[2] = 0;
  // OVMS NetStart (ovms_cellular.cpp:1008-1012): APN and the dial in one line on the data channel. Without our
  // APN (CELL_APN empty) the modem's stored profile 1 is used.
  char c[120];
  if (m_apn[0]) snprintf(c, sizeof(c), "AT+CGDCONT=1,\"IP\",\"%s\";+CGDATA=\"PPP\",1\r", m_apn);
  else snprintf(c, sizeof(c), "AT+CGDATA=\"PPP\",1\r");
  muxTxStr(2, c);
  uint32_t t = millis();
  while (m_dial == 0 && millis() - t < timeoutMs) delay(20);
  if (m_dial != 1) {
    unlock();
    Serial.printf("[CELL] PPP dial: %s\n", m_dial < 0 ? "refused" : "no CONNECT");
    return false;
  }
  m_pppMode = true;                    // from here every port 2 byte goes to lwIP
  ppp_pcb* p = (ppp_pcb*)m_ppp;
  if (!p) {
    p = pppapi_pppos_create(&s_pppNetif, pppOutCb, pppStatusCb, this);
    m_ppp = p;
  }
  if (!p) { m_pppMode = false; unlock(); Serial.println("[CELL] pppos_create failed"); return false; }
  ppp_set_auth(p, PPPAUTHTYPE_PAP, "", "");   // OVMS: PAP with the configured user/password (empty here)
  ppp_set_usepeerdns(p, 1);                   // up to 2 DNS servers from the network (OVMS gsmpppos.cpp:244)
  int seen = m_pppStatusSeen;
  m_pppErr = 0;
  pppapi_connect(p, 0);
  t = millis();
  while (!m_pppUp && m_pppStatusSeen == seen && millis() - t < timeoutMs) delay(20);
  bool ok = m_pppUp;
  unlock();
  if (!ok) pppClose();
  return ok;
}

// lwIP closes the PPP session (LCP terminate), the modem leaves the data mode (NO CARRIER on port 2)
void CellLink::pppClose()
{
  ppp_pcb* p = (ppp_pcb*)m_ppp;
  if (p && m_pppMode) {
    int seen = m_pppStatusSeen;
    pppapi_close(p, 0);
    for (uint32_t t = millis(); m_pppStatusSeen == seen && millis() - t < 5000;) delay(20);
    if (m_pppStatusSeen == seen) {     // no answer from the peer: close without the carrier
      seen = m_pppStatusSeen;
      pppapi_close(p, 1);
      for (uint32_t t = millis(); m_pppStatusSeen == seen && millis() - t < 2000;) delay(20);
    }
  }
  m_pppMode = false;
  m_pppUp = false;
}

// OVMS NetLoss (ovms_cellular.cpp:721-751) -> NetWait (registration) -> NetStart (dial)
bool CellLink::netRecover()
{
  bool had = m_pppMode;
  pppClose();
  lock();
  m_pppLost = false;
  if (had) at("AT+CGATT=0\r", 10000);  // OVMS: detach the PDP only when PPP had been connected
  bool reg = false;
  for (uint32_t t = millis(); !reg && millis() - t < 30000; delay(1000)) reg = registered();
  bool ok = reg && pppDial(30000);
  if (ok) snprintf(m_status, sizeof(m_status), "online ip=%s", m_ipStr);
  else snprintf(m_status, sizeof(m_status), reg ? "PPP failed err=%d" : "not registered", m_pppErr);
  unlock();
  return ok;
}

// netTask chose the modem: lwIP's default route and DNS back to PPP (WiFi's DHCP may have changed both)
void CellLink::makeDefault()
{
  ppp_pcb* p = (ppp_pcb*)m_ppp;
  if (!p || !m_pppUp) return;
  pppapi_set_default(p);
  for (int i = 0; i < 2; i++) {
    if (!m_dns[i]) continue;
    ip_addr_t a;
    IP_ADDR4(&a, 0, 0, 0, 0);
    ip_2_ip4(&a)->addr = m_dns[i];
    dns_setserver(i, &a);
  }
}

// ---------------------------------------------------------- UDP over PPP (NetLink) ---
// The same as WifiLink (netlink.cpp) - lwIP sends through the default route, which netTask sets to PPP
// (makeDefault) when it uses the modem

bool CellLink::open(const char* host, uint16_t port)
{
  if (!up()) return false;
  IPAddress ip;
  if (!WiFi.hostByName(host, ip)) return false;   // Arduino 3: NetworkManager::hostByName -> lwip_getaddrinfo
  s_srvIp = ip;
  s_srvPort = port;
  if (!s_udpBegun) s_udpBegun = s_udp.begin(0);
  return s_udpBegun;
}

bool CellLink::send(const char* data, unsigned int len)
{
  if (!up() || !s_udpBegun || !s_srvPort) return false;
  if (!s_udp.beginPacket(s_srvIp, s_srvPort)) return false;
  return s_udp.write((const uint8_t*)data, len) == len && s_udp.endPacket();
}

int CellLink::receive(char* buf, int cap, unsigned int timeoutMs)
{
  if (!s_udpBegun) return 0;
  uint32_t t = millis();
  do {
    int n = s_udp.parsePacket();
    if (n > 0) return s_udp.read(buf, cap);
    delay(1);
  } while (millis() - t < timeoutMs);
  return 0;
}

void CellLink::close()
{
  if (s_udpBegun) s_udp.stop();
  s_udpBegun = false;
}

// ---------------------------------------------------------------- AT on port 4 ---

bool CellLink::smsSend(const char* num, const char* text)
{
  if (!m_alive) return false;
  lock();
  bool ok = at("AT+CMGF=1\r") && at("AT+CSCS=\"IRA\"\r");
  if (ok) {
    linkFlush();
    m_len = 0;
    m_buf[0] = 0;
    portENTER_CRITICAL(&m_spin);
    m_urcLen = 0;
    m_urc[0] = 0;
    portEXIT_CRITICAL(&m_spin);
    char c[48];
    int n = snprintf(c, sizeof(c), "AT+CMGS=\"%s\"\r", num);
    linkWrite((const uint8_t*)c, n);
    ok = waitFor(">", "ERROR", 5000);           // the prompt, then the text and Ctrl-Z
    if (ok) {
      linkWrite((const uint8_t*)text, strlen(text));
      const uint8_t z = 0x1A;
      linkWrite(&z, 1);
      ok = waitFor("+CMGS:", "ERROR", 60000);   // up to 60 s on a weak network
    } else {
      const uint8_t esc = 0x1B;                 // no prompt: cancel
      linkWrite(&esc, 1);
    }
  }
  unlock();
  return ok;
}

// "00E100640061" (UCS2 hex) -> ASCII, Slovak letters without the accent, anything else '?'
static bool ucs2Hex(const char* s, char* out, int cap)
{
  int n = strlen(s);
  if (n < 4 || n % 4) return false;
  for (int i = 0; i < n; i++) if (!isxdigit((unsigned char)s[i])) return false;
  static const uint16_t from[] = {0xE1, 0xE4, 0x10D, 0x10F, 0xE9, 0xED, 0x13A, 0x13E, 0x148, 0xF3, 0xF4, 0x155, 0x161,
                                  0x165, 0xFA, 0xFD, 0x17E, 0xC1, 0x10C, 0x10E, 0xC9, 0xCD, 0x13D, 0x147, 0xD3, 0x160,
                                  0x164, 0xDA, 0xDD, 0x17D};
  static const char to[] = "aacdeillnoorstuyzACDEILNOSTUYZ";
  int o = 0;
  for (int i = 0; i < n && o < cap - 1; i += 4) {
    char h[5] = {s[i], s[i + 1], s[i + 2], s[i + 3], 0};
    uint16_t u = (uint16_t)strtoul(h, nullptr, 16);
    char ch = '?';
    if (u >= 32 && u < 127) ch = (char)u;
    else if (u == '\n' || u == '\r') ch = ' ';
    else for (unsigned k = 0; k < sizeof(from) / sizeof(from[0]); k++) if (from[k] == u) { ch = to[k]; break; }
    out[o++] = ch;
  }
  out[o] = 0;
  return true;
}

bool CellLink::smsTake(char* from, int fcap, char* text, int tcap)
{
  if (!m_alive) return false;
  from[0] = text[0] = 0;
  lock();
  bool got = false;
  if (at("AT+CMGF=1\r") && at("AT+CSCS=\"IRA\"\r") && at("AT+CMGL=\"ALL\"\r", 5000)) {
    // +CMGL: <index>,"<stat>","<from>",,"<time>"\r\n<text>\r\n ... OK
    char* p = strstr(m_buf, "+CMGL: ");
    if (p) {
      int idx = atoi(p + 7);
      char* q = strchr(p, ',');                 // ,"REC UNREAD","+421...",
      q = q ? strchr(q + 1, ',') : nullptr;
      if (q && q[1] == '"') {
        char* e = strchr(q + 2, '"');
        if (e) snprintf(from, fcap, "%.*s", (int)(e - q - 2), q + 2);
      }
      char* b = strstr(p, "\r\n");
      if (b) {
        b += 2;
        char* e = strstr(b, "\r\n");
        char body[200];
        snprintf(body, sizeof(body), "%.*s", e ? (int)(e - b) : (int)strlen(b), b);
        if (!ucs2Hex(body, text, tcap)) {
          int o = 0;
          for (char* s = body; *s && o < tcap - 1; s++) text[o++] = (*s >= 32 && *s < 127) ? *s : '?';
          text[o] = 0;
        }
      }
      char c[24];
      snprintf(c, sizeof(c), "AT+CMGD=%d\r", idx);
      at(c, 5000);                              // deleted: logged once only
      got = true;
    }
  }
  unlock();
  return got;
}

void CellLink::netInfo(char* out, int cap)
{
  char op[24] = "?";
  int st = -1;
  lock();
  if (at("AT+COPS?\r", 3000, "+COPS:")) {      // +COPS: 0,0,"Plus",7
    char* p = strchr(strstr(m_buf, "+COPS:"), '"');
    char* e = p ? strchr(p + 1, '"') : nullptr;
    if (p && e) snprintf(op, sizeof(op), "%.*s", (int)(e - p - 1), p + 1);
  }
  if (at("AT+CREG?\r", 1000, "+CREG: 0,")) st = strstr(m_buf, "+CREG: 0,")[9] - '0';
  unlock();
  snprintf(out, cap, "%s %s", op, st == 1 ? "home" : st == 5 ? "roam" : st >= 0 ? (st == 2 ? "search" : "noreg") : "?");
}

// registered home (1) or roaming (5) - AT+CREG? as before, AT+CEREG? (LTE) as OVMS polls both
// (simcom_7600.cpp:127); 6 = SMS only (home) kept from before
bool CellLink::registered()
{
  lock();
  bool r = false;
  if (at("AT+CREG?\r", 1000, "+CREG: 0,")) {
    char* p = strstr(m_buf, "+CREG: 0,");
    r = p && (p[9] == '1' || p[9] == '5' || p[9] == '6');
  }
  if (!r && at("AT+CEREG?\r", 1000, "+CEREG: ")) {
    char* p = strstr(m_buf, "+CEREG: ");
    char* q = p ? strchr(p, ',') : nullptr;
    r = q && (q[1] == '1' || q[1] == '5');
  }
  unlock();
  return r;
}

int CellLink::rssi()
{
  if (!at("AT+CSQ\r")) return 0;
  char* p = strstr(m_buf, "+CSQ:");
  if (!p) return 0;
  int csq = atoi(p + 5);
  return csq == 99 ? 0 : csq * 2 - 113;
}

// ddmm.mmmm -> degrees
static float degOf(const char* s)
{
  double v = atof(s);
  int d = (int)(v / 100);
  return d + (v - d * 100) / 60.0;
}

// one AT query, the text after "<tag>" up to the line end into line (under the lock)
bool CellLink::gnssLine(const char* cmd, const char* tag, char* line, int cap)
{
  lock();
  bool got = at(cmd, 1000, tag);
  if (got) waitFor("\r\nOK", nullptr, 300);
  char* p = got ? strstr(m_buf, tag) : nullptr;
  if (!p) { unlock(); return false; }
  p += strlen(tag);
  while (*p == ' ') p++;
  int i = 0;
  while (*p && *p != '\r' && *p != '\n' && i < cap - 1) line[i++] = *p++;
  line[i] = 0;
  unlock();
  return true;
}

// splits at commas; an empty field stays empty
static int fields(char* s, char** tok, int max)
{
  int nt = 0;
  tok[nt++] = s;
  for (; *s && nt < max; s++) if (*s == ',') { *s = 0; tok[nt++] = s + 1; }
  return nt;
}

int CellLink::gnssPushed(GnssFix* f)
{
  memset(f, 0, sizeof(*f));
  if (!m_mux || !m_alive) return 0;
  if (!m_muxOn) return -1;
  // p69: the modem pushes RMC + GNS on port 1 every second
  uint32_t now = millis();
  if (m_nmeaAt && now - m_nmeaAt < 10000) {
    portENTER_CRITICAL(&m_spin);
    GnssFix c = m_nfix;
    uint32_t r = m_rmcAt, g = m_gnsAt;
    portEXIT_CRITICAL(&m_spin);
    if (!r || now - r > 3000 || !c.valid) return 0;
    if (!g || now - g > 3000) { c.sats = 0; c.hdop = 0; }
    *f = c;
    return 1;
  }
  if (m_gpsAt && now - m_gpsAt < 20000) return 0;   // NMEA not started yet
  return -1;                           // no NMEA on port 1 for 10 s: only the AT query (port 4) can read it
}

bool CellLink::gnss(GnssFix* f)
{
  int p = gnssPushed(f);
  if (p >= 0) return p == 1;
  char line[200];
  char* tok[24];
  // CGNSSINFO first (has the satellite count); SIM7600G V2.0.2 in the Peugeot (21.10.2026)
  // answers it empty while CGPSINFO has the fix -> then CGPSINFO
  if (gnssLine("AT+CGNSSINFO\r", "+CGNSSINFO:", line, sizeof(line))) {
    int nt = fields(line, tok, 24);
    int ns = -1;                       // index of the N/S letter
    for (int k = 0; k + 1 < nt; k++) if ((!strcmp(tok[k], "N") || !strcmp(tok[k], "S")) && k >= 2) { ns = k; break; }
    if (ns >= 0 && ns + 9 < nt && tok[ns - 1][0]) {
      int sats = 0;
      for (int k = 1; k < ns - 1; k++) sats += atoi(tok[k]);      // GPS + GLONASS + BEIDOU (+ GALILEO)
      f->lat = degOf(tok[ns - 1]) * (tok[ns][0] == 'S' ? -1 : 1);
      f->lng = degOf(tok[ns + 1]) * (tok[ns + 2][0] == 'W' ? -1 : 1);
      f->date = (uint32_t)atol(tok[ns + 3]);                        // DDMMYY
      f->time = (uint32_t)(atof(tok[ns + 4]) * 100);                // HHMMSS.s -> HHMMSSss
      f->alt = atof(tok[ns + 5]);
      f->kph = atof(tok[ns + 6]) * 1.852f;                          // knots
      f->heading = (uint16_t)atof(tok[ns + 7]);
      f->hdop = ns + 9 < nt ? atof(tok[ns + 9]) : 0;                // PDOP, HDOP, VDOP
      f->sats = sats;
      f->valid = true;
      f->at = millis();
      return true;
    }
  }
  // +CGPSINFO: lat,N/S,lon,E/W,DDMMYY,HHMMSS.s,alt,speed(knots),course - no satellite count
  // (sats 0 = not sent; FleetNest keeps points without a count)
  if (!gnssLine("AT+CGPSINFO\r", "+CGPSINFO:", line, sizeof(line))) return false;
  int nt = fields(line, tok, 24);
  if (nt < 9 || !tok[0][0] || !tok[2][0]) return false;            // no fix (fields empty)
  f->lat = degOf(tok[0]) * (tok[1][0] == 'S' ? -1 : 1);
  f->lng = degOf(tok[2]) * (tok[3][0] == 'W' ? -1 : 1);
  f->date = (uint32_t)atol(tok[4]);
  f->time = (uint32_t)(atof(tok[5]) * 100);
  f->alt = atof(tok[6]);
  f->kph = atof(tok[7]) * 1.852f;                                   // knots (v1F: speed * 1.852)
  f->heading = (uint16_t)atof(tok[8]);
  f->valid = true;
  f->at = millis();
  return true;
}

int CellLink::raw(const char* cmd, unsigned int timeoutMs, char* out, int cap)
{
  if (!m_mux) return snprintf(out, cap, "modem not started (CELL=1 first)");
  lock();
  char c[128];
  snprintf(c, sizeof(c), "%s\r", cmd);
  at(c, timeoutMs);
  int n = snprintf(out, cap, "%s", m_buf);
  unlock();
  return n;
}

// =============================================================== p69 CMUX ===
// 3GPP TS 27.010 basic option, as OVMS3 components/ovms_cellular/src/gsmmux.cpp (frame layout,
// FCS table, SABM / UA / UIH, channel opening one after the other).
#define GSM_EA   0x01
#define GSM_CR   0x02
#define GSM_PF   0x10
#define GSM_SABM 0x2F
#define GSM_UA   0x63
#define GSM_DM   0x0F
#define GSM_UIH  0xEF
#define GSM_SOF  0xF9

static const uint8_t s_fcs8[256] = {
  0x00, 0x91, 0xE3, 0x72, 0x07, 0x96, 0xE4, 0x75, 0x0E, 0x9F, 0xED, 0x7C, 0x09, 0x98, 0xEA, 0x7B,
  0x1C, 0x8D, 0xFF, 0x6E, 0x1B, 0x8A, 0xF8, 0x69, 0x12, 0x83, 0xF1, 0x60, 0x15, 0x84, 0xF6, 0x67,
  0x38, 0xA9, 0xDB, 0x4A, 0x3F, 0xAE, 0xDC, 0x4D, 0x36, 0xA7, 0xD5, 0x44, 0x31, 0xA0, 0xD2, 0x43,
  0x24, 0xB5, 0xC7, 0x56, 0x23, 0xB2, 0xC0, 0x51, 0x2A, 0xBB, 0xC9, 0x58, 0x2D, 0xBC, 0xCE, 0x5F,
  0x70, 0xE1, 0x93, 0x02, 0x77, 0xE6, 0x94, 0x05, 0x7E, 0xEF, 0x9D, 0x0C, 0x79, 0xE8, 0x9A, 0x0B,
  0x6C, 0xFD, 0x8F, 0x1E, 0x6B, 0xFA, 0x88, 0x19, 0x62, 0xF3, 0x81, 0x10, 0x65, 0xF4, 0x86, 0x17,
  0x48, 0xD9, 0xAB, 0x3A, 0x4F, 0xDE, 0xAC, 0x3D, 0x46, 0xD7, 0xA5, 0x34, 0x41, 0xD0, 0xA2, 0x33,
  0x54, 0xC5, 0xB7, 0x26, 0x53, 0xC2, 0xB0, 0x21, 0x5A, 0xCB, 0xB9, 0x28, 0x5D, 0xCC, 0xBE, 0x2F,
  0xE0, 0x71, 0x03, 0x92, 0xE7, 0x76, 0x04, 0x95, 0xEE, 0x7F, 0x0D, 0x9C, 0xE9, 0x78, 0x0A, 0x9B,
  0xFC, 0x6D, 0x1F, 0x8E, 0xFB, 0x6A, 0x18, 0x89, 0xF2, 0x63, 0x11, 0x80, 0xF5, 0x64, 0x16, 0x87,
  0xD8, 0x49, 0x3B, 0xAA, 0xDF, 0x4E, 0x3C, 0xAD, 0xD6, 0x47, 0x35, 0xA4, 0xD1, 0x40, 0x32, 0xA3,
  0xC4, 0x55, 0x27, 0xB6, 0xC3, 0x52, 0x20, 0xB1, 0xCA, 0x5B, 0x29, 0xB8, 0xCD, 0x5C, 0x2E, 0xBF,
  0x90, 0x01, 0x73, 0xE2, 0x97, 0x06, 0x74, 0xE5, 0x9E, 0x0F, 0x7D, 0xEC, 0x99, 0x08, 0x7A, 0xEB,
  0x8C, 0x1D, 0x6F, 0xFE, 0x8B, 0x1A, 0x68, 0xF9, 0x82, 0x13, 0x61, 0xF0, 0x85, 0x14, 0x66, 0xF7,
  0xA8, 0x39, 0x4B, 0xDA, 0xAF, 0x3E, 0x4C, 0xDD, 0xA6, 0x37, 0x45, 0xD4, 0xA1, 0x30, 0x42, 0xD3,
  0xB4, 0x25, 0x57, 0xC6, 0xB3, 0x22, 0x50, 0xC1, 0xBA, 0x2B, 0x59, 0xC8, 0xBD, 0x2C, 0x5E, 0xCF};

static uint8_t fcsOf(const uint8_t* d, size_t n)
{
  uint8_t f = 0xFF;
  while (n--) f = s_fcs8[f ^ *d++];
  return 0xFF - f;
}

// the AT link: port 4 with CMUX, the UART before it (power on, AT+CMUX)
int CellLink::linkAvail()
{
  if (m_muxOn) return (int)xStreamBufferBytesAvailable(m_rx4);
  return s_uart.available();
}

int CellLink::linkRead()
{
  if (m_muxOn) {
    uint8_t b;
    return xStreamBufferReceive(m_rx4, &b, 1, 0) == 1 ? b : -1;
  }
  return s_uart.read();
}

void CellLink::linkWrite(const uint8_t* d, size_t n)
{
  if (m_muxOn) { muxTx(4, d, n); return; }
  xSemaphoreTake(m_txLock, portMAX_DELAY);
  s_uart.write(d, n);
  xSemaphoreGive(m_txLock);
}

void CellLink::linkFlush()
{
  while (linkAvail()) if (linkRead() < 0) break;
}

// UIH frames of at most 127 bytes (one length byte; the modem's N1 is not assumed)
void CellLink::muxTx(int ch, const uint8_t* d, size_t n)
{
  uint8_t f[134];
  xSemaphoreTake(m_txLock, portMAX_DELAY);
  do {
    size_t k = n > 127 ? 127 : n;
    f[0] = GSM_SOF;
    f[1] = (uint8_t)((ch << 2) | GSM_EA);
    f[2] = GSM_UIH | GSM_PF;
    f[3] = (uint8_t)((k << 1) | GSM_EA);
    memcpy(f + 4, d, k);
    f[4 + k] = fcsOf(f + 1, 3);
    f[5 + k] = GSM_SOF;
    s_uart.write(f, k + 6);
    d += k;
    n -= k;
  } while (n > 0);
  xSemaphoreGive(m_txLock);
}

bool CellLink::muxStart()
{
  if (!at("AT+CMUX=0;+CATR=6\r", 2000)) {      // OVMS simcom_7600.cpp:232
    Serial.printf("[CELL] AT+CMUX: %s\n", m_buf);
    return false;
  }
  delay(200);
  lock();
  while (s_uart.available()) s_uart.read();
  m_fpos = m_flen = m_fipos = 0;
  m_fmore = false;
  m_pppMode = false;
  for (int c = 0; c < 5; c++) { m_chOpen[c] = 0; m_lineLen[c] = 0; }
  xStreamBufferReset(m_rx4);
  m_lastFrame = millis();
  m_muxOn = true;                                // the reader task takes the UART from here
  bool ok = true;
  for (int c = 0; c <= 4 && ok; c++) {           // channel 0 (control) first, then 1..4 (OVMS)
    for (int tries = 0; tries < 3 && !m_chOpen[c]; tries++) {
      uint8_t f[6] = {GSM_SOF, (uint8_t)((c << 2) | GSM_EA | GSM_CR), GSM_SABM | GSM_PF, GSM_EA, 0, GSM_SOF};
      f[4] = fcsOf(f + 1, 3);
      xSemaphoreTake(m_txLock, portMAX_DELAY);
      s_uart.write(f, 6);
      xSemaphoreGive(m_txLock);
      for (uint32_t t = millis(); !m_chOpen[c] && millis() - t < 1000;) delay(10);
    }
    ok = m_chOpen[c];
  }
  if (!ok) { unlock(); Serial.println("[CELL] CMUX channel did not open"); return false; }
  at("ATE0\r", 1000);                           // port 4: our AT link
  muxTxStr(2, "ATE0\r");                        // port 2: no echo of the dial line
  unlock();
  Serial.println("[CELL] CMUX up");
  return true;
}

void CellLink::muxStop()
{
  if (!m_muxOn) return;
  m_pppMode = false;
  m_pppUp = false;
  m_muxOn = false;
  delay(30);
  while (s_uart.available()) s_uart.read();
  xStreamBufferReset(m_rx4);
  for (int c = 0; c < 5; c++) m_chOpen[c] = 0;
}

// GNSS on port 1 (OVMS simcom_7600.cpp:86-95: CGPS=0 first, the SIM7600 may already run it), RMC + GNS
// every second; p66 start mode (N / H / C) kept.
void CellLink::startGnss()
{
  const char* go = m_gpsMode == 'H' ? "AT+CGPSHOT\r" : m_gpsMode == 'C' ? "AT+CGPSCOLD\r" : "AT+CGPS=1,1\r";
  m_nmeaAt = m_rmcAt = m_gnsAt = 0;
  muxTxStr(1, "AT+CGPS=0\r");
  delay(2000);
  muxTxStr(1, "AT+CGPSNMEA=258\r");
  delay(50);
  muxTxStr(1, "AT+CGPSINFOCFG=1,258\r");      // every 1 s (manual V3.00 17.2.16: 0-255 s)
  delay(50);
  m_gpsCmdResult = -1;
  muxTxStr(1, go);
  for (uint32_t t = millis(); m_gpsCmdResult < 0 && millis() - t < 2000;) delay(10);
  m_gpsOk = m_gpsCmdResult == 1;
  m_gpsAt = millis();
}

void CellLink::muxLoop()
{
  for (;;) {
    if (!m_muxOn) { vTaskDelay(pdMS_TO_TICKS(10)); continue; }
    int n = s_uart.available();
    if (n <= 0) { vTaskDelay(pdMS_TO_TICKS(2)); continue; }
    while (n-- > 0 && m_muxOn) muxByte((uint8_t)s_uart.read());
  }
}

void CellLink::muxByte(uint8_t b)
{
  if (m_fpos >= (int)sizeof(m_frame)) { m_fpos = 0; m_frameErr++; }
  if (m_fpos == 0 && b != GSM_SOF) return;               // to the start of a frame
  if (m_fpos == 1 && b == GSM_SOF) return;               // the end flag of the previous frame
  m_frame[m_fpos++] = b;
  if (m_fpos == 4) {
    m_fmore = !(b & GSM_EA);
    m_flen = b >> 1;
    if (!m_fmore) { m_flen += 6; m_fipos = 4; } else { m_flen += 7; m_fipos = 5; }
  }
  if (m_fpos == 5 && m_fmore) { m_flen += b << 7; m_fmore = false; }
  if (m_fpos >= 4 && m_fpos == m_flen) {
    if (b == GSM_SOF) muxFrame(); else m_frameErr++;
    m_fpos = 0;
  }
}

void CellLink::muxFrame()
{
  if (fcsOf(m_frame + 1, m_fipos - 1) != m_frame[m_flen - 2]) { m_frameErr++; return; }
  int ch = m_frame[1] >> 2;
  uint8_t ctl = m_frame[2] & ~GSM_PF;
  m_lastFrame = millis();
  m_rxFrames++;
  if (ch > 4) return;
  if (ctl == GSM_UA) { m_chOpen[ch] = 1; return; }
  if (ctl == GSM_DM) { m_chOpen[ch] = 0; return; }
  if (ctl != GSM_UIH) return;
  const uint8_t* d = m_frame + m_fipos;
  int n = m_flen - m_fipos - 2;
  if (ch == 4) {                                   // AT answers
    if (xStreamBufferSend(m_rx4, d, n, 0) != (size_t)n) m_rx4Lost += n;
    return;
  }
  if (ch == 2 && m_pppMode) {
    // PPP: the bytes to lwIP (OVMS ovms_cellular.cpp:1876-1884 -> pppos_input_tcpip). A text frame
    // (not HDLC: no 0x7E) with NO CARRIER = the modem left the data mode
    if (n > 0 && d[0] != 0x7E && memmem(d, n, "NO CARRIER", 10)) {
      if (m_pppUp) m_pppLost = true;
      return;
    }
    if (m_ppp) pppos_input_tcpip((ppp_pcb*)m_ppp, d, n);
    return;
  }
  for (int i = 0; i < n; i++) {                    // ports 1, 3 and port 2 before CONNECT: text lines
    uint8_t c = d[i];
    if (c == '\n') {
      m_lines[ch][m_lineLen[ch]] = 0;
      if (m_lineLen[ch]) portLine(ch, m_lines[ch]);
      m_lineLen[ch] = 0;
    } else if (c != '\r' && m_lineLen[ch] < (int)sizeof(m_lines[0]) - 1) {
      m_lines[ch][m_lineLen[ch]++] = (char)c;
    }
  }
}

// one text line from port 1, 2 (the dial answer) or 3 (every URC, +CATR=6)
void CellLink::portLine(int ch, char* line)
{
  if (line[0] == '$') { nmeaLine(line); return; }
  if (ch == 1) {
    if (!strcmp(line, "OK")) m_gpsCmdResult = 1;
    else if (strstr(line, "ERROR")) m_gpsCmdResult = 0;
    return;
  }
  if (ch == 2) {                                   // OVMS ovms_cellular.cpp:1136-1149
    if (!strncmp(line, "CONNECT", 7)) m_dial = 1;
    else if (strstr(line, "ERROR") || strstr(line, "NO CARRIER")) m_dial = -1;
    return;
  }
  if (ch != 3) return;
  if (!strncmp(line, "+PPPD: DISCONNECTED", 19) && m_pppUp) m_pppLost = true;   // OVMS :1146-1149
  if (!strcmp(line, "OK") || !strcmp(line, "ERROR")) return;
  portENTER_CRITICAL(&m_spin);
  int k = strlen(line);
  if (m_urcLen + k + 4 >= (int)sizeof(m_urc)) m_urcLen = 0;
  m_urc[m_urcLen++] = '\r';
  m_urc[m_urcLen++] = '\n';
  memcpy(m_urc + m_urcLen, line, k);
  m_urcLen += k;
  m_urc[m_urcLen++] = '\r';
  m_urc[m_urcLen++] = '\n';
  m_urc[m_urcLen] = 0;
  portEXIT_CRITICAL(&m_spin);
}

// $..RMC / $..GNS (fields as OVMS gsmnmea.cpp:277-411), checksum checked; time from RMC when no NTP
void CellLink::nmeaLine(const char* line)
{
  const char* star = strrchr(line, '*');
  if (!star || star - line < 7) return;
  uint8_t ck = 0;
  for (const char* p = line + 1; p < star; p++) ck ^= (uint8_t)*p;
  if (strtoul(star + 1, nullptr, 16) != ck) return;
  m_nmeaAt = millis();
  char buf[120];
  int len = star - line < (int)sizeof(buf) - 1 ? star - line : (int)sizeof(buf) - 1;
  memcpy(buf, line, len);
  buf[len] = 0;
  char* tok[20];
  int nt = fields(buf, tok, 20);
  const char* type = tok[0] + 3;
  if (!strcmp(type, "RMC") && nt >= 10) {
    // $..RMC,<time>,<A|V>,<lat>,<N>,<lon>,<E>,<knots>,<course>,<DDMMYY>,...
    if (tok[2][0] != 'A' || !tok[3][0] || !tok[5][0]) return;
    GnssFix c = m_nfix;
    c.lat = degOf(tok[3]) * (tok[4][0] == 'S' ? -1 : 1);
    c.lng = degOf(tok[5]) * (tok[6][0] == 'W' ? -1 : 1);
    c.kph = atof(tok[7]) * 1.852f;
    c.heading = (uint16_t)atof(tok[8]);
    c.date = (uint32_t)atol(tok[9]);
    c.time = (uint32_t)(atof(tok[1]) * 100);
    c.valid = true;
    c.at = millis();
    portENTER_CRITICAL(&m_spin);
    m_nfix = c;
    m_rmcAt = c.at;
    portEXIT_CRITICAL(&m_spin);
    // K3: the system clock from GNSS (no WiFi = no NTP), at most once a minute, only when off > 2 s
    if (!m_timeSetAt || millis() - m_timeSetAt > 60000) {
      struct tm t = {};
      t.tm_mday = c.date / 10000; t.tm_mon = (c.date / 100) % 100 - 1; t.tm_year = c.date % 100 + 100;
      t.tm_hour = c.time / 1000000; t.tm_min = (c.time / 10000) % 100; t.tm_sec = (c.time / 100) % 100;
      time_t g = mktime(&t);                       // TZ is UTC (configTime(0, 0, ...) / default)
      time_t now = time(nullptr);
      if (g > 1735689600 && (now < 1735689600 || labs((long)(now - g)) > 2)) {
        struct timeval tv = {g, 0};
        settimeofday(&tv, nullptr);
      }
      m_timeSetAt = millis();
    }
  } else if (!strcmp(type, "GNS") && nt >= 10) {
    // $..GNS,<time>,<lat>,<N>,<lon>,<E>,<mode>,<sats>,<HDOP>,<alt>,...
    if (!tok[6][0] || (tok[6][0] == 'N' && (tok[6][1] == 'N' || !tok[6][1]))) return;
    portENTER_CRITICAL(&m_spin);
    m_nfix.sats = (uint8_t)atoi(tok[7]);
    m_nfix.hdop = atof(tok[8]);
    m_nfix.alt = atof(tok[9]);
    m_gnsAt = millis();
    portEXIT_CRITICAL(&m_spin);
  }
}

