// SIM7600G - see cellmodem.h (sources, untested state)
#include "cellmodem.h"
#include "max7317.h"
#include "hw.h"
#include <sys/time.h>
#include <time.h>

static HardwareSerial s_uart(1);
static void muxReaderTask(void* p) { ((CellLink*)p)->muxLoop(); }

void CellLink::lock() { if (m_mux) xSemaphoreTakeRecursive(m_mux, portMAX_DELAY); }
void CellLink::unlock() { if (m_mux) xSemaphoreGiveRecursive(m_mux); }

void CellLink::begin()
{
  if (m_mux) return;
  m_mux = xSemaphoreCreateRecursiveMutex();
  m_txLock = xSemaphoreCreateMutex();
  m_rx2 = xStreamBufferCreate(8192, 1);
  xTaskCreatePinnedToCore(muxReaderTask, "cmux", 4096, this, 6, &m_rdTask, 1);
  s_uart.setRxBufferSize(4096);
  s_uart.begin(115200, SERIAL_8N1, HW_MODEM_RX, HW_MODEM_TX);
  mx7317Out(MX_MODEM_PWR, 0);          // key released (OVMS PwrKeyHigh = 0)
}

void CellLink::setApn(const char* apn)
{
  strncpy(m_apn, apn ? apn : "", sizeof(m_apn) - 1);
}

// read until a or b appears (or timeout); m_buf holds what came
bool CellLink::waitFor(const char* a, const char* b, unsigned int timeoutMs)
{
  uint32_t t = millis();
  do {
    while (linkAvail() && m_len < (int)sizeof(m_buf) - 1) {
      m_buf[m_len++] = (char)linkRead();
      m_buf[m_len] = 0;
    }
    if (a && strstr(m_buf, a)) return true;
    if (b && strstr(m_buf, b)) return false;
    if (m_muxOn && a) {                // CMUX: the answer can come as a URC on port 3 (+NETOPEN:, +CCHOPEN:)
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
  while (linkAvail()) linkRead();       // stale bytes (URCs) are not this command's answer
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
  m_netOpen = false;
  m_sockOpen = false;
  snprintf(m_status, sizeof(m_status), "power");
  if (!alive()) {
    // p72 = OVMS PowerOffOn (ovms_cellular.cpp:396, simcom_7600.cpp:165): the modem's state is unknown -
    // after an ESP restart (OTA) it still runs, in CMUX, and answers no plain AT (27.10.2026 19:34).
    // A power cycle works in any case: 3000 ms (off, or on when it was off), 5 s, 500 ms (on; < 2500 ms never off)
    while (s_uart.available()) s_uart.read();
    pressKey(3000);
    delay(5000);
    pressKey(500);
    uint32_t t = millis();
    while (!alive() && millis() - t < 25000) delay(500);
    if (!alive()) { snprintf(m_status, sizeof(m_status), "no answer after power key"); return false; }
  }
  at("ATE0\r");
  if (m_muxWanted && !m_muxOn) {
    if (!muxStart()) {                 // the modem may be half in CMUX: off, then the old way
      Serial.println("[CELL] CMUX failed - power off, start without CMUX");
      muxStop();
      pressKey(3000);
      delay(5000);
      m_muxWanted = false;
      bool r = start();
      m_muxWanted = true;              // next start tries CMUX again
      return r;
    }
  }
  if (!m_muxOn) at("AT+CATR=0\r");      // URCs on every port again (a +CATR=6 from a CMUX start may stay)
  startGnss();
  m_alive = true;
  at("ATI\r", 1000);
  if (!strstr(m_buf, "7600")) Serial.printf("[CELL] ATI: %s\n", m_buf);   // not the expected model: logged, continue
  bool sim = false;
  for (int i = 0; i < 20 && !(sim = at("AT+CPIN?\r", 500, ": READY")); i++) delay(500);
  if (!sim) { snprintf(m_status, sizeof(m_status), "SIM not ready"); return false; }
  snprintf(m_status, sizeof(m_status), "registering");
  bool reg = false;
  for (uint32_t t = millis(); !reg && millis() - t < 90000; delay(1000)) {
    if (at("AT+CREG?\r", 1000, "+CREG: 0,")) {
      char* p = strstr(m_buf, "+CREG: 0,");
      reg = p && (p[9] == '1' || p[9] == '5' || p[9] == '6');
    }
  }
  if (!reg) { snprintf(m_status, sizeof(m_status), "not registered"); return false; }
  // what the modem itself has stored as APN (its own profile) - logged, not changed
  if (at("AT+CGDCONT?\r", 1000)) {
    char* p = strstr(m_buf, "+CGDCONT:");
    char* e = p ? strpbrk(p, "\r\n") : nullptr;
    if (p && e) { *e = 0; snprintf(m_cgdcont, sizeof(m_cgdcont), "%s", p + 9); }
    Serial.printf("[CELL] stored APN profile: %s\n", m_cgdcont);
  }
  if (m_apn[0]) {
    char c[80];
    snprintf(c, sizeof(c), "AT+CGSOCKCONT=1,\"IP\",\"%s\"\r", m_apn);
    at(c);
  }
  at("AT+CSOCKSETPN=1\r");
  at("AT+CIPMODE=0\r");
  // NETOPEN answers OK first, then "+NETOPEN: 0" (or "already opened")
  at("AT+NETOPEN\r", 15000, "+NETOPEN:");
  if (!strstr(m_buf, "+NETOPEN: 0") && !at("AT+NETOPEN?\r", 2000, "+NETOPEN: 1")) {
    snprintf(m_status, sizeof(m_status), "NETOPEN failed");
    return false;
  }
  at("AT+CIPRXGET=1\r");               // buffer mode before any socket (v1F, SIMCom manual 6.3.2)
  m_netOpen = true;
  snprintf(m_status, sizeof(m_status), "online");
  Serial.println("[CELL] online");
  return true;
}

void CellLink::stop()
{
  close();
  m_netOpen = false;
  if (!m_muxOn) at("AT+CGPS=0\r", 1000);
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

static void diagText(char* out, int cap, const char* prefix, const char* in);

// p71: one UDP line - the answer on the data port (m_buf) and on port 3 (m_urc); after each
// open the first 3 good and 12 bad results, so a working link does not flood
void CellLink::udpNote(const char* what, bool ok)
{
  if (!udpDiag || (ok ? m_udpOk++ >= 3 : m_udpErr++ >= 12)) return;
  char both[200], t[120];
  portENTER_CRITICAL(&m_spin);
  snprintf(both, sizeof(both), "%.*s U:%.*s", 100, m_buf, 80, m_urc);
  portEXIT_CRITICAL(&m_spin);
  diagText(t, sizeof(t), what, both);
  udpDiag(t);
}

bool CellLink::open(const char* host, uint16_t port)
{
  if (!m_netOpen) return false;
  lock();
  m_udpOk = m_udpErr = 0;
  char c[120];
  snprintf(c, sizeof(c), "AT+CDNSGIP=\"%s\"\r", host);
  if (at(c, 10000)) {                  // +CDNSGIP: 1,"host","1.2.3.4"
    char* p = strstr(m_buf, host);
    p = p ? strstr(p, ",\"") : nullptr;
    if (p) {
      p += 2;
      char* q = strchr(p, '"');
      if (q && q - p < (int)sizeof(m_ip)) { memcpy(m_ip, p, q - p); m_ip[q - p] = 0; }
    }
  }
  if (!m_ip[0]) { udpNote("UDP DNS ERR ", false); unlock(); return false; }
  m_port = port;
  if (m_sockOpen) at("AT+CIPCLOSE=0\r", 2000);
  at("AT+CIPRXGET=1\r");
  snprintf(c, sizeof(c), "AT+CIPOPEN=0,\"UDP\",\"%s\",%u,8000\r", m_ip, port);
  m_sockOpen = at(c, 3000);
  udpNote(m_sockOpen ? "UDP OPEN OK " : "UDP OPEN ERR ", m_sockOpen);
  unlock();
  return m_sockOpen;
}

bool CellLink::send(const char* data, unsigned int len)
{
  if (!m_sockOpen) return false;
  lock();
  while (linkAvail()) linkRead();
  m_len = 0;
  m_buf[0] = 0;
  portENTER_CRITICAL(&m_spin);
  m_urcLen = 0;
  m_urc[0] = 0;
  portEXIT_CRITICAL(&m_spin);
  char c[80];
  int n = snprintf(c, sizeof(c), "AT+CIPSEND=0,%u,\"%s\",%u\r", len, m_ip, m_port);
  linkWrite((const uint8_t*)c, n);
  bool prompt = waitFor("\r\n>", "\r\nERROR", 1000);   // the prompt, then the bytes
  linkWrite((const uint8_t*)data, len);
  bool ok = waitFor("+CIPSEND:", "\r\nERROR", 2000);
  udpNote(ok ? "UDP SEND OK " : prompt ? "UDP SEND ERR " : "UDP SEND NOPROMPT ", ok);
  unlock();
  return ok;
}

int CellLink::receive(char* buf, int cap, unsigned int timeoutMs)
{
  if (!m_sockOpen) return 0;
  uint32_t t = millis();
  do {
    lock();
    bool got = at("AT+CIPRXGET=2,0,1500\r", 1000);
    // +CIPRXGET: 2,<link>,<read_len>,<rest_len>\r\n<data>\r\nOK
    char* p = got ? strstr(m_buf, "+CIPRXGET: 2,") : nullptr;
    int n = 0;
    if (p) {
      char* q = strchr(p, ',');
      if (q) q = strchr(q + 1, ',');
      int len = q ? atoi(q + 1) : 0;
      char* nl = q ? strchr(q, '\n') : nullptr;
      if (nl && len > 0) {
        n = len < cap ? len : cap;
        memcpy(buf, nl + 1, n);
      }
    }
    if (n > 0) { char w[24]; snprintf(w, sizeof(w), "UDP RX %d ", n); udpNote(w, true); }
    else if (millis() - t >= timeoutMs && timeoutMs >= 1000) udpNote("UDP RX NONE ", false);   // not the 10 ms inbound polls
    unlock();
    if (n > 0) return n;
    if (millis() - t >= timeoutMs) break;
    delay(100);
  } while (true);
  return 0;
}

void CellLink::close()
{
  if (m_sockOpen) at("AT+CIPCLOSE=0\r", 2000);
  m_sockOpen = false;
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

bool CellLink::gnss(GnssFix* f)
{
  memset(f, 0, sizeof(*f));
  if (!m_mux || !m_alive) return false;
  if (m_muxOn) {                       // p69: the modem pushes RMC + GNS on port 1 every second
    uint32_t now = millis();
    if (m_nmeaAt && now - m_nmeaAt < 10000) {
      portENTER_CRITICAL(&m_spin);
      GnssFix c = m_nfix;
      uint32_t r = m_rmcAt, g = m_gnsAt;
      portEXIT_CRITICAL(&m_spin);
      if (!r || now - r > 3000 || !c.valid) return false;
      if (!g || now - g > 3000) { c.sats = 0; c.hdop = 0; }
      *f = c;
      return true;
    }
    if (m_gpsAt && now - m_gpsAt < 20000) return false;   // NMEA not started yet
    // no NMEA on port 1 for 10 s: the old AT query (on port 2) as a fallback
  }
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


// ------------------------------------------------------------- HTTPS (CCH) ---

// a modem answer as diagnostic text: \r\n -> '|', non-printable -> '.', at most `cap` chars
static void diagText(char* out, int cap, const char* prefix, const char* in)
{
  int n = snprintf(out, cap, "%s", prefix);
  for (const char* p = in; *p && n < cap - 1; p++) {
    char ch = *p;
    if (ch == '\r') continue;
    out[n++] = ch == '\n' ? '|' : (ch >= 32 && ch < 127 ? ch : '.');
  }
  out[n < cap ? n : cap - 1] = 0;
}

bool CellLink::tlsOpen(const char* host, uint16_t port)
{
  if (!m_netOpen) return false;
  lock();
  at("AT+CCHCLOSE=0\r", 2000, "+CCHCLOSE:");   // a session left from before (harmless ERROR)
  at("AT+CCHSTOP\r", 3000, "+CCHSTOP:");      // wait for the stop itself, not only its OK (p43)
  // send result report, cache receive mode - before CCHSTART; an ERROR here left the modem pushing
  // the answer by itself (p43, 26.10.2026: CCHRECV only ERROR, the answer never read)
  bool set = at("AT+CCHSET=1,1\r");
  if (!set) { delay(500); set = at("AT+CCHSET=1,1\r"); }
  if (diag) { char t[64]; diagText(t, sizeof(t), set ? "CCH SET OK " : "CCH SET ERR ", m_buf); diag(t); }
  m_pushLen = m_pushPos = 0;
  m_moreData = false;
  bool started = at("AT+CCHSTART\r", 5000);
  if (!started) started = at("AT+CCHSTART\r", 5000);
  if (!started) {
    if (diag) { char t[64]; diagText(t, sizeof(t), "CCH START ERR ", m_buf); diag(t); }
    unlock(); snprintf(m_status, sizeof(m_status), "online, CCHSTART failed"); return false;
  }
  m_diagRecv = 0;
  if (diag && at("AT+CCHSET?\r", 2000)) { char t[64]; diagText(t, sizeof(t), "CCH SET? ", m_buf); diag(t); }
  at("AT+CSSLCFG=\"sslversion\",0,4\r");       // all versions
  at("AT+CSSLCFG=\"authmode\",0,0\r");         // no server certificate check (as v1F, self-signed server)
  at("AT+CSSLCFG=\"ignorelocaltime\",0,1\r");
  at("AT+CSSLCFG=\"enableSNI\",0,1\r");
  at("AT+CCHSSLCFG=0,0\r");
  char c[160];
  snprintf(c, sizeof(c), "AT+CCHOPEN=0,\"%s\",%u,2\r", host, (unsigned)port);
  bool ok = at(c, 30000, "+CCHOPEN: 0,0");
  if (!ok) Serial.printf("[CELL] CCHOPEN %s:%u failed: %s\n", host, (unsigned)port, m_buf);
  if (diag) { char t[64]; diagText(t, sizeof(t), ok ? "CCH OPEN OK " : "CCH OPEN ERR ", m_buf); diag(t); }
  unlock();
  return ok;
}

bool CellLink::tlsSend(const char* data, int len)
{
  lock();
  char c[40];
  snprintf(c, sizeof(c), "AT+CCHSEND=0,%d\r", len);
  bool ok = at(c, 5000, ">");
  if (ok) {
    linkWrite((const uint8_t*)data, len);
    m_len = 0;
    m_buf[0] = 0;
    ok = waitFor("\r\nOK", "\r\nERROR", 10000);
  }
  if (diag) {
    char t[64], pre[24];
    snprintf(pre, sizeof(pre), "CCH SEND %d %s ", len, ok ? "OK" : "ERR");
    diagText(t, sizeof(t), pre, m_buf);
    diag(t);
  }
  unlock();
  return ok;
}

// one AT+CCHRECV: "+CCHRECV: DATA,0,<n>\r\n<n raw bytes>...OK"; no data = no DATA line
int CellLink::tlsRead(uint8_t* buf, int cap, unsigned int timeoutMs)
{
  // as the FreematicsPlus SIM7600 branch (p44, 2026-10-26): in cache mode wait for the modem's
  // "+CCHRECV: 0,<n>" (data ready) or +CCH_PEER_CLOSED, only then AT+CCHRECV. The modem is held
  // (lock) for the whole wait - another task's at() would throw the notice away. Every modem line
  // goes into the diagnostics (first 16 per connection).
  if (m_muxOn) return tlsReadMux(buf, cap, timeoutMs);
  if (m_pushPos < m_pushLen) {
    int n = m_pushLen - m_pushPos < cap ? m_pushLen - m_pushPos : cap;
    memcpy(buf, m_push + m_pushPos, n);
    m_pushPos += n;
    return n;
  }
  uint32_t t0 = millis(), lastProbe = millis();
  lock();
  // RECV EVENT comes only for NEW data: after a full read the rest is still cached (p46)
  bool ready = m_moreData, closed = false;
  m_moreData = false;
  char line[96];
  int ll = 0;
  while (!ready && !closed && millis() - t0 < timeoutMs) {
    if (millis() - lastProbe > 5000) { ready = true; break; }   // no notice in 5 s: try a read anyway
    if (!s_uart.available()) { delay(2); continue; }
    char ch = (char)s_uart.read();
    if (ch != '\n') { if (ch != '\r' && ll < (int)sizeof(line) - 1) { line[ll++] = ch; line[ll] = 0; } continue; }
    line[ll] = 0;
    if (!ll) continue;
    ll = 0;
    if (diag && m_diagRecv < 16) { m_diagRecv++; char t[64]; diagText(t, sizeof(t), "CCH URC ", line); diag(t); }
    char* d = strstr(line, "+CCHRECV: DATA,0,");
    if (d) {                                   // pushed by the modem itself
      int n = atoi(d + 17), got = 0;
      uint32_t tb = millis();
      while (got < n && millis() - tb < 5000) {
        if (!s_uart.available()) { delay(1); continue; }
        int b = s_uart.read();
        if (got < (int)sizeof(m_push)) m_push[got] = (uint8_t)b;
        got++;
        tb = millis();
      }
      m_pushLen = got < (int)sizeof(m_push) ? got : (int)sizeof(m_push);
      m_pushPos = 0;
      unlock();
      return tlsRead(buf, cap, timeoutMs);
    }
    // the data-ready notice: "+CCHEVENT: 0,RECV EVENT" on this SIM7600G (car 26.10.2026), as the
    // FreematicsPlus inbound() pattern "RECV EVENT"; "+CCHRECV: 0,<n>" on other firmware (p45)
    if (strstr(line, "RECV EVENT") || strstr(line, "+CCHRECV: 0,") || strstr(line, "+CCHRECV:0,")) ready = true;
    if (strstr(line, "PEER_CLOSED") || strstr(line, "+CCHCLOSE")) closed = true;
  }
  if (!ready && !closed) { unlock(); return -1; }
  // read what the modem keeps. SIM7500/SIM7600/SIM7800 SSL AT Command Manual V1.00, 2.2.10 - the
  // answer is "OK" FIRST, then "+CCHRECV: DATA,<id>,<len>\r\n<len bytes>" (one or more), then
  // "+CCHRECV: <id>,<err>"; ERROR when nothing is cached ("not allowed"). Stopping at the OK threw
  // the data away (p46, car 26.10.2026: "+CCH_PEER_CLOSED: 0 | OK", then only ERROR).
  if (diag && m_diagRecv < 16 && at("AT+CCHRECV?\r", 1000)) {
    m_diagRecv++;
    char t[64];
    diagText(t, sizeof(t), "CCH LEN ", m_buf);
    diag(t);
  }
  char c[40];
  snprintf(c, sizeof(c), "AT+CCHRECV=0,%d\r", cap);
  s_uart.print(c);
  int got = 0, blocks = 0, err = -1;
  bool failed = false;
  ll = 0;
  uint32_t t = millis();
  while (millis() - t < 5000) {
    if (!s_uart.available()) { delay(1); continue; }
    char ch = (char)s_uart.read();
    t = millis();
    if (ch != '\n') { if (ch != '\r' && ll < (int)sizeof(line) - 1) { line[ll++] = ch; line[ll] = 0; } continue; }
    line[ll] = 0;
    if (!ll) continue;
    ll = 0;
    if (strstr(line, "PEER_CLOSED") || strstr(line, "+CCHCLOSE")) { closed = true; continue; }
    if (!strcmp(line, "ERROR")) { failed = true; break; }
    char* d = strstr(line, "+CCHRECV: DATA,");
    if (d) {                                   // "<id>,<len>" (the manual also shows "DATA, <id>")
      char* q = strchr(d + 15, ',');
      int n = q ? atoi(q + 1) : 0, r = 0;
      uint32_t tb = millis();
      while (r < n && millis() - tb < 5000) {
        if (!s_uart.available()) { delay(1); continue; }
        int b = s_uart.read();
        if (got < cap) buf[got++] = (uint8_t)b;
        r++;
        tb = millis();
      }
      blocks++;
      continue;
    }
    if (!strncmp(line, "+CCHRECV: 0,", 12)) { err = atoi(line + 12); break; }   // end of the answer
  }
  if (diag && m_diagRecv < 16) {
    m_diagRecv++;
    char tt[64];
    snprintf(tt, sizeof(tt), "CCH RECV got=%d blocks=%d err=%d%s%s", got, blocks, err,
             failed ? " ERROR" : "", closed ? " closed" : "");
    diag(tt);
  }
  if (got > 0) { m_moreData = got >= cap; unlock(); return got; }
  unlock();
  if (closed) return 0;
  if (millis() - t0 >= timeoutMs) return -1;
  return tlsRead(buf, cap, timeoutMs - (millis() - t0));   // nothing yet: wait for the next notice
}

void CellLink::tlsClose()
{
  lock();
  at("AT+CCHCLOSE=0\r", 3000, "+CCHCLOSE:");
  at("AT+CCHSTOP\r", 3000);
  unlock();
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

int CellLink::linkAvail()
{
  if (m_muxOn) return (int)xStreamBufferBytesAvailable(m_rx2);
  return s_uart.available();
}

int CellLink::linkRead()
{
  if (m_muxOn) {
    uint8_t b;
    return xStreamBufferReceive(m_rx2, &b, 1, 0) == 1 ? b : -1;
  }
  return s_uart.read();
}

void CellLink::linkWrite(const uint8_t* d, size_t n)
{
  if (m_muxOn) { muxTx(2, d, n); return; }
  xSemaphoreTake(m_txLock, portMAX_DELAY);
  s_uart.write(d, n);
  xSemaphoreGive(m_txLock);
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
  m_binLeft = 0;
  for (int c = 0; c < 5; c++) { m_chOpen[c] = 0; m_lineLen[c] = 0; }
  xStreamBufferReset(m_rx2);
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
  at("ATE0\r", 1000);                           // port 2: our AT link
  muxTxStr(4, "ATE0\r");
  unlock();
  Serial.println("[CELL] CMUX up");
  return true;
}

void CellLink::muxStop()
{
  if (!m_muxOn) return;
  m_muxOn = false;
  delay(30);
  while (s_uart.available()) s_uart.read();
  xStreamBufferReset(m_rx2);
  for (int c = 0; c < 5; c++) m_chOpen[c] = 0;
}

// GNSS on port 1 (OVMS simcom_7600.cpp:86-95: CGPS=0 first, the SIM7600 may already run it), RMC + GNS
// every second; p66 start mode (N / H / C) kept. Without CMUX the old single command.
void CellLink::startGnss()
{
  const char* go = m_gpsMode == 'H' ? "AT+CGPSHOT\r" : m_gpsMode == 'C' ? "AT+CGPSCOLD\r" : "AT+CGPS=1,1\r";
  if (!m_muxOn) {
    m_gpsOk = at(go, 2000);            // an error = already on
    m_gpsAt = millis();
    return;
  }
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
  if (ch == 2) {
    if (xStreamBufferSend(m_rx2, d, n, 0) != (size_t)n) m_rx2Lost += n;
    return;
  }
  for (int i = 0; i < n; i++) {
    uint8_t c = d[i];
    if (m_binLeft > 0) {                           // a pushed +CCHRECV: DATA block (raw bytes)
      portENTER_CRITICAL(&m_spin);
      if (m_pushLen < (int)sizeof(m_push)) m_push[m_pushLen++] = c;
      portEXIT_CRITICAL(&m_spin);
      m_binLeft--;
      continue;
    }
    if (c == '\n') {
      m_lines[ch][m_lineLen[ch]] = 0;
      if (m_lineLen[ch]) portLine(ch, m_lines[ch]);
      m_lineLen[ch] = 0;
    } else if (c != '\r' && m_lineLen[ch] < (int)sizeof(m_lines[0]) - 1) {
      m_lines[ch][m_lineLen[ch]++] = (char)c;
    }
  }
}

// one text line from port 1, 3 or 4 (port 3 = every URC, +CATR=6)
void CellLink::portLine(int ch, char* line)
{
  if (line[0] == '$') { nmeaLine(line); return; }
  if (ch == 1) {
    if (!strcmp(line, "OK")) m_gpsCmdResult = 1;
    else if (strstr(line, "ERROR")) m_gpsCmdResult = 0;
    return;
  }
  if (ch != 3) return;
  if (strstr(line, "RECV EVENT")) m_recvEvent = true;
  if (!strncmp(line, "+CCHRECV: 0,", 12) || !strncmp(line, "+CCHRECV:0,", 11)) { m_recvEvent = true; m_recvEnd = true; }
  if (strstr(line, "PEER_CLOSED") || !strncmp(line, "+CCHCLOSE", 9)) m_peerClosed = true;
  const char* d = strstr(line, "+CCHRECV: DATA,");
  if (d) {                                         // "<id>,<len>" then <len> raw bytes on this port
    const char* q = strchr(d + 15, ',');
    int n = q ? atoi(q + 1) : 0;
    portENTER_CRITICAL(&m_spin);
    if (m_pushPos >= m_pushLen) m_pushPos = m_pushLen = 0;
    portEXIT_CRITICAL(&m_spin);
    m_binLeft = n;
  }
  if (!strcmp(line, "OK") || !strcmp(line, "ERROR")) return;   // never an answer of the data port
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

// HTTPS read over CMUX: the notices come on port 3 (m_recvEvent / m_peerClosed), the answer of
// AT+CCHRECV on port 2 - its DATA blocks may also come as URCs on port 3 (then m_push); the end is
// "+CCHRECV: 0,<err>" on either port (manual SSL 2.2.10)
int CellLink::tlsReadMux(uint8_t* buf, int cap, unsigned int timeoutMs)
{
  portENTER_CRITICAL(&m_spin);
  int have = m_pushLen - m_pushPos;
  portEXIT_CRITICAL(&m_spin);
  if (have > 0 && m_binLeft == 0) {
    int n = have < cap ? have : cap;
    portENTER_CRITICAL(&m_spin);
    memcpy(buf, m_push + m_pushPos, n);
    m_pushPos += n;
    portEXIT_CRITICAL(&m_spin);
    return n;
  }
  uint32_t t0 = millis();
  lock();
  bool ready = m_moreData;
  m_moreData = false;
  while (!ready && !m_recvEvent && !m_peerClosed && millis() - t0 < timeoutMs) {
    if (millis() - t0 > 5000) { ready = true; break; }   // no notice in 5 s: try a read anyway
    delay(5);
  }
  if (!ready && !m_recvEvent && !m_peerClosed) { unlock(); return -1; }
  bool closed = m_peerClosed;
  m_recvEvent = false;
  m_recvEnd = false;
  while (linkAvail()) linkRead();
  char c[40];
  snprintf(c, sizeof(c), "AT+CCHRECV=0,%d\r", cap);
  linkWrite((const uint8_t*)c, strlen(c));
  int got = 0, blocks = 0, err = -1;
  bool failed = false;
  char line[96];
  int ll = 0;
  uint32_t t = millis();
  while (millis() - t < 5000 && !m_recvEnd) {
    int ch = linkRead();
    if (ch < 0) { delay(1); continue; }
    t = millis();
    if (ch != '\n') { if (ch != '\r' && ll < (int)sizeof(line) - 1) { line[ll++] = (char)ch; line[ll] = 0; } continue; }
    line[ll] = 0;
    if (!ll) continue;
    ll = 0;
    if (strstr(line, "PEER_CLOSED") || strstr(line, "+CCHCLOSE")) { closed = true; continue; }
    if (!strcmp(line, "ERROR")) { failed = true; break; }
    char* d = strstr(line, "+CCHRECV: DATA,");
    if (d) {
      char* q = strchr(d + 15, ',');
      int n = q ? atoi(q + 1) : 0, r = 0;
      uint32_t tb = millis();
      while (r < n && millis() - tb < 5000) {
        int b = linkRead();
        if (b < 0) { delay(1); continue; }
        if (got < cap) buf[got++] = (uint8_t)b;
        r++;
        tb = millis();
      }
      blocks++;
      continue;
    }
    if (!strncmp(line, "+CCHRECV: 0,", 12)) { err = atoi(line + 12); break; }
  }
  for (uint32_t tw = millis(); m_binLeft > 0 && millis() - tw < 2000;) delay(2);   // a block still arriving on port 3
  portENTER_CRITICAL(&m_spin);
  int pushed = m_pushLen - m_pushPos;
  if (got == 0 && pushed > 0) {
    int n = pushed < cap ? pushed : cap;
    memcpy(buf, m_push + m_pushPos, n);
    m_pushPos += n;
    got = n;
  }
  portEXIT_CRITICAL(&m_spin);
  if (diag && m_diagRecv < 16) {
    m_diagRecv++;
    char tt[64];
    snprintf(tt, sizeof(tt), "CCH RECV mux got=%d blocks=%d err=%d%s%s", got, blocks, err,
             failed ? " ERROR" : "", closed ? " closed" : "");
    diag(tt);
  }
  if (got > 0) { m_moreData = got >= cap; unlock(); return got; }
  unlock();
  if (closed) return 0;
  if (millis() - t0 >= timeoutMs) return -1;
  return tlsReadMux(buf, cap, timeoutMs - (millis() - t0));
}
