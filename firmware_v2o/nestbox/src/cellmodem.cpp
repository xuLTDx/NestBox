// SIM7600G - see cellmodem.h (sources, untested state)
#include "cellmodem.h"
#include "max7317.h"
#include "hw.h"

static HardwareSerial s_uart(1);

void CellLink::lock() { if (m_mux) xSemaphoreTakeRecursive(m_mux, portMAX_DELAY); }
void CellLink::unlock() { if (m_mux) xSemaphoreGiveRecursive(m_mux); }

void CellLink::begin()
{
  if (m_mux) return;
  m_mux = xSemaphoreCreateRecursiveMutex();
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
    while (s_uart.available() && m_len < (int)sizeof(m_buf) - 1) {
      m_buf[m_len++] = (char)s_uart.read();
      m_buf[m_len] = 0;
    }
    if (a && strstr(m_buf, a)) return true;
    if (b && strstr(m_buf, b)) return false;
    delay(2);
  } while (millis() - t < timeoutMs);
  return false;
}

bool CellLink::at(const char* cmd, unsigned int timeoutMs, const char* expect)
{
  lock();
  while (s_uart.available()) s_uart.read();   // stale bytes (URCs) are not this command's answer
  m_len = 0;
  m_buf[0] = 0;
  if (cmd) s_uart.print(cmd);
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
    pressKey(200);                     // SIM7600: >= 100 ms switches it on, < 2500 ms never off
    uint32_t t = millis();
    while (!alive() && millis() - t < 25000) delay(500);
    if (!alive()) { snprintf(m_status, sizeof(m_status), "no answer after power key"); return false; }
  }
  at("ATE0\r");
  at("AT+CGPS=1,1\r", 2000);           // GNSS first: it does not need the network (an error = already on)
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
  at("AT+CGPS=0\r", 1000);
  if (!at("AT+CPOF\r", 3000)) pressKey(2600);   // v1F end(): AT+CPOF, else the long key press
  m_alive = false;
  snprintf(m_status, sizeof(m_status), "off");
}

bool CellLink::open(const char* host, uint16_t port)
{
  if (!m_netOpen) return false;
  lock();
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
  if (!m_ip[0]) { unlock(); return false; }
  m_port = port;
  if (m_sockOpen) at("AT+CIPCLOSE=0\r", 2000);
  at("AT+CIPRXGET=1\r");
  snprintf(c, sizeof(c), "AT+CIPOPEN=0,\"UDP\",\"%s\",%u,8000\r", m_ip, port);
  m_sockOpen = at(c, 3000);
  unlock();
  return m_sockOpen;
}

bool CellLink::send(const char* data, unsigned int len)
{
  if (!m_sockOpen) return false;
  lock();
  while (s_uart.available()) s_uart.read();
  m_len = 0;
  m_buf[0] = 0;
  char c[80];
  int n = snprintf(c, sizeof(c), "AT+CIPSEND=0,%u,\"%s\",%u\r", len, m_ip, m_port);
  s_uart.write((const uint8_t*)c, n);
  waitFor("\r\n>", "\r\nERROR", 1000);   // the prompt, then the bytes
  s_uart.write((const uint8_t*)data, len);
  bool ok = waitFor("+CIPSEND:", "\r\nERROR", 2000);
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

bool CellLink::gnss(GnssFix* f)
{
  memset(f, 0, sizeof(*f));
  if (!m_mux || !m_alive) return false;
  lock();                              // the whole reply is read under the lock
  bool got = at("AT+CGNSSINFO\r", 1000, "+CGNSSINFO:");
  if (got) waitFor("\r\nOK", nullptr, 300);
  char line[200];
  char* p = got ? strstr(m_buf, "+CGNSSINFO:") : nullptr;
  if (!p) { unlock(); return false; }
  p += 11;
  while (*p == ' ') p++;
  int i = 0;
  while (*p && *p != '\r' && *p != '\n' && i < (int)sizeof(line) - 1) line[i++] = *p++;
  line[i] = 0;
  unlock();
  // tokens; an empty field stays empty
  char* tok[24];
  int nt = 0;
  char* s = line;
  tok[nt++] = s;
  for (; *s && nt < 24; s++) if (*s == ',') { *s = 0; tok[nt++] = s + 1; }
  int ns = -1;                         // index of the N/S letter
  for (int k = 0; k + 1 < nt; k++) if ((!strcmp(tok[k], "N") || !strcmp(tok[k], "S")) && k >= 2) { ns = k; break; }
  if (ns < 0 || ns + 9 >= nt || !tok[ns - 1][0]) return false;   // no fix (fields empty)
  int sats = 0;
  for (int k = 1; k < ns - 1; k++) sats += atoi(tok[k]);          // GPS + GLONASS + BEIDOU (+ GALILEO)
  f->lat = degOf(tok[ns - 1]) * (tok[ns][0] == 'S' ? -1 : 1);
  f->lng = degOf(tok[ns + 1]) * (tok[ns + 2][0] == 'W' ? -1 : 1);
  f->date = (uint32_t)atol(tok[ns + 3]);                          // DDMMYY
  f->time = (uint32_t)(atof(tok[ns + 4]) * 100);                  // HHMMSS.s -> HHMMSSss
  f->alt = atof(tok[ns + 5]);
  f->kph = atof(tok[ns + 6]) * 1.852f;                            // knots
  f->heading = (uint16_t)atof(tok[ns + 7]);
  f->hdop = ns + 9 < nt ? atof(tok[ns + 9]) : 0;                  // PDOP, HDOP, VDOP
  f->sats = sats;
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
