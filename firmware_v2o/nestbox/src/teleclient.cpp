// see teleclient.h - the logic is NestBox_v1F TeleClientUDP, link-neutral
#include "teleclient.h"
#include "store.h"
#include "txqueue.h"
#include "udpmsg.h"
#include "sniff.h"
#include "nbread.h"
#include "standby.h"
#include <sys/time.h>

extern const char FW_BUILD_STR[];
#define FIRMWARE_VARIANT "NestBox_v2O"

char devid[12];

// v1F genDeviceID(): 8 characters from the eFuse MAC (B, D, I, O never used)
void genDeviceID(char* buf)
{
  uint64_t seed = ESP.getEfuseMac() >> 8;
  for (int i = 0; i < 8; i++, seed >>= 5) {
    byte x = (byte)seed & 0x1f;
    if (x >= 10) {
      x = x - 10 + 'A';
      switch (x) {
        case 'B': x = 'W'; break;
        case 'D': x = 'X'; break;
        case 'I': x = 'Y'; break;
        case 'O': x = 'Z'; break;
      }
    } else {
      x += '0';
    }
    buf[i] = x;
  }
  buf[8] = 0;
}

static uint16_t hex2uint16(const char* p)
{
  uint16_t v = 0;
  for (int i = 0; i < 4 && p[i]; i++) {
    char c = p[i];
    int d = (c >= '0' && c <= '9') ? c - '0' : (c >= 'A' && c <= 'F') ? c - 'A' + 10 :
            (c >= 'a' && c <= 'f') ? c - 'a' + 10 : -1;
    if (d < 0) break;
    v = (v << 4) | d;
  }
  return v;
}

bool TeleClientUDP::notify(uint8_t event, const char* payload)
{
  if (!link) return false;
  char buf[48];
  char cache[160];
  CStorageRAM netbuf;
  netbuf.init(cache, sizeof(cache));
  netbuf.header(devid);
  netbuf.dispatch(buf, sprintf(buf, "EV=%X", (unsigned)event));
  netbuf.dispatch(buf, sprintf(buf, "TS=%lu", (unsigned long)millis()));
  netbuf.dispatch(buf, sprintf(buf, "ID=%s", devid));
  if (payload) netbuf.dispatch(payload, strlen(payload));
  netbuf.tailer();
  for (int attempts = 0; attempts < 3; attempts++) {
    if (!link->send(netbuf.buffer(), netbuf.length())) break;
    if (event == EVENT_ACK) return true;
    int n = link->receive(m_rx, sizeof(m_rx) - 1, 3000);
    if (n <= 0) {
      Serial.println("[UDP] Timeout");
      logNetEvent("NET UDP_TIMEOUT");
      continue;
    }
    m_rx[n] = 0;
    rxBytes += n;
    UdpRx rx;
    rx.self = this;
    rx.want = event;
    int bad = 0;
    udpSplit(m_rx, n, udpOnMessage, &rx, &bad);
    if (!rx.found) {
      Serial.printf("[UDP] %s %s\n", bad ? "Checksum mismatch:" : "Invalid reply:", m_rx);
      logNetEvent(bad ? "NET UDP_CHECKSUM_MISMATCH" : "NET UDP_INVALID_REPLY");
      continue;
    }
    char* data = rx.reply;
    if (event == EVENT_LOGIN) {
      char* p = strstr(data, "TM=");            // the server's clock
      if (p) {
        struct timeval tv = {(time_t)atol(p + 3), 0};
        settimeofday(&tv, NULL);
      }
      p = strstr(data, "SN=");
      if (p) {
        char* q = strchr(p, ',');
        if (q) *q = 0;
      }
      feedid = hex2uint16(data);
      login = true;
    } else if (event == EVENT_LOGOUT) {
      login = false;
    }
    return true;
  }
  return false;
}

bool TeleClientUDP::connect(bool quick)
{
  if (!link || !link->up()) return false;
  if (quick) return link->open(SERVER_HOST, SERVER_PORT);
  uint8_t event = login ? EVENT_RECONNECT : EVENT_LOGIN;
  bool success = false;
  for (int attempts = 0; attempts < 3; attempts++) {
    char diag[72];
    snprintf(diag, sizeof(diag), "%s %s %s:%u attempt=%d", event == EVENT_LOGIN ? "LOGIN" : "RECONNECT",
             link->name(), SERVER_HOST, (unsigned)SERVER_PORT, attempts);
    Serial.println(diag);
    logNetEvent(diag);
    if (!link->open(SERVER_HOST, SERVER_PORT)) {
      logNetEvent("NET OPEN_FAIL");
      delay(1000);
      continue;
    }
    // LOGIN carries the build (OTA ordering) and the variant (which hardware/firmware line)
    char fw[80];
    if (event == EVENT_LOGIN) snprintf(fw, sizeof(fw), "FW=%s,VARIANT=%s", FW_BUILD_STR, FIRMWARE_VARIANT);
    if (!notify(event, event == EVENT_LOGIN ? fw : 0)) {
      link->close();
      logNetEvent("NET NOTIFY_TIMEOUT");
      continue;
    }
    logNetEvent("NET NOTIFY_OK");
    success = true;
    break;
  }
  if (!success) logNetEvent("NET CONNECT_FAILED_ALL_ATTEMPTS");
  if (event == EVENT_LOGIN) startTime = millis();
  if (success) lastSyncTime = millis();
  return success;
}

bool TeleClientUDP::ping()
{
  bool success = false;
  for (int n = 0; n < 3 && !success; n++) {
    success = link && link->open(SERVER_HOST, SERVER_PORT);
    if (success) {
      if ((success = notify(EVENT_PING))) break;
      link->close();
      delay(1000);
    }
  }
  if (success) lastSyncTime = millis();
  return success;
}

bool TeleClientUDP::transmit(const char* packet, unsigned int size)
{
  if (!link || !link->send(packet, size)) return false;
  txBytes += size;
  txCount++;
  Serial.printf("[%s] %u bytes sent\n", link->name(), size);
  return true;
}

// one server message (checksum checked): ACK, command, the awaited reply, or a sync
void TeleClientUDP::onMessage(char* msg, UdpRx* rx)
{
  if (!strncmp(msg, "1#ACK=", 6)) {             // after the records are stored
    txqOnAck(strtoul(msg + 6, 0, 10));
    lastSyncTime = millis();
    return;
  }
  char* p = strstr(msg, "EV=");
  if (!p) return;
  int eventID = atoi(p + 3);
  if (eventID == EVENT_COMMAND) {
    char* cmdp = strstr(msg, "CMD=");
    if (cmdp && !strncmp(cmdp + 4, "OTA_READY", 9)) {
      Serial.println("[UDP] CMD:OTA_READY");
      logNetEvent("NET CMD_OTA_READY");
      otaReadyRequested();
    }
    if (cmdp && !strncmp(cmdp + 4, "SNIFF=", 6)) {
      bool on = cmdp[10] == '1';
      const char* c = strchr(cmdp + 10, ',');
      uint16_t min = c && c[1] != '*' ? (uint16_t)atoi(c + 1) : 0;
      sniffRequest(on, min);
      Serial.printf("[UDP] CMD:SNIFF=%d,%u\n", on ? 1 : 0, (unsigned)min);
      logNetEvent(on ? "NET CMD_SNIFF_ON" : "NET CMD_SNIFF_OFF");
    }
    if (cmdp && !strncmp(cmdp + 4, "APN=", 4)) {    // APN from the server, tried first (nbapn.h)
      telemetryApnCommand(cmdp + 8);
      logNetEvent("NET CMD_APN");
    }
    if (cmdp && !strncmp(cmdp + 4, "READ=", 5)) {   // one read request into the car (nbread.h)
      uint32_t seq = 0;
      int r = nbReadSubmit(cmdp + 9, &seq);
      char m[64];
      snprintf(m, sizeof(m), "NET CMD_READ #%u %s", (unsigned)seq, r ? "refused" : "accepted");
      Serial.println(m);
      logNetEvent(m);
    }
    if (cmdp && (!strcmp(cmdp + 4, "ULP_READ") || !strcmp(cmdp + 4, "STANDBY?") ||
                 !strncmp(cmdp + 4, "SLEEP_ULP=", 10))) {   // p59: sleep tests from FleetNest (standby.h)
      Serial.printf("[UDP] CMD:%s\n", cmdp + 4);
      standbyRemote(cmdp + 4);
    }
    if (cmdp && !strncmp(cmdp + 4, "RESEND=", 7)) {
      uint32_t f = (uint32_t)strtoul(cmdp + 11, 0, 10);
      bool started = txqResendRequest(f);
      char m[48];
      snprintf(m, sizeof(m), "NET CMD_RESEND file=%u %s", (unsigned)f, started ? "start" : "running");
      Serial.println(m);
      logNetEvent(m);
    }
    return;
  }
  if (rx->want && eventID == rx->want) {
    strncpy(rx->reply, msg, sizeof(rx->reply) - 1);
    rx->reply[sizeof(rx->reply) - 1] = 0;
    rx->found = true;
    return;
  }
  if (eventID == EVENT_SYNC) feedid = hex2uint16(msg);
}

void udpOnMessage(char* msg, void* ctx)
{
  UdpRx* rx = (UdpRx*)ctx;
  rx->self->onMessage(msg, rx);
}

void TeleClientUDP::inbound()
{
  if (!link) return;
  int len = link->receive(m_rx, sizeof(m_rx) - 1, 10);
  if (len <= 0) return;
  m_rx[len] = 0;
  Serial.printf("[UDP] %s\n", m_rx);
  rxBytes += len;
  UdpRx rx;
  rx.self = this;
  rx.want = 0;
  int bad = 0;
  int n = udpSplit(m_rx, len, udpOnMessage, &rx, &bad);
  if (bad) logNetEvent("NET INBOUND_CHECKSUM_MISMATCH");
  if (n) lastSyncTime = millis();
}

void TeleClientUDP::shutdown()
{
  if (login) {
    notify(EVENT_LOGOUT);
    login = false;
  }
  if (link) link->close();
}
