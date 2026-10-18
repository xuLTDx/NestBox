#include "netlink.h"
#include <WiFi.h>
#include <WiFiUdp.h>

static WiFiUDP s_udp;
static IPAddress s_ip;
static uint16_t s_port;
static bool s_begun;

bool WifiLink::up() { return WiFi.status() == WL_CONNECTED; }

bool WifiLink::open(const char* host, uint16_t port)
{
  if (!up()) return false;
  IPAddress ip;
  if (!WiFi.hostByName(host, ip)) return false;
  s_ip = ip;
  s_port = port;
  if (!s_begun) s_begun = s_udp.begin(0);    // any local port; replies come back to it
  return s_begun;
}

bool WifiLink::send(const char* data, unsigned int len)
{
  if (!s_begun || !s_port) return false;
  if (!s_udp.beginPacket(s_ip, s_port)) return false;
  return s_udp.write((const uint8_t*)data, len) == len && s_udp.endPacket();
}

int WifiLink::receive(char* buf, int cap, unsigned int timeoutMs)
{
  if (!s_begun) return 0;
  uint32_t t = millis();
  do {
    int n = s_udp.parsePacket();
    if (n > 0) return s_udp.read(buf, cap);
    delay(1);
  } while (millis() - t < timeoutMs);
  return 0;
}

void WifiLink::close()
{
  if (s_begun) s_udp.stop();
  s_begun = false;
}
