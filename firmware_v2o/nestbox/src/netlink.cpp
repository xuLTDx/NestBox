#include "netlink.h"
#include <WiFi.h>
#include <WiFiUdp.h>

static WiFiUDP s_udp;
static IPAddress s_ip;
static uint16_t s_port;
static bool s_begun;

bool WifiLink::up() { return WiFi.status() == WL_CONNECTED; }

// PG96 (9.11.2026): the DNS servers are global in lwIP (CONFIG_ESP_NETIF_SET_DNS_PER_DEFAULT_NETIF is off in
// Arduino 3.3.11), and PPP with usepeerdns (cellmodem.cpp) overwrites servers 0/1 with the operator's when it
// comes up (lwIP ipcp.c -> sdns() -> dns_setserver). WiFi stayed the link in use, but its DNS queries went to the
// operator's servers: U9K4M3G6 9.11. 18:25-19:01, modem online, 5 OTA checks over WiFi 'META_HTTP-1' after ~31 s
// and not one request on the OTA server; with the modem off (17:47, 18:30) the same check worked. UDP records
// kept going (the server's address was resolved before PPP came up). CellLink::makeDefault() already set PPP's
// route + DNS when the modem is used - WiFi never got its own back. Now WiFi's DNS (from its DHCP) are kept here
// and put back whenever WiFi is used.
#include <esp_netif.h>
#include <lwip/dns.h>
static uint32_t s_wifiDns[2];

void WifiLink::saveDns()
{
  for (int i = 0; i < 2; i++) {
    const ip_addr_t* d = dns_getserver(i);
    s_wifiDns[i] = (d && IP_IS_V4(d)) ? ip_2_ip4(d)->addr : 0;
  }
}

void WifiLink::makeDefault()
{
  if (WiFi.status() != WL_CONNECTED) return;
  esp_netif_t* n = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
  if (n) esp_netif_set_default_netif(n);    // ESP_NETIF_SET_DEFAULT: always netif_set_default (esp_netif_lwip.c)
  restoreDns();
}

// only the DNS part - cheap (compares first), netTask calls it on every pass while WiFi is the link,
// because PPP may come up (and overwrite the DNS) at any time
void WifiLink::restoreDns()
{
  if (WiFi.status() != WL_CONNECTED) return;
  for (int i = 0; i < 2; i++) {
    if (!s_wifiDns[i]) continue;
    const ip_addr_t* d = dns_getserver(i);
    if (d && IP_IS_V4(d) && ip_2_ip4(d)->addr == s_wifiDns[i]) continue;   // already WiFi's
    ip_addr_t a;
    IP_ADDR4(&a, 0, 0, 0, 0);
    ip_2_ip4(&a)->addr = s_wifiDns[i];
    dns_setserver(i, &a);
  }
}

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

// PG204 (13.11.2026): is there internet behind this WiFi - asked without touching the link in use.
// Why: bench U9K2M3J0 13.11.2026 boot 35, MAC blocked on the router, data on the modem: every 30 min (PG164's new
// try, 21:33 and 22:05) netTask moved the data to WiFi, needed 2 failed logins to learn it is still dead and went
// back - ~2 min 20 s without live data each time ("ERR NET udp timeout", "ERR NET connect WIFI", records late).
// Now the try is this packet: a UDP socket of its own, bound to WiFi's address - ESP-IDF's lwIP routes a packet with
// a bound source address out of the interface that owns it (LWIP_HOOK_IP4_ROUTE_SRC = ip4_route_src_hook,
// lwipopts.h), whatever the default route is (PPP here). The server's name is resolved over the link in use.
// 2 packets, 2 s each for the answer; an answer counts only from the server's address.
#include <lwip/sockets.h>
int WifiLink::probe(const char* host, uint16_t port, const char* pkt, int len, char* rx, int cap)
{
  if (WiFi.status() != WL_CONNECTED || len <= 0) return 0;
  IPAddress srv;
  if (!WiFi.hostByName(host, srv)) return 0;
  int fd = lwip_socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (fd < 0) return 0;
  struct sockaddr_in me = {};
  me.sin_family = AF_INET;
  me.sin_addr.s_addr = (uint32_t)WiFi.localIP();
  struct sockaddr_in to = {};
  to.sin_family = AF_INET;
  to.sin_port = htons(port);
  to.sin_addr.s_addr = (uint32_t)srv;
  struct timeval tv = {2, 0};
  int got = 0;
  if (lwip_bind(fd, (struct sockaddr*)&me, sizeof(me)) == 0 &&
      lwip_setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) == 0) {
    for (int i = 0; i < 2 && !got; i++) {
      if (lwip_sendto(fd, pkt, len, 0, (struct sockaddr*)&to, sizeof(to)) != len) break;
      struct sockaddr_in from = {};
      socklen_t fl = sizeof(from);
      int n = lwip_recvfrom(fd, rx, cap, 0, (struct sockaddr*)&from, &fl);
      if (n > 0 && from.sin_addr.s_addr == to.sin_addr.s_addr) got = n;
    }
  }
  lwip_close(fd);
  return got;
}
