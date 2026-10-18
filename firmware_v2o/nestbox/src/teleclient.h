// UDP client to FleetNest - NestBox_v2O. NestBox_v1F TeleClientUDP
// (firmware_v5/telelogger/teleclient.cpp) with one link (netlink.h) instead of
// a WiFi and a cellular member: login/ping/logout as EV= notifications, record
// packets from the delivery queue, every received buffer split into messages
// (udpmsg.h: ACKs, commands, the awaited reply), the same on any link.
#pragma once
#include <Arduino.h>
#include "netlink.h"

#ifndef SERVER_HOST
#define SERVER_HOST "ultd.mooo.com"
#endif
#ifndef SERVER_PORT
#define SERVER_PORT 6000
#endif

#define EVENT_LOGIN 1
#define EVENT_LOGOUT 2
#define EVENT_SYNC 3
#define EVENT_RECONNECT 4
#define EVENT_COMMAND 5
#define EVENT_ACK 6
#define EVENT_PING 7

#define RECV_BUF_SIZE 1024

class TeleClientUDP;
struct UdpRx {
  TeleClientUDP* self = nullptr;
  uint8_t want = 0;
  bool found = false;
  char reply[160] = "";
};
void udpOnMessage(char* msg, void* ctx);

class TeleClientUDP {
public:
  void reset() { txCount = txBytes = rxBytes = 0; login = false; startTime = millis(); }
  void setLink(NetLink* l) { link = l; }
  bool notify(uint8_t event, const char* payload = 0);
  bool connect(bool quick = false);
  bool transmit(const char* packet, unsigned int size);
  bool ping();
  void inbound();
  void onMessage(char* msg, UdpRx* rx);
  void shutdown();
  NetLink* link = nullptr;
  uint32_t txCount = 0, txBytes = 0, rxBytes = 0;
  uint32_t lastSyncTime = 0, startTime = 0;
  uint16_t feedid = 0;
  bool login = false;
private:
  char m_rx[RECV_BUF_SIZE];
};

extern char devid[12];
void genDeviceID(char* buf);
void logNetEvent(const char* text);   // main.cpp: an "FE,NET ..." line in the SD data file
void otaReadyRequested();             // telemetry.cpp: server command OTA_READY
void telemetryApnCommand(const char* arg);   // telemetry.cpp: server APN= (nbapn.h)
