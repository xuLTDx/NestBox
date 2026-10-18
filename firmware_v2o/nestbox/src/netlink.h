// One datagram link to the server - NestBox_v2O. WiFi now; the SIM7600 modem
// (step 2) gets its own implementation of the same calls, so the UDP client
// (teleclient.cpp) does not care which link is up - as in v1F, where
// TeleClientUDP held a WifiUDP and a CellUDP with these calls.
#pragma once
#include <Arduino.h>

class NetLink {
public:
  virtual ~NetLink() {}
  virtual const char* name() = 0;
  virtual bool up() = 0;                                   // the link itself (WiFi joined / modem registered)
  virtual bool open(const char* host, uint16_t port) = 0;  // the server's address for send()
  virtual bool send(const char* data, unsigned int len) = 0;
  virtual int receive(char* buf, int cap, unsigned int timeoutMs) = 0;  // 0 = nothing
  virtual void close() = 0;
};

// WiFi: Arduino WiFiUDP (v1F libraries/FreematicsPlus WifiUDP did the same,
// except that its open() sent an empty datagram to resolve the host - here
// the host is resolved with DNS and nothing is sent)
class WifiLink : public NetLink {
public:
  const char* name() override { return "WIFI"; }
  bool up() override;
  bool open(const char* host, uint16_t port) override;
  bool send(const char* data, unsigned int len) override;
  int receive(char* buf, int cap, unsigned int timeoutMs) override;
  void close() override;
};
