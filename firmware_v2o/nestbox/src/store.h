// Records as text - NestBox_v2O. From NestBox_v1F telestore.h/.cpp:
//   CStorage / CStorageRAM - "<pid hex>:<value>" fields, the wire packet
//     "<devid>#...*<checksum>" (header/tailer) the server expects;
//   DataLog - the SD copy /DATA/<n>.CSV through the SD writer (sdwriter.cpp),
//     one "<pid>,<value>" line per value, "FE,<text>" event lines
//     (v1F SDLogger; the file logic unchanged, the class trimmed to what is used);
//   sdLock() - the one recursive SD lock every SD user takes.
// PID numbers: libraries/FreematicsPlus/FreematicsBase.h (the protocol's
// numbering, shared with FleetNest apps/telemetry/pids.py).
#pragma once
#include <Arduino.h>

#define PID_TIMESTAMP 0
#define PID_GPS_LATITUDE 0xA
#define PID_GPS_LONGITUDE 0xB
#define PID_GPS_ALTITUDE 0xC
#define PID_GPS_SPEED 0xD
#define PID_GPS_HEADING 0xE
#define PID_GPS_SAT_COUNT 0xF
#define PID_GPS_TIME 0x10
#define PID_GPS_DATE 0x11
#define PID_GPS_HDOP 0x12
#define PID_BATTERY_VOLTAGE 0x24
#define PID_CSQ 0x81
#define PID_DEVICE_TEMP 0x82

void sdLock();
void sdUnlock();

class CStorage {
public:
  virtual void log(uint16_t pid, uint8_t values[], uint8_t count);
  virtual void log(uint16_t pid, uint16_t values[], uint8_t count);
  virtual void log(uint16_t pid, uint32_t values[], uint8_t count);
  virtual void log(uint16_t pid, int32_t values[], uint8_t count);
  virtual void log(uint16_t pid, float values[], uint8_t count, const char* fmt = "%f");
  virtual void timestamp(uint32_t ts);
  virtual void dispatch(const char* buf, byte len) = 0;
protected:
  byte checksum(const char* data, int len);
  char m_delimiter = ':';
};

class CStorageRAM : public CStorage {
public:
  void init(char* cache, unsigned int cacheSize) { m_cacheSize = cacheSize; m_cache = cache; m_cacheBytes = 0; }
  void purge() { m_cacheBytes = 0; }
  unsigned int length() { return m_cacheBytes; }
  char* buffer() { return m_cache; }
  void dispatch(const char* buf, byte len) override;
  void header(const char* devid);
  void tailer();
protected:
  unsigned int m_cacheSize = 0;
  unsigned int m_cacheBytes = 0;
  char* m_cache = 0;
};

class DataLog : public CStorage {
public:
  DataLog() { m_delimiter = ','; }
  uint32_t begin(uint32_t lastId);     // a new /DATA/<n>.CSV (lastId from NVS: no directory listing)
  void dispatch(const char* buf, byte len) override;
  void logEvent(const char* text);     // "FE,<text>"
  bool sync(uint32_t timeoutMs);
  void end();
  uint32_t id() { return m_id; }
private:
  uint32_t m_id = 0;
};
