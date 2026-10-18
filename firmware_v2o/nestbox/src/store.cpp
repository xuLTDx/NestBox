// see store.h - the bodies are NestBox_v1F telestore.cpp
#include "store.h"
#include "nb_sd.h"
#include "sdwriter.h"

void CStorage::log(uint16_t pid, uint8_t values[], uint8_t count)
{
  char buf[256];
  byte n = snprintf(buf, sizeof(buf), "%X%c%u", pid, m_delimiter, (unsigned int)values[0]);
  for (byte m = 1; m < count; m++) n += snprintf(buf + n, sizeof(buf) - n, ";%u", (unsigned int)values[m]);
  dispatch(buf, n);
}

void CStorage::log(uint16_t pid, uint16_t values[], uint8_t count)
{
  char buf[256];
  byte n = snprintf(buf, sizeof(buf), "%X%c%u", pid, m_delimiter, (unsigned int)values[0]);
  for (byte m = 1; m < count; m++) n += snprintf(buf + n, sizeof(buf) - n, ";%u", (unsigned int)values[m]);
  dispatch(buf, n);
}

void CStorage::log(uint16_t pid, uint32_t values[], uint8_t count)
{
  char buf[256];
  byte n = snprintf(buf, sizeof(buf), "%X%c%u", pid, m_delimiter, (unsigned)values[0]);
  for (byte m = 1; m < count; m++) n += snprintf(buf + n, sizeof(buf) - n, ";%u", (unsigned)values[m]);
  dispatch(buf, n);
}

void CStorage::log(uint16_t pid, int32_t values[], uint8_t count)
{
  char buf[256];
  byte n = snprintf(buf, sizeof(buf), "%X%c%d", pid, m_delimiter, (int)values[0]);
  for (byte m = 1; m < count; m++) n += snprintf(buf + n, sizeof(buf) - n, ";%d", (int)values[m]);
  dispatch(buf, n);
}

void CStorage::log(uint16_t pid, float values[], uint8_t count, const char* fmt)
{
  char buf[256];
  char* p = buf + snprintf(buf, sizeof(buf), "%X%c", pid, m_delimiter);
  for (byte m = 0; m < count && (p - buf) < (int)sizeof(buf) - 3; m++) {
    if (m > 0) *(p++) = ';';
    int l = snprintf(p, sizeof(buf) - (p - buf), fmt, values[m]);
    char* q = strchr(p, '.');
    if (q && atoi(q + 1) == 0) {
      *q = 0;
      if (*p == '-' && *(p + 1) == '0') {
        *p = '0';
        *(++p) = 0;
      } else {
        p = q;
      }
    } else {
      p += l;
    }
  }
  dispatch(buf, (int)(p - buf));
}

void CStorage::timestamp(uint32_t ts) { log(PID_TIMESTAMP, &ts, 1); }

byte CStorage::checksum(const char* data, int len)
{
  byte sum = 0;
  for (int i = 0; i < len; i++) sum += data[i];
  return sum;
}

void CStorageRAM::dispatch(const char* buf, byte len)
{
  int remain = m_cacheSize - m_cacheBytes - len - 3;  // room for the checksum
  if (remain < 0) return;
  memcpy(m_cache + m_cacheBytes, buf, len);
  m_cacheBytes += len;
  m_cache[m_cacheBytes++] = ',';
}

void CStorageRAM::header(const char* devid) { m_cacheBytes = sprintf(m_cache, "%s#", devid); }

void CStorageRAM::tailer()
{
  if (m_cache[m_cacheBytes - 1] == ',') m_cacheBytes--;
  m_cacheBytes += sprintf(m_cache + m_cacheBytes, "*%X", (unsigned int)checksum(m_cache, m_cacheBytes));
}

static SemaphoreHandle_t s_sdMutex = 0;

void sdLock()
{
  if (!s_sdMutex) s_sdMutex = xSemaphoreCreateRecursiveMutex();
  if (s_sdMutex) xSemaphoreTakeRecursive(s_sdMutex, portMAX_DELAY);
}

void sdUnlock()
{
  if (s_sdMutex) xSemaphoreGiveRecursive(s_sdMutex);
}

uint32_t DataLog::begin(uint32_t lastId)
{
  char path[24];
  sdLock();
  if (!SD.exists("/DATA")) SD.mkdir("/DATA");
  m_id = lastId + 1;                         // v1F: listing /DATA took 14 s of every boot
  for (;;) {
    sprintf(path, "/DATA/%u.CSV", (unsigned)m_id);
    if (!SD.exists(path)) break;
    m_id++;
  }
  sdUnlock();                                // the SD writer takes the lock itself
  sprintf(path, "/DATA/%u.CSV", (unsigned)m_id);
  Serial.printf("[SD] data file %s\n", path);
  if (!sdwDataOpen(path)) {
    Serial.println("[SD] data file error");
    m_id = 0;
  }
  return m_id;
}

void DataLog::dispatch(const char* buf, byte len)
{
  if (m_id == 0) return;
  char line[258];
  memcpy(line, buf, len);
  line[len] = '\n';
  sdwDataWrite(line, len + 1);
}

void DataLog::logEvent(const char* text)
{
  char buf[250];
  int n = snprintf(buf, sizeof(buf), "FE,%s", text);
  if (n > 0 && n < (int)sizeof(buf)) dispatch(buf, (byte)n);
}

bool DataLog::sync(uint32_t timeoutMs) { return m_id == 0 || sdwDataSync(timeoutMs); }

void DataLog::end()
{
  if (m_id) sdwDataClose(5000);
  m_id = 0;
}
