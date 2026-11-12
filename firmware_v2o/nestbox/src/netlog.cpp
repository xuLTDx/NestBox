// netlog.h - the serial log over the network (PG99)
#include "netlog.h"
#include <esp_log.h>
#include <stdarg.h>

// PG175 (12.11.2026): the ring keeps its content over a panic / watchdog restart. Why: a3 bench 12.11. 16:47 PANIC
// with nothing of what happened before (the ring was new after the restart). The ring sits right after the delivery
// queue's PSRAM reserve (netlogBegin() is the 2nd PSRAM allocation in setup(), main.cpp) - the same address every
// boot, and the Arduino PSRAM test spares it (txqueue.cpp testSPIRAM: only the top 64 KB). A header in PSRAM says
// whether the content is from the boot before (magic + its own address); the head counter lives in it.
#define RING_MAGIC 0x4E4C4F47u
struct RingHdr { uint32_t magic, addr; volatile uint32_t head; };
static RingHdr* s_hdr;
static char* s_ring;
#define s_head (s_hdr->head)                  // bytes ever written; ring index = s_head % NETLOG_SIZE
static char* s_prev64;                        // the boot before's last NETLOG_SIZE bytes (after a crash), or null
static uint32_t s_prevLen;
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;
static vprintf_like_t s_prev;

static void add(const char* s, int n)
{
  if (!s_ring || n <= 0) return;
  portENTER_CRITICAL(&s_mux);
  for (int i = 0; i < n; i++) s_ring[(s_head + i) % NETLOG_SIZE] = s[i];
  s_head += n;
  portEXIT_CRITICAL(&s_mux);
}

// every ESP_LOGx: a copy into the ring, then on to the UART as before
static int hook(const char* fmt, va_list ap)
{
  char t[256];
  va_list c;
  va_copy(c, ap);
  int n = vsnprintf(t, sizeof(t), fmt, c);
  va_end(c);
  if (n > (int)sizeof(t) - 1) n = sizeof(t) - 1;
  add(t, n);
  return s_prev ? s_prev(fmt, ap) : vprintf(fmt, ap);
}

void netlogBegin()
{
  if (s_ring) return;
  uint8_t* m = (uint8_t*)ps_malloc(sizeof(RingHdr) + NETLOG_SIZE);
  if (!m) return;                             // no PSRAM: no network log (never on this board)
  RingHdr* h = (RingHdr*)m;
  char* ring = (char*)(m + sizeof(RingHdr));
  esp_reset_reason_t r = esp_reset_reason();
  bool crash = r == ESP_RST_PANIC || r == ESP_RST_TASK_WDT || r == ESP_RST_INT_WDT || r == ESP_RST_WDT || r == ESP_RST_SW;
  if (crash && h->magic == RING_MAGIC && h->addr == (uint32_t)m && h->head) {
    uint32_t n = h->head < NETLOG_SIZE ? h->head : NETLOG_SIZE;
    s_prev64 = (char*)ps_malloc(n);
    if (s_prev64) {
      for (uint32_t i = 0; i < n; i++) s_prev64[i] = ring[(h->head - n + i) % NETLOG_SIZE];
      s_prevLen = n;
    }
  }
  h->magic = RING_MAGIC;
  h->addr = (uint32_t)m;
  h->head = 0;
  s_hdr = h;
  s_ring = ring;
  s_prev = esp_log_set_vprintf(hook);
}

// PG175: the boot before's log (after a panic / watchdog / software restart), taken once by dbglog.cpp
bool netlogTakePrev(char** buf, uint32_t* len)
{
  if (!s_prev64) return false;
  *buf = s_prev64;
  *len = s_prevLen;
  s_prev64 = nullptr;
  return true;
}
uint32_t netlogHead() { return s_ring ? s_head : 0; }

// PG175: raw bytes from *from (no header line, as netlogRead has); bytes the ring overwrote meanwhile -> *lost
int netlogCopy(uint32_t* from, char* out, int cap, uint32_t* lost)
{
  if (!s_ring) return 0;
  portENTER_CRITICAL(&s_mux);
  uint32_t head = s_head;
  portEXIT_CRITICAL(&s_mux);
  uint32_t oldest = head > NETLOG_SIZE ? head - NETLOG_SIZE : 0;
  if (*from > head) *from = head;            // a new boot's ring (never here: the task starts after netlogBegin)
  if (*from < oldest) { *lost += oldest - *from; *from = oldest; }
  uint32_t n = head - *from;
  if ((int)n > cap) n = cap;
  for (uint32_t i = 0; i < n; i++) out[i] = s_ring[(*from + i) % NETLOG_SIZE];
  *from += n;
  return (int)n;
}

void netlogLine(const char* text)
{
  char t[200];
  int n = snprintf(t, sizeof(t), "(%lu) %s\n", (unsigned long)millis(), text);
  if (n > (int)sizeof(t) - 1) n = sizeof(t) - 1;
  add(t, n);
}

int netlogRead(uint32_t from, char* out, int cap)
{
  if (!s_ring) return snprintf(out, cap, "no netlog\n");
  portENTER_CRITICAL(&s_mux);
  uint32_t head = s_head;
  portEXIT_CRITICAL(&s_mux);
  uint32_t oldest = head > NETLOG_SIZE ? head - NETLOG_SIZE : 0;
  if (from < oldest || from > head) from = from > head ? head : oldest;
  int hdr = snprintf(out, cap, "head=%lu from=%lu ", (unsigned long)head, (unsigned long)from);
  int room = cap - hdr - 24;
  uint32_t n = head - from;
  if ((int)n > room) n = room > 0 ? room : 0;
  hdr += snprintf(out + hdr, cap - hdr, "next=%lu\n", (unsigned long)(from + n));
  for (uint32_t i = 0; i < n; i++) out[hdr + i] = s_ring[(from + i) % NETLOG_SIZE];
  return hdr + n;
}

// ------------------------------------------------------------------ PG101 ---
NbOut nbout;
static volatile bool s_lineStart = true;

static void addTimed(const uint8_t* b, size_t n)
{
  size_t i = 0;
  while (i < n) {
    if (s_lineStart) {
      char t[16];
      int k = snprintf(t, sizeof(t), "(%lu) ", (unsigned long)millis());
      add(t, k);
      s_lineStart = false;
    }
    size_t j = i;
    while (j < n && b[j] != '\n') j++;
    if (j < n) { add((const char*)b + i, j - i + 1); s_lineStart = true; i = j + 1; }
    else { add((const char*)b + i, n - i); i = n; }
  }
}

size_t NbOut::write(uint8_t c) { addTimed(&c, 1); return Serial.write(c); }
size_t NbOut::write(const uint8_t* b, size_t n) { addTimed(b, n); return Serial.write(b, n); }
