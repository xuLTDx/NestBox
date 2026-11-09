// netlog.h - the serial log over the network (PG99)
#include "netlog.h"
#include <esp_log.h>
#include <stdarg.h>

static char* s_ring;
static volatile uint32_t s_head;              // bytes ever written; ring index = s_head % NETLOG_SIZE
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
  s_ring = (char*)ps_malloc(NETLOG_SIZE);
  if (!s_ring) return;                        // no PSRAM: no network log (never on this board)
  s_prev = esp_log_set_vprintf(hook);
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
