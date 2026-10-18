// Sniff mode - see sniff.h. Carried over from NestBox_v1F sniff.cpp (queue,
// line format, stats, SD writer hand-off, 60-day purge, auto-off); new here:
// three buses (TWAI + 2x MCP2515), a bus column, per-bus speed probe and stats.
#include "sniff.h"
#include <SD_MMC.h>
#include <Preferences.h>
#include "driver/twai.h"
#include "esp_timer.h"
#include "sdwriter.h"
#include "mcp2515.h"
#include "max7317.h"
#include "hw.h"

extern const char FW_BUILD_STR[];
uint32_t bootId();
void sdLock();
void sdUnlock();

#define SNIFF_DIR "/SNIFF"
#define LIVE_FRAMES 200
// receive tasks -> SD writer (PSRAM): 65536 frames = ~1.5 MB; with ~1580 frames/s
// on Peugeot 6/14 (2026-10-12) that is ~40 s of a stalled card
static const uint32_t QUEUE_TRY[] = {65536, 32768, 8192};
#define SPEED_PROBE_MS 2000
#define STATS_EVERY_MS 60000
#define NVS_SAVE_EVERY_MIN 5

struct Frame {
  int64_t us;
  uint32_t id;
  uint8_t bus, ext, dlc;
  uint8_t data[8];
};

static Frame* s_queue;
static uint32_t s_queueCap;
static volatile uint32_t s_qHead, s_qTail;
static volatile uint32_t s_dropped;           // queue full: lost before SD
static portMUX_TYPE s_qMux = portMUX_INITIALIZER_UNLOCKED;
static Frame s_live[LIVE_FRAMES];
static uint32_t s_liveHead, s_liveCount;
static volatile uint32_t s_frames;            // all buses

// per bus (index 0 = CAN1)
static const uint32_t s_speeds[] = {500, 250, 125};
struct BusState {
  bool present;
  bool on;                                    // taking part in this session
  int speedIdx;
  bool speedFound;
  uint32_t probeStart, probeFrames;
  volatile uint32_t frames;
};
static BusState s_bus[SNIFF_BUSES];
static uint8_t s_busMask = 0x07;
static Mcp2515 s_mcp1(2, HW_MCP1_CS, HW_MCP1_INT), s_mcp2(3, HW_MCP2_CS, HW_MCP2_INT);
static Mcp2515* mcpFor(int i) { return i == 1 ? &s_mcp1 : i == 2 ? &s_mcp2 : nullptr; }

static volatile bool s_twaiRunning, s_pauseRx, s_rxPaused;
static SemaphoreHandle_t s_canMux;

static volatile bool s_active;
static volatile int8_t s_req = -1;
static volatile uint16_t s_reqMin;
static uint16_t s_minLeft;
static uint32_t s_minuteMs;
static uint8_t s_sinceSave;

static volatile bool s_fileOk;
static uint32_t s_fileNo;
static char s_path[32];
static volatile uint32_t s_written, s_sdErrors, s_sdLost;
static uint32_t s_lastStats;
static volatile bool s_closing;
static const char* volatile s_closeTag = "# end";
static String s_vehicle = "UNSPECIFIED";
static int s_sdLimitPct = SNIFF_SD_LIMIT_PCT;
static volatile bool s_refused;
static bool s_tasksMade;

static Preferences s_prefs;                  // namespace "sniff"

// ---------------------------------------------------------------- queue ----
static void pushFrame(const Frame& f)
{
  portENTER_CRITICAL(&s_qMux);
  uint32_t next = (s_qHead + 1) % s_queueCap;
  if (next == s_qTail) {
    s_dropped++;
  } else {
    s_queue[s_qHead] = f;
    s_qHead = next;
  }
  s_live[s_liveHead] = f;
  s_liveHead = (s_liveHead + 1) % LIVE_FRAMES;
  if (s_liveCount < LIVE_FRAMES) s_liveCount++;
  s_frames++;
  s_bus[f.bus - 1].frames++;
  portEXIT_CRITICAL(&s_qMux);
}

// MCP2515 receive tasks call this (bus 2 / 3)
static void onMcpFrame(uint8_t bus, const CanRx& r)
{
  if (!s_active || !s_queue) return;
  Frame f = {};
  f.us = r.us;
  f.id = r.id;
  f.bus = bus;
  f.ext = r.ext;
  f.dlc = r.dlc;
  memcpy(f.data, r.data, 8);
  pushFrame(f);
}

// ----------------------------------------------------------- CAN1 TWAI ----
static twai_timing_config_t timingFor(uint32_t kbit)
{
  switch (kbit) {
    case 250: return TWAI_TIMING_CONFIG_250KBITS();
    case 125: return TWAI_TIMING_CONFIG_125KBITS();
    default:  return TWAI_TIMING_CONFIG_500KBITS();
  }
}

static bool twaiStart()
{
  mx7317Out(MX_CAN1_RS, 1);                   // transceiver transmitter off (listen only)
  twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)HW_CAN1_TX, (gpio_num_t)HW_CAN1_RX,
                                                        TWAI_MODE_LISTEN_ONLY);
  g.rx_queue_len = 128;
  g.tx_queue_len = 0;
  twai_timing_config_t t = timingFor(s_speeds[s_bus[0].speedIdx]);
  twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  if (twai_driver_install(&g, &t, &f) != ESP_OK) return false;
  if (twai_start() != ESP_OK) { twai_driver_uninstall(); return false; }
  s_twaiRunning = true;
  return true;
}

static void twaiStop()
{
  if (!s_twaiRunning) return;
  s_twaiRunning = false;
  twai_stop();
  twai_driver_uninstall();
}

static void twaiRxTask(void*)
{
  for (;;) {
    if (s_pauseRx || !s_twaiRunning) {
      s_rxPaused = true;
      vTaskDelay(pdMS_TO_TICKS(5));
      continue;
    }
    s_rxPaused = false;
    twai_message_t m;
    if (twai_receive(&m, pdMS_TO_TICKS(20)) != ESP_OK) continue;
    Frame f = {};
    f.us = esp_timer_get_time();
    f.id = m.identifier;
    f.bus = 1;
    f.ext = m.extd;
    f.dlc = m.data_length_code > 8 ? 8 : m.data_length_code;
    memcpy(f.data, m.data, f.dlc);
    pushFrame(f);
  }
}

static void pauseRx()
{
  s_pauseRx = true;
  for (int i = 0; i < 100 && !s_rxPaused; i++) delay(2);
}

#ifdef TEST_SNIFF_LOAD
// bench: synthetic frames at ~TEST_SNIFF_LOAD per second on "bus 1" while the
// mode is on (no car needed). ID 0x7AA, data = 64-bit sequence number; the box
// checks the file itself with SNIFF_CHECK=<n>.
static void loadTask(void*)
{
  static uint64_t seq = 0;
  const int perTick = TEST_SNIFF_LOAD / 500;
  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(2));
    if (!s_active) continue;
    for (int i = 0; i < perTick; i++) {
      Frame f = {};
      f.us = esp_timer_get_time();
      f.id = 0x7AA;
      f.bus = 1;
      f.dlc = 8;
      memcpy(f.data, &seq, 8);
      seq++;
      pushFrame(f);
    }
  }
}
#endif

// --------------------------------------------------------------- buses ----
static bool busStart(int i)
{
  if (i == 0) return twaiStart();
  return mcpFor(i)->start(s_speeds[s_bus[i].speedIdx]);
}

static void busStop(int i)
{
  if (i == 0) twaiStop();
  else mcpFor(i)->stop();
}

// ----------------------------------------------------------------- SD ----
static int formatFrame(char* out, int cap, const Frame& f)
{
  int n = snprintf(out, cap, "%lu.%03lu,%u,%lX,%c,%u,", (unsigned long)(f.us / 1000),
                   (unsigned long)(f.us % 1000), f.bus, (unsigned long)f.id, f.ext ? 'x' : 's', f.dlc);
  for (int i = 0; i < f.dlc && n < cap - 4; i++) n += snprintf(out + n, cap - n, i ? " %02X" : "%02X", f.data[i]);
  return n;
}

static char s_text[2048];
static int s_textLen;
static portMUX_TYPE s_tMux = portMUX_INITIALIZER_UNLOCKED;

static void addLine(const char* line)
{
  int n = strlen(line);
  portENTER_CRITICAL(&s_tMux);
  if (s_textLen + n + 1 <= (int)sizeof(s_text)) {
    memcpy(s_text + s_textLen, line, n);
    s_textLen += n;
    s_text[s_textLen++] = '\n';
  }
  portEXIT_CRITICAL(&s_tMux);
}

// one stats line for the whole box, then per bus:
//   frames = received on all buses; written + sd_lost + dropped + queued = frames
//   bus<n>: f = frames, kbit, then the bus's own losses
static void statsLine(char* line, int cap)
{
  int n = snprintf(line, cap, "# stats t=%lu unix=%lu frames=%lu written=%lu dropped=%lu sd_err=%lu sd_lost=%lu",
                   (unsigned long)millis(), (unsigned long)time(nullptr), (unsigned long)s_frames,
                   (unsigned long)s_written, (unsigned long)s_dropped, (unsigned long)s_sdErrors,
                   (unsigned long)s_sdLost);
  for (int i = 0; i < SNIFF_BUSES && n < cap - 80; i++) {
    const BusState& b = s_bus[i];
    if (!b.on) continue;
    n += snprintf(line + n, cap - n, " | can%d f=%lu %lu%s", i + 1, (unsigned long)b.frames,
                  (unsigned long)s_speeds[b.speedIdx], b.speedFound ? "k" : "k?");
    if (i == 0) {
      twai_status_info_t st = {};
      if (s_twaiRunning) twai_get_status_info(&st);
      n += snprintf(line + n, cap - n, " rx_missed=%lu rx_overrun=%lu bus_err=%lu",
                    (unsigned long)st.rx_missed_count, (unsigned long)st.rx_overrun_count,
                    (unsigned long)st.bus_error_count);
    } else {
      McpStats st = mcpFor(i)->stats();
      n += snprintf(line + n, cap - n, " rx_overflow=%lu err_irq=%lu eflg=%02X rec=%u",
                    (unsigned long)st.rxOverflow, (unsigned long)st.errIrq, st.eflg, st.rec);
    }
  }
}

int sniffFill(char* buf, int cap, uint32_t* frames, bool* last)
{
  int n = 0;
  *frames = 0;
  *last = false;
  portENTER_CRITICAL(&s_tMux);
  if (s_textLen && s_textLen <= cap) {
    memcpy(buf, s_text, s_textLen);
    n = s_textLen;
    s_textLen = 0;
  }
  portEXIT_CRITICAL(&s_tMux);
  char line[400];
  if (millis() - s_lastStats >= STATS_EVERY_MS && n + (int)sizeof(line) < cap) {
    s_lastStats = millis();
    statsLine(line, sizeof(line));
    Serial.println(line);
    int l = strlen(line);
    memcpy(buf + n, line, l);
    n += l;
    buf[n++] = '\n';
  }
  while (s_qTail != s_qHead && n + 100 < cap) {
    Frame f = s_queue[s_qTail];
    s_qTail = (s_qTail + 1) % s_queueCap;
    n += formatFrame(buf + n, cap - n, f);
    buf[n++] = '\n';
    (*frames)++;
  }
  if (s_closing && s_qTail == s_qHead && n + 2 * (int)sizeof(line) < cap) {
    s_written += *frames;
    statsLine(line, sizeof(line));
    s_written -= *frames;
    int l = strlen(line);
    memcpy(buf + n, line, l);
    n += l;
    buf[n++] = '\n';
    const char* e = s_closeTag;
    l = strlen(e);
    memcpy(buf + n, e, l);
    n += l;
    buf[n++] = '\n';
    *last = true;
  }
  return n;
}

void sniffConfirmed(uint32_t frames) { s_written += frames; }

void sniffNoteSdError(const char* op, int err)
{
  s_sdErrors++;
  char line[64];
  snprintf(line, sizeof(line), "# sderr t=%lu op=%s errno=%d", (unsigned long)millis(), op, err);
  addLine(line);
}

void sniffSdDead(uint32_t lostFrames)
{
  uint32_t h = s_qHead;
  s_sdLost += lostFrames + (h + s_queueCap - s_qTail) % s_queueCap;
  s_qTail = h;
  s_fileOk = false;
  s_closing = false;
  s_req = 0;
}

static int sdUsedPct()
{
  sdLock();
  uint64_t total = SD_MMC.totalBytes(), used = SD_MMC.usedBytes();
  sdUnlock();
  return total ? (int)(used * 100 / total) : 100;
}

// new session file /SNIFF/S<n>.LOG, n above every file already there (never overwrite)
static void openFile()
{
  uint32_t n = s_prefs.getUInt("no", 0);
  char path[32];
  sdLock();
  if (!SD_MMC.exists(SNIFF_DIR)) SD_MMC.mkdir(SNIFF_DIR);
  File dir = SD_MMC.open(SNIFF_DIR);
  File f;
  while (dir && (f = dir.openNextFile())) {
    const char* fn = f.name();
    const char* p = strrchr(fn, '/');
    if (p) fn = p + 1;
    if (fn[0] == 'S' && (uint32_t)atoi(fn + 1) > n) n = atoi(fn + 1);
    f.close();
  }
  if (dir) dir.close();
  sdUnlock();
  n++;
  s_prefs.putUInt("no", n);
  s_fileNo = n;
  snprintf(path, sizeof(path), SNIFF_DIR "/S%lu.LOG", (unsigned long)n);
  strcpy(s_path, path);
  s_written = s_sdErrors = s_sdLost = 0;
  portENTER_CRITICAL(&s_tMux);
  s_textLen = 0;
  portEXIT_CRITICAL(&s_tMux);
  time_t now = time(nullptr);
  char line[200];
  char buses[16] = "";
  for (int i = 0; i < SNIFF_BUSES; i++)
    if (s_bus[i].on) snprintf(buses + strlen(buses), sizeof(buses) - strlen(buses), "%s%d", buses[0] ? "+" : "", i + 1);
  snprintf(line, sizeof(line), "# NestBox_v2O sniff (listen-only) buses=%s vehicle=%s fw=%s boot=%lu",
           buses, s_vehicle.c_str(), FW_BUILD_STR, (unsigned long)bootId());
  addLine(line);
  snprintf(line, sizeof(line), "# clock unix=%lu at_ms=%lu", (unsigned long)(now > 1735689600 ? now : 0),
           (unsigned long)(esp_timer_get_time() / 1000));
  addLine(line);
  addLine("# format: <ms>.<us>,<bus>,<ID hex>,<s=11-bit|x=29-bit>,<dlc>,<data hex>");
  s_fileOk = sdwSniffOpen(path);
  Serial.printf("[SNIFF] file %s %s\n", path, s_fileOk ? "open" : "FAILED");
}

static void closeFile(const char* tag)
{
  if (!s_fileOk) return;
  s_closeTag = tag;
  s_closing = true;
  sdwSniffClose(5000);
  if (!sdwSniffOpenNow()) {
    s_fileOk = false;
    s_closing = false;
  } else {
    Serial.println("[SNIFF] file not closed within 5 s (writer still busy)");
  }
}

// files older than SNIFF_KEEP_DAYS by their "# clock unix=" line; never a file without a clock
int sniffPurgeOld(time_t now)
{
  if (now < 1735689600) return 0;
  int removed = 0;
  char names[16][16];
  int nNames = 0;
  sdLock();
  File dir = SD_MMC.open(SNIFF_DIR);
  File f;
  while (dir && nNames < 16 && (f = dir.openNextFile())) {
    const char* fn = f.name();
    const char* p = strrchr(fn, '/');
    if (p) fn = p + 1;
    uint32_t no = fn[0] == 'S' ? (uint32_t)atoi(fn + 1) : 0;
    if (no && !(s_fileOk && no == s_fileNo)) {
      char head[400];
      int l = f.read((uint8_t*)head, sizeof(head) - 1);
      head[l > 0 ? l : 0] = 0;
      const char* c = strstr(head, "# clock unix=");
      unsigned long unix = c ? strtoul(c + 13, nullptr, 10) : 0;
      if (unix > 1735689600 && (unsigned long)now > unix && (unsigned long)now - unix > SNIFF_KEEP_DAYS * 86400UL)
        snprintf(names[nNames++], sizeof(names[0]), "%s", fn);
    }
    f.close();
  }
  if (dir) dir.close();
  for (int i = 0; i < nNames; i++) {
    char path[32];
    snprintf(path, sizeof(path), SNIFF_DIR "/%s", names[i]);
    if (SD_MMC.remove(path)) {
      removed++;
      Serial.printf("[SNIFF] removed %s (older than %u days)\n", path, (unsigned)SNIFF_KEEP_DAYS);
    }
  }
  sdUnlock();
  return removed;
}

// ------------------------------------------------------------- control ---
static void saveMinutes()
{
  s_prefs.putUShort("min", s_active ? s_minLeft : 0);
  s_sinceSave = 0;
}

static void makeTasks()
{
  if (s_tasksMade) return;
  s_tasksMade = true;
  for (uint32_t cap : QUEUE_TRY) {
    s_queue = (Frame*)ps_malloc(sizeof(Frame) * cap);
    if (s_queue) { s_queueCap = cap; break; }
  }
  if (!s_queue) {
    s_queue = (Frame*)malloc(sizeof(Frame) * 512);
    s_queueCap = 512;
  }
  Serial.printf("[SNIFF] queue %lu frames\n", (unsigned long)s_queueCap);
  xTaskCreatePinnedToCore(twaiRxTask, "can1rx", 3072, nullptr, 10, nullptr, 0);
#ifdef TEST_SNIFF_LOAD
  xTaskCreate(loadTask, "sniffload", 2048, nullptr, 5, nullptr);
#endif
}

static bool startNow(uint16_t minutes)
{
  int used = sdUsedPct();
  if (used >= s_sdLimitPct) {
    s_refused = true;
    Serial.printf("[SNIFF] NOT started: SD %d %% full (limit %d %%)\n", used, s_sdLimitPct);
    return false;
  }
  s_refused = false;
  if (sdwSniffOpenNow()) {
    sdwSniffClose(5000);
    if (sdwSniffOpenNow()) return false;
  }
  makeTasks();
  s_qHead = s_qTail = 0;
  s_frames = s_dropped = 0;
  s_liveCount = s_liveHead = 0;
  for (int i = 0; i < SNIFF_BUSES; i++) {
    BusState& b = s_bus[i];
    b.on = b.present && (s_busMask & (1 << i));
    b.speedIdx = 0;
    b.speedFound = false;
    b.frames = 0;
  }
  s_closing = false;
  s_lastStats = millis();
  openFile();
  s_active = true;                            // before the buses: the MCP tasks push only while active
  xSemaphoreTake(s_canMux, portMAX_DELAY);
  for (int i = 0; i < SNIFF_BUSES; i++) {
    BusState& b = s_bus[i];
    if (!b.on) continue;
    bool ok = busStart(i);
    b.probeStart = millis();
    b.probeFrames = 0;
    Serial.printf("[SNIFF] CAN%d listen-only %s\n", i + 1, ok ? "started" : "FAILED");
  }
  xSemaphoreGive(s_canMux);
  s_minLeft = minutes;
  s_minuteMs = millis();
  saveMinutes();
  Serial.printf("[SNIFF] ON %u min\n", minutes);
  return true;
}

static void stopBuses()
{
  xSemaphoreTake(s_canMux, portMAX_DELAY);
  pauseRx();
  for (int i = 0; i < SNIFF_BUSES; i++) if (s_bus[i].on) busStop(i);
  s_pauseRx = false;
  xSemaphoreGive(s_canMux);
}

void sniffStopNow()
{
  if (!s_active) return;
  stopBuses();
  s_active = false;
  closeFile("# end");
  saveMinutes();
  Serial.println("[SNIFF] OFF");
}

void sniffCloseForRestart()
{
  if (!s_active) return;
  saveMinutes();                              // the exact minutes left
  stopBuses();
  s_active = false;
  closeFile("# restart");
  Serial.println("[SNIFF] closed for restart");
}

void sniffSetSdLimit(int pct) { s_sdLimitPct = pct; }
void sniffSetBuses(uint8_t mask) { s_busMask = mask & 0x07; s_prefs.putUChar("buses", s_busMask); }
uint8_t sniffBuses() { return s_busMask; }

// per bus: no frame within SPEED_PROBE_MS -> the next speed
static void speedTick()
{
  for (int i = 0; i < SNIFF_BUSES; i++) {
    BusState& b = s_bus[i];
    if (!b.on || b.speedFound) continue;
    if (b.frames > b.probeFrames) {
      b.speedFound = true;
      char line[64];
      snprintf(line, sizeof(line), "# can%d speed %lu kbit/s", i + 1, (unsigned long)s_speeds[b.speedIdx]);
      addLine(line);
      Serial.println(line);
      continue;
    }
    if (millis() - b.probeStart < SPEED_PROBE_MS) continue;
    xSemaphoreTake(s_canMux, portMAX_DELAY);
    if (i == 0) pauseRx();
    busStop(i);
    b.speedIdx = (b.speedIdx + 1) % (sizeof(s_speeds) / sizeof(s_speeds[0]));
    busStart(i);
    b.probeFrames = b.frames;
    b.probeStart = millis();
    s_pauseRx = false;
    xSemaphoreGive(s_canMux);
  }
}

bool sniffInit()
{
  s_prefs.begin("sniff", false);
  s_vehicle = s_prefs.getString("vehicle", s_vehicle);
  s_busMask = s_prefs.getUChar("buses", 0x07);
  s_canMux = xSemaphoreCreateMutex();
  s_bus[0].present = true;                    // TWAI is inside the ESP32
  s_bus[1].present = s_mcp1.begin(onMcpFrame);
  s_bus[2].present = s_mcp2.begin(onMcpFrame);
  Serial.printf("[SNIFF] buses: CAN1 yes, CAN2 %s, CAN3 %s\n", s_bus[1].present ? "yes" : "NO",
                s_bus[2].present ? "yes" : "NO");
  uint16_t m = s_prefs.getUShort("min", 0);
  if (m) sniffRequest(true, m);
  return m != 0;
}

void sniffRequest(bool on, uint16_t minutes)
{
  if (minutes > SNIFF_MAX_MIN) minutes = SNIFF_MAX_MIN;
  s_reqMin = minutes ? minutes : SNIFF_DEFAULT_MIN;
  s_req = on ? 1 : 0;
}

int sniffTick()
{
  int ev = 0;
  int8_t req = s_req;
  if (req >= 0) {
    s_req = -1;
    if (req == 1) {
      if (s_active) {
        s_minLeft = s_reqMin;
        saveMinutes();
      } else {
        ev = startNow(s_reqMin) ? SNIFF_EV_STARTED : SNIFF_EV_REFUSED;
        if (ev == SNIFF_EV_REFUSED) saveMinutes();
      }
    } else if (s_active) {
      sniffStopNow();
      ev = SNIFF_EV_STOPPED;
    }
  }
  if (!s_active) return ev;
  speedTick();
  if (millis() - s_minuteMs >= 60000) {
    s_minuteMs += 60000;
    if (s_minLeft) s_minLeft--;
    if (!s_minLeft) {
      Serial.println("[SNIFF] time is up");
      sniffStopNow();
      return SNIFF_EV_STOPPED;
    }
    if (++s_sinceSave >= NVS_SAVE_EVERY_MIN) saveMinutes();
  }
  return ev;
}

bool sniffActive() { return s_active; }
uint16_t sniffMinutesLeft() { return s_active ? s_minLeft : 0; }
uint32_t sniffFileNo() { return s_fileNo; }

// ----------------------------------------------------------------- web ---
int sniffStatus(char* buf, int size)
{
  int n = snprintf(buf, size,
      "{\"on\":%d,\"refused\":%d,\"queue\":%lu,\"min\":%u,\"file\":%lu,\"sd\":%d,\"frames\":%lu,"
      "\"written\":%lu,\"sderr\":%lu,\"sdlost\":%lu,\"dropped\":%lu,\"vehicle\":\"%s\",\"buses\":[",
      s_active ? 1 : 0, s_refused ? 1 : 0, (unsigned long)s_queueCap, (unsigned)sniffMinutesLeft(),
      (unsigned long)s_fileNo, s_fileOk ? 1 : 0, (unsigned long)s_frames, (unsigned long)s_written,
      (unsigned long)s_sdErrors, (unsigned long)s_sdLost, (unsigned long)s_dropped, s_vehicle.c_str());
  for (int i = 0; i < SNIFF_BUSES && n < size - 160; i++) {
    const BusState& b = s_bus[i];
    n += snprintf(buf + n, size - n, "%s{\"bus\":%d,\"present\":%d,\"on\":%d,\"speed\":%lu,\"found\":%d,\"frames\":%lu",
                  i ? "," : "", i + 1, b.present ? 1 : 0, b.on ? 1 : 0, (unsigned long)s_speeds[b.speedIdx],
                  b.speedFound ? 1 : 0, (unsigned long)b.frames);
    if (i == 0) {
      twai_status_info_t st = {};
      if (s_twaiRunning) twai_get_status_info(&st);
      n += snprintf(buf + n, size - n, ",\"missed\":%lu,\"overrun\":%lu,\"buserr\":%lu}",
                    (unsigned long)st.rx_missed_count, (unsigned long)st.rx_overrun_count,
                    (unsigned long)st.bus_error_count);
    } else {
      McpStats st = mcpFor(i)->stats();
      n += snprintf(buf + n, size - n, ",\"overflow\":%lu,\"errirq\":%lu,\"eflg\":%u,\"rec\":%u}",
                    (unsigned long)st.rxOverflow, (unsigned long)st.errIrq, st.eflg, st.rec);
    }
  }
  n += snprintf(buf + n, size - n, "]}");
  return n;
}

int sniffLive(char* buf, int size, uint32_t id, uint32_t mask)
{
  int n = 0;
  char line[96];
  portENTER_CRITICAL(&s_qMux);
  uint32_t count = s_liveCount, head = s_liveHead;
  portEXIT_CRITICAL(&s_qMux);
  for (uint32_t i = 0; i < count; i++) {
    const Frame& f = s_live[(head + LIVE_FRAMES - count + i) % LIVE_FRAMES];
    if (mask && (f.id & mask) != (id & mask)) continue;
    int l = formatFrame(line, sizeof(line), f);
    if (n + l + 2 >= size) {
      int cut = n / 2;
      char* nl = (char*)memchr(buf + cut, '\n', n - cut);
      if (!nl) break;
      cut = nl + 1 - buf;
      memmove(buf, buf + cut, n - cut);
      n -= cut;
    }
    memcpy(buf + n, line, l);
    n += l;
    buf[n++] = '\n';
  }
  if (n < size) buf[n] = 0;
  return n;
}

bool sniffSetVehicle(const char* tag)
{
  String clean;
  for (const char* p = tag; *p && clean.length() < 40; p++)
    if (isalnum((unsigned char)*p) || *p == '_' || *p == '-') clean += *p;
  if (!clean.length()) return false;
  s_vehicle = clean;
  s_prefs.putString("vehicle", s_vehicle);
  if (s_active) {
    String line = "# vehicle=" + s_vehicle + " from t=" + String(millis());
    addLine(line.c_str());
  }
  return true;
}

#ifdef TEST_SNIFF_LOAD
// ------------------------------------------------------------- bench ----
// SNIFF_CHECK=<n>: read /SNIFF/S<n>.LOG on the box and check the generator's
// sequence (v1F checkTask, with the bus column): every frame line well-formed,
// the 0x7AA counter without gap or repeat, count = last stats "written".
static char s_checkResult[320] = "none";
static volatile uint32_t s_checkFile;
static volatile bool s_checkRunning;

static void checkTask(void*)
{
  char path[48];
  snprintf(path, sizeof(path), SDW_MOUNT SNIFF_DIR "/S%lu.LOG", (unsigned long)s_checkFile);
  uint32_t lines = 0, frames = 0, bad = 0, gaps = 0, repeats = 0, sderr = 0, lastWritten = 0;
  uint64_t prev = 0, firstGapAt = 0;
  bool havePrev = false, endTag = false;
  static char buf[16384];
  char line[200];
  int ll = 0;
  sdLock();
  FILE* f = fopen(path, "r");
  sdUnlock();
  if (!f) {
    snprintf(s_checkResult, sizeof(s_checkResult), "S%lu: open failed", (unsigned long)s_checkFile);
    s_checkRunning = false;
    vTaskDelete(nullptr);
  }
  for (;;) {
    sdLock();
    int n = fread(buf, 1, sizeof(buf), f);
    sdUnlock();
    if (n <= 0) break;
    for (int i = 0; i < n; i++) {
      char c = buf[i];
      if (c != '\n') { if (ll < (int)sizeof(line) - 1) line[ll++] = c; else bad++; continue; }
      line[ll] = 0;
      ll = 0;
      lines++;
      if (line[0] == '#') {
        if (!strncmp(line, "# sderr", 7)) sderr++;
        if (!strncmp(line, "# end", 5) || !strncmp(line, "# restart", 9)) endTag = true;
        const char* w = strstr(line, " written=");
        if (!strncmp(line, "# stats", 7) && w) lastWritten = strtoul(w + 9, nullptr, 10);
        continue;
      }
      // <ms>.<us>,1,7AA,s,8,XX XX XX XX XX XX XX XX
      char* p[6] = {};
      p[0] = strchr(line, ',');
      for (int k = 1; k < 5 && p[k - 1]; k++) p[k] = strchr(p[k - 1] + 1, ',');
      if (!p[4]) { bad++; continue; }
      frames++;
      if (strncmp(p[1] + 1, "7AA,", 4)) continue;
      const char* h = p[4] + 1;
      uint64_t seq = 0;
      bool ok = strlen(h) == 23;
      for (int k = 0; ok && k < 8; k++) {
        int v = 0;
        for (int m = 0; m < 2; m++) {
          char ch = h[k * 3 + m];
          v <<= 4;
          if (ch >= '0' && ch <= '9') v |= ch - '0';
          else if (ch >= 'A' && ch <= 'F') v |= ch - 'A' + 10;
          else ok = false;
        }
        if (k < 7 && h[k * 3 + 2] != ' ') ok = false;
        seq |= (uint64_t)v << (8 * k);
      }
      if (!ok) { bad++; continue; }
      if (havePrev) {
        if (seq == prev + 1) {}
        else if (seq <= prev) repeats++;
        else { if (!gaps) firstGapAt = prev; gaps++; }
      }
      prev = seq;
      havePrev = true;
    }
  }
  sdLock();
  fclose(f);
  sdUnlock();
  if (ll) bad++;
  snprintf(s_checkResult, sizeof(s_checkResult),
      "S%lu: lines=%lu frames=%lu stats_written=%lu match=%d bad=%lu gaps=%lu first_gap_after=%llu repeats=%lu sderr=%lu end=%d",
      (unsigned long)s_checkFile, (unsigned long)lines, (unsigned long)frames, (unsigned long)lastWritten,
      frames == lastWritten ? 1 : 0, (unsigned long)bad, (unsigned long)gaps, (unsigned long long)firstGapAt,
      (unsigned long)repeats, (unsigned long)sderr, endTag ? 1 : 0);
  Serial.println(s_checkResult);
  s_checkRunning = false;
  vTaskDelete(nullptr);
}

int sniffCheck(uint32_t fileNo, char* out, int cap)
{
  if (fileNo) {
    if (s_checkRunning) return snprintf(out, cap, "busy");
    if (s_fileOk && fileNo == s_fileNo) return snprintf(out, cap, "S%lu is open - stop sniff first", (unsigned long)fileNo);
    s_checkFile = fileNo;
    s_checkRunning = true;
    snprintf(s_checkResult, sizeof(s_checkResult), "S%lu: running", (unsigned long)fileNo);
    xTaskCreate(checkTask, "sniffchk", 4096, nullptr, 1, nullptr);
  }
  return snprintf(out, cap, "%s", s_checkResult);
}
#endif
