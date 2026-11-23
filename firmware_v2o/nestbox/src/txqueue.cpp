// Delivery queue - see txqueue.h for the design. Copied from NestBox_v1F
// (firmware_v5/telelogger) - only the SD include differs (nb_sd.h).
#include <Arduino.h>
#include "nb_sd.h"     // v2O: SD = SD_MMC
#include "clocktrust.h"     // PG259
#include <nvs.h>
#include <esp_heap_caps.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <time.h>
#include <LittleFS.h>   // PG223: the internal flash file system (was SPIFFS) - survives a power loss mid-write
#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>
#include <stdarg.h>
#include "sddigest.h"
#include "netlog.h"                  // PG101: nbout = Serial + /api/serial
#include "txqueue.h"
// esp_spiram_writeback_cache() of ESP-IDF 4.4 is esp_psram_extram_writeback_cache() in ESP-IDF 5
// (esp_psram/include/esp_private/esp_psram_extram.h - declared here: that header clashes with the Arduino
// core's C++ declaration of esp_psram_extram_test)
extern "C" void esp_psram_extram_writeback_cache(void);
#include "esp_system.h"     // esp_register_shutdown_handler()
#include "soc/soc.h"        // SOC_EXTRAM_DATA_LOW

extern nvs_handle_t nvs;
void statusLine(const char* text);   // ota.cpp
void sdLock();    // telestore.cpp - the SD card is shared with the logger
void sdUnlock();

// ---------------------------------------------------------------------------
// The Arduino core runs a PSRAM test at every boot (psramInit() ->
// testSPIRAM(), a weak hook) that writes every 8th word of the whole PSRAM -
// measured 2026-09-26: 12 % of the reserve destroyed by each restart, 0 %
// with the test skipped. Test the top 64 KB instead: the queue reserve is the
// first PSRAM allocation, at the bottom. A failing chip is still caught here
// and by esp_psram_init() itself (the core then runs without PSRAM).
// Arduino 3 / ESP-IDF 5.5 (2026-11-05, read in the sources): psramInit() still calls this weak hook
// (esp32-hal-psram.c:46, 83); ESP-IDF clears PSRAM only with SPIRAM_ALLOW_BSS_SEG_EXTERNAL_MEMORY (off).
extern "C" bool testSPIRAM(void)
{
  volatile uint32_t* p = (volatile uint32_t*)(SOC_EXTRAM_DATA_LOW + 0x400000 - 0x10000);   // soc.h: 0x3F800000
  const int n = 0x10000 / 4;
  for (int i = 0; i < n; i++) p[i] = (uint32_t)i * 2654435761u;
  for (int i = 0; i < n; i++) {
    if (p[i] != (uint32_t)i * 2654435761u) return false;
  }
  return true;
}

// ---------------------------------------------------------------------------
#define Q_MAGIC   0x51584654  // "TFXQ"
#define E_MAGIC   0x52434552  // "RECR"
#define W_MAGIC   0x50415257  // "WRAP"
#define Q_VERSION 1
#define ST_PENDING  0
#define ST_INFLIGHT 1
#define ST_ACKED    2
// QEnt.flags (outside the entry CRC, like state)
#define F_SD 1  // the SD writer confirmed the record's SD copy (sdwriter.cpp)
#define F_SB 2  // standby record, also kept in the SPIFFS file SB_PATH

struct QHdr {
  uint32_t magic, version, addr, cap, head, tail, count, lastPos, crc;
};
struct QEnt {
  uint32_t magic, boot, ts, file;
  uint16_t len;
  uint8_t state, flags;  // flags: was "pad" (always 0) before 2026-10-12
  uint32_t packet, crc;
};
struct SdCursor {  // NVS blob TXQ_CUR: the oldest record not confirmed yet
  uint32_t magic, file, boot, ts, pos, sdActive;
};

static SemaphoreHandle_t s_mux;
static SemaphoreHandle_t s_ackMux;   // PG223: the .ACK marks (below)
static uint8_t* s_mem;
static uint32_t s_cap;
static bool s_psram;
static QHdr* H;
#define DSTART ((uint32_t)sizeof(QHdr))

static uint32_t s_boot, s_file, s_lastTs;
// fingerprint of the open /DATA file (records the card confirmed) and of the last closed one,
// reported once (FleetNest SD = database check; not kept over a reboot - the next boot opens
// a new file and the STOP record already carried the trip's fingerprint)
static SdDigest s_dg, s_dgClosed;
static bool s_dgClosedNew;
// SD replay vs. the card's state (2026-10-13 bench: after a power-on the
// network was up ~5 s after boot, the card ~18 s; the replay found no file,
// took that for "nothing left" and stopped - 38 records never sent, lost=0)
static bool s_fileKnown;          // txqSetFile() called this boot: the card's state is known
static uint32_t s_maxFile;        // newest /DATA file of this boot (also while s_file is 0)
static bool s_replayDeferred;     // the card is not there: the cursor waits in NVS for a boot with it
static uint32_t s_replayAbandoned;  // replays ended at a missing file (counted, never silent)

// bench diagnostics (TEST_TXQ): every queue decision as an SD event line
// (telelogger.ino sets the sink to logger.logEvent)
static void (*s_evSink)(const char*);
void txqSetEventSink(void (*sink)(const char*)) { s_evSink = sink; }
#ifdef TEST_TXQ
static void ev(const char* fmt, ...) __attribute__((format(printf, 1, 2)));
static void ev(const char* fmt, ...)
{
  if (!s_evSink) return;
  char b[120];
  int n = snprintf(b, sizeof(b), "TXQ ");
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(b + n, sizeof(b) - n, fmt, ap);
  va_end(ap);
  s_evSink(b);
}
#else
#define ev(...) do {} while (0)
#endif
static uint32_t s_packetNo;
static uint32_t s_lost;           // records dropped without any other copy
static uint32_t s_lastThin;       // parked thinning (no SD, queue >= 80 %)
static uint32_t s_lastPriority;
static uint32_t s_lastSave;
static SdCursor s_saved;

// Standby records (2026-10-12): the SD card is off in standby, so the 3 h
// position reports had only the PSRAM copy - gone with a power loss. Each one
// is also appended to a small file in the internal flash (SPIFFS partition,
// 128 KB); the file is deleted once the server has confirmed every standby
// record, and replayed with the records' own identity after a lost reserve.
// PG223 (14.11.2026, Vojtech "PG223 ano"): the same file now also takes EVERY record the server has not
// confirmed when the box goes to deep sleep (txqSleepSave). Why: the PSRAM reserve is lost in every deep sleep
// (bench U9K2M3J0 12.11.: "0 records kept (reserve lost)"), the unconfirmed records were left on the SD card
// only and came back through an SD replay (Peugeot 12.11. boots 226-228: records the server already had were
// sent again after each wake, live data one record per ~19 s meanwhile; without a card they were lost).
// After the wake they go straight back into the queue with their own identity - no card needed, no replay.
// The file system is LittleFS (was SPIFFS) on the same "spiffs" partition, mounted at /spiffs as before;
// nothing was kept there in a3 (the web pages are in the firmware), so the one-time format loses nothing.
#define SB_PATH "/spiffs/txq_sb.bin"
#define SB_TMP  "/spiffs/txq_sb.new"
#define SB_MAX_BYTES (1536 * 1024)   // the whole 1 MB reserve + headers
#define SB_MAGIC 0x32425251  // "QRB2" (PG223: + file, flags; "QRBS" files of older builds are not read)
struct SbHdr { uint32_t magic, boot, ts, file; uint16_t len; uint8_t flags, pad; };
static bool s_sbMounted, s_sbFile;
// the result of the last txqSleepSave(), reported by the wake ("TXQ sleep ..."): RTC memory survives deep sleep,
// the debug log and the queue do not (the log is closed before, the queue is what is being saved)
RTC_DATA_ATTR static uint32_t s_slN, s_slBytes, s_slMs;
RTC_DATA_ATTR static uint8_t s_slState;       // 0 no sleep save before this boot, 1 ok, 2 failed
static uint32_t s_sbSaved, s_sbNotSaved, s_sbRestored;
static void sbCheckDone();
static void sbRestore();

// SD replay (records the queue does not hold any more)
static bool s_sdActive;
static uint32_t s_sdFile, s_sdBoot, s_sdTs, s_sdPos;
struct Flight { uint32_t no, sent; bool used, sd, sdEof, sdDone; uint32_t sdNextTs, sdNextPos; };
static Flight s_flight[TXQ_MAX_INFLIGHT];
static uint32_t s_lastBuiltNo;

static void lock() { if (s_mux) xSemaphoreTakeRecursive(s_mux, portMAX_DELAY); }
static void unlock() { if (s_mux) xSemaphoreGiveRecursive(s_mux); }

// PSRAM writes sit in the CPU cache; "disabling cache on the ESP32 discards
// the data in the SPI RAM cache" (ESP-IDF spiram.h) - a restart does that, so
// the newest queue entries were lost across a restart (2026-10-13 bench: the
// last 3 records before RESET were gone after the boot). Written back after
// every change of the reserve, and by a shutdown handler for any esp_restart().
static void persist()
{
  if (s_psram) esp_psram_extram_writeback_cache();
}

static uint32_t crc32b(uint32_t crc, const uint8_t* p, int n)
{
  crc = ~crc;
  while (n--) {
    crc ^= *p++;
    for (int k = 0; k < 8; k++) crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1)));
  }
  return ~crc;
}
static uint32_t hdrCrc() { return crc32b(0, (const uint8_t*)H, offsetof(QHdr, crc)); }
static void hdrSeal() { H->crc = hdrCrc(); }
static QEnt* ent(uint32_t pos) { return (QEnt*)(s_mem + pos); }
static uint32_t entCrc(const QEnt* e)
{
  uint32_t c = crc32b(0, (const uint8_t*)&e->boot, 12);
  c = crc32b(c, (const uint8_t*)&e->len, 2);
  return crc32b(c, (const uint8_t*)(e + 1), e->len);
}
static uint32_t entSize(uint32_t len) { return (sizeof(QEnt) + len + 3) & ~3u; }
// position of the entry at pos, following an explicit or implicit wrap
static uint32_t norm(uint32_t pos)
{
  if (pos + sizeof(QEnt) > s_cap || ent(pos)->magic == W_MAGIC) return DSTART;
  return pos;
}
static uint32_t nextPos(uint32_t pos) { return pos + entSize(ent(pos)->len); }

static void resetRing()
{
  memset(H, 0, sizeof(QHdr));
  H->magic = Q_MAGIC;
  H->version = Q_VERSION;
  H->addr = (uint32_t)s_mem;
  H->cap = s_cap;
  H->head = H->tail = H->lastPos = DSTART;
  hdrSeal();
}

void txqEarlyInit()
{
  s_mux = xSemaphoreCreateRecursiveMutex();
  s_ackMux = xSemaphoreCreateMutex();
  s_mem = (uint8_t*)heap_caps_malloc(TXQ_RESERVE_BYTES, MALLOC_CAP_SPIRAM);
  s_psram = s_mem != nullptr;
  s_cap = TXQ_RESERVE_BYTES;
  if (!s_mem) {
    // no PSRAM (chip failed): a small queue in internal RAM, never a crash
    s_mem = (uint8_t*)malloc(TXQ_FALLBACK_BYTES);
    s_cap = TXQ_FALLBACK_BYTES;
  }
  H = (QHdr*)s_mem;
  // any esp_restart() (OTA, RESET, wake): the reserve's newest writes to PSRAM
  esp_register_shutdown_handler([]() { persist(); });
}

// Walk what survived a restart; stop at the first damaged entry.
static uint32_t s_validN, s_validCount;  // what validateRing() found (diagnostics)
static bool validateRing()
{
  if (!s_psram || H->magic != Q_MAGIC || H->version != Q_VERSION || H->addr != (uint32_t)s_mem ||
      H->cap != s_cap || H->crc != hdrCrc() || H->head >= s_cap || H->tail >= s_cap) {
    return false;
  }
  uint32_t pos = H->tail, n = 0, last = H->tail;
  for (uint32_t i = 0; i < H->count; i++) {
    pos = norm(pos);
    QEnt* e = ent(pos);
    if (e->magic != E_MAGIC || e->len > 2048 || pos + entSize(e->len) > s_cap || e->crc != entCrc(e)) break;
    if (e->state == ST_INFLIGHT) e->state = ST_PENDING;
    last = pos;
    pos = nextPos(pos);
    n++;
  }
  if (n != H->count) nbout.printf("[TXQ] reserve damaged after %u of %u records\n", (unsigned)n, (unsigned)H->count);
  s_validN = n;
  s_validCount = H->count;
  H->count = n;
  H->head = n ? pos : DSTART;
  if (!n) H->tail = DSTART;
  H->lastPos = last;
  hdrSeal();
  return true;
}

// PG303 (23.11.2026, Vojtech "303 ano"): a LittleFS bigger than its partition is erased before the mount.
// Why: 2Z 23.11. 12:32 - the PG177/PG300 table change made "spiffs" 64 KB smaller; since PG223 (14.11.) it holds
// LittleFS and Arduino's LittleFS.begin mounts with grow_on_mount=true -> lfs_fs_grow_ asserts
// "lfs.c:5272 (block_count >= lfs->block_count)" -> restart at every boot (boot 691, 692, 693 ... on USB). LittleFS can
// grow, never shrink. The 13.11. test passed only because the partition still held SPIFFS. General: any table change
// that shrinks the partition is covered. Read from the superblock (littlefs v2, both blocks of the pair 0/1:
// revision(4) tag(4) "littlefs"(8) tag(4) version(4) block_size(4) block_count(4)); erasing both blocks = no file
// system -> LittleFS.begin(true) formats it to the partition size. Lost: the standby records (the table change runs
// only with an empty queue) and /spiffs/profile.txt -> WIFI_PL_T cleared, the list + profile are fetched again at once.
static void fsShrinkGuard()
{
  const esp_partition_t* p = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_SPIFFS, "spiffs");
  if (!p) return;
  uint32_t fsBytes = 0;
  for (int b = 0; b < 2; b++) {
    uint8_t h[32];
    if (esp_partition_read(p, b * 4096, h, sizeof(h)) != ESP_OK || memcmp(h + 8, "littlefs", 8)) continue;
    uint32_t bs, bc;
    memcpy(&bs, h + 24, 4);
    memcpy(&bc, h + 28, 4);
    if (bs == 4096 && (uint64_t)bs * bc > fsBytes) fsBytes = bs * bc;
  }
  if (fsBytes <= p->size) return;
  bool ok = esp_partition_erase_range(p, 0, 2 * 4096) == ESP_OK;
  nvs_erase_key(nvs, "WIFI_PL_T");
  nvs_commit(nvs);
  char m[64];
  snprintf(m, sizeof(m), "FS %luK > part %luK - erased %s", (unsigned long)(fsBytes / 1024),
           (unsigned long)(p->size / 1024), ok ? "OK" : "ERR");
  nbout.println(m);
  statusLine(m);
}

void txqBegin(uint32_t bootId)
{
  lock();
  s_boot = bootId;
  bool kept = validateRing();
  if (!kept) resetRing();
  // the internal flash file system for the standby records (also the httpd's
  // static files); /live.html, an old diagnostic page, removed (user, 2026-10-12)
  fsShrinkGuard();                            // PG303: never mount a LittleFS bigger than its partition (assert loop)
  s_sbMounted = LittleFS.begin(true, "/spiffs", 10, "spiffs");   // true: formatted once after SPIFFS
  if (s_sbMounted && LittleFS.exists("/txq_sb.new")) LittleFS.remove("/txq_sb.new");   // a save cut off by a power loss
  if (s_sbMounted && LittleFS.exists("/txq_sb.bin")) {
    s_sbFile = true;
    if (kept) sbCheckDone();  // the records are still in the reserve, or all confirmed
    else sbRestore();
  }
  size_t len = sizeof(s_saved);
  bool haveSaved = nvs_get_blob(nvs, "TXQ_CUR", &s_saved, &len) == ESP_OK &&
                   len == sizeof(s_saved) && s_saved.magic == Q_MAGIC;
  s_sdActive = false;
  if (haveSaved && s_saved.file && (s_saved.sdActive || !kept)) {
    // the reserve did not survive (power loss) or an SD replay was running:
    // continue from the oldest record that was not confirmed
    s_sdActive = true;
    s_sdFile = s_saved.file;
    s_sdBoot = s_saved.boot;
    s_sdTs = s_saved.ts;
    s_sdPos = s_saved.pos;
  }
  nbout.printf("[TXQ] %s %u KB, %u records kept%s, SD replay %s, standby records restored %u\n",
      s_psram ? "PSRAM" : "internal RAM", (unsigned)(s_cap >> 10), (unsigned)H->count,
      kept ? "" : " (reserve lost)", s_sdActive ? "from saved cursor" : "off", (unsigned)s_sbRestored);
  persist();
  uint32_t restored = s_sbRestored;
  unlock();
  char m[57];
  if (s_slState && esp_reset_reason() == ESP_RST_DEEPSLEEP) {   // PG223: what the sleep saved, what came back
    snprintf(m, sizeof(m), "TXQ sleep n=%u %uB %ums %s back=%u", (unsigned)s_slN, (unsigned)s_slBytes,
             (unsigned)s_slMs, s_slState == 1 ? "ok" : "FAILED", (unsigned)restored);
    statusLine(m);
  } else if (restored) {                       // after a power loss: an older save
    snprintf(m, sizeof(m), "TXQ %u records back from flash (power loss)", (unsigned)restored);
    statusLine(m);
  }
  s_slState = 0;
}

void txqSetFile(uint32_t fileId)
{
  lock();
  if (fileId != s_dg.file) {
    if (s_dg.file && s_dg.count) { s_dgClosed = s_dg; s_dgClosedNew = true; }
    sdDigestStart(&s_dg, fileId, s_boot);
  }
  s_file = fileId;
  s_fileKnown = true;
  if (fileId > s_maxFile) s_maxFile = fileId;
  unlock();
}
uint32_t txqBootId() { return s_boot; }

// after the SD log is open (the sink set): what the boot found
void txqBootReport()
{
  lock();
  uint32_t firstTs = 0, lastTs = 0, firstBoot = 0, lastBoot = 0;
  if (H->count) {
    QEnt* f = ent(norm(H->tail));
    QEnt* l = ent(H->lastPos);
    firstTs = f->ts; firstBoot = f->boot; lastTs = l->ts; lastBoot = l->boot;
  }
  ev("BOOT valid=%u/%u kept=%u first=%u:%u last=%u:%u cursor=%u:%u:%u:%u act=%d", (unsigned)s_validN,
     (unsigned)s_validCount, (unsigned)H->count, (unsigned)firstBoot, (unsigned)firstTs, (unsigned)lastBoot,
     (unsigned)lastTs, (unsigned)s_saved.file, (unsigned)s_saved.boot, (unsigned)s_saved.ts,
     (unsigned)s_saved.pos, (int)s_saved.sdActive);
  unlock();
}
bool txqUsingPsram() { return s_psram; }

static uint32_t usedBytes()
{
  if (!H->count) return 0;
  if (H->head > H->tail) return H->head - H->tail;
  return (s_cap - H->tail) + (H->head - DSTART);
}
uint32_t txqUsedPercent() { lock(); uint32_t p = usedBytes() * 100 / (s_cap - DSTART); unlock(); return p; }

static void dropTail()
{
  uint32_t pos = norm(H->tail);
  QEnt* e = ent(pos);
  if (e->state != ST_ACKED) {
    // replayed from SD only if the card really has it (2026-10-12: a record
    // whose SD copy was never confirmed is counted lost, not skipped silently)
    if (e->file && (e->flags & F_SD)) {
      if (!s_sdActive) {  // the SD copy takes over from here
        s_sdActive = true;
        s_sdFile = e->file;
        s_sdBoot = e->boot;
        s_sdTs = e->ts;
        s_sdPos = 0;
        nbout.printf("[TXQ] full - SD replay from file %u ts %u\n", (unsigned)e->file, (unsigned)e->ts);
        ev("DROP %u:%u st=%u fl=%u -> replay start", (unsigned)e->boot, (unsigned)e->ts, e->state, e->flags);
      } else {
        ev("DROP %u:%u st=%u fl=%u replay on at %u:%u", (unsigned)e->boot, (unsigned)e->ts, e->state, e->flags,
           (unsigned)s_sdFile, (unsigned)s_sdTs);
      }
    } else {
      s_lost++;
      ev("DROP %u:%u st=%u fl=%u file=%u LOST", (unsigned)e->boot, (unsigned)e->ts, e->state, e->flags,
         (unsigned)e->file);
    }
  }
  H->tail = nextPos(pos);
  if (--H->count == 0) H->head = H->tail = H->lastPos = DSTART;
}

// advance the tail over records the server has confirmed
static void releaseAcked()
{
  while (H->count && ent(norm(H->tail))->state == ST_ACKED) {
    uint32_t pos = norm(H->tail);
    H->tail = nextPos(pos);
    if (--H->count == 0) H->head = H->tail = H->lastPos = DSTART;
  }
}

static bool findSpace(uint32_t need, uint32_t& at)
{
  if (!H->count) {
    H->head = H->tail = H->lastPos = DSTART;
    at = DSTART;
    return DSTART + need <= s_cap;
  }
  if (H->head > H->tail) {  // free: [head, end) and [start, tail)
    if (H->head + need <= s_cap) { at = H->head; return true; }
    if (DSTART + need < H->tail) {
      if (H->head + sizeof(QEnt) <= s_cap) ent(H->head)->magic = W_MAGIC;
      at = DSTART;
      return true;
    }
    return false;
  }
  // wrapped (head <= tail, head == tail = full): free is [head, tail)
  if (H->head + need < H->tail) { at = H->head; return true; }
  return false;
}

// only what the user listed (2026-09-26): time code (PID 0 + GPS time/date),
// position, speed, heading, and in an engine event record the start/stop and
// its time - plus the no-fix flag that says the position is the box's last fix
static int essentials(const char* rec, int len, char* out, int cap)
{
  static const char* keep[] = {"0", "10", "11", "A", "B", "D", "E", "385", "380", "381"};
  int o = 0;
  const char* p = rec;
  const char* end = rec + len;
  while (p < end) {
    const char* q = (const char*)memchr(p, ',', end - p);
    if (!q) q = end;
    const char* colon = (const char*)memchr(p, ':', q - p);
    if (colon) {
      for (const char* k : keep) {
        size_t kl = strlen(k);
        if ((size_t)(colon - p) == kl && !memcmp(p, k, kl) && o + (q - p) + 1 < cap) {
          if (o) out[o++] = ',';
          memcpy(out + o, p, q - p);
          o += q - p;
          break;
        }
      }
    }
    p = q + 1;
  }
  return o;
}

static float fieldValue(const char* rec, int len, const char* key, float def)
{
  char pat[8];
  int pl = snprintf(pat, sizeof(pat), ",%s:", key);
  for (int i = 0; i + pl <= len; i++) {
    if (!memcmp(rec + i, pat, pl)) return atof(rec + i + pl);
  }
  return def;
}

static QEnt* pushLocked(uint32_t boot, uint32_t ts, const char* rec, int len, uint32_t file, uint8_t flags);

// ---- standby records in SPIFFS ----
static void sbAppend(const QEnt* e)
{
  if (!s_sbMounted) { s_sbNotSaved++; return; }
  FILE* f = fopen(SB_PATH, "ab");
  if (!f) { s_sbNotSaved++; return; }
  bool ok = fseek(f, 0, SEEK_END) == 0 && ftell(f) + sizeof(SbHdr) + e->len + 4 <= SB_MAX_BYTES;
  if (ok) {
    SbHdr h = {SB_MAGIC, e->boot, e->ts, e->file, e->len, (uint8_t)(e->flags & F_SD), 0};
    uint32_t crc = crc32b(crc32b(0, (const uint8_t*)&h, sizeof(h)), (const uint8_t*)(e + 1), e->len);
    ok = fwrite(&h, sizeof(h), 1, f) == 1 && fwrite(e + 1, 1, e->len, f) == e->len &&
         fwrite(&crc, 4, 1, f) == 1 && fflush(f) == 0 && fsync(fileno(f)) == 0;
  }
  fclose(f);
  if (ok) { s_sbSaved++; s_sbFile = true; }
  else s_sbNotSaved++;
}

// no standby record waiting for the server any more: the file goes
static void sbCheckDone()
{
  if (!s_sbFile) return;
  uint32_t pos = H->tail;
  for (uint32_t i = 0; i < H->count; i++) {
    pos = norm(pos);
    QEnt* e = ent(pos);
    if ((e->flags & F_SB) && e->state != ST_ACKED) return;
    pos = nextPos(pos);
  }
  remove(SB_PATH);
  s_sbFile = false;
}

// after a lost reserve: the standby records back into the queue, with their
// own boot and ts (the server drops the ones it already has)
static void sbRestore()
{
  FILE* f = fopen(SB_PATH, "rb");
  if (!f) return;
  static char rec[2048];
  SbHdr h;
  while (fread(&h, sizeof(h), 1, f) == 1 && h.magic == SB_MAGIC && h.len <= sizeof(rec)) {
    uint32_t crc;
    if (fread(rec, 1, h.len, f) != h.len || fread(&crc, 4, 1, f) != 1) break;
    if (crc != crc32b(crc32b(0, (const uint8_t*)&h, sizeof(h)), (const uint8_t*)rec, h.len)) break;
    if (pushLocked(h.boot, h.ts, rec, h.len, h.file, F_SB | (h.flags & F_SD))) s_sbRestored++;
  }
  fclose(f);
  s_sbFile = true;
}

bool txqPush(uint32_t ts, const char* rec, int len)
{
  return txqPushEx(ts, rec, len, false);
}

bool txqPushEx(uint32_t ts, const char* rec, int len, bool standby)
{
  if (len <= 0) return false;
  char tmp[512];
  lock();
  s_lastTs = ts;
  if (!s_file) {
    // no SD copy: the queue is all there is - make it last
    uint32_t used = usedBytes() * 100 / (s_cap - DSTART);
    bool event = fieldValue(rec, len, "380", -1) >= 0;
    if (used >= 80 && !event) {
      // thinned always (driving or parked): one record per TXQ_THIN_MS
      if (s_lastThin && ts - s_lastThin < TXQ_THIN_MS) { unlock(); return true; }
      s_lastThin = ts;
    }
    if (used >= 50) {  // engine events too (start/stop kept), but never thinned
      len = essentials(rec, len, tmp, sizeof(tmp));
      rec = tmp;
    }
  }
  QEnt* e = pushLocked(s_boot, ts, rec, len, s_file, standby ? F_SB : 0);
  if (e && standby) sbAppend(e);
  persist();
  unlock();
  return e != nullptr;
}

static QEnt* pushLocked(uint32_t boot, uint32_t ts, const char* rec, int len, uint32_t file, uint8_t flags)
{
  if (len > 2048) len = 2048;
  uint32_t need = entSize(len), at;
  if (need + DSTART > s_cap) return nullptr;
  while (!findSpace(need, at)) {
    if (!H->count) return nullptr;
    dropTail();
  }
  QEnt* e = ent(at);
  e->boot = boot;
  e->ts = ts;
  e->file = file;
  e->len = len;
  e->state = ST_PENDING;
  e->flags = flags;
  e->packet = 0;
  memcpy(e + 1, rec, len);
  e->crc = entCrc(e);
  e->magic = E_MAGIC;
  H->head = at + need;
  H->lastPos = at;
  H->count++;
  hdrSeal();
  return e;
}

// SD writer task: these records (this boot, ts ascending) are on the card
void txqSdConfirmed(const uint32_t* ts, int n)
{
  if (n <= 0) return;
  lock();
  if (s_dg.file && s_dg.file == s_file)
    for (int k = 0; k < n; k++) sdDigestAdd(&s_dg, ts[k]);
  uint32_t pos = H->tail;
  int j = 0;
  for (uint32_t i = 0; i < H->count && j < n; i++) {
    pos = norm(pos);
    QEnt* e = ent(pos);
    if (e->boot == s_boot && e->file) {
      while (j < n && ts[j] < e->ts) j++;
      if (j < n && ts[j] == e->ts) e->flags |= F_SD;
    }
    pos = nextPos(pos);
  }
  persist();
  unlock();
}

// ---------------------------------------------------------------------------
static int flightsUsed()
{
  int n = 0;
  for (auto& f : s_flight) if (f.used) n++;
  return n;
}
// PG228: packets sent and not answered yet (telemetry.cpp: is anything waiting for the server right now)
int txqInflight() { lock(); int n = flightsUsed(); unlock(); return n; }
static Flight* newFlight()
{
  for (auto& f : s_flight) {
    if (!f.used) { memset(&f, 0, sizeof(f)); f.used = true; return &f; }
  }
  return nullptr;
}

static void expire()
{
  uint32_t now = millis();
  for (auto& f : s_flight) {
    if (!f.used || now - f.sent < TXQ_ACK_TIMEOUT_MS) continue;
    f.used = false;  // SD replay simply restarts from its cursor
    ev("EXPIRE pkt=%u sd=%d", (unsigned)f.no, f.sd ? 1 : 0);
    if (!f.sd) {
      uint32_t pos = H->tail;
      for (uint32_t i = 0; i < H->count; i++) {
        pos = norm(pos);
        QEnt* e = ent(pos);
        if (e->state == ST_INFLIGHT && e->packet == f.no) e->state = ST_PENDING;
        pos = nextPos(pos);
      }
    }
  }
}

// ---- records made before the clock was set (power-on, no GPS fix yet) ----
// Such a record has no time (PID 10/11); Traccar would file it under the time
// of the last position it knows (2026-09-27: 57 s of a drive landed on the
// evening before). They wait in the queue until the clock is valid, then get
// their time computed from their ms timestamp: t = now - (millis() - ts).
#define TXQ_UNTIMED_HOLD_MS 600000  // after 10 min without a clock: sent as they are
static bool clockValid() { return clockTrusted(); }  // > 2025, PG259: and set since a deep-sleep wake
static bool recHasTime(const char* rec, int len)
{
  for (int i = 0; i + 4 <= len; i++) {
    if (rec[i] == ',' && rec[i + 1] == '1' && rec[i + 2] == '0' && rec[i + 3] == ':') return true;
  }
  return false;
}
// 0 = send as is, 1 = append the computed time, -1 = hold it back for now
static int untimedAction(uint32_t boot, uint32_t ts, const char* rec, int len)
{
  if (recHasTime(rec, len) || boot != s_boot) return 0;  // other boot: its millis base is gone
  if (clockValid()) return 1;
  return millis() - ts < TXQ_UNTIMED_HOLD_MS ? -1 : 0;
}
static int timeFields(char* out, int cap, uint32_t ts)
{
  time_t t = time(nullptr) - (time_t)((millis() - ts) / 1000);
  struct tm u;
  gmtime_r(&t, &u);
  return snprintf(out, cap, ",10:%u,11:%u",
      (unsigned)(u.tm_hour * 1000000 + u.tm_min * 10000 + u.tm_sec * 100),
      (unsigned)(u.tm_mday * 10000 + (u.tm_mon + 1) * 100 + u.tm_year % 100));
}

static int header(char* out, int cap, const char* devid, uint32_t boot, uint32_t no)
{
  return snprintf(out, cap, "%s#%X:%u,%X:%u", devid, PID_TX_BOOT, (unsigned)boot, PID_TX_PACKET, (unsigned)no);
}
static int tailer(char* out, int n, int cap)
{
  uint8_t sum = 0;
  for (int i = 0; i < n; i++) sum += (uint8_t)out[i];
  return n + snprintf(out + n, cap - n, "*%X", sum);
}

// ---- PG223: /DATA/<n>.ACK - which records of /DATA/<n>.CSV the server has confirmed ----
// Why: the NVS cursor is one position ("everything before it is confirmed"). With the newest record and the
// queue going out before the SD replay, the confirmed records are no longer one block: after a long outage the
// fresh ones are confirmed while the middle is still being replayed. A power loss then restarted the replay at
// the cursor and sent everything behind it again (Peugeot 12.11.: records the server already had, sent again
// after each wake). The .ACK file marks every confirmed record by its ts (4 bytes, append only); the replay
// leaves those out. A mark not written yet (ACK_FLUSH_MS, power loss) only means that record is sent once
// more - the server drops it by its identity. RESEND= from the server ignores the marks (the server says it
// does NOT have the records).
#define ACK_PEND      512
#define ACK_FLUSH_MS  5000
#define ACK_CACHE_MAX 131072         // ts of one file kept in PSRAM (512 KB = 36 h at 1 record/s)
#define SD_SCAN_LINES 4000           // lines read per call while nothing is sent: the SD lock is given back
struct AckPend { uint32_t file, ts; };
static AckPend s_ackPend[ACK_PEND], s_ackOut[ACK_PEND];
static int s_ackN;
static volatile bool s_ackFlushing;
static uint32_t s_ackFlushAt, s_ackWritten, s_ackErr, s_ackDropped;
static uint32_t* s_ackC;             // cache of one file's marks: [0, sorted) sorted, the rest as it came
static uint32_t s_ackCFile, s_ackCN, s_ackCSorted;
static bool s_sdIgnoreAck;           // a RESEND replay
static uint32_t s_replayAcked;

static bool ackTake(TickType_t t) { return s_ackMux && xSemaphoreTake(s_ackMux, t) == pdTRUE; }
static void ackGive() { xSemaphoreGive(s_ackMux); }

// called holding the queue lock: never waits (a mark that does not fit is only a record sent once more)
static void ackNote(uint32_t file, uint32_t ts)
{
  if (!file) return;
  if (!ackTake(0)) { s_ackDropped++; return; }
  if (s_ackN < ACK_PEND) s_ackPend[s_ackN++] = {file, ts};
  else s_ackDropped++;
  ackGive();
}

static int ackCmp(const void* a, const void* b)
{
  uint32_t x = *(const uint32_t*)a, y = *(const uint32_t*)b;
  return x < y ? -1 : x > y ? 1 : 0;
}

// the marks waiting in RAM go to the card; called from the net task and before a restart / sleep
void txqAckFlush(bool force)
{
  if (!s_ackMux) return;
  if (!force && millis() - s_ackFlushAt < ACK_FLUSH_MS && s_ackN < ACK_PEND / 2) return;
  if (!ackTake(pdMS_TO_TICKS(200))) return;
  if (s_ackFlushing || !s_ackN) { ackGive(); s_ackFlushAt = millis(); return; }
  s_ackFlushing = true;
  int n = s_ackN;
  memcpy(s_ackOut, s_ackPend, n * sizeof(AckPend));
  s_ackN = 0;
  ackGive();
  s_ackFlushAt = millis();
  if (s_maxFile) {                   // a card this boot (else: dropped, the records are resent after a replay)
    int i = 0;
    while (i < n) {
      uint32_t file = s_ackOut[i].file;
      static uint32_t ts[ACK_PEND];
      int k = 0;
      for (int j = i; j < n; j++)
        if (s_ackOut[j].file == file) ts[k++] = s_ackOut[j].ts;
      char path[32];
      snprintf(path, sizeof(path), "/sd/DATA/%u.ACK", (unsigned)file);
      sdLock();
      int fd = open(path, O_WRONLY | O_CREAT | O_APPEND, 0644);
      bool ok = fd >= 0 && write(fd, ts, k * 4) == k * 4 && fsync(fd) == 0;
      if (fd >= 0) close(fd);
      sdUnlock();
      if (ok) s_ackWritten += k; else s_ackErr++;
      if (ok && ackTake(pdMS_TO_TICKS(200))) {     // the cache of the file being replayed follows
        if (s_ackC && s_ackCFile == file)
          for (int j = 0; j < k && s_ackCN < ACK_CACHE_MAX; j++) s_ackC[s_ackCN++] = ts[j];
        ackGive();
      }
      while (i < n && s_ackOut[i].file == file) i++;   // the batch is grouped by file in practice
      for (int j = i; j < n; j++)                       // the same file further on: done above
        if (s_ackOut[j].file == file) s_ackOut[j].file = 0;
      while (i < n && !s_ackOut[i].file) i++;
    }
  }
  s_ackFlushing = false;
}

// SD replay (net task, holding the SD lock): has the server confirmed this record of this file?
static bool ackHas(uint32_t file, uint32_t ts)
{
  if (!ackTake(pdMS_TO_TICKS(500))) return false;
  if (s_ackCFile != file) {          // the marks of this file from the card, once per file
    if (!s_ackC) s_ackC = (uint32_t*)heap_caps_malloc(ACK_CACHE_MAX * 4, MALLOC_CAP_SPIRAM);
    s_ackCFile = file;
    s_ackCN = s_ackCSorted = 0;
    if (s_ackC) {
      char path[32];
      snprintf(path, sizeof(path), "/sd/DATA/%u.ACK", (unsigned)file);
      int fd = open(path, O_RDONLY);
      if (fd >= 0) {
        int got;
        while (s_ackCN < ACK_CACHE_MAX &&
               (got = read(fd, s_ackC + s_ackCN, (ACK_CACHE_MAX - s_ackCN) * 4 > 16384 ? 16384 : (ACK_CACHE_MAX - s_ackCN) * 4)) >= 4)
          s_ackCN += got / 4;
        close(fd);
      }
    }
  }
  if (s_ackC && s_ackCN - s_ackCSorted > 0 && (s_ackCSorted == 0 || s_ackCN - s_ackCSorted > 1024)) {
    qsort(s_ackC, s_ackCN, 4, ackCmp);
    s_ackCSorted = s_ackCN;
  }
  bool has = false;
  if (s_ackC) {
    has = s_ackCSorted && bsearch(&ts, s_ackC, s_ackCSorted, 4, ackCmp) != nullptr;
    for (uint32_t i = s_ackCSorted; !has && i < s_ackCN; i++) has = s_ackC[i] == ts;
  }
  for (int i = 0; !has && i < s_ackN; i++) has = s_ackPend[i].file == file && s_ackPend[i].ts == ts;
  ackGive();
  return has;
}

// ---- SD replay: read records back from /DATA/<file>.CSV ----
// PG-p14: how a replay ends goes to FleetNest as a status line (before: only in TEST_TXQ builds)
static uint32_t s_replayRecs, s_replaySkip;   // records put into replay packets / skipped (no boot id)
static void replayEnd(const char* why)
{
  char m[57];
  snprintf(m, sizeof(m), "REPLAY end file=%u sent=%u skip=%u %s", (unsigned)s_sdFile, (unsigned)s_replayRecs,
           (unsigned)s_replaySkip, why);
  statusLine(m);
  if (s_replayAcked) {                 // PG223: left out, the server has them (.ACK)
    snprintf(m, sizeof(m), "REPLAY acked=%u not sent again", (unsigned)s_replayAcked);
    statusLine(m);
  }
  s_replayRecs = 0;
  s_replaySkip = 0;
  s_replayAcked = 0;
  s_sdIgnoreAck = false;
}
// stop where the queue takes over (its oldest record that has an SD copy). PG-p19: taken as a copy under
// the lock before the card is read. It only moves forward (records leave the queue at its tail), so a read
// with an old copy stops early at worst - the next packet goes on (the replay ends only when a build finds
// nothing more, see txqOnAck)
struct SdStop { bool have; uint32_t file, ts; };
static SdStop sdStopNow()
{
  SdStop s = {false, 0, 0};
  uint32_t pos = H->tail;
  for (uint32_t i = 0; i < H->count; i++) {
    pos = norm(pos);
    QEnt* e = ent(pos);
    if (e->file) { s.have = true; s.file = e->file; s.ts = e->ts; return s; }
    pos = nextPos(pos);
  }
  return s;
}
static bool sdStopKey(const SdStop& s, uint32_t file, uint32_t ts)
{
  return s.have && (file > s.file || (file == s.file && ts >= s.ts));
}

// PG-p19 (3.11.2026): one replay packet is read from the card WITHOUT the queue lock.
// why: txqPush() (the record task, every record) waits for the queue lock without a limit, and the card
// was read under it - Peugeot 3.11 boot 147/149: each RESEND stopped the records 66-76 s (before PG-p14
// the whole files were read at once), the missing seconds are not on SD either (records never made).
// The card can be slow by itself (the SD writer's fsync, OTA/sniff writes under sdLock), so no queue
// state is touched while reading: sdReadPacket() works on a copy of the cursor (SdRead), buildSdPacket()
// applies the result under the lock - unless a RESEND set a new cursor meanwhile (s_sdGen), then the
// read is dropped and done again from the new cursor. sdLock stays (user 3.11, PG35).
// Only the net task builds packets (txqNextPacket), so s_recBuf and the flight are not shared.
struct SdRead {
  // in: the cursor and what the read needs from the queue
  uint32_t file, boot, ts, pos, curFile, myBoot;
  SdStop stop;
  bool ignoreAck;
  // out
  bool opened, eof, done, partial;
  uint32_t nextTs, nextPos, skip, acked;
  int recLen, nrec;
  uint32_t tsl[64];                  // PG223: ts of the records in the packet (for their .ACK marks)
};
static uint32_t s_sdPktFile, s_sdPktTs[64];   // PG223: the records of the SD packet in flight (one at a time)
static int s_sdPktN;
static char s_recBuf[TXQ_PACKET_MAX + 700];  // a record is never cut: start one only below TXQ_PACKET_MAX
static uint32_t s_sdGen;                      // +1 when a RESEND sets the cursor anew

static void sdReadPacket(SdRead& r)
{
  char path[24];
  sprintf(path, "/DATA/%u.CSV", (unsigned)r.file);
  sdLock();
  File file = SD.open(path, FILE_READ);
  if (!file) {
    sdUnlock();
    r.opened = false;
    return;
  }
  r.opened = true;
  if (r.pos) file.seek(r.pos);
  char line[160];
  char* recBuf = s_recBuf;
  int recLen = 0;         // records accepted into the packet
  int cur = -1;           // start of the record being read in recBuf, -1 = skipping
  bool curHasData = false, curHasTime = false;
  uint32_t curTs = 0, curPos = 0;
  bool eof = false, done = false;
  uint32_t nextTs = 0, nextPos = 0;
  int nrec = 0;           // PG-p14: records in this packet
  uint32_t lines = 0;
  while (true) {
    uint32_t linePos = file.position();
    lines++;
    int n = file.available() ? file.readBytesUntil('\n', line, sizeof(line) - 1) : -1;
    if (n < 0) {
      eof = true;
      if (r.file == r.curFile && cur >= 0) {
        // the file being written: its last record may still be growing - not
        // sent now, read again (complete) from its start next time
        recLen = cur;  // cur = where its ",0:<ts>" began
        nrec--;
        nextTs = curTs;
        nextPos = curPos;
        cur = -1;
      } else {
        nextTs = curTs ? curTs + 1 : r.ts;
        nextPos = linePos;
      }
      break;
    }
    line[n] = 0;
    char* comma = strchr(line, ',');
    if (!comma) continue;
    *comma = 0;
    const char* key = line;
    const char* val = comma + 1;
    if (!strcmp(key, "FE")) {
      if (!strncmp(val, "BOOTID=", 7)) r.boot = strtoul(val + 7, 0, 10);
      // PG-p14: v2O files before the BOOTID line have only "FE,BOOT <n> ID=<devid> ..." (telemetry.cpp)
      else if (!r.boot && !strncmp(val, "BOOT ", 5) && val[5] >= '1' && val[5] <= '9' && strstr(val, " ID="))
        r.boot = strtoul(val + 5, 0, 10);
      continue;
    }
    if (!strcmp(key, "0")) {
      uint32_t ts = strtoul(val, 0, 10);
      if (cur >= 0 && !curHasData) { recLen = cur; nrec--; }  // an empty record: leave it out
      else if (cur >= 0 && !curHasTime && r.boot == r.myBoot && clockValid())
        recLen += timeFields(recBuf + recLen, sizeof(s_recBuf) - recLen, curTs);
      cur = -1;  // the previous record (if any) is complete
      if (!r.boot) { r.skip++; continue; }  // file without a boot id (old firmware): not replayable
      // PG223: a long stretch with nothing to send (already confirmed / before the cursor) is read in parts -
      // the SD writer must get the card in between (it calls the card dead after 30 s without a write)
      if (!recLen && lines > SD_SCAN_LINES) { r.partial = true; nextTs = r.ts; nextPos = linePos; break; }
      if (ts < r.ts) continue;
      if (sdStopKey(r.stop, r.file, ts)) { done = true; nextTs = ts; nextPos = linePos; break; }
      if (!r.ignoreAck && ackHas(r.file, ts)) { r.acked++; continue; }
      if (recLen >= TXQ_PACKET_MAX) { nextTs = ts; nextPos = linePos; break; }
      cur = recLen;
      nrec++;
      if (nrec >= 1 && nrec <= 64) r.tsl[nrec - 1] = ts;
      curHasData = false;
      curHasTime = false;
      if (recLen) recBuf[recLen++] = ',';
      recLen += snprintf(recBuf + recLen, sizeof(s_recBuf) - recLen, "0:%u", (unsigned)ts);
      curTs = ts;
      curPos = linePos;
      continue;
    }
    if (cur < 0) continue;
    curHasData = true;
    if (!strcmp(key, "10")) curHasTime = true;
    int kl = strlen(key), vl = strlen(val);
    if (recLen + kl + vl + 2 < (int)sizeof(s_recBuf)) {
      recBuf[recLen++] = ',';
      memcpy(recBuf + recLen, key, kl); recLen += kl;
      recBuf[recLen++] = ':';
      memcpy(recBuf + recLen, val, vl); recLen += vl;
    }
  }
  if (cur >= 0 && !curHasData) { recLen = cur; nrec--; }
  else if (cur >= 0 && !curHasTime && r.boot == r.myBoot && clockValid())
    recLen += timeFields(recBuf + recLen, sizeof(s_recBuf) - recLen, curTs);
  file.close();
  sdUnlock();
  r.eof = eof;
  r.done = done;
  r.nextTs = nextTs;
  r.nextPos = nextPos;
  r.recLen = recLen;
  r.nrec = nrec;
}

// called by txqNextPacket() holding the queue lock ONCE (it is released here while the card is read)
static int buildSdPacket(char* out, int cap, const char* devid, Flight* f)
{
  SdRead r = {};
  r.file = s_sdFile; r.boot = s_sdBoot; r.ts = s_sdTs; r.pos = s_sdPos;
  r.curFile = s_file; r.myBoot = s_boot;
  r.ignoreAck = s_sdIgnoreAck;
  r.stop = sdStopNow();
  uint32_t gen = s_sdGen;
  unlock();
  sdReadPacket(r);
  lock();
  // a RESEND (or the end) meanwhile: next time from the new cursor
  if (gen != s_sdGen || !s_sdActive || s_sdFile != r.file || s_sdPos != r.pos || s_sdTs != r.ts) return 0;
  s_sdBoot = r.boot;
  s_replaySkip += r.skip;
  s_replayAcked += r.acked;
  if (!r.opened) {
    if (!s_maxFile) {
      // no SD file this boot (no card / card gone): keep the cursor in NVS
      // for a boot that has the card, send the queue meanwhile
      s_sdActive = false;
      s_replayDeferred = true;
      SdCursor c = {Q_MAGIC, s_sdFile, s_sdBoot, s_sdTs, s_sdPos, 1};
      s_saved = c;
      nvs_set_blob(nvs, "TXQ_CUR", &c, sizeof(c));
      nvs_commit(nvs);
      nbout.printf("[TXQ] SD replay deferred (no SD file this boot) at file %u ts %u\n", (unsigned)s_sdFile, (unsigned)s_sdTs);
    } else if (s_sdFile < s_maxFile) {
      // deleted (SD space cleanup) or not written: go on with the next file
      s_sdFile++; s_sdTs = 0; s_sdPos = 0; s_sdBoot = 0;
    } else {
      s_sdActive = false;
      s_replayAbandoned++;
      nbout.printf("[TXQ] SD replay ended: /DATA/%u.CSV missing\n", (unsigned)s_sdFile);
      replayEnd("missing");
    }
    return 0;
  }
  if (!r.recLen) {
    if (r.partial) { s_sdPos = r.nextPos; return 0; }   // PG223: go on from here at the next call
    if (r.done || (r.eof && s_sdFile >= s_maxFile)) {
      ev("REPLAY end file=%u ts=%u done=%d eof=%d stop=%u", (unsigned)s_sdFile, (unsigned)s_sdTs, r.done, r.eof,
         (unsigned)r.nextTs);
      s_sdActive = false;
      replayEnd(r.done ? "done" : "eof");
      return 0;
    }
    if (r.eof) {
      ev("REPLAY eof file=%u -> next", (unsigned)s_sdFile);
      s_sdFile++; s_sdTs = 0; s_sdPos = 0; s_sdBoot = 0;
    }
    return 0;
  }
  ev("REPLAY pkt=%u file=%u from=%u next=%u done=%d eof=%d", (unsigned)f->no, (unsigned)s_sdFile,
     (unsigned)s_sdTs, (unsigned)r.nextTs, r.done, r.eof);
  s_replayRecs += r.nrec;
  f->sd = true;
  f->sdEof = r.eof;
  f->sdDone = r.done;
  f->sdNextTs = r.nextTs;
  f->sdNextPos = r.nextPos;
  s_sdPktFile = s_sdFile;
  s_sdPktN = r.nrec < 0 ? 0 : r.nrec > 64 ? 64 : r.nrec;
  memcpy(s_sdPktTs, r.tsl, s_sdPktN * 4);
  int o = header(out, cap, devid, s_sdBoot, f->no);
  out[o++] = ',';
  memcpy(out + o, s_recBuf, r.recLen);
  o += r.recLen;
  return tailer(out, o, cap);
}

#ifdef TEST_TXQ
volatile bool g_txqTestPause = false;  // bench: an outage without touching the server
#endif

#ifdef TEST_SLEEP_BACKLOG
volatile bool g_txqTestHold = false;   // bench build only (standby.cpp): nothing is sent before the test sleep
#endif

int txqNextPacket(char* out, int cap, const char* devid)
{
#ifdef TEST_SLEEP_BACKLOG
  if (g_txqTestHold) return 0;
#endif
#ifdef TEST_TXQ
  if (g_txqTestPause) return 0;
#endif
#ifdef TEST_TXQ_HOLD_MS
  if (millis() < TEST_TXQ_HOLD_MS) return 0;  // bench: build a backlog -> full-size packets
#endif
  lock();
  // an SD replay from before this boot: nothing goes out (older records
  // first, always) until the boot has set up the card (txqSetFile)
  if (s_sdActive && !s_fileKnown) { unlock(); return 0; }
  expire();
  if (flightsUsed() >= TXQ_MAX_INFLIGHT) { unlock(); return 0; }
  Flight* f = newFlight();
  f->no = ++s_packetNo;
  f->sent = millis();
  int n = 0;

  // 1. the newest record first when a backlog is being worked off - the map
  //    and a theft/tow position must not wait behind hours of old data
  if (H->count > 1 && millis() - s_lastPriority > 5000) {
    QEnt* last = ent(H->lastPos);
    QEnt* first = ent(norm(H->tail));
    int act = untimedAction(last->boot, last->ts, (const char*)(last + 1), last->len);
    if (last->state == ST_PENDING && last->ts - first->ts > 20000 && last->boot == first->boot && act >= 0) {
      s_lastPriority = millis();
      n = header(out, cap, devid, last->boot, f->no);
      out[n++] = ',';
      memcpy(out + n, last + 1, last->len);
      n += last->len;
      if (act == 1) n += timeFields(out + n, cap - n, last->ts);
      last->state = ST_INFLIGHT;
      ev("PRIO pkt=%u %u:%u", (unsigned)f->no, (unsigned)last->boot, (unsigned)last->ts);
      last->packet = f->no;
    }
  }
  // 2. queued records, oldest first, same boot in one packet
  if (!n) {
    uint32_t pos = H->tail, boot = 0;
    for (uint32_t i = 0; i < H->count; i++) {
      pos = norm(pos);
      QEnt* e = ent(pos);
      if (e->state == ST_PENDING) {
        int act = untimedAction(e->boot, e->ts, (const char*)(e + 1), e->len);
        if (act < 0) break;  // no clock yet: this and the newer records wait
        if (!n) {
          boot = e->boot;
          n = header(out, cap, devid, boot, f->no);
        } else if (e->boot != boot) {
          break;
        }
        if (n + 1 + e->len + 32 > cap || n + 1 + e->len > TXQ_PACKET_MAX + 64) break;
        out[n++] = ',';
        memcpy(out + n, e + 1, e->len);
        n += e->len;
        if (act == 1) n += timeFields(out + n, cap - n, e->ts);
        e->state = ST_INFLIGHT;
        e->packet = f->no;
      }
      pos = nextPos(pos);
    }
  }
  // 3. SD replay (older than anything in the queue) in the background: one packet at a time, and only when
  //    the queue has nothing to send right now. PG223 (Vojtech 14.11.2026 "nove data prve"): before, the
  //    replay came first and the queue stood still until it ended - Peugeot 12.11. drive 18:30-19:27 got one
  //    live record per ~19 s; and an error in the old data must not keep the new data from getting through.
  if (!n && s_sdActive) {
    bool sdBusy = false;
    for (auto& g : s_flight) if (g.used && g.sd) sdBusy = true;
    if (!sdBusy) n = buildSdPacket(out, cap, devid, f);
    if (n) { s_lastBuiltNo = f->no; unlock(); return n; }
  }
  if (!n) { f->used = false; s_packetNo--; unlock(); return 0; }
#ifdef TEST_TXQ
  {
    // first and last record of the packet
    uint32_t a0 = 0, a1 = 0, cnt = 0, bt = 0, pos = H->tail;
    for (uint32_t i = 0; i < H->count; i++) {
      pos = norm(pos);
      QEnt* e = ent(pos);
      if (e->state == ST_INFLIGHT && e->packet == f->no) { if (!cnt) a0 = e->ts; a1 = e->ts; bt = e->boot; cnt++; }
      pos = nextPos(pos);
    }
    ev("SEND pkt=%u %u:%u..%u n=%u", (unsigned)f->no, (unsigned)bt, (unsigned)a0, (unsigned)a1, (unsigned)cnt);
  }
#endif
  n = tailer(out, n, cap);
  s_lastBuiltNo = f->no;
  unlock();
  return n;
}

void txqSendFailed()
{
  lock();
  for (auto& f : s_flight) {
    if (f.used && f.no == s_lastBuiltNo) f.sent = millis() - TXQ_ACK_TIMEOUT_MS;  // expire now
  }
  expire();
  unlock();
}

void txqOnAck(uint32_t packetNo)
{
  lock();
  for (auto& f : s_flight) {
    if (!f.used || f.no != packetNo) continue;
    f.used = false;
    if (f.sd) {
      ev("ACK sd pkt=%u done=%d eof=%d next=%u", (unsigned)packetNo, f.sdDone, f.sdEof, (unsigned)f.sdNextTs);
      for (int k = 0; k < s_sdPktN; k++) ackNote(s_sdPktFile, s_sdPktTs[k]);   // PG223
      s_sdPktN = 0;
      if (f.sdDone || (f.sdEof && s_sdFile >= s_maxFile)) {
        // not "done" yet (2026-10-13): while this packet was out, the full
        // queue may have dropped more records relying on this replay - go on
        // from where the packet ended; the replay stops only when a build
        // finds nothing more to send (buildSdPacket: recLen 0 + done/eof)
        s_sdTs = f.sdNextTs;
        s_sdPos = f.sdNextPos;
      } else if (f.sdEof) {
        s_sdFile++; s_sdTs = 0; s_sdPos = 0; s_sdBoot = 0;
      } else {
        s_sdTs = f.sdNextTs;
        s_sdPos = f.sdNextPos;
      }
    } else {
      uint32_t pos = H->tail;
      for (uint32_t i = 0; i < H->count; i++) {
        pos = norm(pos);
        QEnt* e = ent(pos);
        if (e->state == ST_INFLIGHT && e->packet == packetNo) {
          e->state = ST_ACKED;
          if (e->flags & F_SD) ackNote(e->file, e->ts);   // PG223: marked in /DATA/<file>.ACK
        }
        pos = nextPos(pos);
      }
      releaseAcked();
      hdrSeal();
      sbCheckDone();
    }
  }
  persist();
  unlock();
}

uint32_t txqPending()
{
  lock();
  uint32_t n = 0, pos = H->tail;
  for (uint32_t i = 0; i < H->count; i++) {
    pos = norm(pos);
    if (ent(pos)->state != ST_ACKED) n++;
    pos = nextPos(pos);
  }
  if (s_sdActive) n++;
  unlock();
  return n;
}

void txqSave(bool force)
{
  if (!force && millis() - s_lastSave < 30000) return;
  s_lastSave = millis();
  if (s_replayDeferred) return;  // the cursor for a boot with the card stays as it is
  lock();
  SdCursor c = {Q_MAGIC, 0, 0, 0, 0, 0};
  if (s_sdActive) {
    c = {Q_MAGIC, s_sdFile, s_sdBoot, s_sdTs, s_sdPos, 1};
  } else {
    uint32_t pos = H->tail;
    for (uint32_t i = 0; i < H->count; i++) {
      pos = norm(pos);
      QEnt* e = ent(pos);
      if (e->state != ST_ACKED && e->file) { c = {Q_MAGIC, e->file, e->boot, e->ts, 0, 0}; break; }
      pos = nextPos(pos);
    }
    if (!c.file && s_file) c = {Q_MAGIC, s_file, s_boot, s_lastTs + 1, 0, 0};
  }
  unlock();
  if (memcmp(&c, &s_saved, sizeof(c))) {
    s_saved = c;
    nvs_set_blob(nvs, "TXQ_CUR", &c, sizeof(c));
    nvs_commit(nvs);
  }
}

// PG223: before deep sleep (standby.cpp, after prepareRestart) - every record the server has not confirmed
// goes into the internal flash; txqBegin() of the wake puts them back (sbRestore). Written to a new file and
// renamed, so a power loss in the middle leaves the previous file. When it worked and no SD replay is running,
// the NVS cursor points behind the last record of the open file: the wake has nothing to replay. When it failed (no file
// system, full) the cursor stays as txqSave() left it and the wake replays from the card, as before.
void txqSleepSave()
{
  lock();
  uint32_t t0 = millis(), n = 0, bytes = 0;
  bool ok = s_sbMounted;
  FILE* f = ok ? fopen(SB_TMP, "wb") : nullptr;
  if (!f) ok = false;
  uint32_t pos = H->tail;
  for (uint32_t i = 0; i < H->count && ok; i++) {
    pos = norm(pos);
    QEnt* e = ent(pos);
    if (e->state != ST_ACKED) {
      SbHdr h = {SB_MAGIC, e->boot, e->ts, e->file, e->len, (uint8_t)(e->flags & F_SD), 0};
      uint32_t crc = crc32b(crc32b(0, (const uint8_t*)&h, sizeof(h)), (const uint8_t*)(e + 1), e->len);
      ok = bytes + sizeof(h) + e->len + 4 <= SB_MAX_BYTES && fwrite(&h, sizeof(h), 1, f) == 1 &&
           fwrite(e + 1, 1, e->len, f) == e->len && fwrite(&crc, 4, 1, f) == 1;
      if (ok) { n++; bytes += sizeof(h) + e->len + 4; }
    }
    pos = nextPos(pos);
  }
  if (f) {
    if (ok) ok = fflush(f) == 0 && fsync(fileno(f)) == 0;
    fclose(f);
  }
  if (ok && n) ok = rename(SB_TMP, SB_PATH) == 0;
  else if (ok) remove(SB_PATH);               // nothing waits: no file
  if (!ok || !n) remove(SB_TMP);
  if (ok) {
    s_sbFile = n > 0;
    if (!s_sdActive) {
      // "after the last record of the open file": a record made between here and the sleep itself is on the
      // card only and is still found (pos = the file's size now, so the wake does not read the file through)
      SdCursor c = {Q_MAGIC, 0, 0, 0, 0, 0};
      if (s_file) {
        char path[32];
        snprintf(path, sizeof(path), "/sd/DATA/%u.CSV", (unsigned)s_file);
        struct stat st;
        sdLock();
        bool have = stat(path, &st) == 0;
        sdUnlock();
        c = {Q_MAGIC, s_file, s_boot, s_lastTs + 1, have ? (uint32_t)st.st_size : 0, 0};
      }
      s_saved = c;
      nvs_set_blob(nvs, "TXQ_CUR", &c, sizeof(c));
      nvs_commit(nvs);
    }
  }
  s_slN = n; s_slBytes = bytes; s_slMs = millis() - t0; s_slState = ok ? 1 : 2;
  nbout.printf("[TXQ] sleep save: %u records, %u bytes in %u ms - %s\n", (unsigned)n, (unsigned)bytes,
               (unsigned)(millis() - t0), ok ? "ok" : "FAILED (SD replay after the wake)");
  unlock();
}

#ifdef TEST_TXQ
void txqTestLoseReserve()
{
  txqSave(true);
  lock();
  H->magic = 0;  // next boot: "reserve lost", continue from the saved cursor
  unlock();
}
#endif

void txqResendFrom(uint32_t fileId)
{
  lock();
  s_sdActive = fileId != 0;
  s_sdGen++;     // PG-p19: a replay read running now (without the lock) is dropped, not applied
  s_sdFile = fileId;
  s_sdIgnoreAck = fileId != 0;   // PG223: the server says it does not have them - the .ACK marks do not count
  s_sdBoot = 0;  // read from the file's BOOTID line
  s_replayRecs = 0;
  s_replaySkip = 0;
  s_sdTs = 0;
  s_sdPos = 0;
  for (auto& f : s_flight) if (f.used && f.sd) f.used = false;
  unlock();
  txqSave(true);
}

// the oldest /DATA file the queue may still have to replay from (0 = none):
// SDLogger::purgeOldFiles() never deletes it or anything newer
uint32_t txqOldestFileNeeded()
{
  lock();
  uint32_t f = 0;
  auto take = [&](uint32_t x) { if (x && (!f || x < f)) f = x; };
  if (s_sdActive) take(s_sdFile);
  uint32_t pos = H->tail;
  for (uint32_t i = 0; i < H->count; i++) {
    pos = norm(pos);
    QEnt* e = ent(pos);
    if (e->state != ST_ACKED) take(e->file);
    pos = nextPos(pos);
  }
  if (s_saved.magic == Q_MAGIC) take(s_saved.file);
  unlock();
  return f;
}

void txqStatus(char* buf, int size)
{
  lock();
  uint32_t sdWait = 0, pos = H->tail;
  for (uint32_t i = 0; i < H->count; i++) {
    pos = norm(pos);
    QEnt* e = ent(pos);
    if (e->file && !(e->flags & F_SD) && e->state != ST_ACKED) sdWait++;
    pos = nextPos(pos);
  }
  uint32_t newestBoot = H->count ? ent(H->lastPos)->boot : 0, newestTs = H->count ? ent(H->lastPos)->ts : 0;
  snprintf(buf, size, "TXQ %s used=%u%% records=%u pending=%u inflight=%d sd=%s%u/%u lost=%u sdwait=%u sb=%u/%u/%u/%d replay_deferred=%d replay_abandoned=%u newest=%u:%u ack=%u/%u/%u",
      s_psram ? "psram" : "ram", (unsigned)(usedBytes() * 100 / (s_cap - DSTART)), (unsigned)H->count,
      (unsigned)txqPending(), flightsUsed(), s_sdActive ? "on:" : "off:", (unsigned)s_sdFile, (unsigned)s_sdTs,
      (unsigned)s_lost, (unsigned)sdWait, (unsigned)s_sbSaved, (unsigned)s_sbNotSaved, (unsigned)s_sbRestored,
      s_sbFile ? 1 : 0, s_replayDeferred ? 1 : 0, (unsigned)s_replayAbandoned, (unsigned)newestBoot, (unsigned)newestTs,
      (unsigned)s_ackWritten, (unsigned)s_ackErr, (unsigned)s_ackDropped);
  unlock();
}

// SD file fingerprints (sddigest.h) - FleetNest compares them with its database
bool txqSdDigest(uint32_t out[6])
{
  lock();
  bool ok = s_dg.file && s_dg.count;
  if (ok) sdDigestValues(&s_dg, out);
  unlock();
  return ok;
}

bool txqSdDigestClosed(uint32_t out[6])
{
  lock();
  bool ok = s_dgClosedNew;
  if (ok) { sdDigestValues(&s_dgClosed, out); s_dgClosedNew = false; }
  unlock();
  return ok;
}

// RESEND=<file> from the server: it repeats a command until it sees the result, so a replay
// that already runs at or before this file (and goes on to the newest) is left alone -
// restarting it every minute would never finish. Returns true when a replay was started.
bool txqResendRequest(uint32_t fileId)
{
  if (!fileId) return false;
  lock();
  bool covered = s_sdActive && s_sdFile && s_sdFile <= fileId;
  unlock();
  if (covered) return false;
  txqResendFrom(fileId);
  return true;
}
