// One-time partition table change - see part.h (PG177, 12.11.2026)
#include "part.h"
#include <Arduino.h>
#include <string.h>
#include <nvs.h>
#include <esp_flash.h>
#include <esp_partition.h>
#include "txqueue.h"
#include "ota.h"
#include "otaverify.h"

extern nvs_handle_t nvs;                     // telemetry.cpp
void statusLine(const char* text);           // ota.cpp
void logNetEvent(const char* text);          // telemetry.cpp
extern const char FW_BUILD_STR[];            // main.cpp
void prepareRestart(const char* why);        // main.cpp
float read12V();                             // telemetry.cpp

#define PART_ADDR       0x8000               // CONFIG_PARTITION_TABLE_OFFSET (libs sdkconfig)
#define PART_SECTOR     0x1000
#define PART_TABLE_LEN  0xC00                // ESP_PARTITION_TABLE_MAX_LEN
#define PART_MIN_V      13.0f                // engine running (charging) or the bench supply 13.5 V
#define PART_STABLE_MS  30000UL              // that long without a break. Why 30 s: a crank / a single low reading never
                                             // passes (PG148: single false readings 12.33-12.58 V on a steady 13.5 V)

// generated 12.11.2026 with ESP-IDF 5.5 gen_esp32part.py: OLD = partitions_v2o.csv (Peugeot USB 18.10., U9K2M3J0),
// NEW = + "coredump, data, coredump, 0xFF0000, 0x10000", spiffs 0x7E0000 (/home/claude/Claude/pg177); MD5 entry
// included and checked; the rest of the table area is 0xFF in both
static const uint8_t PART_OLD[192] = {
  0xaa, 0x50, 0x01, 0x02, 0x00, 0x90, 0x00, 0x00, 0x00, 0x50, 0x00, 0x00, 0x6e, 0x76, 0x73, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0xaa, 0x50, 0x01, 0x00, 0x00, 0xe0, 0x00, 0x00, 0x00, 0x20, 0x00, 0x00, 0x6f, 0x74, 0x61, 0x64,
  0x61, 0x74, 0x61, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0xaa, 0x50, 0x00, 0x10, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x40, 0x00, 0x61, 0x70, 0x70, 0x30,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0xaa, 0x50, 0x00, 0x11, 0x00, 0x00, 0x41, 0x00, 0x00, 0x00, 0x40, 0x00, 0x61, 0x70, 0x70, 0x31,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0xaa, 0x50, 0x01, 0x82, 0x00, 0x00, 0x81, 0x00, 0x00, 0x00, 0x7f, 0x00, 0x73, 0x70, 0x69, 0x66,
  0x66, 0x73, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0xeb, 0xeb, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
  0x6a, 0x41, 0xd0, 0xe0, 0xbd, 0x5c, 0xb0, 0xc3, 0xe4, 0x9f, 0x8b, 0x3d, 0xa9, 0xfe, 0xf9, 0x2f,
};
static const uint8_t PART_NEW[224] = {
  0xaa, 0x50, 0x01, 0x02, 0x00, 0x90, 0x00, 0x00, 0x00, 0x50, 0x00, 0x00, 0x6e, 0x76, 0x73, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0xaa, 0x50, 0x01, 0x00, 0x00, 0xe0, 0x00, 0x00, 0x00, 0x20, 0x00, 0x00, 0x6f, 0x74, 0x61, 0x64,
  0x61, 0x74, 0x61, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0xaa, 0x50, 0x00, 0x10, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x40, 0x00, 0x61, 0x70, 0x70, 0x30,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0xaa, 0x50, 0x00, 0x11, 0x00, 0x00, 0x41, 0x00, 0x00, 0x00, 0x40, 0x00, 0x61, 0x70, 0x70, 0x31,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0xaa, 0x50, 0x01, 0x82, 0x00, 0x00, 0x81, 0x00, 0x00, 0x00, 0x7e, 0x00, 0x73, 0x70, 0x69, 0x66,
  0x66, 0x73, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0xaa, 0x50, 0x01, 0x03, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 0x01, 0x00, 0x63, 0x6f, 0x72, 0x65,
  0x64, 0x75, 0x6d, 0x70, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0xeb, 0xeb, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
  0x53, 0x28, 0xc1, 0x19, 0x17, 0x02, 0x6a, 0xaa, 0x50, 0xc3, 0x54, 0x99, 0x83, 0xfd, 0xed, 0x91,
};

// PG201 (13.11.2026): the 3 sector buffers come from PSRAM only while the change runs. Why: as statics (15 KB of
// internal RAM for good) the free heap after login fell from ~94 KB (12.11.) to ~79 KB and the HTTPS connection over
// WiFi with PPP up failed at once ("OTA-PULL ERR=META_HTTP-1", "LOG ... ERR http=-1", bench 13.11. 17:41, largest
// block 34.8 KB) - mbedTLS needs its buffers from internal RAM.
static uint8_t* s_sector;                    // the whole sector as it is in flash (the old one, for the way back)
static uint8_t* s_want;
static uint8_t* s_back;
static uint32_t s_highSince;
static bool s_tried;                         // once per boot

// PG300 (23.11.2026, Vojtech "300 áno" + "toto treba mať vo fw ak sa náhodou zase bude meniť tabuľka oddielov"):
// the change is AUTOMATIC and GENERAL. The firmware knows a list of tables (TABLES, oldest first); the last one is the
// TARGET. A box whose table is one of the older known ones changes to the target by itself after its build is
// confirmed, at the first stable >= 13.0 V for 30 s (engine running) - never on USB (< 7 V: unplugging the cable at
// the wrong moment = no boot). Why: with PG177 the change waited for a FleetNest command PART=CD sent by a person who
// knows the engine runs (J0 23.11.: "to zase niekto musí sedieť pri tebe a povoliť?"); a box never sent it keeps the
// old table for ever. A NEXT table change = add its bytes to TABLES as the new last entry, nothing else.
// Safety for any pair: nvs, otadata and both app partitions must be byte for byte the same entries in both tables
// (sameCore) - the running app, its settings and the OTA state never move; else "PART ERR moves ..." and nothing written.
// NVS: PART_CD = 1 armed by PART=CD (also on USB, bench - Vojtech 12.11.); PART_SKIP / PART_OFF see skipped().
struct KnownTable { const char* name; const uint8_t* data; uint16_t len; };
static const KnownTable TABLES[] = {
  {"v2o", PART_OLD, sizeof(PART_OLD)},       // partitions_v2o.csv (Peugeot USB 18.10.)
  {"v2o_cd", PART_NEW, sizeof(PART_NEW)},    // + coredump 64 KB (PG177)
};
#define N_TABLES (int)(sizeof(TABLES) / sizeof(TABLES[0]))
#define TARGET (N_TABLES - 1)

// PART_SKIP = "<target>@<build>": tried by this build - not again by itself; a NEWER build tries once more (Vojtech
// 23.11.: "má ísť cez fw" - 2Z 12:32 the try of build 12:30:44 hit the LittleFS assert, the fixed build 12:47:56 must
// try again by itself, no command). PART_OFF = "<target>": PART=0, off for good for this target (any build).
static void skipKey(char* out, size_t cap) { snprintf(out, cap, "%s@%s", TABLES[TARGET].name, FW_BUILD_STR); }
static bool strIs(const char* key, const char* want)
{
  char n[48] = "";
  size_t l = sizeof(n);
  return nvs_get_str(nvs, key, n, &l) == ESP_OK && !strcmp(n, want);
}
static bool skipped()
{
  char k[48];
  skipKey(k, sizeof(k));
  return strIs("PART_OFF", TABLES[TARGET].name) || strIs("PART_SKIP", k);
}
static void skipTarget()                     // this build: not again by itself
{
  char k[48];
  skipKey(k, sizeof(k));
  nvs_set_str(nvs, "PART_SKIP", k);
  nvs_set_u8(nvs, "PART_CD", 0);
  nvs_commit(nvs);
}
static bool armed()
{
  uint8_t v = 0;
  nvs_get_u8(nvs, "PART_CD", &v);
  return v == 1;
}

bool partArm(bool on)
{
  s_tried = false;
  if (!on) { nvs_set_str(nvs, "PART_OFF", TABLES[TARGET].name); skipTarget(); return true; }
  nvs_erase_key(nvs, "PART_SKIP");
  nvs_erase_key(nvs, "PART_OFF");
  return nvs_set_u8(nvs, "PART_CD", 1) == ESP_OK && nvs_commit(nvs) == ESP_OK;
}

static int tableKind(const uint8_t* t)     // index in TABLES, -1 unknown
{
  for (int k = 0; k < N_TABLES; k++) {
    if (memcmp(t, TABLES[k].data, TABLES[k].len)) continue;
    bool rest = true;
    for (int i = TABLES[k].len; i < PART_TABLE_LEN && rest; i++) if (t[i] != 0xFF) rest = false;
    if (rest) return k;
  }
  return -1;
}

// entries of 32 bytes: 0xAA 0x50, type, subtype, offset, size, label[16], flags; ends at 0xEB 0xEB (MD5) / 0xFF
static const uint8_t* findEntry(const KnownTable& t, uint8_t type, uint8_t sub)
{
  for (int i = 0; i + 32 <= t.len && t.data[i] == 0xAA && t.data[i + 1] == 0x50; i += 32)
    if (t.data[i + 2] == type && t.data[i + 3] == sub) return t.data + i;
  return nullptr;
}
static const char* sameCore(const KnownTable& a, const KnownTable& b)   // nullptr = safe, else what would move
{
  static const struct { uint8_t type, sub; const char* name; } core[] = {
    {0x01, 0x02, "nvs"}, {0x01, 0x00, "otadata"}, {0x00, 0x10, "app0"}, {0x00, 0x11, "app1"}};
  for (auto& c : core) {
    const uint8_t* x = findEntry(a, c.type, c.sub);
    const uint8_t* y = findEntry(b, c.type, c.sub);
    if (!x || !y || memcmp(x, y, 32)) return c.name;
  }
  return nullptr;
}

const char* partState()
{
  static char s[56];
  uint8_t* t = (uint8_t*)ps_malloc(PART_TABLE_LEN);
  bool rd = t && esp_flash_read(NULL, t, PART_ADDR, PART_TABLE_LEN) == ESP_OK;
  int k = rd ? tableKind(t) : -2;
  free(t);
  snprintf(s, sizeof(s), "table=%s target=%s mode=%s", k >= 0 ? TABLES[k].name : k == -1 ? "unknown" : "readerr",
           TABLES[TARGET].name, armed() ? "armed" : skipped() ? "off" : "auto");
  return s;
}

// region_protected of the copy: the table sector is allowed, everything else asks the original hook
static const esp_flash_os_functions_t* s_origOs;
static esp_err_t allowTable(void* arg, size_t start, size_t size)
{
  if (start >= PART_ADDR && start + size <= PART_ADDR + PART_SECTOR) return ESP_OK;
  return s_origOs->region_protected ? s_origOs->region_protected(arg, start, size) : ESP_OK;
}

static bool writeSector(const uint8_t* data)
{
  esp_flash_t* chip = esp_flash_default_chip;
  static esp_flash_os_functions_t os;
  s_origOs = chip->os_func;
  os = *s_origOs;
  os.region_protected = allowTable;
  chip->os_func = &os;
  esp_err_t e = esp_flash_erase_region(chip, PART_ADDR, PART_SECTOR);
  if (e == ESP_OK) e = esp_flash_write(chip, data, PART_ADDR, PART_SECTOR);
  chip->os_func = s_origOs;
  return e == ESP_OK;
}

void partTick()
{
  if (s_tried) return;
  static uint32_t lastCheck;
  if (millis() - lastCheck < 1000) return;
  lastCheck = millis();
  bool arm = armed();
  if (!arm && skipped()) { s_highSince = 0; return; }
  if (!arm) {                                // PG300 auto: only a box with an OLDER known table; read once per boot
    static int8_t kind = -2;
    if (kind == -2) {
      uint8_t* t = (uint8_t*)ps_malloc(PART_TABLE_LEN);
      kind = t && esp_flash_read(NULL, t, PART_ADDR, PART_TABLE_LEN) == ESP_OK ? tableKind(t) : -1;
      free(t);
      if (kind == -1) { skipTarget(); statusLine("PART ERR unknown table - nothing written"); }
    }
    if (kind < 0 || kind == TARGET) { s_tried = true; return; }   // the target already (normal): nothing, no line
  }
  float v = read12V();
  // under 7 V = no 12 V at all = the bench on USB (as ota.cpp flashAllowed()); in a car the 12 V is never under 7 V.
  // Why allowed: the first test of the change is on USB to see the boot on the serial port (Vojtech 12.11.) -
  // only when armed by PART=CD, never automatic (PG300)
  if ((!arm || v >= 7.0f) && v < PART_MIN_V) { s_highSince = 0; return; }
  if (!s_highSince) s_highSince = millis();
  if (millis() - s_highSince < PART_STABLE_MS) return;
  // the queue survives the restart (PSRAM reserve + persist on esp_restart, txqueue.cpp); <= 5 = only the live
  // records of the last seconds (1 per second while the engine runs, ACK ~1-2 s) - an old backlog (standby records in
  // SPIFFS, which may be formatted) waits until delivered
  if (txqPending() > 5 || otaBusy() || otaVerifyPending()) return;
  s_tried = true;
  char m[64];
  if (!s_sector) s_sector = (uint8_t*)ps_malloc(PART_SECTOR);
  if (!s_want) s_want = (uint8_t*)ps_malloc(PART_SECTOR);
  if (!s_back) s_back = (uint8_t*)ps_malloc(PART_SECTOR);
  if (!s_sector || !s_want || !s_back) { statusLine("PART ERR no memory"); return; }
  if (esp_flash_read(NULL, s_sector, PART_ADDR, PART_SECTOR) != ESP_OK) { statusLine("PART ERR read"); return; }
  int k = tableKind(s_sector);
  if (k == TARGET) { skipTarget(); snprintf(m, sizeof(m), "PART %s already", TABLES[TARGET].name); statusLine(m); return; }
  if (k < 0) { skipTarget(); statusLine("PART ERR unknown table - nothing written"); return; }
  const char* moves = sameCore(TABLES[k], TABLES[TARGET]);
  if (moves) { skipTarget(); snprintf(m, sizeof(m), "PART ERR moves %s - nothing written", moves); statusLine(m); return; }
  for (int i = PART_TABLE_LEN; i < PART_SECTOR; i++)
    if (s_sector[i] != 0xFF) { skipTarget(); statusLine("PART ERR sector tail not empty"); return; }
  memset(s_want, 0xFF, PART_SECTOR);
  memcpy(s_want, TABLES[TARGET].data, TABLES[TARGET].len);
  skipTarget();                              // first: a crash / restart during the change never repeats it by itself
  nvs_set_u8(nvs, "PART_NEW", 1);
  nvs_commit(nvs);
  snprintf(m, sizeof(m), "PART %s -> %s write start", TABLES[k].name, TABLES[TARGET].name);
  logNetEvent(m);
  bool ok = writeSector(s_want);
  uint8_t* back = s_back;
  bool same = ok && esp_flash_read(NULL, back, PART_ADDR, PART_SECTOR) == ESP_OK && !memcmp(back, s_want, PART_SECTOR);
  if (!same) {
    bool restored = writeSector(s_sector) && esp_flash_read(NULL, back, PART_ADDR, PART_SECTOR) == ESP_OK &&
                    !memcmp(back, s_sector, PART_SECTOR);
    nvs_set_u8(nvs, "PART_NEW", 0);
    nvs_commit(nvs);
    snprintf(m, sizeof(m), "PART ERR verify (write=%d) old %s", ok ? 1 : 0, restored ? "restored" : "NOT restored");
    statusLine(m);
    logNetEvent(m);
    return;
  }
  snprintf(m, sizeof(m), "PART %s written - restart", TABLES[TARGET].name);
  statusLine(m);
  logNetEvent(m);
  delay(5000);                               // the status line into a record (the queue survives the restart)
  prepareRestart("PART_CD");
  esp_restart();
}

void partBootReport()
{
  uint8_t v = 0;
  if (nvs_get_u8(nvs, "PART_NEW", &v) != ESP_OK || v != 1) return;
  nvs_set_u8(nvs, "PART_NEW", 0);
  nvs_commit(nvs);
  const esp_partition_t* cd = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_COREDUMP, NULL);
  const esp_partition_t* sp = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_SPIFFS, NULL);
  char m[64];
  if (cd) snprintf(m, sizeof(m), "PART OK %s coredump %luK @%lx spiffs %luK", TABLES[TARGET].name, (unsigned long)(cd->size / 1024),
                   (unsigned long)cd->address, sp ? (unsigned long)(sp->size / 1024) : 0UL);
  else snprintf(m, sizeof(m), "PART ERR no coredump after restart");
  statusLine(m);
  logNetEvent(m);
}
