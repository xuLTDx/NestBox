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

static bool armed()
{
  uint8_t v = 0;
  nvs_get_u8(nvs, "PART_CD", &v);
  return v == 1;
}

bool partArm(bool on)
{
  s_tried = false;
  return nvs_set_u8(nvs, "PART_CD", on ? 1 : 0) == ESP_OK && nvs_commit(nvs) == ESP_OK;
}

static int tableKind(const uint8_t* t)     // 0 unknown, 1 old, 2 new
{
  auto rest = [&](int from) { for (int i = from; i < PART_TABLE_LEN; i++) if (t[i] != 0xFF) return false; return true; };
  if (!memcmp(t, PART_OLD, sizeof(PART_OLD)) && rest(sizeof(PART_OLD))) return 1;
  if (!memcmp(t, PART_NEW, sizeof(PART_NEW)) && rest(sizeof(PART_NEW))) return 2;
  return 0;
}

const char* partState()
{
  static char s[40];
  uint8_t* t = (uint8_t*)ps_malloc(PART_TABLE_LEN);
  int k = t && esp_flash_read(NULL, t, PART_ADDR, PART_TABLE_LEN) == ESP_OK ? tableKind(t) : -1;
  free(t);
  snprintf(s, sizeof(s), "table=%s armed=%d", k == 1 ? "old" : k == 2 ? "new" : k == 0 ? "unknown" : "readerr",
           armed() ? 1 : 0);
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
  if (!armed()) { s_highSince = 0; return; }
  float v = read12V();
  // under 7 V = no 12 V at all = the bench on USB (as ota.cpp flashAllowed()); in a car the 12 V is never under 7 V.
  // Why allowed: the first test of the change is on USB to see the boot on the serial port (Vojtech 12.11.)
  if (v >= 7.0f && v < PART_MIN_V) { s_highSince = 0; return; }
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
  if (k == 2) { partArm(false); statusLine("PART CD already"); return; }
  if (k != 1) { partArm(false); statusLine("PART ERR unknown table - nothing written"); return; }
  for (int i = PART_TABLE_LEN; i < PART_SECTOR; i++)
    if (s_sector[i] != 0xFF) { partArm(false); statusLine("PART ERR sector tail not empty"); return; }
  memset(s_want, 0xFF, PART_SECTOR);
  memcpy(s_want, PART_NEW, sizeof(PART_NEW));
  partArm(false);                            // first: a crash / restart during the change never repeats it by itself
  nvs_set_u8(nvs, "PART_NEW", 1);
  nvs_commit(nvs);
  logNetEvent("PART CD write start");
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
  statusLine("PART CD written - restart");
  logNetEvent("PART CD written - restart");
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
  if (cd) snprintf(m, sizeof(m), "PART OK coredump %luK @%lx spiffs %luK", (unsigned long)(cd->size / 1024),
                   (unsigned long)cd->address, sp ? (unsigned long)(sp->size / 1024) : 0UL);
  else snprintf(m, sizeof(m), "PART ERR no coredump after restart");
  statusLine(m);
  logNetEvent(m);
}
