// Debug log to SD, switched on/off (PG175, 12.11.2026).
// Why: Vojtech 27.10. ("Prečo nemáš debug kde ti to viazne") and 12.11. ("debug log ... musí byť, zapnúť vypnúť"):
// every field problem so far (CMUX 27.10, Peugeot 11 h stop 11.-12.11, a3 PANIC 12.11) had no log of the moment.
//  - DEBUG=1 / DEBUG=0 (FleetNest command, web /api/control), kept in NVS, default off: when on, everything of the
//    /api/serial ring (our lines + ESP-IDF log, netlog.cpp) is appended to /DEBUG/D<boot>.LOG every 2 s.
//    Not written synchronously from the print itself: v1F's Serial->SD mirror (16.9.2026) reset the box when /api/list
//    printed while iterating the card; here a separate task copies the ring, so a print never touches the card.
//    Bytes the ring overwrote before the copy (card slow / busy) are counted in the file ("[DBG] lost <n> bytes").
//  - always, also with DEBUG off: after a panic / watchdog restart the boot before's last 64 KB of the ring
//    (netlogTakePrev) -> /DEBUG/C<that boot>.LOG once, status line "DBG crash log C<boot> <bytes>".
//  - files in /DEBUG older than 7 days are deleted (hourly, with a valid clock).
//  - fetch: FleetNest LOG=D<boot> / LOG=C<boot> (ota.cpp, the PG49 upload; server file id 1000000+ / 2000000+boot).
#pragma once
#include <stdint.h>
#include <time.h>

void dbglogBegin(uint32_t bootId, bool sdOk);   // setup(), after the NVS handle is open (telemetryBegin)
bool dbglogSet(bool on);                        // stored in NVS; true = done
bool dbglogOn();
void dbglogPurge(time_t now);                   // loop(), hourly

// PG270 (18.11.2026): the detail lines are printed only with DEBUG=1 - on the UART, in /api/serial and so on the SD.
// Why: Vojtech 27.10. and 12.11. asked for a debug log to switch on and off; PG175 switched only the SD copy, the
// detail lines ("[WIFI] 79 bytes sent" + "[UDP] 1#ACK=.." every 5 s, "| heap=" after each OTA line, SDPINS at boot)
// stayed on with DEBUG=0 (2Z 18.11.: they were most of the ring). With DEBUG=0 the status lines and errors stay.
// With DEBUG=1 also "[MEM] ..." every 60 s (internal heap, its minimum, largest DMA block, PSRAM) - 2Z 18.11. HTTPS
// failed at a largest block of 27-37 KB and worked at 45 KB, the log had the memory only next to OTA lines.
// Before dbglogBegin() (NVS not read yet) DEBUG counts as off.
#include <Print.h>
class DbgOut : public Print {
public:
  size_t write(uint8_t c) override;
  size_t write(const uint8_t* b, size_t n) override;
};
extern DbgOut dbgout;                           // as nbout, but only with DEBUG=1
void dbgLine(const char* text);                 // as netlogLine, but only with DEBUG=1
// PG273 (18.11.2026): "[MEM] <where> heap= min= dma_max=" right at a step, only with DEBUG=1. Why: 2Z lost 5-15 KB
// of internal RAM (HTTPS then failed at <=74 KB); measured only: the first web request -4.3 KB (boot 6). These lines
// show what the own WiFi (AP on/off) takes. (A per-pass "[MEM] web" line was tried 18.11. and removed: other tasks
// change the heap during the server wait, so it was noise; the web effect is read from the minute [MEM] lines.)
void dbgMem(const char* where);
