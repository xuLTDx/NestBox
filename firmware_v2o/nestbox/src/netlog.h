// PG99 (9.11.2026): the serial log over the network - /api/serial on the local page (web.cpp).
// Why: every driver message (ESP-IDF "E (..) sdmmc_..: .. 0x101") was on the USB serial only; on 9.11. the user
// had to move the new box U9K4M3G6 between USB and the 12 V supply again and again for it, and the SD error
// during OTA (EIO, 4 of 5 downloads) could not be seen without USB. In the car there is never USB.
// What is kept: every ESP-IDF log line (esp_log_set_vprintf - sdmmc, diskio, wifi, lwip, ...) and the lines
// given to netlogLine() (sdwriter errors), and since PG101 every print of our code (nbout).
// RAM only (PSRAM ring, NETLOG_SIZE bytes, the oldest bytes overwritten), lost on restart.
#pragma once
#include <Arduino.h>

#define NETLOG_SIZE 65536

void netlogBegin();                                   // setup(), as early as possible
void netlogLine(const char* text);                    // one of our lines (a '\n' is added)

// PG101 (9.11.2026): every Serial.print/println/printf of our code goes through nbout - to the UART as before
// AND into the ring (each line starts with "(<ms>) "). Why: /api/serial had the ESP-IDF lines only; the SD
// failure 72 s after boot (U9K4M3G6 boot 12, 0x109) could not be tied to what our code did just before.
class NbOut : public Print {
public:
  size_t write(uint8_t c) override;
  size_t write(const uint8_t* b, size_t n) override;
};
extern NbOut nbout;
// /api/serial?from=<n>: the bytes from position n (older than the ring = from the oldest kept);
// first line "head=<bytes ever written> from=<n> next=<n>", then the text
int netlogRead(uint32_t from, char* out, int cap);
