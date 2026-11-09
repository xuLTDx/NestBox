// PG99 (9.11.2026): the serial log over the network - /api/serial on the local page (web.cpp).
// Why: every driver message (ESP-IDF "E (..) sdmmc_..: .. 0x101") was on the USB serial only; on 9.11. the user
// had to move the new box U9K4M3G6 between USB and the 12 V supply again and again for it, and the SD error
// during OTA (EIO, 4 of 5 downloads) could not be seen without USB. In the car there is never USB.
// What is kept: every ESP-IDF log line (esp_log_set_vprintf - sdmmc, diskio, wifi, lwip, ...) and the lines
// given to netlogLine() (sdwriter errors). Not every Serial.printf of our code - those go to the UART only.
// RAM only (PSRAM ring, NETLOG_SIZE bytes, the oldest bytes overwritten), lost on restart.
#pragma once
#include <Arduino.h>

#define NETLOG_SIZE 65536

void netlogBegin();                                   // setup(), as early as possible
void netlogLine(const char* text);                    // one of our lines (a '\n' is added)
// /api/serial?from=<n>: the bytes from position n (older than the ring = from the oldest kept);
// first line "head=<bytes ever written> from=<n> next=<n>", then the text
int netlogRead(uint32_t from, char* out, int cap);
