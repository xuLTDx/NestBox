// NestBox_v2O: the SD card is on SDMMC (SD_MMC), not SPI. The code carried over
// from v1F calls "SD."; both are fs::FS with the same calls - this header is
// the only difference. The card is mounted once in main.cpp (sdMount).
#pragma once
#include <SD_MMC.h>
#define SD SD_MMC
