// NestBox_v2O - OVMS v3.3 hardware map.
// Source: OVMS3 main/ovms_peripherals.h (default GPIO map, base 3.1 = what the
// v3.3 product build uses; the module's own firmware reports "product v3.3-5"
// and finds its SIM7600 on these pins). Read 2026-10-18, see ../NAVRH.md.
#pragma once

// shared SPI bus (VSPI): 2x MCP2515 + MAX7317
#define HW_SPI_MISO 19
#define HW_SPI_MOSI 23
#define HW_SPI_CLK  18
#define HW_SPI_HZ   10000000   // OVMS uses 10 MHz for all three chips

#define HW_MAX7317_CS   21
#define HW_MCP1_CS      5      // CAN2
#define HW_MCP1_INT     34
#define HW_MCP2_CS      27     // CAN3
#define HW_MCP2_INT     35

// CAN1 = ESP32 TWAI
#define HW_CAN1_TX 25
#define HW_CAN1_RX 26

// SD card, SDMMC 1-bit (CLK/CMD/D0 are the fixed SDMMC slot-1 pins)
#define HW_SD_CLK 14
#define HW_SD_CMD 15
#define HW_SD_D0  2
#define HW_SD_CD  39           // card detect (input only)

// modem SIM7600G (step 2)
#define HW_MODEM_RX 13         // ESP32 RX <- modem TX
#define HW_MODEM_TX 4

// MAX7317 ports
#define MX_MODEM_PWR 0         // modem power key
#define MX_EXT12V    1         // switched 12 V output
#define MX_CAN1_RS   2         // CAN1 transceiver RS: 1 = transmitter off (listen only), 0 = on
#define MX_MODEM_DTR 3

#define HW_SW2 0               // button SW2 (boot / firmware download)
