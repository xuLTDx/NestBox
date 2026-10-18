#include "max7317.h"
#include "spibus.h"
#include "hw.h"

// No bulk write at start. OVMS writes <0x0A> <1> here; with that in v2O
// (2026-10-18) every reset from outside (USB RTS / esptool) after the first run
// came up in DOWNLOAD_BOOT (GPIO0 low) until the board lost power - the
// MAX7317 keeps its ports over an ESP32 reset. Cause NOT verified (MAX7317
// register 0x0A and the board wiring not checked); only the ports that are
// needed are written, one at a time.
void mx7317Begin() {}

void mx7317Out(uint8_t port, uint8_t level)
{
  uint8_t b[2] = {port, level};
  spiXfer(HW_MAX7317_CS, b, nullptr, 2);
}
