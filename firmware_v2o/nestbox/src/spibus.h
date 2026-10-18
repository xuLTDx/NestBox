// The shared SPI bus (VSPI) of OVMS v3: MAX7317 and both MCP2515. One mutex,
// chip select by hand, so a transfer of one chip is never cut by another.
#pragma once
#include <Arduino.h>

void spiBusBegin();
// out = bytes to send, in = bytes received (same length, may be null)
void spiXfer(int cs, const uint8_t* out, uint8_t* in, int len);
