#include "max7317.h"
#include "spibus.h"
#include "hw.h"

void mx7317Begin()
{
  uint8_t b[2] = {0x0A, 1};    // as OVMS max7317::max7317()
  spiXfer(HW_MAX7317_CS, b, nullptr, 2);
}

void mx7317Out(uint8_t port, uint8_t level)
{
  uint8_t b[2] = {port, level};
  spiXfer(HW_MAX7317_CS, b, nullptr, 2);
}
