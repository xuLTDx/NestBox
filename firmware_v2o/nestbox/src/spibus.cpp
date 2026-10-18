#include "spibus.h"
#include <SPI.h>
#include "hw.h"

static SPIClass s_spi(VSPI);
static SemaphoreHandle_t s_mux;

void spiBusBegin()
{
  if (s_mux) return;
  s_mux = xSemaphoreCreateMutex();
  const int cs[] = {HW_MAX7317_CS, HW_MCP1_CS, HW_MCP2_CS};
  for (int p : cs) {           // all chips deselected before the bus starts (OVMS does the same)
    pinMode(p, OUTPUT);
    digitalWrite(p, HIGH);
  }
  s_spi.begin(HW_SPI_CLK, HW_SPI_MISO, HW_SPI_MOSI, -1);
}

void spiXfer(int cs, const uint8_t* out, uint8_t* in, int len)
{
  xSemaphoreTake(s_mux, portMAX_DELAY);
  s_spi.beginTransaction(SPISettings(HW_SPI_HZ, MSBFIRST, SPI_MODE0));
  digitalWrite(cs, LOW);
  for (int i = 0; i < len; i++) {
    uint8_t r = s_spi.transfer(out ? out[i] : 0);
    if (in) in[i] = r;
  }
  digitalWrite(cs, HIGH);
  s_spi.endTransaction();
  xSemaphoreGive(s_mux);
}
