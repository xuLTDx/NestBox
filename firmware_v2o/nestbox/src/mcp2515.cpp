// MCP2515 listen-only driver - see mcp2515.h.
#include "mcp2515.h"
#include "spibus.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "soc/gpio_struct.h"

// instructions and registers (MCP2515 data sheet; same values as OVMS3 mcp2515_regdef.h)
#define CMD_RESET       0xC0
#define CMD_READ        0x03
#define CMD_WRITE       0x02
#define CMD_BITMODIFY   0x05
#define CMD_READ_RXBUF  0x90     // + 0 = RXB0 from SIDH, + 4 = RXB1; clears RXnIF when CS goes high
#define CMD_READ_STATUS 0xA0
#define REG_BFPCTRL  0x0C
#define REG_CANSTAT  0x0E
#define REG_CANCTRL  0x0F
#define REG_REC      0x1D
#define REG_CNF3     0x28
#define REG_CANINTE  0x2B
#define REG_CANINTF  0x2C
#define REG_EFLG     0x2D
#define REG_RXB0CTRL 0x60
#define REG_RXB1CTRL 0x70
#define MODE_MASK    0xE0
#define MODE_CONFIG  0x80
#define MODE_LISTEN  0x60
#define MODE_SLEEP   0x20
#define INTF_ERRIF   0x20
#define EFLG_RX1OVR  0x80
#define EFLG_RX0OVR  0x40

uint8_t Mcp2515::readReg(uint8_t reg)
{
  uint8_t o[3] = {CMD_READ, reg, 0}, i[3];
  spiXfer(m_cs, o, i, 3);
  return i[2];
}

void Mcp2515::writeReg(uint8_t reg, uint8_t v)
{
  uint8_t o[3] = {CMD_WRITE, reg, v};
  spiXfer(m_cs, o, nullptr, 3);
}

void Mcp2515::bitModify(uint8_t reg, uint8_t mask, uint8_t v)
{
  uint8_t o[4] = {CMD_BITMODIFY, reg, mask, v};
  spiXfer(m_cs, o, nullptr, 4);
}

bool Mcp2515::setMode(uint8_t mode)
{
  bitModify(REG_CANCTRL, MODE_MASK, mode);
  for (int i = 0; i < 50; i++) {             // up to ~100 ms, as OVMS (MCP2515_TIMEOUT)
    if ((readReg(REG_CANSTAT) & MODE_MASK) == mode) return true;
    delay(2);
  }
  return false;
}

void IRAM_ATTR mcpIsr(void* arg)
{
  Mcp2515* m = (Mcp2515*)arg;
  // level interrupt: mask it until the task has emptied the chip (else it fires again at once).
  // A direct register write: gpio_intr_disable() is not in IRAM (OVMS3 mcp2515.cpp, MCP2515_isr)
  GPIO.pin[m->m_int].int_ena = 0;
  BaseType_t woken = pdFALSE;
  if (m->m_task) vTaskNotifyGiveFromISR(m->m_task, &woken);
  if (woken) portYIELD_FROM_ISR();
}

static void mcpTask(void* arg)
{
  Mcp2515* m = (Mcp2515*)arg;
  for (;;) m->service();
}

void Mcp2515::service()
{
  // woken by the interrupt; every 100 ms anyway, so a lost wake-up costs at most that
  ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100));
  if (!m_running) {
    gpio_intr_disable((gpio_num_t)m_int);
    return;
  }
  // drain: while INT is low, read what is pending
  for (int guard = 0; guard < 64 && digitalRead(m_int) == LOW; guard++) {
    uint8_t o[2] = {CMD_READ_STATUS, 0}, i[2];
    spiXfer(m_cs, o, i, 2);
    uint8_t st = i[1];                        // bit0 RX0IF, bit1 RX1IF
    for (int b = 0; b < 2; b++) {
      if (!(st & (1 << b))) continue;
      uint8_t ro[14] = {(uint8_t)(CMD_READ_RXBUF + (b ? 4 : 0))}, r[14];
      spiXfer(m_cs, ro, r, 14);               // SIDH SIDL EID8 EID0 DLC D0..D7
      CanRx f = {};
      f.us = esp_timer_get_time();
      if (r[2] & 0x08) {                      // IDE: extended
        f.ext = 1;
        f.id = ((uint32_t)r[1] << 21) | ((uint32_t)(r[2] & 0xE0) << 13) |
               ((uint32_t)(r[2] & 0x03) << 16) | ((uint32_t)r[3] << 8) | r[4];
      } else {
        f.id = ((uint32_t)r[1] << 3) | (r[2] >> 5);
      }
      f.dlc = r[5] & 0x0F;
      if (f.dlc > 8) f.dlc = 8;
      memcpy(f.data, r + 6, 8);
      m_st.frames++;
      if (m_fn) m_fn(m_bus, f);
      if (m_capId && f.id == m_capId && !f.ext && m_cap) xQueueSend(m_cap, &f, 0);
    }
    // errors: overflow = a frame lost in the chip; count it, clear the flags
    uint8_t intf = readReg(REG_CANINTF);
    if (intf & INTF_ERRIF) {
      m_st.errIrq++;
      uint8_t ef = readReg(REG_EFLG);
      m_st.eflg = ef;
      if (ef & EFLG_RX0OVR) m_st.rxOverflow++;
      if (ef & EFLG_RX1OVR) m_st.rxOverflow++;
      if (ef & (EFLG_RX0OVR | EFLG_RX1OVR)) bitModify(REG_EFLG, EFLG_RX0OVR | EFLG_RX1OVR, 0);
      m_st.rec = readReg(REG_REC);
      bitModify(REG_CANINTF, INTF_ERRIF, 0);
    }
    // any other flag (MERRF, WAKIF; TX flags never set - we never transmit)
    intf = readReg(REG_CANINTF);
    if (intf & 0xDC) bitModify(REG_CANINTF, intf & 0xDC, 0);
  }
  if (m_running) gpio_intr_enable((gpio_num_t)m_int);
}

bool Mcp2515::begin(CanRxFn fn)
{
  m_fn = fn;
  pinMode(m_int, INPUT);                      // GPIO 34/35: input only, the board has the pull-up
  uint8_t r = CMD_RESET;
  spiXfer(m_cs, &r, nullptr, 1);
  delay(10);
  // after reset the chip is in configuration mode (CANSTAT 0x80) - that is the probe
  m_present = (readReg(REG_CANSTAT) & MODE_MASK) == MODE_CONFIG;
  if (!m_present) {
    Serial.printf("[CAN%u] MCP2515 not answering\n", m_bus);
    return false;
  }
  writeReg(REG_BFPCTRL, 0x3C);                // transceiver transmitter off (OVMS: listen mode)
  setMode(MODE_SLEEP);
  xTaskCreatePinnedToCore(mcpTask, m_bus == 2 ? "can2rx" : "can3rx", 3072, this, 10, &m_task, 0);
  gpio_set_intr_type((gpio_num_t)m_int, GPIO_INTR_LOW_LEVEL);
  gpio_install_isr_service(ESP_INTR_FLAG_IRAM);  // already installed = harmless error
  gpio_isr_handler_add((gpio_num_t)m_int, mcpIsr, this);
  gpio_intr_disable((gpio_num_t)m_int);
  return true;
}

bool Mcp2515::start(uint32_t kbit)
{
  if (!m_present) return false;
  m_kbit = kbit;
  stop();
  uint8_t r = CMD_RESET;
  spiXfer(m_cs, &r, nullptr, 1);
  delay(10);
  writeReg(REG_CANINTE, 0);
  // bit timing for the board's 16 MHz crystal - the values OVMS uses (mcp2515.cpp Start())
  uint8_t c1, c2, c3;
  switch (kbit) {
    case 250: c1 = 0x41; c2 = 0xF1; c3 = 0x85; break;
    case 125: c1 = 0x03; c2 = 0xF0; c3 = 0x86; break;
    default:  c1 = 0x00; c2 = 0xF0; c3 = 0x86; break;   // 500
  }
  uint8_t o[5] = {CMD_WRITE, REG_CNF3, c3, c2, c1};     // CNF3, CNF2, CNF1 are consecutive
  spiXfer(m_cs, o, nullptr, 5);
  writeReg(REG_RXB0CTRL, 0x64);               // any frame, roll over to RXB1 when RXB0 is full
  writeReg(REG_RXB1CTRL, 0x60);               // any frame
  writeReg(REG_BFPCTRL, 0x3C);                // transmitter off
  uint8_t back[3];
  uint8_t ro[5] = {CMD_READ, REG_CNF3, 0, 0, 0}, ri[5];
  spiXfer(m_cs, ro, ri, 5);
  back[0] = ri[2]; back[1] = ri[3]; back[2] = ri[4];
  if (back[0] != c3 || back[1] != c2 || back[2] != c1) {
    Serial.printf("[CAN%u] bit timing not taken (%02X %02X %02X)\n", m_bus, back[2], back[1], back[0]);
    return false;
  }
  writeReg(REG_CANINTE, 0x23);                // RX0, RX1, error
  if (!setMode(MODE_LISTEN)) {
    Serial.printf("[CAN%u] listen mode not taken\n", m_bus);
    return false;
  }
  m_running = true;
  gpio_intr_enable((gpio_num_t)m_int);
  xTaskNotifyGive(m_task);                    // anything already pending
  return true;
}

void Mcp2515::stop()
{
  if (!m_present) return;
  m_running = false;
  gpio_intr_disable((gpio_num_t)m_int);
  writeReg(REG_CANINTE, 0);
  setMode(MODE_SLEEP);
}

// ---- READ exchange -------------------------------------------------------
#define REG_TXB0CTRL 0x30
#define CMD_LOAD_TX0 0x40     // load TX buffer 0 from TXB0SIDH
#define CMD_RTS_TX0  0x81
#define MODE_NORMAL  0x00

bool Mcp2515::exchangeBegin(uint32_t kbit, uint32_t rxId)
{
  if (!m_present) return false;
  m_wasRunning = m_running;
  if (!m_running && !start(kbit)) return false;
  if (!m_cap) m_cap = xQueueCreate(8, sizeof(CanRx));
  xQueueReset(m_cap);
  m_capId = rxId;                             // before the request goes out: the reply can be quick
  writeReg(REG_BFPCTRL, 0x0C);               // transceiver transmitter on (OVMS SetTransceiverMode ACTIVE)
  return setMode(MODE_NORMAL);
}

bool Mcp2515::sendFrame(uint32_t id, const uint8_t d[8])
{
  uint8_t o[14] = {CMD_LOAD_TX0, (uint8_t)(id >> 3), (uint8_t)(id << 5), 0, 0, 8};
  memcpy(o + 6, d, 8);
  spiXfer(m_cs, o, nullptr, 14);
  uint8_t rts = CMD_RTS_TX0;
  spiXfer(m_cs, &rts, nullptr, 1);
  for (int i = 0; i < 25; i++) {              // up to ~50 ms for the frame to leave
    if (!(readReg(REG_TXB0CTRL) & 0x08)) return true;   // TXREQ cleared = sent
    delay(2);
  }
  bitModify(REG_CANCTRL, 0x10, 0x10);         // abort (ABAT): never repeated on the bus
  delay(2);
  bitModify(REG_CANCTRL, 0x10, 0x00);
  return false;
}

bool Mcp2515::waitFrame(uint32_t id, uint8_t d[8], uint8_t* dlc, uint32_t timeoutMs)
{
  (void)id;                                   // m_capId set by exchangeBegin()
  CanRx f;
  bool got = m_cap && xQueueReceive(m_cap, &f, pdMS_TO_TICKS(timeoutMs)) == pdTRUE;
  if (got) { memcpy(d, f.data, 8); *dlc = f.dlc; }
  return got;
}

void Mcp2515::exchangeEnd()
{
  m_capId = 0;
  writeReg(REG_BFPCTRL, 0x3C);               // transmitter off
  if (m_wasRunning) setMode(MODE_LISTEN);
  else stop();
}
