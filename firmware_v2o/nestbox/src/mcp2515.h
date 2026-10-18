// MCP2515 CAN controller (CAN2 / CAN3 of OVMS v3), LISTEN ONLY.
//
// Written for NestBox_v2O; OVMS3 components/mcp2515 was the reference for the
// register values (bit timing for its 16 MHz crystal, RX buffer control,
// BFPCTRL = transceiver transmitter off) and for the one lesson its history
// cost them (07-08/2026): the INT pin is a LEVEL, held low while any flag is
// set - an edge interrupt can be missed and the receive path then hangs. Here
// the interrupt only wakes a task; the task masks the pin, empties both RX
// buffers until INT is high again, and only then unmasks it.
//
// Nothing is lost silently: the chip has two RX buffers, a frame that comes
// while both are full is lost in the chip - EFLG RX0OVR/RX1OVR - and counted
// (rxOverflow). The controller never transmits: listen-only mode (no ACK, no
// error frames) and the transceiver's transmitter switched off.
#pragma once
#include <Arduino.h>

struct CanRx {                 // one received frame
  int64_t us;                  // esp_timer time of reading it from the chip
  uint32_t id;
  uint8_t ext, dlc;
  uint8_t data[8];
};
typedef void (*CanRxFn)(uint8_t bus, const CanRx& f);

struct McpStats {
  uint32_t frames;             // read from the chip
  uint32_t rxOverflow;         // lost in the chip (both buffers full)
  uint32_t errIrq;             // error interrupts (EFLG changes)
  uint8_t eflg, rec;           // last error flags, receive error counter
};

class Mcp2515 {
public:
  Mcp2515(uint8_t bus, int cs, int intPin) : m_bus(bus), m_cs(cs), m_int(intPin) {}
  // probe the chip; false = not answering (the bus is then never started)
  bool begin(CanRxFn fn);
  bool start(uint32_t kbit);   // listen only at 500 / 250 / 125 kbit/s
  void stop();                 // reset + sleep
  bool running() const { return m_running; }
  McpStats stats() const { return m_st; }
  uint8_t bus() const { return m_bus; }
  void service();              // the RX task (internal)
  // READ command (nbread.h): normal mode + transmitter on for the exchange only
  bool exchangeBegin(uint32_t kbit, uint32_t rxId);   // starts the bus if it was off; frames from rxId are kept
  bool sendFrame(uint32_t id, const uint8_t d[8]);
  bool waitFrame(uint32_t id, uint8_t d[8], uint8_t* dlc, uint32_t timeoutMs);
  void exchangeEnd();                  // listen only again (or off, as before)
  uint32_t speed() const { return m_kbit; }
private:
  uint8_t readReg(uint8_t reg);
  void writeReg(uint8_t reg, uint8_t v);
  void bitModify(uint8_t reg, uint8_t mask, uint8_t v);
  bool setMode(uint8_t mode);
  uint8_t m_bus;
  int m_cs, m_int;
  bool m_present = false;
  volatile bool m_running = false;
  CanRxFn m_fn = nullptr;
  TaskHandle_t m_task = nullptr;
  McpStats m_st = {};
  uint32_t m_kbit = 500;
  bool m_wasRunning = false;
  volatile uint32_t m_capId = 0;               // READ: frames from this ID go to m_cap
  QueueHandle_t m_cap = nullptr;
  friend void IRAM_ATTR mcpIsr(void*);
};
