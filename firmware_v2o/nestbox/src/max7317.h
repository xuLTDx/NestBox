// MAX7317 port expander (SPI): modem power key, CAN1 transceiver RS, 12 V out.
// Protocol as OVMS3 components/max7317 uses it: a write is 2 bytes
// <port 0..9> <level>. A port written 1 is high (open drain released), 0 is low.
// OVMS also writes <0x0A> <1> at start - v2O does not (see max7317.cpp).
#pragma once
#include <Arduino.h>

void mx7317Begin();
void mx7317Out(uint8_t port, uint8_t level);
