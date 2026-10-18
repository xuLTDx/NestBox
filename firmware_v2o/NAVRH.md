# NestBox_v2O – návrh (na schválenie)

Firmvér NestBox pre hardvér **OVMS v3**. Rozhodnutie (Vojtech 2026-10-18): z NestBox_v1F
(`firmware_v5/telelogger/`) sa berie **len kostra a náš overený kód**; ďalšie funkcie sa
pridávajú **po jednej**, každá overená. Pôvodný firmvér OVMS (`github.com/openvehicles/Open-Vehicle-Monitoring-System-3`,
MIT) slúži **iba ako vzor** pri písaní ovládačov – nič sa nepreberá naslepo.

Značky: **OVERENÉ** = prečítané v kóde / zmerané; **NEOVERENÉ** = treba overiť.

## 1. Hardvér OVMS v3 (z kódu OVMS3 `main/ovms_peripherals.h`, `support/sdkconfig.default.hw31`)

| Časť | Zapojenie | Poznámka |
|---|---|---|
| ESP32 | flash 16 MB, PSRAM áno (veľkosť sa zisťuje pri štarte) | podľa sdkconfig hw31; náš kus NEOVERENÉ |
| Verzia dosky | hw 3.1 (predvolená) | náš kus NEOVERENÉ – prečítať z modulu |
| SPI zbernica (VSPI) | MISO 19, MOSI 23, CLK 18 | spoločná pre MCP2515 ×2 a MAX7317 |
| **MAX7317** (rozširovač výstupov, SPI, CS 21) | P0 napájanie modemu, P1 výstup 12 V, **P2 CAN1 vysielač (RS)**, P3 DTR modemu | **bez neho nejde ani CAN1** – RS = 1 vypne vysielač (len počúvanie) |
| CAN1 (vstavaný CAN radič ESP32) | TX 25, RX 26 | OBD 6/14 (kábel) |
| CAN2 (MCP2515 č. 1) | CS 5, INT 34 | OBD 3/8 (kábel) |
| CAN3 (MCP2515 č. 2) | CS 27, INT 35 | rozhodne sa |
| Vysielače MCP2515 | cez výstupy RXnBF čipu (BFPCTRL) | 0b00111100 = vysielač vypnutý |
| SD karta | SDMMC 1-bit: CLK 14, CMD 15, D0 2, detekcia karty 39 | v1F má SD cez SPI → iné pripojenie, rovnaký zápis |
| Modem SIM7600G | UART TX 4 / RX 13 (hw 3.1), napájanie cez MAX7317 | krok 2 |
| Tlačidlo SW2 | GPIO 0 | nahrávanie firmvéru |
| GPS | z modemu (NMEA) | krok 2 |

## 2. Krok 1 – sniff (pred testom Peugeota)

Cieľ: nahrať **CAN1 + CAN2 (+ CAN3)** naraz, len počúvanie, na SD s potvrdeným zápisom,
s počítaním každej straty. Bez modemu, bez servera.

| Časť | Odkiaľ | Zmena |
|---|---|---|
| Prostredie | v1F: PlatformIO, espressif32 6.5.0, Arduino | doska `esp32dev` + PSRAM, oddiely pre 16 MB |
| `sdwriter` (zápis s fsync, potvrdená dĺžka, opakovanie) | v1F, overené | pripojenie cez SD_MMC 1-bit namiesto SPI |
| sniff (fronta v PSRAM, formát riadku, štatistika, 60 dní) | v1F `sniff.cpp`, overené na Peugeote 12.10. | **viac zberníc**: v riadku číslo zbernice; jedna fronta pre všetky |
| CAN1 | v1F (TWAI, len počúvanie), overené | piny 25/26 + RS cez MAX7317 = 1 |
| **MAX7317** | nový, malý (zápis registra cez SPI) | vzor OVMS `max7317.cpp` |
| **MCP2515** (CAN2/3) | **nový** | len počúvanie, 500/250/125 kbit/s; prerušenie → čítanie oboch RX bufferov; počítadlá: pretečenie RX (strata), chyby zbernice; vzor OVMS `mcp2515.cpp` (po opravách 07–08/2026: INT ako úroveň, nie hrana) |
| WiFi (NTLIRIS, uLTD_ext) + HTTP: stav, zoznam a stiahnutie súborov, štart/stop sniffu | v1F `dataserver.cpp` (časť) | len to, čo treba na sniff |
| OTA | v1F (pokračovanie, SHA256) | **krok 1b** – kým nie je, nahráva sa cez USB |

**Overenie kroku 1 (na stole, potom v aute):**
- na stole: generátor rámcov nemáme → overiť štart zberníc, SD (zápis, výpadok karty, vytiahnutie napájania = žiadna diera), WiFi;
- v Peugeote naraz s **NestBox_v1F na 6/14** (overený nástroj): CAN1 v2O = v1F rámec po rámci (rovnaké ID a počty za minútu); CAN2 (3/8) – počet strát MCP2515 musí byť 0, inak riešiť (filtre, rýchlejšie SPI).

## 3. Riziká (NEOVERENÉ, ukáže test)

- **Prvé nahranie cez USB** (esptool) – potrebný prístup k USB modulu. Tým sa prepíše aj tabuľka
  oddielov OVMS; **návrat na pôvodný OVMS = znova cez USB** (jeho obraz si pred tým stiahneme).
- **MCP2515 pri plnej zbernici 500 kbit/s:** čip má len 2 prijímacie buffre; ak ich ESP32
  nestihne vyprázdniť, rámec sa stratí (počíta sa). Na 6/14 Peugeota bolo ~1 580 rámcov/s
  (S19.LOG) – preto 6/14 ostáva na CAN1 (vstavaný radič, FIFO 64 B).
- **Napájanie z OBD (pin 16) cez DB9 9** – spotreba a správanie pri štarte motora nezmerané.
- **Polarita RS / BFPCTRL** prevzatá z kódu OVMS, nie zmeraná.

## 4. Ďalšie kroky (po jednom, každý po schválení)

1b OTA (z v1F) · 2 modem SIM7600 + GPS (rozhodnúť AT vs. PPP) · 3 fronta s ACK + protokol
FleetNest (z v1F) · 4 odtlačok SD (PID 0x398), RESEND · 5 STOP / státie / spánok (12 V cez ADC –
OVMS `esp32adc`, NEOVERENÉ) · 6 dekódovanie K0 (tachometer 0x552 OVERENÉ, palivo, AdBlue,
limit 0x1E9 – podľa nahrávok).
