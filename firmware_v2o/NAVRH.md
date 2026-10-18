# NestBox_v2O – návrh (na schválenie)

Firmvér NestBox pre hardvér **OVMS v3**. Rozhodnutie (Vojtech 2026-10-18): z NestBox_v1F
(`firmware_v5/telelogger/`) sa berie **len kostra a náš overený kód**; ďalšie funkcie sa
pridávajú **po jednej**, každá overená. Pôvodný firmvér OVMS (`github.com/openvehicles/Open-Vehicle-Monitoring-System-3`,
MIT) slúži **iba ako vzor** pri písaní ovládačov – nič sa nepreberá naslepo.

Značky: **OVERENÉ** = prečítané v kóde / zmerané; **NEOVERENÉ** = treba overiť.

## 1. Hardvér OVMS v3 (z kódu OVMS3 `main/ovms_peripherals.h`, `support/sdkconfig.default.hw31`)

| Časť | Zapojenie | Poznámka |
|---|---|---|
| Flash | 16 MB | podľa sdkconfig hw31 a oddielov v3-35 na module |
| Verzia dosky | **OVMS v3.3**, modem doska OVMS MDM v3.3 (SIM7600G), vyrobené 2026.04.13 | foto 2026-09-30; v kóde OVMS sa v3.3 stavia ako „base 3.1“ (`Kconfig` pozná len 3.0 / 3.1) – piny 3.1 (modem RX 13 / TX 4). Bežiaci firmvér `product v3.3-5` modem SIM7600 vidí (`m.hardware`) |
| ESP32 / PSRAM (zmerané 2026-10-18) | ESP32 rev 3, 2 jadrá; PSRAM 4 MB (`module memory`: SPIRAM 4194252 B) | |
| Antény (SIM7600) | ANT1 = GNSS, ANT2 = MAIN, ANT3 = AUX | potlač na module |
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

## 4. Celý firmvér (Vojtech 2026-10-18: „krok 2 by som už skúsil komplet, nie len sniff“)

Logika v obrázku: `LOGIKA.mmd` (diagram). Zásada: **všetko nad hardvérom je kód v1F** (overený
v aute); vymieňa sa iba vrstva, ktorá v v1F volá dosku Freematics. Zmerané v kóde v1F (počet
volaní): `sys.*` (GPS, linka ku koprocesoru, bzučiak), `obd.*` (OBD cez koprocesor, napätie,
úsporný režim), `mems->*` (pohyb), `teleClient.wifi/cell` (WiFi a modem SIM7670), `SD.*` (SPI).

| v1F (Freematics) | v2O (OVMS) | Stav |
|---|---|---|
| fronta s ACK, SD repl, odtlačok SD 0x398, RESEND, správy `udpmsg`, príkazy OTA_READY/SNIFF, OTA s pokračovaním + SHA256, STOP/ŠTART udalosti, hlásenie počas státia 3 h, webové API | **bez zmeny** (prenesené) | overené v aute (v1F) |
| `sdwriter` SD cez SPI | SD_MMC 1-bit (piny 14/15/2) | nové pripojenie |
| WiFi (`ClientWIFI`) | to isté – ESP32 | bez zmeny |
| modem SIM7670 (`CellSIMCOM`, AT: UDP `CIP*`, HTTPS `CCH*`) | **SIM7600G** – rovnaký prístup cez AT príkazy, napájanie cez MAX7317 | AT príkazy SIM7600 vs. SIM7670 – **overiť na module** (NEOVERENÉ) |
| GPS Freematics (`sys.gps*`) | GNSS v modeme SIM7600 (anténa ANT1) | nové |
| OBD cez koprocesor (`obd.readPID`, ELM327) | priamo CAN1: OBD režim 01 cez ISO-TP (Passat), počúvanie (K0) | nové; UDS tachometer Passat už je (`canodo`) |
| napätie (`obd.getVoltage`) | ADC ESP32 (OVMS `esp32adc`, kanál ADC1_0) | prepočet **zmerať** |
| úsporný režim / prebudenie koprocesorom | ESP32 hlboký spánok + časovač (OVMS: `12v.wakeup_interval`) + kontrola napätia/CAN | **nový návrh – overiť odber** |
| pohyb (MEMS) | OVMS **nemá** akcelerometer (v periférach nie je) | ŠTART/STOP podľa motora (CAN), nie pohybu |
| motor beží (OBD RPM > 0) | CAN: Passat – OBD RPM; K0 – správa so stavom motora (vybrať z nahrávky) | K0 NEOVERENÉ |
| bzučiak, LED | nie sú | vynechané |

**Poradie (každý bod overený na stole / v aute pred ďalším):**
1. hardvér: MAX7317, SD_MMC + `sdwriter`, CAN1 + MCP2515 (CAN2/3), sniff na všetkých zberniciach, WiFi + webové API, OTA (z v1F) – od tejto chvíle už bez USB;
2. modem SIM7600 (AT) + GNSS; fronta s ACK + UDP do FleetNest; príkazy; odtlačok SD;
3. motor / ŠTART–STOP / státie / spánok, napätie;
4. dekódovanie áut: K0 (tachometer 0x552 OVERENÉ; palivo, AdBlue, limit 0x1E9 podľa nahrávok), Passat (OBD + UDS z v1F).

**Pred testom Peugeota stačí bod 1** (sniff 3 zberníc); v aute zároveň v1F na 6/14 ako meradlo.
FleetNest: OVMS sa hlási ako zariadenie typu NestBox_v2O (hodnota `ovms`) – vlastné ID z MAC.

## 5. Stav (2026-10-18 večer)

| Krok | Kód | Overené |
|---|---|---|
| 1 hardvér + sniff 3 zberníc + WiFi + web + OTA (push) | hotový (`nestbox/`) | na module cez NAS: štart, SD, WiFi, oba MCP2515 odpovedajú, OTA cez WiFi, záťaž 3000 rámcov/s 5 min bez straty, tvrdý reset uprostred zápisu bez diery. **Otvorené:** po resete zvonka štart do režimu nahrávania (podozrenie MAX7317 0x0A, odstránené, neoverené). CAN rámce z auta – Peugeot |
| 2a záznamy do FleetNest cez WiFi (fronta + ACK z v1F) | hotový | nie (na module zajtra; zariadenie treba pridať vo FleetNest) |
| 2b modem SIM7600G + GNSS | hotový, **vypnutý** (`CELL=1` zapne) | nie – na USB modem možno nenaštartuje, test v aute |
| pull-OTA (v1F), štart/stop motora, státie, spánok, dekódovanie áut | – | – |
