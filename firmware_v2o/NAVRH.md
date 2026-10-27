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

## 6. Spánok a prebúdzanie – NÁVRH na rozhodnutie po teste Peugeota

**Čo platí dnes (overené v kóde):**
- **v1F** (`telelogger.ino`, `config.h`): po STOP a nehybnosti (`STATIONARY_TIME_TABLE` 10/60/180 s) ide do standby –
  koprocesor do úsporného režimu, ESP32 meria napätie (prah `ENGINE_OFF_VOLTAGE` 12,8 V) a pri jeho stúpnutí (motor
  nabíja) sa zobudí; hlásenie polohy každé 3 h (`STANDBY_REPORT_INTERVAL`), voliteľne hlboký spánok s časovačom.
- **OVMS** (`main/ovms_boot.cpp`): hlboký spánok ESP32 **iba s budením časovačom** (iný zdroj budenia v kóde nie je);
  hneď po prebudení v zavádzači zmeria 12 V (ADC1 kanál 0 / 195,7) a pod nastavenou hranicou spí znova.
- **v2O dnes nespí vôbec** – v aute by bežal nepretržite (odber z autobatérie NEMERANÝ).

**Návrh pre v2O (podľa v1F, prebúdzanie podľa toho, čo doska OVMS vie):**
1. STOP motora (0x208 ticho 10 s) → ešte X minút bdie (dohnať frontu na server, odtlačok SD, uzavrieť súbor).
2. Spánok: modem vypnúť (AT+CPOF), CAN1 vysielač už je vypnutý, MCP2515 do sleep, SD odpojiť, ESP32 hlboký spánok.
3. **Prebudenie** – kandidáti (každý treba overiť na doske):
   - **časovač** (ako OVMS): každých N minút krátko zobudiť, zmerať 12 V a pozrieť CAN1 – je auto hore?; každé 3 h
     hlásenie polohy (ako v1F);
   - **napätie** 12 V nad prahom (motor nabíja) – iba pri prebudení časovačom (ESP32 v spánku ADC nesleduje);
   - **prevádzka na CAN** – MCP2515 má budenie zo spánku pri aktivite na zbernici a jeho INT je na GPIO 34/35, ktoré
     vedia budiť ESP32 (NEOVERENÉ: dátový list MCP2515 ani zapojenie som v tomto rozhovore nečítal) – na CAN2 by
     muselo byť niečo, čo sa pri odomknutí auta zobudí;
4. Odber v spánku a pri krátkom prebúdzaní **zmerať** (USB merač / v aute) – podľa toho interval N.

**Na rozhodnutie po teste:** ktorá zbernica sa v Peugeote budí prvá (odomknutie, dvere) – ukáže nahrávka 3/8 a 6/14.

## 7. Ladiace funkcie – ROZHODNÚŤ pred finálnym firmvérom (Vojtech 2026-10-27: „zapíš rozhodnutie, či nechať alebo odstrániť“)

Rozhodnutie je **OTVORENÉ** – do finálneho fw sa rieši pri každom bode zvlášť. Stav overený v kóde 2026-10-27
(fw „Oct 27 2026 08:31:15“, NestBox d1c09d8):

| # | Čo | Kde | Čo môže vo finále robiť | Návrh |
|---|----|-----|-------------------------|-------|
| D1 | Spánok s časovačom / s ULP na požiadanie: `SLEEP_TEST=`, `SLEEP_ULP=` | `web.cpp:145-152`, `standby.cpp` | box zaspí na príkaz aj v aute; pri bežiacom motore ho ULP má hneď zobudiť (NEOVERENÉ do testu) → reštart + medzera v dátach | po teste vypnúť prepínačom pri builde |
| D2 | `ULP_READ` | `standby.cpp` `ulpMeasure` | `loop()` stojí ~1,35 s (web, WiFi, standby); CAN1 číta vlastná úloha (`canbus1.cpp:80`) → CAN1 nedotknuté; CAN2/3 sniff NEOVERENÉ | po teste vypnúť |
| D3 | Príkazy z FleetNestu `ULP_READ`, `STANDBY?`, `SLEEP_ULP=` (p59) | `teleclient.cpp`, FleetNest `commands.py` | ako D1/D2, ale na diaľku (cez mobilnú sieť); zadať ich smie správca firmy | po teste vypnúť vo fw aj vo FleetNeste |
| D4 | `AT=<príkaz>` priamo do modemu | `web.cpp` | ľubovoľný AT príkaz modemu z lokálnej WiFi; v `web.cpp` som nenašiel žiadne overenie prístupu (heslo) | odstrániť alebo vypnúť |
| D5 | Lokálna webstránka `/api/control` (VEH=, CELL=, SNIFF…, RESTART) | `web.cpp` | kto je na rovnakej WiFi, mení nastavenia bez hesla (pozri D4) | rozhodnúť: heslo / len servis |
| D6 | Riadky stavu do FleetNestu (PID 0x399): kroky OTA, diagnostika CCH (p42, front 24 riadkov), `BOOT reset/wake`, `WIFI <sieť> IP`, `CMD …` | `ota.cpp`, `main.cpp` | malé dáta navyše cez mobilnú sieť (iba pri udalosti) | ponechať (BOOT/WIFI/OTA), CCH diagnostiku po vyriešení OTA cez sieť zmenšiť |
| D7 | Flash OTA aj pri bežiacom motore, keď `veh` je prázdne (motor sa nerozozná) | `ota.cpp:26-31`, `telemetry.cpp:337` | flash a reštart za jazdy | **pred finálom opraviť** (p56: motor podľa napätia) – na test povolené (81) |
| D8 | `STANDBY_AUTO 0` – automatický spánok vypnutý | `standby.h:23` | v2O v aute nespí, stále odoberá prúd | **pred finálom zapnúť** po úspešnom teste ULP |
| D9 | Automatické čítania K0 (BSI/CMM DIDs) surovo do PID 0x39F | `telemetry.cpp` | dotazy do auta každých 10 min | rozhodnúť: funkcia alebo ladenie |
| D10 | FleetNest admin: príkaz s ľubovoľným textom bez kontroly | FleetNest `apps/telemetry/admin.py` `CommandAdmin` | obíde kontrolu príkazov (firmvér neznáme ignoruje) | zakázať pridávanie v admine alebo volať `commands.validate` |

**Rozhodnuté (Vojtech 2026-10-27, p60):** ladiace funkcie D1–D4 sú za prepínačom pri builde `NESTBOX_DIAG`
(`platformio.ini` `-DNESTBOX_DIAG=1` teraz; vo finálnom builde riadok preč alebo =0, bez neho je vypnutý) – kód ostane
v repozitári, vo finálnom firmvéri nebude (overené: build s =0 neobsahuje text `SLEEP_ULP=`).
**D9 – ponechať** (Vojtech 2026-10-27: „potrebujeme“): je to funkcia, nie ladenie. Pozor: odometer to nie je –
ten v2O pri K0 číta pasívne z CAN1 rámca 0x552 (`cardecode.cpp`); D9 = km/mesiace do servisu, riedenie a tlak oleja,
objem AdBlue (dekódovanie ešte chýba).
Poznámka: časť 6 vyššie („ESP32 v spánku ADC nesleduje“) je prekonaná – ULP koprocesor meria 12 V aj v spánku
(`standby.cpp`, overené podľa ESP-IDF 4.4 `ulp_fsm/ulp_adc`); funkčnosť v aute ukáže test.

## 8. Porovnanie po častiach: v1F (kostra) vs. OVMS3 na tej istej doske (p70, 2026-10-27)

Pravidlo (Vojtech 2026-10-27): z v1F brať len kostru, k tomu z OVMS3 čo je lepšie – pri každej časti
porovnať a rozhodnúť. Zdroje: OVMS3 clone `~/GitHub/OVMS3` @86dfa0afe, NestBox `firmware_v2o/nestbox/src`.
Kód sa podľa tejto tabuľky NEMENÍ bez rozhodnutia.

| # | Časť | v2O dnes (pôvod) | OVMS3 na tejto doske | Lepšie | Odporúčanie |
|---|------|------------------|----------------------|--------|-------------|
| K1 | **Dáta cez modem** | v1F: UDP cez AT sokety `CIP*`, HTTPS (OTA) cez `CCH*` v modeme, jedna UART linka pre všetko (`cellmodem.cpp`) | `AT+CMUX=0` → 4 virtuálne kanály (CTRL/NMEA, DATA, POLL, CMD), dáta = **PPP + lwIP** (`gsmpppos.cpp`) – sokety ESP32 rovnaké ako pri WiFi; stavový automat s obnovou: NetLoss → opakovania → vypnúť/zapnúť modem, MUX bez rámca 3 min → reštart modemu (`ovms_cellular.cpp:721-751, 905-916`); pretečenie UART sa vyprázdni a počíta (`:190-200`) | **OVMS** | **p69**. Odpadne kód CCH (týždeň opráv OTA 20.–26.10.) a AT sokety; 27.10. po OTA nešli záznamy cez modem 40 min (príčina neznáma). Pozor: ovládač SIM7600 v OVMS má popis „Experimental support“ (štúdia 18.10.) |
| K2 | **GPS** | v1F: `AT+CGNSSINFO` / `AT+CGPSINFO` pri každom zázname na tej istej linke ako dáta | NMEA na vlastnom kanáli, modem posiela sám (`AT+CGPSNMEA=258`, `AT+CGPSINFOCFG=5,258`, `simcom_7600.cpp:91-95`); manuál V3.00 17.2.16: `<time>` 0–255 s → 1 s ide | **OVMS** | súčasť p69, interval 1 s |
| K3 | Čas z GPS | NTP len pri WiFi (`main.cpp` `configTime`); systémový čas z GPS sa nenastavuje (v kóde v2O nie je `settimeofday`) | voliteľne čas z GPS (`modem enable.gpstime`, `gsmnmea.cpp:247, 420`) | **OVMS** | pri p69 nastaviť čas z GPS (bez WiFi dnes čas záznamov z RTC) |
| K4 | Fronta, ACK servera, náhrada zo SD | v1F `txqueue` (PSRAM rezerva, ACK po uložení, SD replay, overené 0 strát) | server v3 MQTT QoS0, nič sa pri výpadku nebufferuje (štúdia 28.9.) | **v1F** | ponechať |
| K5 | SD karta | v1F jeden zapisovač s potvrdením (`sdwriter`), nikdy neformátuje (overené 18.10.) | `sdcard(..., autoformat)` → `format_if_mount_failed` (`sdcard.cpp:115`) | **v1F** | ponechať |
| K6 | OTA | v1F/v2O: pull-OTA, staging na SD, SHA256, pokračovanie (Range), značka obrazu, potvrdenie serveru, pravidlá flashu | HTTP (nie HTTPS) z `api.openvehicles.com`, kontrola len veľkosti (`ovms_ota.cpp:1606-1640`) | **v1F** | ponechať logiku; prenos po p69 cez lwIP (rovnaký HTTPS kód ako WiFi), CCH preč |
| K7 | CAN1 (ESP32 TWAI) | ESP-IDF `twai_*`: normálne **len počúvanie** (`TWAI_MODE_LISTEN_ONLY` + vysielač budiča vypnutý cez `Rs`) – nevysiela ani ACK, do bus-off sa dostať nemôže; pri `READ` sa ovládač nainštaluje nanovo v normálnom režime, rámec ide **jednorazovo** (`ss = 1`, bez opakovania), potom `twai_driver_uninstall` a späť len počúvanie (`canbus1.cpp:30-50, 124-165`) – každá výmena začína s čistým radičom; ESP-IDF 4.4 v Arduine má zapnuté opravy chýb čipu `CONFIG_TWAI_ERRATA_FIX_BUS_OFF_REC`, `_TX_INTR_LOST`, `_RX_FRAME_INVALID`, `_RX_FIFO_CORRUPT` (sdkconfig.h:213-216) | vlastný ovládač, lebo OVMS na CAN aktívne vysiela; tie isté obchádzky chýb čipu (bus-off errata `esp32can.cpp:191-226`) a plný reset, keď obchádzka zlyhá (`:857`) | **rovnaké** (pre nás) | nič – overené 27.10. na otázku Vojtecha |
| K8 | CAN2/3 (MCP2515) | vlastný podľa OVMS3 (hladinové IRQ, BFPCTRL, sleep) | OVMS ovládač, prerábaný 07–08/2026 | rovnaké | ponechať |
| K9 | Spánok | vlastný: ULP 12 V + časovač + 3 h hlásenie (overené 27.10.) | hlboký spánok len pri slabej 12 V, budenie časovačom | **v2O** | ponechať |
| K10 | 12 V meranie | ADC1 ch0 / 195,7 (hodnota OVMS) | to isté (`system.adc factor12v`) | rovnaké | kalibrovať na kus (nekalibrované) |
| K11 | Napájanie modemu | PWRKEY 200 ms zap., `AT+CPOF` alebo 2,6 s vyp. | PWRKEY cyklus 500 ms zap. / 3000 ms vyp. (`simcom_7600.cpp:157, 165`) | blízke | pri p69 prevziať časy OVMS |
| K12 | WiFi | 2 pevné siete, sken + pripojenie, opakovanie 30 s (`main.cpp`) | režimy client / AP / apclient, skenovanie, reconnect, kvalita signálu (`esp32wifi.cpp`) | OVMS (viac funkcií) | nízka priorita; AP na servis zvážiť |
| K13 | Lokálna web stránka / `/api/control` | bez prihlásenia (D4/D5) | webserver s prihlásením (session, `web_framework.cpp:522`) | **OVMS** | pred finálom heslo alebo vypnúť (D5) |
| K14 | Watchdog | v2O v riadku BOOT hlási dôvod resetu (aj `TASK_WDT`) a posledný krok (`laststep`) | pri resete watchdogom ukladá mená zaseknutých úloh do RTC a vypíše ich v `boot status` (`ovms_boot.cpp:221-224`) – **ale na ESP-IDF ≥ 4 len "(unavailable)"** (`ovms_boot.cpp:686-689`); my sme na IDF 4.4 | **nič navyše** | zrušené (opravené 27.10. po otázke Vojtecha) |
| K15 | Protokol na server | v1F Freematics UDP → FleetNest | OVMS v2/v3 na vlastný server | **v1F** | ponechať |

Poradie podľa dopadu: **K1+K2+K3 (p69)** → K13 (D5) → K11 (s p69) → K12.
