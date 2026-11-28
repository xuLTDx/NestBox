# NestBox_v2O a3 – ako firmvér funguje TERAZ (referencia)

Vojtech 24.11.2026: „Ako nechceš si takéto informácie niekde popísať? Čo ako funguje, kde, … štruktúru fw s funkciami,
odbočkami, podmienkami + čo máme zistené, čo sa rieši.“ Toto je JEDINÝ popis aktuálneho správania (NAVRH.md = história
návrhov, LOGIKA.mmd = stav 18.10., zastaraný). Pri každej zmene firmvéru sa upraví TU, v tom istom commite.
Zdroj: vetva v2o-arduino3, worktree /home/claude/Claude/nestbox_a3, súbory firmware_v2o/nestbox/src/. Riadky = stav 24.11.2026.
Na J0 beží build Nov 27 2026 14:28:13 (OTA VALID 27.11. 14:35:59, b54; commit 623cc07). Na v2o_a3: build Nov 28 2026 12:34:31.

Súvisiace (čítať pred tvrdením):
- merania (odber, napätia, časy): /home/claude/notes/MERANIA.md
- údaje auta K0 (čo je overené, zdroj, PID): /home/claude/Claude/toyota_sniff/K0_TABULKA.md
- stav projektu, boxy: /home/claude/notes/PROJEKT_STAV.md
- sniffy (všetky, stav): /home/claude/Claude/cardata/SNIFFY.md

## 0. Kde bežím ja, kde NAS, siete, WiFi, USB, zdroj (overené 25.11.2026; Vojtech: „aby si nemal zmätky“)
| čo | kde / adresa | čo z toho vyplýva |
|---|---|---|
| ja (Claude) | server **uLTD**, LAN 192.168.10.22/25 (firma, brána 192.168.10.1), WireGuard wg0 10.74.18.4 | účet claude bez sudo → sudo príkazy pre účet **ultd**; žiadne USB tu nie je |
| OTA server nestbox-ota, FleetNest | ten istý stroj uLTD (192.168.10.22) | box ho vidí cez internet (modem) aj z uLTD_ext priamo |
| **NAS** | doma / stôl, LAN 192.168.0.18/25 (brána 192.168.0.1), wg0 10.74.18.1; `ssh nas` = claudef | trasa 192.168.0.0/24 ide zo servera cez wg0 → server sa dostane na boxy na NTLIRIS (overené: srv → 192.168.0.29 HTTP 401) |
| WiFi **uLTD_ext** | firma, 192.168.10.x, siaha do auta na parkovisku | box v aute → priamo na server (J0 24.11. b25/b26 = 192.168.10.9) |
| WiFi **NTLIRIS** | stôl pri NAS, 192.168.0.x | box len na stole (USB alebo zdroj), do auta nedosiahne; J0 b27 = 192.168.0.29 |
| USB | len na NAS: /dev/ttyUSB0 (CP2102, claudef, dialout) | len prvý flash / zamrznutý box; OTVORENIE PORTU REŠTARTUJE BOX (18.11.) → neotvárať bez dôvodu. ttyUSB0 vznikol 24.11. 15:08 = začiatok J0 b27 (podľa času, port neotváraný) |
| 12 V zdroj | laboratórny zdroj na stole, číta ho len Vojtech (displej) | napätie vidím len z boxu (/api/live, statusy); USB ALEBO zdroj, nikdy oboje; na USB nejde modem/GPS |
| auto | len modem (FleetNest príkazy, statusy, cell OTA); WiFi iba na uLTD_ext na firme | box v aute mimo firmy = žiadne WiFi kroky |

SERIAL CEZ WiFi (bez USB): http://<IP>/api/serial?from=<n> (web.cpp:695 handlerSerial, netlog.cpp netlogRead) = celý
výpis ako na USB (nbout = Serial + kruh); odpoveď „head= from= … next=<n>“, jeden kúsok na volanie → slučka from=next.
Funguje zo servera aj NAS, keď je box na uLTD_ext alebo NTLIRIS; heslo webu cez curl -K - (nikdy nevypisovať).
SD log boxu: /api/log/<n>. USB treba len na prvý flash / zamrznutý box.
Možnosti k boxu: FleetNest DB (st.sh, len čítanie), príkazy cmd.sh (cez modem/WiFi pri ďalšom spojení), web boxu
http://<IP>/api/... (zo servera aj NAS, keď je box na uLTD_ext alebo NTLIRIS), pull-OTA (kanál v2o_a3), USB na NAS.

Build (28.11. CD5, premenované z ovms33): `pio run -e v2o` (bežný do áut), skúšobné `v2o_load/_crash/_fullq/_batt`; výstup `.pio/build/v2o/`.
CD6 (28.11.): bežný `v2o` BEZ ladiacich príkazov (D1–D4, WDT_TEST, GPS_TEST, AT=) a bez PROBE/SCAN; tie sú len v `v2o_test`
(-DNESTBOX_DIAG=1, stôl) a `v2o_probe` (-DNB_PROBE=1, nové auto). Ladiaci log DEBUG=1/0 ostáva v `v2o`. Pravidlo 17.

## 1. Moduly (súbor → čo robí)

| súbor | čo |
|---|---|
| main.cpp | setup() (poradie štartu), loop(), rozhodovanie o WiFi (wifiSearch/wifiTick) |
| standby.cpp/.h | spánok, prebudenie ULP od 12 V, hlásenie pri státí každé 3 h |
| telemetry.cpp | záznamy (každú 1 s pri motore), motor beží?, čítania UDS (E/U/F riadky profilu), príkazy z FleetNest, modem úloha |
| cardecode.cpp | hodnoty z CAN1 vysielaní (K0 pevne + profil C riadky) |
| profile.cpp/.h | profil auta zo servera (/spiffs/profile.txt): V, M, E, C, U, F, O riadky |
| txqueue.cpp | rad na doručenie: záznam sa zahodí až po ACK servera; PSRAM, prežije reštart; zvyšok z SD |
| teleclient.cpp | UDP k FleetNest ingest (ACK po uložení) |
| ota.cpp | pull-OTA, WiFi zoznam (places), profil, LOG upload (PG49), stavové riadky (statusLine/otaLog), PG340 sniff upload |
| otaverify.cpp | nový build platí až po prvom prihlásení na server (inak späť) |
| sniff.cpp | sniff 3 zbernice → /SNIFF/S<n>.LOG, minúty v NVS, 60 dní čistenie, PG340 delenie súboru |
| sdwriter.cpp | jediný zapisovač SD (potvrdené zápisy) |
| cellmodem.cpp | SIM7600: CMUX + PPP, GNSS, SMS |
| wifiplaces.cpp | zoznam WiFi miest (poloha, polomer, sieť, heslo) v NVS |
| web.cpp | lokálne API (/api/control, /api/sniff/…, /api/serial, /api/log, /api/ota) |
| errreport.cpp | „ERR <oblasť> <čo>“ / „OK …“ riadky, opakovania x<n> |
| crash.cpp, dbglog.cpp, netlog.cpp, wdt.cpp | pád (RTC + coredump), debug log na SD (DEBUG=1), /api/serial kruh, watchdog |
| clocktrust.cpp | po spánku sa hodinám neverí, kým ich nenastaví server/GNSS/NTP |
| install.cpp | inštalačný režim (vlastná WiFi NestBox-<ID>, mini stránka) |
| canbus1.cpp, mcp2515.cpp | CAN1 (TWAI), CAN2/CAN3 (MCP2515) – len počúvanie |

## 2. Štart (main.cpp setup, 592)
standbyEarly() (prebudenie zo spánku: overí 12 V, prípadne hneď spí) → PSRAM/netlog → crash/dbglog → SD mount + sdwStart →
CAN1 + sniffInit() (sniff pokračuje, ak mal minúty v NVS) → telemetryBegin() → otaBegin() (úloha ota) → wifiPlacesBegin →
webStart → wdt. loop(): web, can1Tick, readTick, sniffTick, wifiTick, instTick, standbyTick, errTick, čistenie sniffov/debug.

## 3. Hlavné toky s podmienkami

### 3.1 Záznamy a doručenie
- Motor beží (profil M riadok; K0: 0x208 ×0,125 > 0; ďalšie vetvy telemetryEngineOn – v tomto popise ešte NEPREJDENÉ) → záznam každú 1 s (DRIVE_REC_MS, telemetry.cpp:79).
- Každý záznam → SD /DATA/<n>.CSV + rad (txqueue). Zahodí sa až po ACK servera. Identita = (boot, ms od bootu).
- Sieť: WiFi s internetom má prednosť (telemetryWifiForData), inak modem PPP.
- Kedy WiFi „nie“ (build Nov 27 2026 20:53:21, Vojtech 27.11. „1-3 áno“): server cez WiFi mlčí (30 s bez odpovede pri
  čakajúcich paketoch / 2 neúspešné prihlásenia / skúšobný paket každé 2 min) -> box pošle 1 ping na server CEZ MODEM
  (telemetry.cpp wifiServerBad, netlink.cpp nbUdpProbe). Modem odpovie = cesta cez WiFi zlá -> dáta na modem
  („NET wifi-noserver modem-ok inet=<0/1> by <probe|silent|logins>“, potom „ERR NET wifi-nointernet“). Modem tiež mlčí =
  server dole -> ostane na WiFi, dáta čakajú v rade („NET server-silent both inet=… - stays on WiFi“, platí 2 min).
  Riadok len pri zmene výsledku (PG352). Meno servera ešte nezistené -> raz cez DNS modemu (PG354).
  inet = TCP 1.1.1.1/8.8.8.8:53 cez WiFi, len informácia. Prečo: WiFi môže mať internet a blokovať náš server (port,
  captive portal, DNS); 15:49:49 b55 „wifi-nointernet“ na hotspote s internetom. NEOVERENÉ na boxe (ping cez PPP pri WiFi).
- PG357 (build po 12:34:31): krok úlohy record / net / loop (od wdtFeed po wdtFeed) > 20 s → „SLOW <úloha> <krok> <s>s“;
  OTA úloha: WiFi zoznam / LOG > 20 s, upload sniffu (1 POST) > 60 s → „SLOW ota <krok> <s>s“ (nie cell, nie sťahovanie buildu).
- Rad (txqueue.h): 1 MB PSRAM (~1 h jazdy po 1 záznam/s), prežije reštart a pád, nie výpadok napájania ani hlboký spánok.
  Plný → najstarší von: so SD kópiou sa neskôr dohrá zo SD, bez nej sa počíta ako stratený (lost=). Bez SD: od 50 % len
  čas/poloha/rýchlosť/smer, od 80 % 1 záznam za 10 s. Záznamy zo státia (SD vyp.) aj do SPIFFS.
- Stavové riadky (PID 0x399 = 921): statusLine()/otaLog() – idú v záznamoch, v DB čitateľné cez fn_db/st.sh.

### 3.2 WiFi – kedy hľadá (main.cpp wifiSearch, 207)
- Bez zoznamu miest → nehľadá („nolist“).
- Inštalačný režim / príkaz WIFI=1 (30 min, WIFI_CMD_MS) / 3 min po štarte motora (WIFI_START_MS) → všetky siete.
- Bez polohy → všetky. Inak len siete, ktorých miesto obsahuje polohu („place“), mimo → nehľadá („away“).
- Pri pripojení riadok „WIFI <sieť> IP <ip>“ (len raz pri pripojení, nie priebežne).
- Siete: uLTD_ext (192.168.10.x) = firma, dosiahne do auta; NTLIRIS (192.168.0.x) = stôl (USB/zdroj), do auta nedosiahne.

### 3.3 Spánok (standby.cpp standbyTick ~431)
Zaspí, keď VŠETKO platí: STANDBY_AUTO; 12 V pod 12,8 V nepretržite 10 s (PG148) a nie pod 7 V (stôl); CAN1 ticho 5 min
(STANDBY_QUIET_MS, pri hlásení zo státia 10 s); hlásenie zo státia doručené; nový build nečaká na overenie; SMS o batérii
odoslaná; OTA sťahovanie beží najviac 3 min navyše (STANDBY_OTA_WAIT_MS); PG340: posielanie sniffu najviac 20 min navyše
(STANDBY_SNIFFUP_WAIT_MS); obe čakania začínajú odznova pri každom novom parkovaní (build Nov 28 11:57:24, predtým raz za boot). Pred spánkom: sniff súbor a rad zavreté, modem + WiFi vypnuté. Hlboký spánok: ULP meria 12 V
každých 200 ms, zobudí pri ≥ 12,8 V; každé 3 h hlásenie zo státia (GNSS max 150 s, 1 záznam, doručenie max 60 s).

### 3.4 OTA, WiFi zoznam, profil (ota.cpp otaTask, slučka každé 2 s)
- Kontrola buildu: každých 6 h (OTA_PERIODIC_MS) alebo OTA_READY / OTA_CHECK. Sťahuje s pokračovaním (Range), flash až pri
  vypnutom motore a ≥ 11,5 V. Nikdy nižšia verzia.
- Hlásenie zo státia (build Nov 28 2026 11:57:24, Vojtech 28.11. „3 áno“): hneď po sieti len kontrola buildu + sťahovanie
  (nie WiFi zoznam, LOG, sniff, flash); nie pri nízkej batérii; sťahovanie pokračuje pri ďalšom prebudení (OTAD_OFF, 64 KB).
  Prečo: J0 27.11. 20:54 OTA_READY v státí, do 28.11. 09:31 (5 prebudení) build nestiahol (otaTask čakal 60 s).
- Čakajúci build (STAGED) blokuje posielanie sniffu len keď sa dá hneď flashnúť (motor vyp. + napätie), build 11:57:24.
- PG361 (diagnostika): na konci sťahovania buildu „OTA-PULL CAN1 ov recv=+a flash=+b other=+c“ (príjem / mazanie+zápis flash+NVS / zvyšok), ak pribudol overrun.
- WiFi zoznam + profil: každé 2 h (PLACES_EVERY_S 7200, ota.cpp:277) alebo WIFI_SYNC. Profil platí od ďalšieho štartu.
- LOG=<súbor>[,<od>]: súbor z SD na server po 8 KB s pokračovaním (PG49) – WiFi alebo modem.

### 3.5 Sniff (sniff.cpp)
- SNIFF=1,<min> (max 1440). Minúty v NVS → pokračuje po každom štarte, kým neuplynú. Každý štart = nový /SNIFF/S<n>.LOG.
- Počas sniffu box posiela PID 0x393 (915) = zostávajúce minúty → v DB vidno každý sniff aj bez stiahnutia.
- Build 20:53:21: po KAŽDOM príkaze SNIFF ide 0x393 raz aj bez sniffu (0) → FleetNest potvrdí aj SNIFF=0 na už vypnutý
  sniff. Prečo: cmd 946 27.11. SNIFF=0 o 14:32 po samovypnutí 14:29:14 → poslané 50×, expired.
- Nezačne pri SD ≥ 70 %. Súbory staršie 60 dní (podľa „# clock unix=“) sa mažú.
- PG340 (build Nov 25 2026 00:17:18, na J0 od 25.11. 00:27 b28): CAN1 ticho 60 s a súbor má rámce → súbor sa zavrie („# split“),
  ďalší rámec CAN1 otvorí nový → 1 súbor = 1 jazda.

### 3.6 PG340 posielanie sniffov (build Nov 25 2026 00:17:18, na J0 od 25.11. 00:27 b28)
ota.cpp sniffUpTick (v otaTask): podmienky = WiFi s internetom, OTA token, nebeží OTA sťahovanie ani LOG, sniff práve
nezapisuje. Vyberie najstarší S<n> nad NVS snup/done (rozpracovaný je vždy ten najstarší). Prázdny POST = „čo máš“
(next=), potom POST po 1 MB od next (zo SD po 16 KB). Server nechá všetko, čo prišlo (aj prerušené). Celý → done=n,
„SNIFF-UP S<n> OK <B> <s> s“. Chyba → za 30 s znova sa spýta.
Oprava 27.11. (Vojtech „1 oprav“; build Nov 27 2026 13:42:14 na v2o_a3, spolu s DNS a prázdnym súborom nižšie): rozposlaný súbor drží „posielam“ (spánok čaká) aj keď WiFi chvíľu vypadne
alebo je označená bez internetu, najviac 5 min od posledného pokusu (SNUP_HOLD_MS; mimo WiFi box nečaká); medzi dvoma
súbormi sa „posielam“ nezruší; pri nízkej batérii (s_battLvl) spánok na posielanie nečaká. Prečo: J0 27.11. b46/b47
zaspal 5 min po stíchnutí CAN s S62 na 13 MB zo 79 MB (12:41:20, 12:58:41, „WiFi silent -> back to the modem“).
Prázdny otvorený súbor neblokuje (sniffIdleNow, sniff.cpp): posiela sa, keď nie je otvorený súbor ALEBO CAN1 je ticho ≥ 60 s;
otvorený súbor sa sám nikdy neposiela. Prečo: J0 b50 13:34 po reštarte s tichým CAN S67 s 0 rámcami -> split nikdy -> S62 neodišiel.
DNS: mená cez lwip_getaddrinfo (netlink.cpp lwipResolve), TLS connect(ip, port, host) - nie Arduino hostByName (PANIC b48, 6a). Nová SD karta (nižšie čísla) → done=0.
Počítadlá CAN1 (overrun/missed/buserr) = súčet za boot, reštart ovládača (READ, hľadanie rýchlosti) ich už nenuluje (PG355, canbus1.cpp stop()).
Diagnostika (build 20:53:21, dočasná): „SNIFF-UP S<n> CAN1 ov sd=+a wifi=+b other=+c @<off>“ po každom POST s prírastkom
CAN1 overrun (SD čítanie / WiFi zápis / zvyšok). Prečo: 27.11. overrun x54/x52/x13 presne v oknách uploadu (can_overrun/ROZBOR.md).
Počas jazdy sa neposiela (build Nov 28 2026 12:05:02, Vojtech 28.11. „sprav to takto“): jazda = rýchlosť > 5 km/h z CAN (0x38D, čerstvá 5 s), inak z GPS (fix do 10 s) – telemetry.cpp telemetryDriving. Bežiaci motor na mieste neblokuje (na firme stojí, WiFi silná). Rozbehnutý POST (≤ 1 MB) dobehne. Prečo: upload ide s CAN1 overrun (27.11.).
Server nestbox-ota 2.8.0 (/home/claude/GitHub/NestBox-OTA ota_server.py _sniff_post): /home/nestbox-ota/sniffs/
<box>_S<n>_<veľkosť>.LOG (.part/ rozpracované) → systemd path unit claude cardata-sniff.path → cardata/receive.py.

### 3.7 PG358 ochrana proti slučke pádov (crashloop.cpp/.h, Vojtech 28.11. „358 áno“)
- 5 reštartov pádom (PANIC, TASK_WDT, INT_WDT, WDT, vlastný watchdog) za 10 min = slučka (počíta RTC pamäť, nie POWERON).
- Prvá slučka: návrat na druhú partíciu, ak má build označený „dobrý“ (24 h po OTA VALID bez slučky, NVS CL_GOOD) a sha
  padajúceho buildu je známe (z OTA) → sha do OTAV_REJ (OTA ho znova neinštaluje, kým nepríde novší).
  Riadok „CRASHLOOP rollback sha=… -> <partícia> <dátum> <čas>“. Sťahovanie do partície s dobrým buildom značku zmaže.
- Inak vypne jednu časť na 30 min / 2 h / 6 h: modem (pád v cell/CMUX/PPP; len kým WiFi nesie dáta), WiFi (krok WIFI…),
  inak sniff + čítania z auta. Nikdy obe siete. Záznamy na SD stále. „CRASHLOOP n=5 10min last=<úloha>/<krok> -> <časť> off <min>min“,
  na konci „CRASHLOOP <časť> on again“. Úroveň sa vynuluje po 24 h bez slučky.
- 2 pády za 10 min alebo vypnutá časť → OTA kontrola hneď po sieti (nie po 60 s), „OTA-PULL CHECK crash loop“.
- Obmedzenie: pád ešte pred crashLoopBegin (setup pred otaBegin) sa nezaráta. Build označený dobrý: „CRASHLOOP build marked good (24 h)“.

## 4. Príkazy (FleetNest commands / web /api/control)
SNIFF=1[,min] | SNIFF=0 | SNIFF? | SNIFF_BUSES= | SNIFF_LIMIT= | SNIFF_PURGE | SNIFF_VEH= ; OTA_READY | OTA_CHECK | OTA? |
OTA_TOKEN= | OTA_HOST= | OTA_PORT= ; WIFI_SYNC | WIFI=1/0 ; LOG=<n|D<boot>|C<boot>>[,od] ; DEBUG=1/0 | DEBUG? ;
READ= | PROBE= | SCAN= | CAR? | TEL? | VEH= | REC_S= ; STANDBY? | SLEEP_TEST= | SLEEP_ULP= | ULP_READ ; CELL= | CELL_APN= |
AT= | SMS… ; SDW? | PART? | RESTART | INST=0.
Zaradiť príkaz: /home/claude/Claude/fn_db/cmd.sh <box> '<príkaz>'.

## 5. Čo máme zistené (odkazy, nie kópie)
- Údaje K0 zo zbernice / čítania: K0_TABULKA.md (overené, kandidáti, vyradené).
- Odber: prevádzka 0,1–0,17 A; spánok na 12 V 0,017–0,020 A = 17–20 mA (Vojtech 25.11.; MERANIA.md).
- Sťahovanie sniffu cez WiFi z boxu: ~170 kB/s (fetch.py 24.11., J0 na NTLIRIS).
- J0 v Toyote na firme (27.11.): uLTD_ext len −81 až −88 dBm, box ho pod prahom preskakuje („-skip“), pripojí pri −86;
  posielanie sniffu 46 kB/s, keď ide (12:38–12:41), potom TLS na ultd.mooo.com:6001 zlyhá po 32 s; UDP prihlásenie ide.
  Bližšie parkovať ani ďalší AP NEDÁ (Vojtech 27.11. „Nie nedá“) -> s týmto signálom treba počítať.
- K0: CAN1 (OBD 6/14) po vypnutí motora beží naplno (~900–3600 rámcov/s), kým auto nie je zamknuté; ticho až po zamknutí
  (riešené skôr v chate: Peugeot motor vyp. 11:20:58, zamknuté, CAN ticho od 11:24:15; Vojtech 27.11. „to už sme riešili“).
  ROZPOR 27.11.: J0 v Toyote, motor vyp. ~12:36:40, auto NEzamknuté (Vojtech) -> SNIFF-UP S62 start 12:38:13, t. j. CAN1 ticho
  60 s už ~30 s po vypnutí motora bez zamknutia. Staršie sniffy J0 S30/S38/S42: posledný rámec CAN1 64-113 s po otáčkach 0.
  Kedy K0 CAN1 stíchne (motor vyp. / zamknutie / čas) NEOVERENÉ - overiť z S62 (# split, posledné rámce) - Peugeot vs Toyota?

## 6. Čo ešte box / FleetNest NEROBÍ alebo robí len čiastočne (stav 25.11.; zdroj K0_TABULKA.md)
| čo | dnes | čo chýba |
|---|---|---|
| AdBlue | len čítaním CMM D558/D517/D513 → 0x3A2/0x3A6 | mierka NEOVERENÁ (treba doliatie so známym L); na zbernici nenájdené |
| značka z kamery | 0x5F2 b2 → 0x3B1 v profile | PG304 otvorený: príznaky b3 00/C0 nevysvetlené; „50“ 24.11. 14:15:01 v dátach nenájdená |
| výsledný limit | FleetNest logika D (24/31 hlásení) | PG327 – lepšia logika kamera + mapa |
| prekročenie rýchlosti | – | počíta FleetNest z limitu, závisí od PG327 |
| tankovanie | FleetNest zo skoku 0x54E / D8C4 | priamy signál nenájdený (0x432 vyradené) |
| palivo litre, spolujazdec, servis, olej | len čítaním (UDS) | na zbernici nenájdené (spolujazdec najlepšie 92 %) |
| motor beží – všetky vetvy | profil M + K0 0x208 | popis vetiev telemetryEngineOn v tomto dokumente chýba |
| CAN1 overrun (prebehy) | PG308 nasadené (LASTPOS max 5 min za jazdy) – opakovaní menej, nezmizlo | ďalší podozriví: TXQ_CUR NVS každých 30 s, TWAI ISR mimo IRAM; overiť jazdou na builde 25.11. (riadok 6a) |
| CAN3 | – | PG317 LS.CAR (zásah do káblov) |

## 6a. Čo sa riešilo a čo je vyriešené (dátum, dôkaz; zatvára len Vojtech)
| vec | čo sa spravilo | výsledok (overené) | stav |
|---|---|---|---|
| Prechod Peugeot master -> a3 (generálka PG287-PG294) | 23.11.: J0 = 1:1 Peugeot (erase + USB: starý zavádzač Arduino 2.0.14, stará tabuľka, master 12.11. 11:51:49), potom ako Peugeot: crash build -> návrat cez STARÝ zavádzač, a3 cez OTA v aute, nová tabuľka (PART v2o_cd), profil K0. Claude/peugeot_generalka/PLAN.md | 10:32 OTA ROLLBACK crash-or-reset; 12:19:34 OTA VALID a3; 13:20 PART OK + FS erased OK + PROFIL K0, bez pádu | overené; Peugeot prepnutie = PG305 (28.11. žiadosť FleetNest) |
| CAN1 overrun PG307/PG308 | 23.11.: rozbor Claude/can_overrun/ROZBOR.md – HW RX FIFO TWAI (~5 rámcov) sa plní, keď je cache vypnutá pri zápise do flash (ISR nie je v IRAM); najviac pri OTA zápise (+211, +187). PG308 (5714041, build 23.11. 14:43:42): LASTPOS do NVS max raz za 5 min za jazdy (predtým každých 50 m). Na J0 od 24.11. 09:19 (b22). | DB 0x399: pred PG308 po štarte +14..+29, potom x1–x4 / 10 min; po PG308 (b24–b26) po štarte +23..+30, potom x1 / 10 min; počas OTA x22; missed +144 raz (b25). Menej, nezmizlo. | čiastočne – otvorené PG308 |
| PG340 sniffy posiela box sám | firmvér 430a503 (build Nov 25 2026 00:17:18), nestbox-ota 2.8.0 d85c0ee, cardata receive.py (oprava „už máme“ 25.11.), sniffy.py (párovanie b73) | 25.11. 12:05–12:09 J0 poslal 32 chýbajúcich sniffov, 14:29–14:30 2Z S1–S10; všetky na serveri, u mňa, na NAS; na J0 aj 2Z SD nič neposlané neostalo | funguje na malých súboroch; veľký súbor z jazdy NEOVERENÝ |
| PANIC lwIP DNS/SNTP (PG263 a znova 27.11.) | 16.11. PG263: dns_setserver len pod LOCK_TCPIP_CORE (J0 b85, úloha net) | 27.11. 13:03–13:04 J0 b48 (build Nov 25 00:17:18, Toyota na firme, hneď po pripojení uLTD_ext pri slabom signále): ERR SYS panic IllegalInstruction ota, bt = __assert_func <- sys_untimeout <- sntp_retry <- sntp_dns_found <- dns_call_found <- dns_clear_cache (ten istý reťazec ako b85, ale úloha ota); COREDUMP 36068 + crash log C48 na SD boxu | OTVORENÉ – PRÍČINA NÁJDENÁ (coredump K48, gdb): Arduino 3.3.11 NetworkManager::hostByName (libraries/Network/src/NetworkManager.cpp:84) volá dns_clear_cache() BEZ zámku jadra lwIP, keď sa zmení „má IPv4 / IPv6“ (pripojenie / výpadok WiFi alebo PPP); ak práve čaká DNS dopyt SNTP, jeho callback -> sntp_retry -> sys_untimeout -> assert „Required to lock TCPIP core“. Volané z ota.cpp:844 sniffUpPost -> WifiConn::connect (ota.cpp:202) -> NetworkClientSecure::connect(host). Spoločná príčina s b85 (16.11.): dns_clear_cache bez zámku pri čakajúcom SNTP; PG263 opravil len naše volania. Ďalšie cesty cez hostByName: netlink.cpp:72 nbResolve, telemetry.cpp:921-922. Napätie nie (C48: bez štartu, 12 V OK; ENGINE START až v b49). Návrh opravy čaká na Vojtecha. |

## 7. Plán (poradie podľa dohody; uzatvára len Vojtech)
1. PG340: inštalácia nestbox-ota 2.8.0 (sudo ultd) → príjem sniffov na serveri (cardata-sniff.path, seed) → OTA J0 (PG342, vlastné áno) → prvý príchod na uLTD_ext overí posielanie.
2. Spoločné vyhodnotenie sniffov (aj jazdy 20. a 21.11.) – evaluate.py len spolu.
3. PG305: 26.11. otázka prepnúť Peugeot na v2o_a3.
4. AdBlue mierka pri najbližšom doliatí so známym množstvom.
5. PG304 / PG327 značky a limit z vyhodnotenia sniffov.
7. U9K2652Z (2Z): Vojtech 25.11.: v poriadku, že má build v2o_a3 (vlastný profil), na 2Z sa ešte bude robiť diag a sniff. 25.11. 14:27 zapnutý na zdroji (NTLIRIS 192.168.0.33), 14:28 OTA na build Nov 25 2026 00:17:18 (VALID 14:28:54, b1048), 14:29-14:30 poslal S1-S10 (najväčší S7 688525 B za 5 s), na boxe mu nič neostalo. J0 od 25.11. 14:28 v Toyote (b30, cell online). Plán (Vojtech 26.11. „zajtra asi“): 2Z do Caddy 27.11. - jazda + SNIFF (zapnem cmd.sh, keď napíše že jazdí); overiť čítanie profilu 2K, DID 1018 za jazdy, READ 02BD; Launch dvere + motor podľa odpovede (2K_TABULKA.md).
6. Z PROJEKT_STAV.md 6 (otvorené dlhšie): PG222 mazanie potvrdených /DATA pri plnej SD, box bez SIM, šifrovanie flash/NVS, ACK test v aute, CELL REG LOST v aute, OTA cez cell v aute.
