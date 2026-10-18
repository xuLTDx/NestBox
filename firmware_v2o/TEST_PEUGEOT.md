# Test Peugeot Traveller (AA458DN) – kontrolný zoznam

Cieľ: nahrať **obe zbernice** naraz (OBD 6/14 a 3/8) počas diagnostiky Launchom a bežnej jazdy, overiť NestBox_v2O
v aute vedľa NestBox_v1F, zistiť palivo, AdBlue, rýchlostný limit a zdroje prebúdzania.

## Pred odchodom (doma, WiFi)
- [ ] v2O (OVMS): firmvér aktuálny (stránka http://192.168.0.117/), SD karta, `SNIFF_VEH=Peugeot_K0`.
- [ ] **Sniff v2O spustiť doma** na dlhý čas (napr. 240 min) – po vypnutí a zapnutí v aute sám pokračuje so zvyškom
      minút do nového súboru (overené na stole 2026-10-18). V aute WiFi nie je a modem v2O zatiaľ nie je overený.
- [ ] v1F (Freematics): sniff zo stránky FleetNest (`SNIFF=1,240`) alebo doma z jeho stránky.
- [ ] Kábel OVMS: CAN1 = OBD 6/14 (DB9 7/2), CAN2 = OBD 3/8 (DB9 5/4), 12 V = OBD 16 (DB9 9), zem = OBD 4 (DB9 3);
      CAN3 podľa tvojho rozhodnutia. **Y-kábel**, aby v OBD boli naraz v1F aj OVMS (a Launch).
- [ ] Hodinky / mobil na zapisovanie časov (stačí fotka s časom).

## V aute – zapísať čas pri každom kroku
1. Zapojiť (Y-kábel: v1F + OVMS), počkať 1 min.
2. Odomknúť / zamknúť auto (diaľkovým), otvoriť a zavrieť dvere vodiča, posuvné dvere – **časy**.
3. Zapaľovanie bez motora 1 min – čas. **Ručička paliva a AdBlue na palubovke** (fotka).
4. Naštartovať – čas; voľnobeh 1 min.
5. Launch: moduly BSI, NAC (telefón), motor (DCM62), ESP90, MDPLC – pri každom **čas začiatku a konca** (dopyty
   Launchu a odpovede auta sa nahrajú – podľa nich potom READ).
6. Telefón pripojiť a odpojiť cez Bluetooth – časy.
7. Jazda so **zapnutou navigáciou**; pri pár značkách rýchlosti zapísať **čas a limit** (porovnanie s 0x1E9).
8. Pri jazde aspoň raz zastaviť na štart-stop (motor sa sám vypne) – čas.
9. Koniec: motor vypnúť, zamknúť – časy; nechať zapojené ešte 5 min (zaspávanie zberníc).

## Po teste (Claude)
- Stiahnuť sniff súbory oboch zariadení, porovnať 6/14: v2O CAN1 = v1F rámec po rámci; straty MCP2515 na 3/8.
- Hľadať: palivo / AdBlue (proti fotkám), limit 0x1E9 (proti zapísaným značkám), ktorá zbernica sa budí prvá,
  dopyty Launchu (ID a služby) → kandidáti pre READ; tachometer 0x552 = palubovka.
- Záznamy v2O vo FleetNest (VXQH2SY2 → AA458DN): ŠTART/STOP z otáčok, tachometer.
