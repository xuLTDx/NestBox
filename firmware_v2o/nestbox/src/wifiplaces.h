// PG205 + PG208 (13.-14.11.2026): WiFi is searched only where a known network can be - the places of the box's
// company that have a WiFi in FleetNest (position, radius, name, password), as v1F did (telelogger.ino
// "Geofence-based WiFi known locations", 22.9.2026: "po celu dobu na cell sa zbytocne wifi neskusa az ked sa vojde
// do okruhu business adresy").
// Why: v2O scanned for its 2 built-in networks every 30 s everywhere (main.cpp wifiTick) - Vojtech 13.11.2026:
// "naco zbytocne skusat WiFi niekde kde to netreba". PG208 (Vojtech 14.11.2026 "208 ano"): the networks themselves
// (name + password) come with the list, so a new depot or a phone hotspot needs no new firmware; the 2 built-in
// ones are used only while the box has no list.
// The list: GET /api/nestbox/ota/<token>/wifi.txt (HTTPS), one network per line, fields split by TAB:
//   lat <TAB> lon <TAB> radius m <TAB> name <TAB> password
// radius 0 = a network without an address (a phone hotspot): searched only in the windows where all networks are
// (after an engine start, after WIFI=1, without a position - main.cpp wifiSearch).
// Kept in NVS (survives sleep and power loss), NOT encrypted: the box must read the password to join, a key in the
// same flash protects nothing (agreed 14.11.2026; the server keeps it encrypted, the networks should be guest-like).
// The passwords are never printed or sent anywhere.
#pragma once
#include <Arduino.h>

#define WIFI_PLACES_MAX 16

struct WifiPlace {
  float lat, lng, radiusM;                     // radiusM 0 = no address
  char ssid[33];
  char pass[65];
};

void wifiPlacesBegin();                        // load from NVS
// the downloaded list; false = not the list (kept as it was). *total = lines, *usable = stored
bool wifiPlacesStore(const String& body, bool* changed, int* total, int* usable);
int wifiPlacesCount();                         // 0 = no list: the built-in networks, searched everywhere, as before
bool wifiPlaceGet(int i, WifiPlace* out);
bool wifiPlaceHolds(const WifiPlace& p, float lat, float lng);   // the position is inside its radius
