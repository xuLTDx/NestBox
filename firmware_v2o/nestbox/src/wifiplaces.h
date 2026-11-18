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
int wifiPlacesCount();                         // 0 = no list: no WiFi is searched (PG230, 14.11.2026: no built-in networks)
bool wifiPlaceGet(int i, WifiPlace* out);
bool wifiPlaceHolds(const WifiPlace& p, float lat, float lng);   // the position is inside its radius

// PG233 (18.11.2026, install.h): the network chosen on the box's mini page. Kept apart from the server list (NVS
// WIFI_USER, the list download never overwrites it) and served as the list's last entry by wifiPlacesCount() /
// wifiPlaceGet(). Saved without an address (radius 0 = searched like a phone hotspot); the first time the box is
// joined to it with a position, that position + WIFI_USER_RADIUS_M is kept, from then on it is searched only there
// (Vojtech 18.11.2026: "v okoli, moze byt 200m" - a box in a car must not search everywhere, PG205).
#define WIFI_USER_RADIUS_M 200.0f
bool wifiUserSet(const char* ssid, const char* pass);   // false = bad length
bool wifiUserGet(WifiPlace* out);
bool wifiUserLearn(float lat, float lng);    // true = the position was taken now
