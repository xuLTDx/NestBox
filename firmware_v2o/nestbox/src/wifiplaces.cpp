#include "wifiplaces.h"
#include <nvs.h>
#include <math.h>

extern nvs_handle_t nvs;                       // telemetry.cpp ("storage")

// in PSRAM, not static: 16 x 112 bytes of internal memory are better left to TLS (PG201: 15 KB of static buffers
// broke HTTPS over WiFi, 13.11.2026)
static WifiPlace* s_pl;
static volatile int s_n;
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;   // written by the ota task, read by loop()

static bool alloc()
{
  if (!s_pl) s_pl = (WifiPlace*)ps_calloc(WIFI_PLACES_MAX, sizeof(WifiPlace));
  return s_pl != nullptr;
}

void wifiPlacesBegin()
{
  if (!alloc()) return;
  size_t l = WIFI_PLACES_MAX * sizeof(WifiPlace);
  if (nvs_get_blob(nvs, "WIFI_NETS", s_pl, &l) != ESP_OK || l % sizeof(WifiPlace)) { s_n = 0; return; }
  for (int i = 0; i < (int)(l / sizeof(WifiPlace)); i++) {    // whatever the flash held: the strings end
    s_pl[i].ssid[sizeof(s_pl[i].ssid) - 1] = 0;
    s_pl[i].pass[sizeof(s_pl[i].pass) - 1] = 0;
  }
  s_n = l / sizeof(WifiPlace);
}

bool wifiPlacesStore(const String& body, bool* changed, int* total, int* usable)
{
  if (!alloc()) return false;
  WifiPlace* tmp = (WifiPlace*)ps_calloc(WIFI_PLACES_MAX, sizeof(WifiPlace));
  if (!tmp) return false;
  int n = 0, lines = 0;
  bool bad = false;
  for (int at = 0; at < (int)body.length() && !bad;) {
    int nl = body.indexOf('\n', at);
    if (nl < 0) nl = body.length();
    String ln = body.substring(at, nl);
    at = nl + 1;
    if (ln.endsWith("\r")) ln.remove(ln.length() - 1);
    if (!ln.length()) continue;
    int t1 = ln.indexOf('\t'), t2 = t1 < 0 ? -1 : ln.indexOf('\t', t1 + 1), t3 = t2 < 0 ? -1 : ln.indexOf('\t', t2 + 1),
        t4 = t3 < 0 ? -1 : ln.indexOf('\t', t3 + 1);
    if (t4 < 0) { bad = true; break; }         // not the list (an error page): keep what we have
    lines++;
    String ssid = ln.substring(t3 + 1, t4), pass = ln.substring(t4 + 1);
    WifiPlace p = {};
    p.lat = ln.substring(0, t1).toFloat();
    p.lng = ln.substring(t1 + 1, t2).toFloat();
    p.radiusM = ln.substring(t2 + 1, t3).toFloat();
    if (!ssid.length() || ssid.length() > 32 || pass.length() > 64 || p.radiusM < 0 || n >= WIFI_PLACES_MAX) continue;
    if (p.radiusM > 0 && !p.lat && !p.lng) continue;   // an address without a position
    strcpy(p.ssid, ssid.c_str());
    strcpy(p.pass, pass.c_str());
    tmp[n++] = p;
  }
  if (!bad) {
    *total = lines;
    *usable = n;
    *changed = n != s_n || memcmp(tmp, s_pl, n * sizeof(WifiPlace));
    if (*changed) {
      portENTER_CRITICAL(&s_mux);
      memcpy(s_pl, tmp, n * sizeof(WifiPlace));
      s_n = n;
      portEXIT_CRITICAL(&s_mux);
      if (n) nvs_set_blob(nvs, "WIFI_NETS", tmp, n * sizeof(WifiPlace));
      else nvs_erase_key(nvs, "WIFI_NETS");
      nvs_commit(nvs);
    }
  }
  memset(tmp, 0, WIFI_PLACES_MAX * sizeof(WifiPlace));       // passwords do not stay in freed memory
  free(tmp);
  return !bad;
}

int wifiPlacesCount() { return s_n; }

bool wifiPlaceGet(int i, WifiPlace* out)
{
  bool ok = false;
  portENTER_CRITICAL(&s_mux);
  if (s_pl && i >= 0 && i < s_n) { *out = s_pl[i]; ok = true; }
  portEXIT_CRITICAL(&s_mux);
  return ok;
}

// metres between two positions; a flat-earth step is exact enough for a radius of some hundred metres
bool wifiPlaceHolds(const WifiPlace& p, float lat, float lng)
{
  if (p.radiusM <= 0) return false;
  const float R = 6371000.0f, D = 0.017453293f;
  float x = (lng - p.lng) * D * cosf((lat + p.lat) * 0.5f * D);
  float y = (lat - p.lat) * D;
  return R * sqrtf(x * x + y * y) <= p.radiusM;
}
