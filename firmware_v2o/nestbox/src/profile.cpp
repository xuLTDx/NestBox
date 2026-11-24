// see profile.h (PG283)
#include "profile.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "cardecode.h"

#define PROF_PATH "/spiffs/profile.txt"
#define PROF_TMP  "/spiffs/profile.tmp"
#define PROF_FILE_MAX 16384                  // a whole K0 profile is ~2 KB of lines; more = a broken answer

static Profile* s_prof;                      // the profile in use this boot (PSRAM), nullptr = none
static char s_storedVer[17];                 // the version in the file (may be newer than s_prof until a restart)
static ProfVal* s_val;                       // one per CAN rule of s_prof
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;

// "1,2,3" -> out[]; "-" or "" -> 0 items
static uint8_t parseList(const char* s, uint32_t* out, uint8_t max)
{
  uint8_t n = 0;
  if (!s || !*s || !strcmp(s, "-")) return 0;
  while (*s && n < max) {
    char* e;
    out[n++] = strtoul(s, &e, 10);
    if (e == s) break;
    s = *e == ',' ? e + 1 : e;
    if (!*e) break;
  }
  return n;
}

// one line split at TAB in place; returns the number of fields
static int split(char* line, char** f, int max)
{
  int n = 0;
  f[n++] = line;
  for (char* p = line; *p && n < max; p++)
    if (*p == '\t') { *p = 0; f[n++] = p + 1; }
  return n;
}

// the whole text -> p; false = not a profile file (first line not "V")
static bool parse(const char* text, Profile* p)
{
  memset(p, 0, sizeof(*p));
  p->udsEveryS = 60;
  char* buf = strdup(text);
  if (!buf) return false;
  bool ok = false;
  char* save = nullptr;
  for (char* line = strtok_r(buf, "\r\n", &save); line; line = strtok_r(nullptr, "\r\n", &save)) {
    char* f[16];
    int n = split(line, f, 16);
    uint32_t l[8];
    if (!strcmp(f[0], "V") && n >= 2) {
      snprintf(p->ver, sizeof(p->ver), "%s", f[1]);
      snprintf(p->code, sizeof(p->code), "%s", n >= 3 ? f[2] : "");
      ok = true;
    } else if (!ok) {
      break;                                 // the first line must be V
    } else if (!strcmp(f[0], "M") && n >= 2) {
      if (!strcmp(f[1], "obd")) p->motorObd = true;
      else if (!strcmp(f[1], "can") && n >= 5) {
        p->motorCan = true;
        p->motorId = strtoul(f[2], nullptr, 10);
        p->motorNb = parseList(f[3], l, 4);
        for (int i = 0; i < p->motorNb; i++) p->motorBytes[i] = l[i];
        p->motorMult = atof(f[4]);
      }
    } else if (!strcmp(f[0], "E") && n >= 2) {
      int s = atoi(f[1]);
      if (s >= 10) p->udsEveryS = s;         // < 10 s would keep the bus busy all the time
    } else if (!strcmp(f[0], "C") && n >= 14 && p->nCan < PROF_CAN_MAX) {
      ProfCan& c = p->can[p->nCan];
      c.bus = atoi(f[1]);
      c.id = strtoul(f[2], nullptr, 10);
      c.dlcMin = atoi(f[3]);
      c.nb = parseList(f[4], l, 8);
      for (int i = 0; i < c.nb; i++) c.bytes[i] = l[i];
      c.shift = atoi(f[5]);
      c.mask = strcmp(f[6], "-") ? strtoul(f[6], nullptr, 10) : 0;
      c.mult = atof(f[7]);
      c.add = atof(f[8]);
      c.hasInv = strcmp(f[9], "-") != 0;
      c.inv = c.hasInv ? strtoul(f[9], nullptr, 10) : 0;
      c.raw = f[10][0] == '1';
      c.pid = strtoul(f[11], nullptr, 10);
      c.field = strcmp(f[12], "-") ? atoi(f[12]) : -1;
      c.change = f[13][0] == 'z';
      if (c.nb && c.pid) p->nCan++;
    } else if (!strcmp(f[0], "U") && n >= 6 && p->nUds < PROF_UDS_MAX) {
      ProfUds& u = p->uds[p->nUds];
      u.num = atoi(f[1]);
      u.tx = strtoul(f[2], nullptr, 10);
      u.rx = strtoul(f[3], nullptr, 10);
      uint8_t nb = parseList(f[4], l, 2);
      for (int i = 0; i < nb; i++) u.buses[i] = l[i];
      u.nDid = parseList(f[5], l, 3);
      for (int i = 0; i < u.nDid; i++) u.did[i] = l[i];
      if (u.nDid) p->nUds++;
    } else if (!strcmp(f[0], "O") && n >= 7 && p->nObd < PROF_OBD_MAX) {
      ProfObd& o = p->obd[p->nObd];
      o.tx = strtoul(f[1], nullptr, 10);
      o.rx = strtoul(f[2], nullptr, 10);
      uint8_t nb = parseList(f[3], l, 2);
      for (int i = 0; i < nb; i++) o.buses[i] = l[i];
      o.obdPid = strtoul(f[4], nullptr, 10);
      o.pid = strtoul(f[5], nullptr, 10);
      o.everyS = atoi(f[6]);
      if (o.everyS < 2) o.everyS = 10;
      if (o.pid) p->nObd++;
    } else if (!strcmp(f[0], "F") && n >= 7 && p->nFast < PROF_FAST_MAX) {   // PG319 (profile.h)
      ProfFast& q = p->fast[p->nFast];
      q.tx = strtoul(f[1], nullptr, 10);
      q.rx = strtoul(f[2], nullptr, 10);
      uint8_t nb = parseList(f[3], l, 2);
      for (int i = 0; i < nb; i++) q.buses[i] = l[i];
      q.did = strtoul(f[4], nullptr, 10);
      q.pid = strtoul(f[5], nullptr, 10);
      q.everyS = atoi(f[6]);
      if (q.everyS < 1) q.everyS = 1;
      if (q.pid && q.tx && q.rx && nb) p->nFast++;
    }
  }
  free(buf);
  return ok;
}

static char* readFile(const char* path)
{
  FILE* f = fopen(path, "rb");
  if (!f) return nullptr;
  char* b = (char*)ps_malloc(PROF_FILE_MAX + 1);
  size_t n = b ? fread(b, 1, PROF_FILE_MAX, f) : 0;
  fclose(f);
  if (!b) return nullptr;
  b[n] = 0;
  return b;
}

void profileBegin()
{
  char* text = readFile(PROF_PATH);
  if (!text) return;
  Profile* p = (Profile*)ps_malloc(sizeof(Profile));
  if (p && parse(text, p)) {
    snprintf(s_storedVer, sizeof(s_storedVer), "%s", p->ver);
    if (p->ver[0] && p->code[0]) {           // version "" = the car has no profile: the old way (VEH)
      s_val = (ProfVal*)ps_calloc(p->nCan ? p->nCan : 1, sizeof(ProfVal));
      if (s_val) { s_prof = p; p = nullptr; }
    }
  }
  free(p);
  free(text);
}

const Profile* profileGet() { return s_prof; }
const char* profileStoredVer() { return s_storedVer; }

bool profileStore(const String& body, bool* changed, char* summary, int cap)
{
  *changed = false;
  if (body.length() > PROF_FILE_MAX) { snprintf(summary, cap, "too long %u", body.length()); return false; }
  Profile* p = (Profile*)ps_malloc(sizeof(Profile));
  if (!p) { snprintf(summary, cap, "no memory"); return false; }
  bool ok = parse(body.c_str(), p);
  if (ok) {
    snprintf(summary, cap, "%s %.8s c=%u u=%u o=%u f=%u%s", p->code[0] ? p->code : "none", p->ver, p->nCan, p->nUds,
             p->nObd, p->nFast, p->motorObd ? " m=obd" : p->motorCan ? " m=can" : "");
    if (strcmp(p->ver, s_storedVer)) {
      FILE* f = fopen(PROF_TMP, "wb");      // tmp + rename: a power loss never leaves half a file
      bool w = f && fwrite(body.c_str(), 1, body.length(), f) == body.length();
      if (f) fclose(f);
      if (w) { remove(PROF_PATH); w = rename(PROF_TMP, PROF_PATH) == 0; }
      if (w) { snprintf(s_storedVer, sizeof(s_storedVer), "%s", p->ver); *changed = true; }
      else { snprintf(summary, cap, "file write failed"); ok = false; }
    }
  } else {
    snprintf(summary, cap, "bad file");
  }
  free(p);
  return ok;
}

static uint32_t bytesBE(const uint8_t* d, const uint8_t* idx, uint8_t n)
{
  uint32_t v = 0;
  for (uint8_t i = 0; i < n && i < 4; i++) v = (v << 8) | d[idx[i]];
  return v;
}

void profileCanFrame(uint8_t bus, uint32_t id, uint8_t dlc, const uint8_t* d)
{
  const Profile* p = s_prof;
  if (!p) return;
  uint32_t now = millis();
  if (p->motorCan && id == p->motorId && p->motorNb) {
    bool fits = true;
    for (int i = 0; i < p->motorNb; i++) if (p->motorBytes[i] >= dlc) fits = false;
    if (fits) {
      uint32_t raw = bytesBE(d, p->motorBytes, p->motorNb);
      uint32_t all = p->motorNb >= 4 ? 0xFFFFFFFFu : (1u << (8 * p->motorNb)) - 1;
      if (raw != all) carSetRpm(raw * p->motorMult);   // all bits set = invalid (K0 0x208 0xFFFF)
    }
  }
  for (uint8_t i = 0; i < p->nCan; i++) {
    const ProfCan& c = p->can[i];
    if (c.id != id || c.bus != bus || dlc < c.dlcMin) continue;
    bool fits = true;
    for (int k = 0; k < c.nb; k++) if (c.bytes[k] >= dlc) fits = false;
    if (!fits) continue;
    ProfVal& v = s_val[i];
    if (c.raw) {
      portENTER_CRITICAL(&s_mux);
      bool diff = !v.at || v.rawN != c.nb;
      for (int k = 0; k < c.nb; k++) { if (v.raw[k] != d[c.bytes[k]]) diff = true; v.raw[k] = d[c.bytes[k]]; }
      if (diff) v.chg++;
      v.rawN = c.nb; v.at = now;
      portEXIT_CRITICAL(&s_mux);
      continue;
    }
    uint32_t raw = bytesBE(d, c.bytes, c.nb);
    if (c.hasInv && raw == c.inv) continue;
    raw >>= c.shift;
    if (c.mask) raw &= c.mask;
    float x = raw * (c.mult ? c.mult : 1.0f) + c.add;
    uint32_t val = x < 0 ? (uint32_t)(int32_t)(x - 0.5f) : (uint32_t)(x + 0.5f);
    portENTER_CRITICAL(&s_mux);
    if (!v.at || v.val != val) v.chg++;
    v.val = val; v.at = now;
    portEXIT_CRITICAL(&s_mux);
  }
}

bool profileCanValue(uint8_t i, ProfVal* out)
{
  if (!s_prof || i >= s_prof->nCan) return false;
  portENTER_CRITICAL(&s_mux);
  *out = s_val[i];
  portEXIT_CRITICAL(&s_mux);
  return out->at != 0;
}
