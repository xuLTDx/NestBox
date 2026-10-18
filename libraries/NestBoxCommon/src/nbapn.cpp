// see nbapn.h
#include "nbapn.h"

static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;
static char s_status[64];
static bool s_new;
static char s_trial[40];

static void setStatus(const char* fmt, const char* a, const char* b = "")
{
  char t[64];
  snprintf(t, sizeof(t), fmt, a, b);
  portENTER_CRITICAL(&s_mux);
  strncpy(s_status, t, sizeof(s_status) - 1);
  s_new = true;
  portEXIT_CRITICAL(&s_mux);
  Serial.printf("[APN] %s\n", t);
}

void nbApnCommand(nvs_handle_t nvs, const char* arg, const char* buildDefault)
{
  char a[40];
  snprintf(a, sizeof(a), "%s", strcmp(arg, "DEFAULT") ? arg : buildDefault);
  for (char* p = a; *p; p++) if (*p == '*' || *p == ',' || *p == '\r' || *p == '\n') { *p = 0; break; }
  char cur[40] = "";
  size_t l = sizeof(cur);
  nvs_get_str(nvs, "CELL_APN", cur, &l);
  if (!strcmp(a, cur[0] ? cur : buildDefault)) {           // nothing to try: already in use
    nvs_erase_key(nvs, "APN_NEW");
    nvs_commit(nvs);
    setStatus("APN SAME %s", a);
    return;
  }
  nvs_set_str(nvs, "APN_NEW", a);
  nvs_set_u8(nvs, "APN_TRY", 0);
  nvs_commit(nvs);
  setStatus("APN TRIAL %s (next modem start)", a);
}

const char* nbApnPick(nvs_handle_t nvs, const char* current)
{
  size_t l = sizeof(s_trial);
  s_trial[0] = 0;
  if (nvs_get_str(nvs, "APN_NEW", s_trial, &l) == ESP_OK && s_trial[0]) return s_trial;
  return current;
}

void nbApnAfterLogin(nvs_handle_t nvs, bool loginOk)
{
  char t[40] = "";
  size_t l = sizeof(t);
  if (nvs_get_str(nvs, "APN_NEW", t, &l) != ESP_OK || !t[0]) return;   // no trial running
  if (loginOk) {
    nvs_set_str(nvs, "CELL_APN", t);
    nvs_erase_key(nvs, "APN_NEW");
    nvs_erase_key(nvs, "APN_TRY");
    nvs_commit(nvs);
    setStatus("APN OK %s", t);
    return;
  }
  uint8_t n = 0;
  nvs_get_u8(nvs, "APN_TRY", &n);
  n++;
  if (n >= NB_APN_TRIES) {
    nvs_erase_key(nvs, "APN_NEW");
    nvs_erase_key(nvs, "APN_TRY");
    nvs_commit(nvs);
    setStatus("APN REVERT %s failed, old APN again", t);
  } else {
    nvs_set_u8(nvs, "APN_TRY", n);
    nvs_commit(nvs);
    char m[8];
    snprintf(m, sizeof(m), "%u", n);
    setStatus("APN TRY %s failed #%s", t, m);
  }
}

int nbApnTakeStatus(uint8_t* out, int cap)
{
  int n = 0;
  portENTER_CRITICAL(&s_mux);
  if (s_new) {
    n = strnlen(s_status, cap);
    memcpy(out, s_status, n);
    s_new = false;
  }
  portEXIT_CRITICAL(&s_mux);
  return n;
}
