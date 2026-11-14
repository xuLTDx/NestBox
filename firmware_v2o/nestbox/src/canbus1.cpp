// see canbus1.h - the TWAI part is the v1F sniff code (listen-only start/stop, rx task, probe)
#include "canbus1.h"
#include "netlog.h"                  // PG101: nbout = Serial + /api/serial
#include "driver/twai.h"
#include "esp_timer.h"
#include "max7317.h"
#include "hw.h"
#include "nbread.h"

void carDecode(uint32_t id, bool ext, uint8_t dlc, const uint8_t* data);   // cardecode.cpp

#include <Preferences.h>
#define SPEED_PROBE_MS 2000
float read12V();                              // telemetry.cpp
// PG250 (14.11.2026, Vojtech "Pg250 ano"): the speed the box once heard frames at is kept (NVS "can1"/"kbit") and the
// box listens on it from the boot; while the bus is silent the speed is not changed. Why: without frames the probe
// restarted the TWAI driver every 2 s for ever (bench U9K2M3J0 14.11.2026: /api/live can1 frames=0, kbit cycling,
// ~43000 restarts a day) - and a car that wakes while the box listens on a wrong speed loses its first frames
// (engine START, rpm) until the probe comes round. Another speed is tried only when (a) the bus shows errors
// (traffic at another speed - TWAI bus_error_count rises; NEOVERENE that the listen-only controller counts them,
// the "[CAN1] probe" line shows it in the car) or (b) no speed was ever found and the 12 V says the engine runs
// (>= 12.8 V, standby.h ENGINE_OFF_VOLTAGE): then the 3 speeds in turn as before.
static bool s_known;                          // a speed from NVS
static uint32_t s_probeErr;                   // bus_error_count at the probe start
static const uint32_t s_speeds[] = {500, 250, 125};
static int s_speedIdx;
static bool s_found;
static volatile bool s_running, s_pause, s_paused;
static volatile uint32_t s_frames, s_lastMs;
static uint32_t s_probeStart, s_probeFrames;
static volatile Can1Fn s_hook;
static SemaphoreHandle_t s_mux;

static twai_timing_config_t timingFor(uint32_t kbit)
{
  switch (kbit) {
    case 250: return TWAI_TIMING_CONFIG_250KBITS();
    case 125: return TWAI_TIMING_CONFIG_125KBITS();
    default:  return TWAI_TIMING_CONFIG_500KBITS();
  }
}

static bool start(bool active = false)
{
  mx7317Out(MX_CAN1_RS, active ? 0 : 1);      // transmitter off, except for a READ exchange
  twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)HW_CAN1_TX, (gpio_num_t)HW_CAN1_RX,
                                                        active ? TWAI_MODE_NORMAL : TWAI_MODE_LISTEN_ONLY);
  g.rx_queue_len = 128;
  g.tx_queue_len = active ? 2 : 0;
  twai_timing_config_t t = timingFor(s_speeds[s_speedIdx]);
  twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  if (twai_driver_install(&g, &t, &f) != ESP_OK) return false;
  if (twai_start() != ESP_OK) { twai_driver_uninstall(); return false; }
  s_running = true;
  return true;
}

static void stop()
{
  if (!s_running) return;
  s_running = false;
  twai_stop();
  twai_driver_uninstall();
}

static void rxTask(void*)
{
  for (;;) {
    if (s_pause || !s_running) {
      s_paused = true;
      vTaskDelay(pdMS_TO_TICKS(5));
      continue;
    }
    s_paused = false;
    twai_message_t m;
    if (twai_receive(&m, pdMS_TO_TICKS(20)) != ESP_OK) continue;
    int64_t us = esp_timer_get_time();
    uint8_t dlc = m.data_length_code > 8 ? 8 : m.data_length_code;
    s_frames++;
    s_lastMs = millis();
    carDecode(m.identifier, m.extd, dlc, m.data);
    Can1Fn h = s_hook;
    if (h) h(us, m.identifier, m.extd, dlc, m.data);
  }
}

void can1Begin()
{
  s_mux = xSemaphoreCreateMutex();
  {                                           // PG250
    Preferences p;
    p.begin("can1", true);
    int k = p.getInt("kbit", -1);
    p.end();
    for (int i = 0; i < 3; i++) if ((int)s_speeds[i] == k) { s_speedIdx = i; s_known = true; }
  }
  bool ok = start();
  s_probeStart = millis();
  s_probeFrames = 0;
  xTaskCreatePinnedToCore(rxTask, "can1rx", 3072, nullptr, 10, nullptr, 0);
  nbout.printf("[CAN1] listen-only %s %lu kbit/s%s\n", ok ? "started" : "FAILED", (unsigned long)s_speeds[s_speedIdx],
               s_known ? " (kept)" : "");
}

void can1Tick()
{
  if (s_found) return;           // the speed is kept for the rest of the boot (a sleeping car only goes quiet)
  if (s_frames > s_probeFrames) {
    s_found = true;
    nbout.printf("[CAN1] speed %lu kbit/s\n", (unsigned long)s_speeds[s_speedIdx]);
    if (!s_known) {                           // PG250: kept for the next boots
      Preferences p;
      p.begin("can1", false);
      p.putInt("kbit", (int)s_speeds[s_speedIdx]);
      p.end();
      s_known = true;
    }
    return;
  }
  if (millis() - s_probeStart < SPEED_PROBE_MS) return;
  {                                           // PG250: change the speed only with a reason
    twai_status_info_t st = {};
    uint32_t err = (s_running && twai_get_status_info(&st) == ESP_OK) ? st.bus_error_count : 0;
    bool errors = err > s_probeErr;
    bool engine = !s_known && read12V() >= 12.8f;
    if (errors) nbout.printf("[CAN1] probe: %lu bus errors at %lu kbit/s - next speed\n",
                             (unsigned long)(err - s_probeErr), (unsigned long)s_speeds[s_speedIdx]);
    s_probeErr = err;
    if (!errors && !engine) { s_probeStart = millis(); return; }   // silent: keep listening on this speed
  }
  xSemaphoreTake(s_mux, portMAX_DELAY);
  s_pause = true;
  for (int i = 0; i < 100 && !s_paused; i++) delay(2);
  stop();
  s_speedIdx = (s_speedIdx + 1) % 3;
  start();
  s_probeFrames = s_frames;
  s_probeErr = 0;                             // a new driver counts from 0
  s_probeStart = millis();
  s_pause = false;
  xSemaphoreGive(s_mux);
}

Can1Stats can1Stats()
{
  Can1Stats s = {};
  s.running = s_running;
  s.kbit = s_speeds[s_speedIdx];
  s.speedFound = s_found;
  s.frames = s_frames;
  s.lastFrameMs = s_lastMs;
  twai_status_info_t st = {};
  if (s_running && twai_get_status_info(&st) == ESP_OK) {
    s.rxMissed = st.rx_missed_count;
    s.rxOverrun = st.rx_overrun_count;
    s.busErr = st.bus_error_count;
  }
  return s;
}

void can1SetSniffHook(Can1Fn fn) { s_hook = fn; }

// ---- READ exchange -------------------------------------------------------
static bool xSend(void*, uint32_t id, const uint8_t d[8])
{
  twai_message_t m = {};
  m.identifier = id;
  m.ss = 1;                                   // single shot: an unanswered frame is not repeated (v1F canodo)
  m.data_length_code = 8;
  memcpy(m.data, d, 8);
  return twai_transmit(&m, pdMS_TO_TICKS(20)) == ESP_OK;
}

static bool xRecv(void*, uint32_t id, uint8_t d[8], uint8_t* dlc, uint32_t timeoutMs)
{
  uint32_t t0 = millis();
  while (millis() - t0 < timeoutMs) {
    twai_message_t m;
    if (twai_receive(&m, pdMS_TO_TICKS(20)) != ESP_OK) continue;
    uint8_t n = m.data_length_code > 8 ? 8 : m.data_length_code;
    s_frames++;
    s_lastMs = millis();
    carDecode(m.identifier, m.extd, n, m.data);
    Can1Fn h = s_hook;
    if (h) h(esp_timer_get_time(), m.identifier, m.extd, n, m.data);
    if (m.identifier == id && !m.extd) { memset(d, 0, 8); memcpy(d, m.data, n); *dlc = n; return true; }
  }
  return false;
}

int can1Exchange(const NbRead* r, uint8_t* resp, int cap, int* respLen, uint8_t* nrc)
{
  xSemaphoreTake(s_mux, portMAX_DELAY);
  s_pause = true;
  for (int i = 0; i < 100 && !s_paused; i++) delay(2);
  stop();
  int result = NBR_NOT_READY;
  if (start(true)) {
    NbCanPort p = {xSend, xRecv, nullptr};
    result = nbReadExchange(&p, r, resp, cap, respLen, nrc);
    stop();
  }
  start(false);                               // listen only again, transmitter off
  s_pause = false;
  xSemaphoreGive(s_mux);
  return result;
}
