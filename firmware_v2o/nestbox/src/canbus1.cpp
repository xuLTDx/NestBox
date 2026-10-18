// see canbus1.h - the TWAI part is the v1F sniff code (listen-only start/stop, rx task, probe)
#include "canbus1.h"
#include "driver/twai.h"
#include "esp_timer.h"
#include "max7317.h"
#include "hw.h"
#include "nbread.h"

void carDecode(uint32_t id, bool ext, uint8_t dlc, const uint8_t* data);   // cardecode.cpp

#define SPEED_PROBE_MS 2000
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
  bool ok = start();
  s_probeStart = millis();
  s_probeFrames = 0;
  xTaskCreatePinnedToCore(rxTask, "can1rx", 3072, nullptr, 10, nullptr, 0);
  Serial.printf("[CAN1] listen-only %s\n", ok ? "started" : "FAILED");
}

void can1Tick()
{
  if (s_found) return;           // the speed is kept for the rest of the boot (a sleeping car only goes quiet)
  if (s_frames > s_probeFrames) {
    s_found = true;
    Serial.printf("[CAN1] speed %lu kbit/s\n", (unsigned long)s_speeds[s_speedIdx]);
    return;
  }
  if (millis() - s_probeStart < SPEED_PROBE_MS) return;
  xSemaphoreTake(s_mux, portMAX_DELAY);
  s_pause = true;
  for (int i = 0; i < 100 && !s_paused; i++) delay(2);
  stop();
  s_speedIdx = (s_speedIdx + 1) % 3;
  start();
  s_probeFrames = s_frames;
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
