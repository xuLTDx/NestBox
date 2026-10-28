// see cardecode.h
#include "cardecode.h"

static char s_profile[8] = "K0";
static CarValues s_v;
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;

void carSetProfile(const char* p) { snprintf(s_profile, sizeof(s_profile), "%s", p ? p : ""); }
const char* carProfile() { return s_profile; }

CarValues carValues()
{
  portENTER_CRITICAL(&s_mux);
  CarValues v = s_v;
  portEXIT_CRITICAL(&s_mux);
  return v;
}

// CAN1 receive task (canbus1.cpp): every frame
void carDecode(uint32_t id, bool ext, uint8_t dlc, const uint8_t* d)
{
  if (ext || strcmp(s_profile, "K0")) return;
  uint32_t now = millis();
  if (id == 0x552 && dlc >= 7) {
    uint32_t km = ((uint32_t)d[4] << 16) | ((uint32_t)d[5] << 8) | d[6];
    if (km != 0xFFFFFF) {                     // PSA-RE: 0xFFFFFF = invalid
      portENTER_CRITICAL(&s_mux);
      s_v.odoKm = km; s_v.odoAt = now;
      portEXIT_CRITICAL(&s_mux);
    }
  } else if (id == 0x612 && dlc >= 4) {
    if (d[3] != 0xFF) {
      portENTER_CRITICAL(&s_mux);
      s_v.fuelRaw = d[3]; s_v.fuelL = d[3] * 0.5f; s_v.fuelAt = now;
      portEXIT_CRITICAL(&s_mux);
    }
  } else if (id == 0x208 && dlc >= 2) {
    uint16_t raw = ((uint16_t)d[0] << 8) | d[1];
    if (raw != 0xFFFF) {
      portENTER_CRITICAL(&s_mux);
      s_v.rpm = raw * 0.125f; s_v.rpmAt = now;
      if (raw) s_v.rpmPosAt = now;
      portEXIT_CRITICAL(&s_mux);
    }
  } else if (id == 0x348 && dlc >= 6) {
    portENTER_CRITICAL(&s_mux);
    s_v.engState = d[5] & 0x0F; s_v.engAt = now;
    portEXIT_CRITICAL(&s_mux);
  } else if (id == 0x412 && dlc >= 7) {      // p81 doors (bitfield, byte 7 bits 7-3)
    portENTER_CRITICAL(&s_mux);
    s_v.doors = (d[6] >> 3) & 0x1F; s_v.doorsAt = now;
    portEXIT_CRITICAL(&s_mux);
  } else if (id == 0x488 && dlc >= 1) {      // p81 coolant
    if (d[0] != 0xFF) {
      portENTER_CRITICAL(&s_mux);
      s_v.coolC = (int16_t)d[0] - 40; s_v.coolAt = now;
      portEXIT_CRITICAL(&s_mux);
    }
  } else if (id == 0x38D && dlc >= 2) {      // p81 speed from the wheels
    uint16_t raw = ((uint16_t)d[0] << 8) | d[1];
    if (raw != 0xFFFF) {
      portENTER_CRITICAL(&s_mux);
      s_v.speedKmh = raw * 0.01f; s_v.speedAt = now;
      portEXIT_CRITICAL(&s_mux);
    }
  } else if (id == 0x5A8 && dlc >= 8) {
    portENTER_CRITICAL(&s_mux);
    s_v.ureaKm = ((uint32_t)d[0] << 7) | (d[1] >> 1);
    memcpy(s_v.urea, d, 8);
    s_v.ureaAt = now;
    portEXIT_CRITICAL(&s_mux);
  }
}

void carFromRead(uint32_t txId, const uint8_t* r, int len)
{
  if (len < 4 || r[0] != 0x62) return;
  uint16_t did = ((uint16_t)r[1] << 8) | r[2];
  uint32_t now = millis();
  portENTER_CRITICAL(&s_mux);
  if (txId == 0x752 && did == 0xD8C3 && len >= 5) { s_v.fuelBsiCl = ((uint16_t)r[3] << 8) | r[4]; s_v.fuelBsiAt = now; }
  else if (txId == 0x6A8 && did == 0xD558) { s_v.ureaMeasRaw = r[3]; s_v.ureaMeasAt = now; }
  else if (txId == 0x6A8 && did == 0xD517 && len >= 5) { s_v.ureaEstMl = (uint16_t)(r[4] * 62.5f + 0.5f); s_v.ureaEstAt = now; }
  portEXIT_CRITICAL(&s_mux);
}
