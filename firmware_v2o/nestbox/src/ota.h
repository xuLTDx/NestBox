// Pull-OTA - NestBox_v2O step C. The v1F way (telelogger.ino performPullOtaCheck /
// performPullOtaFlash, server nestbox-ota ota_server.py, was freematics-ota):
//   GET https://<host>:<port>/api/nestbox/ota/<token>/meta.json?build=<build>&variant=NestBox_v2O
//     -> {"available":true,"size":N,"sha256":"..."} when the registry's target build is newer;
//   GET .../firmware.bin (Range: bytes=N- to resume) into /ota_fw.bin on SD, SHA256 over the
//     whole file, /ota_resume.txt "<size>\n<sha256>" identifies a partial download;
//   GET .../ota_confirm?size=N&sha256=<16 hex> after the check passed;
//   flash from SD - here only while the engine is off (v1F: at standby / boot), then restart.
// New in v2O: the downloaded image must contain NESTBOX_IMAGE_MARK (this firmware's own
// marker) - the server no longer checks the variant (ota_server.py docstring, 2026-09-16),
// so a v1F image behind a wrong token would otherwise be flashed into the OVMS.
// TLS without certificate check, as v1F (WifiHTTP: client.setInsecure()).
// WiFi only for now; over the modem later (SIM7600 HTTPS - untested).
// Every step: an SD event line + PID 0x399 text in the next records (v1F otaLog).
#pragma once
#include <Arduino.h>

extern const char NESTBOX_IMAGE_MARK[];

void otaBegin();                     // after SD + NVS: settings, the OTA task
void otaRequest();                   // server OTA_READY / web: check now
bool otaBusy();                      // a check / download runs now (standby waits for it)
int otaTakeStatus(uint8_t* out, int cap);   // the oldest pending step line (records)
void statusLine(const char* text);   // p58: a text line to FleetNest the same way (BOOT, WIFI)
int otaStatus(char* buf, int size);  // web
bool otaSet(const char* key, const char* value);   // OTA_TOKEN / OTA_HOST / OTA_PORT
