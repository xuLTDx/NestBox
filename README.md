# NestBox

In-car device firmware for the **FleetNest** fleet system (`xuLTDx/FleetNest`).
Name: generation + hardware letter.

| Name | Hardware | State |
|---|---|---|
| **NestBox_v1F** | [Freematics ONE+](https://freematics.com/products/freematics-one-plus/) | in use (VW Passat B8), this repo's `firmware_v5/telelogger/` |
| **NestBox_v2O** | OVMS v3.3 (SIM7600 modem, 3 CAN buses) | bench-tested (2026-10-19), first car test pending; `firmware_v2o/nestbox/`. Built from the v1F skeleton and our verified code (delivery queue + server ACK, SD digest, server messages, resumable OTA, sniff, READ); OVMS code only as a reference for drivers. Design: [`firmware_v2o/NAVRH.md`](firmware_v2o/NAVRH.md) |

NestBox_v1F logs OBD, GPS and device data to the SD card, sends it live to FleetNest
(UDP, the Freematics message format) and updates itself over WiFi or cellular from a
self-hosted OTA server (`nestbox-ota`, repo `xuLTDx/NestBox-OTA`). Until 2026-10-14 the
server was a Traccar fork.

The v1F firmware lives in [`firmware_v5/telelogger/`](firmware_v5/telelogger/). How it works
internally, subsystem by subsystem: [`firmware_v5/telelogger/ARCHITECTURE.md`](firmware_v5/telelogger/ARCHITECTURE.md).

## What it does

- **No data lost to coverage gaps.** Every sample is always written to SD, whether or
  not a network is up. After an outage, unsent log files are replayed to FleetNest in
  order before any new live data, so trip history and distance stay consistent.
- **Correct timestamps on late data.** GPS date and time are sent together, so data
  replayed days later keeps its real date. A lost GPS fix is reported as lost instead
  of repeating the last position.
- **Updates over WiFi or cellular (pull-OTA).** When a new build is published, the
  server asks the device to check (an `OTA_READY` command from FleetNest). The device downloads the
  firmware to SD, verifies SHA256, and only then flashes it. An interrupted download
  resumes where it stopped (HTTP `Range`), including after a reboot. A failed
  or corrupt download never gets flashed; the old firmware keeps running.
- **Two WiFi networks and cellular fallback.** WiFi first, cellular (SIM7670) when out of
  range. While on cellular, the device retries WiFi only when GPS puts it near a
  known place that has WiFi. Those places come from the company's places in
  FleetNest. WiFi is always off in standby, to spare the car battery.
- **Local HTTP API and web page** (port 80, also over the device's own fallback access
  point): live sensor values (`/api/live`, `live.html`), runtime settings
  (`/api/control?cmd=KEY=value`), SD log files (`/api/list`, `/api/log`, …).
- **No Bluetooth** since 2026-10-12 (removed, see `platformio.ini`).

## Hardware

Tested on one device: Freematics ONE+ with ESP32 (16 MB flash, 8 MB PSRAM) and a
**SIM7670E-LN** cellular modem. The cellular code targets the SIM7670's `AT+CCH*`
(TLS) and `AT+CIP*` (UDP) commands; other SIMCom modems keep the upstream code paths
but are untested here.

## Build and flash

[PlatformIO](https://platformio.org). NestBox_v1F, from `firmware_v5/telelogger/`:

```
pio run -t upload        # build and flash over USB
pio run -t uploadfs      # web files in dashboard/ (live.html, …)
```

- **WiFi passwords are never stored in the repo.** `wifi_secrets.py` asks for them at
  build time; they can also be set on the device later over the HTTP API.
- **Automatic OTA publishing is optional.** Copy `publish_ota_config.example.json` to
  `publish_ota_config.json` (git-ignored, holds the OTA token and the server's SSH
  target).
  With that file present, **every** `pio run` publishes the build to the OTA server
  and triggers the device to update, so rename it aside while experimenting.
- Everything else is runtime configuration stored on the device (NVS): server, WiFi,
  OTA token and host, standby, OBD options. See `ARCHITECTURE.md` §A6.

NestBox_v2O, from `firmware_v2o/nestbox/`:

```
pio run -e ovms33 -t upload   # build and flash over USB
```

- Uses the same `wifi_secrets.py` as v1F. The build never publishes anything; a build is
  published to `nestbox-ota` by type (`type:NestBox_v2O`, see NestBox-OTA `publish_receiver.py`)
  or pushed to the box's own `/api/ota` over WiFi.
- `-e ovms33_load` = bench build that generates 3000 CAN frames/s itself (SD path test).

## Server side

Kept in separate repositories:

- **FleetNest** (`xuLTDx/FleetNest`, since 2026-10-14): receives the UDP protocol (ACK after
  storage), commands (OTA_READY, SNIFF, RESEND, READ, APN), SD = database check (PID 0x398).
- **Traccar fork** (until 2026-10-14, stopped): received the device's UDP protocol and adds a Slovak trip logbook
  (*Kniha jázd*), business addresses, and odometer calibration from GPS distance.
- **nestbox-ota** (`xuLTDx/NestBox-OTA`, renamed from freematics-ota 2026-10-18): small HTTPS
  service that serves firmware to the device (`/api/nestbox/ota/<token>/meta.json`,
  `firmware.bin` with resume support), tells it when to update, and provides the
  known-WiFi-locations list.

## Known issues

- A reset in the middle of an SD write can leave the SD card unresponsive until the
  device is power-cycled.

## Repository layout

| Path | |
|---|---|
| `firmware_v5/telelogger/` | NestBox_v1F firmware |
| `firmware_v2o/` | NestBox_v2O: design (`NAVRH.md`, `LOGIKA.mmd`), car test checklist, firmware in `nestbox/` |
| `libraries/NestBoxCommon/` | code shared by v1F and v2O (READ command, APN from the server) |
| `libraries/FreematicsPlus/` | device, GPS, OBD, MEMS and network (WiFi/cellular/BLE) library |
| `libraries/httpd/` | small HTTP server used for the local API |

## Credits and license

Based on Stanley Huang's [Freematics](https://github.com/stanleyhuangyc/Freematics)
firmware and libraries, distributed under the BSD license stated in their source file
headers.
