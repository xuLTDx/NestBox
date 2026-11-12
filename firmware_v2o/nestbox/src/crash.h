// Crash capture (PG175, 12.11.2026).
// Why: bench U9K2M3J0 12.11.2026 16:47:18 the a3 build restarted with "BOOT reset=PANIC step=NET LINK WIFI" on the
// 12 V supply - the panic dump went only to the USB serial port (ESP-IDF panic output is esp_rom_printf, not the log
// hook of netlog.cpp), /api/serial is lost on a restart and the flash has no coredump partition
// (partitions_v2o.csv, "No core dump partition found!" at every boot). Vojtech 27.10. + 12.11.: a debug log must be.
// What: a handler set with set_arduino_panic_handler() (Arduino 3 wraps esp_panic_handler itself, esp32-hal-misc.c
// 457-462, and walks the backtrace as ESP-IDF panic_arch.c); before the normal panic output the reason, the core, the task, the fault address and up to CRASH_BT return
// addresses go into RTC memory not cleared at boot (survives the panic restart, not a power-on), and the PSRAM cache
// is written back (the /api/serial ring, netlog.cpp, survives the restart only if its last lines reached PSRAM).
// The next boot sends "ERR SYS panic <reason> <task>" + "PANIC bt <addresses>" lines (FleetNest area SYS) and the
// full text as an SD event. Addresses -> function + line with addr2line on the build's firmware.elf (kept per build).
#pragma once

void crashBegin();                  // setup(), early: the panic handler
void crashReportLast();              // setup(), after telemetryBegin(): the lines of a panic before this boot
