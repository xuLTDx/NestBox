#!/usr/bin/env python3
"""PG266 (18.11.2026, Vojtech "266 ano"): new NestBox_v2O box on USB -> firmware + FleetNest registration + OTA token
in one run. Why: 17.11.2026 box U9K2652Z (for VW Caddy RK762CR) had no token and no WiFi after the USB upload, the
setup went over 2 Claude sessions with manual steps (Vojtech: "uz je to domotane"); FN85 (9.11.) planned this script,
it was never written. Design: /home/claude/notes/handoff/2026-11-18_nestbox-v2o_to_fleetnest_novy-box.md.

Runs on the FleetNest host (secrets are here), the box hangs on the NAS USB (ssh nas, /dev/ttyUSB0):
  1. MAC over esptool -> box ID (same formula as firmware genDeviceID, teleclient.cpp:30; checked 18.11. against
     U9K2652Z, U9K2M3J0, U9K4M3G6).
  2. Backup of the whole 16 MB flash, only if NAS ~/v2o_a3/box_<mac>/orig_16M.bin does not exist (the original OVMS
     firmware is kept once, a re-run never overwrites it).
  3. FleetNest POST /api/nahratie/zariadenie/<ID>/ {"channel":"v2o_a3"} -> token (+ activation code for a new box;
     an existing box gets the same token and no code - FN85).
  4. Image: bootloader + partition table + otadata from the local a3 factory build (first 0x10000 bytes), NVS
     (namespace "storage": OTA_TOKEN, INST_WIFI_PW) at 0x9000, app at 0x10000 = the file the OTA server offers on
     channel v2o_a3 (/home/nestbox-ota/channels.json) - the box starts on the build every other a3 box gets.
  5. One write-flash at 0x0 (esptool verifies the hash), reset, the serial log must show the build, the ID and the SD.
  6. The label (ID, code, QR) is made by FleetNest (FN245, 18.11.: /zariadenia/sklad/, code kept there encrypted),
     not here.
Prints only the ID, OK/CHYBA per step and where the label is - never the token, the code or the password
(rule: no secrets in output or logs).

  novy_box.py [--port /dev/ttyUSB0] [--factory <local factory.bin>]
Needs the venv /home/claude/Claude/pg266/venv (esp-idf-nvs-partition-gen)."""
import argparse, hashlib, json, os, re, secrets, string, subprocess, sys, tempfile, urllib.request

NAS = "nas"
ESPTOOL = "~/venvs/esptool/bin/esptool"
NAS_PY = "~/venvs/esptool/bin/python"            # has pyserial
CHANNELS = "/home/nestbox-ota/channels.json"
CHANNEL = "v2o_a3"
FACTORY = "/home/claude/Claude/nestbox_a3/firmware_v2o/nestbox/.pio/build/v2o/firmware.factory.bin"
FLEETNEST = "https://fleetnest.ultd.sk"
ENV = "/home/claude/secrets/fleetnest.env"
INST_PW_FILE = "/home/claude/secrets/nestbox-install-wifi.txt"
NVS_OFF, NVS_SIZE, APP_OFF = 0x9000, 0x5000, 0x10000      # partitions_v2o.csv


def step(name, ok, detail=""):
    print(f"{'OK   ' if ok else 'CHYBA'} {name}{(' - ' + detail) if detail else ''}", flush=True)
    if not ok:
        sys.exit(1)


def nas(cmd, timeout=900):
    r = subprocess.run(["ssh", NAS, cmd], capture_output=True, text=True, timeout=timeout)
    return r.returncode, r.stdout + r.stderr


def box_id(mac):
    """teleclient.cpp genDeviceID: ESP.getEfuseMac() (MAC bytes little-endian) >> 8, 8 x 5 bits."""
    seed = int.from_bytes(bytes.fromhex(mac.replace(":", "")), "little") >> 8
    out = ""
    for _ in range(8):
        x = seed & 0x1F
        seed >>= 5
        c = chr(x - 10 + 65) if x >= 10 else chr(48 + x)
        out += {"B": "W", "D": "X", "I": "Y", "O": "Z"}.get(c, c)
    return out


def env_value(key):
    for line in open(ENV):
        if line.startswith(key + "="):
            return line.split("=", 1)[1].strip().strip('"').strip("'")
    return ""


def install_password():
    """One password for every box in install mode (Vojtech 18.11.: "jednotne heslo", he writes it into the file
    himself). WPA2 needs 8-63 characters; no comma (NVS CSV)."""
    pw = open(INST_PW_FILE).read().strip() if os.path.exists(INST_PW_FILE) else ""
    step("instalacne heslo", 8 <= len(pw) <= 63 and "," not in pw and "\n" not in pw,
         "" if pw else f"prazdne - zapis ho do {INST_PW_FILE}")
    return pw


def register(ident):
    req = urllib.request.Request(f"{FLEETNEST}/api/nahratie/zariadenie/{ident}/", method="POST",
                                 data=json.dumps({"channel": CHANNEL}).encode(),
                                 headers={"Authorization": "Bearer " + env_value("FLASH_API_TOKEN"),
                                          "Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(req, timeout=30) as r:
            return r.status, json.loads(r.read())
    except urllib.error.HTTPError as e:
        return e.code, {}


def nvs_image(token, pw, path):
    from esp_idf_nvs_partition_gen import nvs_partition_gen as gen
    with tempfile.TemporaryDirectory() as d:
        csv = os.path.join(d, "nvs.csv")
        with open(csv, "w") as f:                 # values never contain a comma: hex token, [A-Za-z0-9] password
            f.write("key,type,encoding,value\nstorage,namespace,,\n")
            f.write(f"OTA_TOKEN,data,string,{token}\nINST_WIFI_PW,data,string,{pw}\n")
        args = argparse.Namespace(input=csv, output=os.path.basename(path), size=hex(NVS_SIZE), version=2,
                                  outdir=os.path.dirname(path))
        import contextlib, io
        with contextlib.redirect_stdout(io.StringIO()):   # the generator's own messages are not shown
            gen.generate(args, is_encr_enabled=False, encr_key=None)




def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--factory", default=FACTORY)
    # a bench test of a build not yet on the OTA channel (e.g. PG233 on U9K2652Z 18.11.2026); never for a box that
    # goes into a car before that build is on the channel
    ap.add_argument("--app", help="local app .bin instead of the channel's")
    a = ap.parse_args()
    port = a.port

    rc, out = nas(f"{ESPTOOL} --port {port} read-mac", 60)
    m = re.search(r"MAC:\s*([0-9a-fA-F:]{17})", out)
    step("MAC", rc == 0 and m is not None, m.group(1) if m else out.strip().splitlines()[-1:])
    mac = m.group(1).lower()
    ident = box_id(mac)
    print(f"      box {ident}", flush=True)

    bdir = f"~/v2o_a3/box_{mac.replace(':', '')}"
    rc, _ = nas(f"test -s {bdir}/orig_16M.bin")
    if rc == 0:
        step("zaloha", True, "uz existuje, neprepisujem")
    else:
        rc, out = nas(f"mkdir -p {bdir} && {ESPTOOL} --port {port} --baud 460800 read-flash 0 0x1000000 "
                      f"{bdir}/orig_16M.bin >/dev/null 2>&1 && sha256sum {bdir}/orig_16M.bin", 1800)
        step("zaloha", rc == 0, out.split()[0][:16] if rc == 0 else "read-flash zlyhal")

    ch = json.load(open(CHANNELS))[CHANNEL]
    if a.app:
        app = open(a.app, "rb").read()
        m = re.search(rb"[A-Z][a-z]{2} [ 0-9]\d \d{4} \d\d:\d\d:\d\d", app)
        ch = {"target_build": m.group(0).decode() if m else "?"}
        step("firmver (lokalny)", m is not None, f"{a.app} {ch['target_build']}")
    else:
        app = open(ch["firmware"], "rb").read()
        step("firmver kanala", ch["target_build"].encode() in app, f"{CHANNEL} {ch['target_build']}")
    fac = open(a.factory, "rb").read()
    step("bootloader", len(fac) > APP_OFF and fac[0x1000] == 0xE9, a.factory)   # ESP32 bootloader at 0x1000

    status, reg = register(ident)
    token = reg.get("ota_token") or ""
    step("FleetNest", status in (200, 201) and re.fullmatch(r"[0-9a-fA-F]{16,64}", token) is not None,
         f"http {status}, {'nove zariadenie' if reg.get('created') else 'existuje'}")
    code = reg.get("activation_code")

    with tempfile.TemporaryDirectory(dir="/home/claude/Claude/pg266") as d:
        os.chmod(d, 0o700)
        nvs = os.path.join(d, "nvs.bin")
        nvs_image(token, install_password(), nvs)
        nv = open(nvs, "rb").read()
        step("NVS obraz", len(nv) == NVS_SIZE)
        img = bytearray(fac[:APP_OFF])
        img[NVS_OFF:NVS_OFF + NVS_SIZE] = nv
        img += app
        path = os.path.join(d, "box.bin")
        open(path, "wb").write(img)
        rdir = f"/tmp/pg266_{secrets.token_hex(4)}"
        rc, _ = nas(f"mkdir -m 700 {rdir}")
        rc2 = subprocess.run(["scp", "-q", path, f"{NAS}:{rdir}/box.bin"]).returncode
        rc, out = nas(f"{ESPTOOL} --port {port} --baud 460800 write-flash 0x0 {rdir}/box.bin 2>&1 | "
                      f"grep -E 'Hash of data verified|rror'; rm -rf {rdir}", 600)
        step("zapis", rc2 == 0 and "Hash of data verified" in out, hashlib.sha256(img).hexdigest()[:16])

    serial = (f"timeout 40 {NAS_PY} -c \"\nimport serial,time\ns=serial.Serial()\ns.port='{port}';"
              "s.baudrate=115200;s.timeout=1;s.dtr=False;s.rts=True\ns.open();time.sleep(0.2);s.rts=False\n"
              "t=time.time()\nwhile time.time()-t<30:\n  l=s.readline()\n  if l: print(l.decode(errors='replace').rstrip())\n\"")
    rc, out = nas(serial, 60)
    step("boot build", ch["target_build"] in out, ch["target_build"])
    step("boot ID", f"device ID {ident}" in out, ident)
    sd = re.search(r"\[SD\] (\d+) MB", out)
    step("SD karta", sd is not None, f"{sd.group(1)} MB" if sd else "nenajdena")

    print(f"HOTOVO {ident} - stitok (ID, kod, QR): FleetNest {FLEETNEST}/zariadenia/sklad/"
          + ("" if code else " (box uz bol vo FleetNest, kod tam vyrobi Admin)"))


if __name__ == "__main__":
    main()
