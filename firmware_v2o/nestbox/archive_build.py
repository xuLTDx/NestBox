# PlatformIO post-script (PG291, 22.11.2026, Vojtech "291 ano"): every build is archived whole, nothing is overwritten.
# Why: 22.11. the test "J0 = exactly the Peugeot flash" needed the bootloader + partition table of build 12.11. 11:51:49;
# the OTA server keeps only the app (.bin), .pio/build is overwritten by the next build, and without the ELF of the
# exact build a core dump from a car cannot be decoded. Removing this = the same gap again.
# Where: /home/claude/Claude/fw_archive/<branch>_<env>/<YYYYMMDD_HHMMSS of FW_BUILD_STR>/ + a copy on NAS
# (ssh nas, /home/claudef/fw_archive/...). All builds are kept (~19 MB each, 17 MB of it ELF) - which build is on
# which box is only known later. The NAS copy failing never fails the build (a warning, archive stays local).
import hashlib
import os
import re
import shutil
import subprocess
from datetime import datetime

Import("env")  # noqa: F821

ARCH = "/home/claude/Claude/fw_archive"
NAS_ARCH = "/home/claudef/fw_archive"
MONTHS = "Jan Feb Mar Apr May Jun Jul Aug Sep Oct Nov Dec".split()


def build_str(app):
    # FW_BUILD_STR = __DATE__ " " __TIME__, e.g. "Nov 22 2026 15:41:39" / "Nov  8 2026 17:30:12"
    with open(app, "rb") as f:
        data = f.read()
    m = re.search(rb"(%s) ([ \d]\d) (\d{4}) (\d\d:\d\d:\d\d)" % "|".join(MONTHS).encode(), data)
    if not m:
        return None, None
    s = m.group(0).decode()
    t = datetime.strptime("%s %s %s %s" % (m.group(1).decode(), m.group(2).decode().strip(), m.group(3).decode(),
                                          m.group(4).decode()), "%b %d %Y %H:%M:%S")
    return s, t.strftime("%Y%m%d_%H%M%S")


def git(*args):
    try:
        return subprocess.run(["git"] + list(args), cwd=env.subst("$PROJECT_DIR"), capture_output=True,  # noqa: F821
                              text=True, timeout=10).stdout.strip()
    except Exception:
        return ""


def archive(source, target, env):
    bdir = env.subst("$BUILD_DIR")
    app = os.path.join(bdir, env.subst("${PROGNAME}.bin"))
    bstr, stamp = build_str(app)
    if not stamp:
        print("ARCHIVE ERR build string not found in %s - NOT archived" % app)
        return
    branch = git("rev-parse", "--abbrev-ref", "HEAD").replace("/", "-") or "nogit"
    dest = os.path.join(ARCH, "%s_%s" % (branch, env.subst("$PIOENV")), stamp)
    os.makedirs(dest, exist_ok=True)
    files = {
        "firmware.bin": app,
        "firmware.elf": os.path.join(bdir, env.subst("${PROGNAME}.elf")),
        "bootloader.bin": os.path.join(bdir, "bootloader.bin"),
        "partitions.bin": os.path.join(bdir, "partitions.bin"),
        "boot_app0.bin": os.path.join(env.PioPlatform().get_package_dir("framework-arduinoespressif32"),
                                      "tools", "partitions", "boot_app0.bin"),
    }
    sums = []
    for name, src in files.items():
        if not os.path.exists(src):
            print("ARCHIVE WARN missing %s" % src)
            continue
        shutil.copy2(src, os.path.join(dest, name))
        with open(src, "rb") as f:
            sums.append("%s  %s" % (hashlib.sha256(f.read()).hexdigest(), name))
    with open(os.path.join(dest, "SHA256SUMS"), "w") as f:
        f.write("\n".join(sums) + "\n")
    dirty = "dirty" if git("status", "--porcelain", "--untracked-files=no") else "clean"
    with open(os.path.join(dest, "INFO.txt"), "w") as f:
        f.write("build: %s\nenv: %s\nbranch: %s\ncommit: %s (%s at build time)\n"
                "flash: bootloader 0x1000, partitions 0x8000, boot_app0 0xe000, firmware 0x10000\n"
                % (bstr, env.subst("$PIOENV"), branch, git("rev-parse", "--short", "HEAD"), dirty))
    print("ARCHIVE %s -> %s" % (bstr, dest))
    rel = os.path.relpath(dest, ARCH)
    try:
        r = subprocess.run(["rsync", "-a", "--mkpath", "-e", "ssh -o BatchMode=yes -o ConnectTimeout=5",
                            dest + "/", "nas:%s/%s/" % (NAS_ARCH, rel)], capture_output=True, text=True, timeout=120)
        print("ARCHIVE NAS %s" % ("ok" if r.returncode == 0 else "WARN rc=%d %s" % (r.returncode, r.stderr.strip()[:200])))
    except Exception as e:
        print("ARCHIVE NAS WARN %s" % e)


env.AddPostAction("$BUILD_DIR/${PROGNAME}.bin", archive)  # noqa: F821
