# PlatformIO pre-script: main.cpp is compiled on every build, so FW_BUILD_STR (__DATE__ " " __TIME__)
# is new with every firmware.bin. SCons rebuilds by content (MD5), not by time - without this a
# change in another file kept the old build string, and the OTA server (which compares build
# strings) offered nothing (2026-10-18).
import os
Import("env")  # noqa: F821
obj = os.path.join(env.subst("$BUILD_DIR"), "src", "main.cpp.o")  # noqa: F821
if os.path.exists(obj):
    os.remove(obj)
