import json
import math
import time

import serial

s = serial.Serial("COM21", 115200, timeout=1.0)
time.sleep(0.3)


def grab():
    s.reset_input_buffer()
    time.sleep(0.05)
    raw = b""
    end = time.time() + 0.4
    while time.time() < end:
        raw += s.read(4096)
    for line in raw.split(b"\n"):
        line = line.strip()
        if line.startswith(b"{") and b'"x"' in line:
            try:
                return json.loads(line.decode())
            except Exception:
                pass
    return None


for _ in range(4):
    d = grab()
    if d:
        break
    time.sleep(0.2)

s.close()

if d:
    print("RAW FIRMWARE FRAME (one ~100 ms feedback packet):")
    print(json.dumps(d, indent=2))
    print()
    print("Joint angles it reports (rad -> deg):")
    for k in ("tit", "b", "s", "e", "t", "r", "g"):
        if k in d:
            print("  %-4s = %8.5f rad = %7.2f deg" % (k, d[k], math.degrees(d[k])))
    print()
    print("Computed tool position it reports:")
    print("  x=%.2f  y=%.2f  z=%.2f  (mm)" % (d["x"], d["y"], d["z"]))
