import json
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


for i in range(6):
    d = grab()
    if d:
        print("sample %d: x=%.2f y=%.2f z=%.2f" % (i, d["x"], d["y"], d["z"]))
    time.sleep(0.4)

s.close()
