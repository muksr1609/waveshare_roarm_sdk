#!/usr/bin/env python3
"""Safe probe: connect with stock SDK, read feedback 5 times, disconnect. No motion commands."""
import math, sys, time

import roarm_sdk
print("SDK path:", roarm_sdk.__file__, flush=True)
assert "/mnt/c/" not in roarm_sdk.__file__, "repo-local SDK imported - abort"

from roarm_sdk.roarm import roarm

arm = roarm(roarm_type="roarm_m3", port="/dev/ttyUSB0", baudrate=115200)
ok = 0
for i in range(5):
    t0 = time.time()
    v = arm.feedback_get()
    el = time.time() - t0
    good = isinstance(v, list) and len(v) == 10 and all(isinstance(x, (int, float)) and math.isfinite(x) for x in v)
    ok += 1 if good else 0
    print(f"sample {i}: ok={good} elapsed={el*1000:.0f}ms value={v}", flush=True)
    time.sleep(0.2)
arm.disconnect()
print(f"PROBE RESULT: {ok}/5 feedback reads OK", flush=True)
sys.exit(0 if ok >= 4 else 1)
