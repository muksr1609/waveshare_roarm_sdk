#!/usr/bin/env python3
"""Final checks: feedback failure counts, second freeze state torque, mid-motion torque."""
import json, os, re

base = os.path.dirname(os.path.abspath(__file__))

d = json.load(open(os.path.join(base, "results.json")))
for name in ("A", "B", "C"):
    t = d["tests"][name]
    ok = sum(r.get("n_feedback_ok", 0) for r in t.get("transitions", []))
    fail = sum(r.get("n_feedback_fail", 0) for r in t.get("transitions", []))
    print(f"Test {name}: feedback ok={ok} fail={fail}")
t = d["tests"]["D"]
ok = sum(l["feedback_ok"] for l in t["legs"])
fail = sum(l["feedback_fail"] for l in t["legs"])
print(f"Test D: feedback ok={ok} fail={fail} (300 commands sent)")

pat = re.compile(r"'b': ([0-9.]+).*?'tB': (-?[0-9.]+), 'tS': (-?[0-9.]+), 'tE': (-?[0-9.]+), 'tT': (-?[0-9.]+), 'tR': (-?[0-9.]+), 'tG': (-?[0-9.]+).*?'v': (\d+)")
frozen2, mid, atA = [], [], []
for line in open(os.path.join(base, "console.log"), errors="ignore"):
    if "1051" not in line:
        continue
    m = pat.search(line)
    if not m:
        continue
    b = float(m.group(1))
    vals = [float(m.group(i)) for i in range(2, 8)]
    v = m.group(8)
    if abs(b - 0.7256) < 0.002:
        frozen2.append(vals)
    elif 0.705 < b < 0.745:  # mid-motion band (between the two freeze positions)
        mid.append(vals)
    elif abs(b - 0.7547) < 0.005:
        atA.append(vals)

def avg(lst):
    n = len(lst)
    return [round(sum(x[i] for x in lst) / n, 1) for i in range(6)] if n else None

print(f"torque at A (base~0.7547): n={len(atA)} {avg(atA)}")
print(f"torque frozen2 (base~0.7256): n={len(frozen2)} {avg(frozen2)}")
print(f"torque mid-motion (0.705<b<0.745): n={len(mid)} {avg(mid)}")
vpat = re.compile(r"'v': (\d+)")
vs = set()
for line in open(os.path.join(base, "console.log"), errors="ignore"):
    if "1051" in line:
        m2 = vpat.search(line)
        if m2:
            vs.add(m2.group(1))
print("battery v samples:", sorted(vs))
