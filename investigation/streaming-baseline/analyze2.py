#!/usr/bin/env python3
"""Extract torque + progression details from console.log and results.json."""
import json, os, re

base = os.path.dirname(os.path.abspath(__file__))

# 1) torque comparison: samples near pose A vs frozen state (base ~0.6995)
pat = re.compile(r"'b': ([0-9.]+).*?'tB': (-?[0-9.]+), 'tS': (-?[0-9.]+), 'tE': (-?[0-9.]+), 'tT': (-?[0-9.]+), 'tR': (-?[0-9.]+), 'tG': (-?[0-9.]+)")
atA, frozen = [], []
for line in open(os.path.join(base, "console.log"), errors="ignore"):
    if "1051" not in line:
        continue
    m = pat.search(line)
    if not m:
        continue
    b = float(m.group(1))
    torq = tuple(float(m.group(i)) for i in range(2, 8))
    if abs(b - 0.7547) < 0.005:
        atA.append(torq)
    elif abs(b - 0.6995) < 0.002:
        frozen.append(torq)

def avg(lst):
    n = len(lst)
    return [round(sum(x[i] for x in lst) / n, 1) for i in range(6)] if n else None

print(f"torque samples at A (base~0.7547): n={len(atA)} avg tB,tS,tE,tT,tR,tG = {avg(atA)}")
print(f"torque samples frozen (base~0.6995): n={len(frozen)} avg tB,tS,tE,tT,tR,tG = {avg(frozen)}")

# 2) D-leg progression detail for cycle 1 A->B
d = json.load(open(os.path.join(base, "results.json")))
leg = d["tests"]["D"]["legs"][0]
print(f"\nD cycle1 {leg['leg']} pre={leg['pre_rad']}")
print("  step  alpha   err_to_cmd  progress")
for s in leg["steps"][::3]:
    k, alpha, err, prog = s
    print(f"  {k:3d}  {alpha:5.3f}   {err if err is not None else 'FAIL':>8}   {prog}")

# 3) hand joint check across all tests
def hand_range(test, key):
    hs = []
    for t in test.get(key, []):
        for s in t.get("samples", []):
            if s[1]:
                hs.append(s[1][5])
    return (min(hs), max(hs)) if hs else None
print("\nhand joint range during Test A samples:", hand_range(d["tests"]["A"], "transitions"))
print("hand joint range during Test D legs:",
      [(s[1][5]) for leg in d["tests"]["D"]["legs"] for s in leg["steps"]][:0] or "see samples")
