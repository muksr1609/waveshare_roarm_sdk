#!/usr/bin/env python3
"""Compact analysis of results.json for the T=102 transport experiment."""
import json, math, os

d = json.load(open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "results.json")))

meta = d["meta"]
print("=== META ===")
print("SDK:", meta["roarm_sdk_version"], meta["roarm_sdk_path"])
print("A rad:", meta["pose_A_rad"], "deg:", meta["pose_A_deg"])
print("B rad:", meta["pose_B_rad"], "deg:", meta["pose_B_deg"])
print("baseline fb success:", d["baseline"]["success_rate"])

JN = ["base", "shldr", "elbow", "wrist", "roll", "hand"]

def show_sparse(name):
    t = d["tests"][name]
    print(f"\n=== TEST {name} (speed={t['speed']}, acc={t['acc']}) : "
          f"{t.get('accepted','?')}/{t.get('sent','?')} accepted ===")
    for r in t["transitions"]:
        pre, tgt = r["pre_rad"], r["target_rad"]
        base_pre, base_tgt = pre[0], tgt[0]
        print(f"  {r['to']}: pre_base={base_pre:.4f} tgt_base={base_tgt:.4f} "
              f"accepted={r.get('accepted')} no_op={r.get('already_at_target', False)} "
              f"lat={r.get('first_motion_latency_s')}s "
              f"min_err={r.get('min_err_to_target')} final_err={r.get('final_err_to_target')} "
              f"settled={r.get('settled')}")
        # where did base end up vs the 5 deg span
        last = r["samples"][-1][1] if r["samples"] and r["samples"][-1][1] else None
        if last:
            frac = (last[0] - base_pre) / (base_tgt - base_pre) if abs(base_tgt - base_pre) > 1e-9 else 1.0
            print(f"        last_base={last[0]:.4f}  base_progress={frac*100:.0f}%  "
                  f"last_j={ [round(v,4) for v in last] }")

show_sparse("A")
show_sparse("B")

t = d["tests"]["C"]
print(f"\n=== TEST C (repeated @10Hz, speed={t['speed']}, acc={t['acc']}) : "
      f"{t.get('success')}/{len(t['transitions'])} successful, packets_total={t.get('sent_total')} ===")
for r in t["transitions"]:
    if r.get("already_at_target"):
        print(f"  {r['to']}: no-op (already at target)")
        continue
    pre, tgt = r["pre_rad"], r["target_rad"]
    print(f"  {r['to']}: pre_base={pre[0]:.4f} tgt_base={tgt[0]:.4f} "
          f"packets_before_motion={r.get('packets_before_motion')} "
          f"lat={r.get('motion_latency_s')}s final_err={r.get('final_err_to_target')} "
          f"settled={r.get('settled')}")

t = d["tests"]["D"]
print(f"\n=== TEST D (stream @20Hz 1.5s legs, speed={t['speed']}, acc={t['acc']}) : "
      f"{t.get('legs_started')}/{t.get('legs_total')} legs started, fb_rate={t.get('feedback_success_rate')} ===")
for leg in t["legs"]:
    steps = leg["steps"]
    errs = [s[2] for s in steps if s[2] is not None]
    progs = [s[3] for s in steps if s[3] is not None]
    print(f"  cyc{leg['cycle']} {leg['leg']}: started={leg['started']} lat={leg.get('start_latency_s')}s "
          f"fb_ok={leg['feedback_ok']}/{leg['feedback_ok']+leg['feedback_fail']} "
          f"regressions={leg['regressions']} max_step_err={max(errs) if errs else None} "
          f"final_err={leg.get('final_err')} prog_first={progs[0] if progs else None} "
          f"prog_last={progs[-1] if progs else None}")
