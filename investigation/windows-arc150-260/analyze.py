#!/usr/bin/env python3
"""Offline feasibility analysis for the 150mm/260deg arc from the measured home
pose (no robot, no motion). Pinpoints which safety gate the 260deg arc violates
and finds the largest feasible sweep from home."""
import importlib.util, math, os

here = os.path.dirname(os.path.abspath(__file__))
ik_path = os.path.join(os.path.dirname(os.path.dirname(here)), "roarm_sdk", "waveshare_ik.py")
spec = importlib.util.spec_from_file_location("waveshare_ik", ik_path)
ik = importlib.util.module_from_spec(spec); spec.loader.exec_module(ik)
ik_fn = ik.compute_joint_rad_by_pos

# Measured home pose (from the dry run of arc150-260.py).
x0, y0, z0 = 347.92, 7.47, 203.15
pitch0, roll0, grip0 = 0.0598, -0.0031, 0.0261
start = [0.0215, 0.0138, 1.6214, -0.0046, -0.0031]

JOINT_LIMITS = [(-3.3,3.3),(-1.9,1.9),(-1.2,3.3),(-1.9,1.9),(-3.3,3.3)]
JOINT_NAMES = ["base","shoulder","elbow","wrist","roll"]
MARGIN = math.radians(2.5)
MAXSTEP = math.radians(5.0)
SANITY = math.radians(3.0)

def unwrap(joints):
    out=[list(j) for j in joints]
    for k in range(5):
        prev=out[0][k]
        for i in range(1,len(out)):
            v=out[i][k]
            while v-prev>math.pi: v-=2*math.pi
            while v-prev<-math.pi: v+=2*math.pi
            out[i][k]=v; prev=v
    return out

def build(R, S, cx, cy, sign):
    Srad=math.radians(S)
    n=max(101, int(round(R*Srad/5.0))+1)
    a0=math.atan2(y0-cy, x0-cx)
    wp=[]; ikj=[]
    for i in range(n):
        a=a0+sign*Srad*i/(n-1)
        x,y=cx+R*math.cos(a), cy+R*math.sin(a)
        try: j5=ik_fn(x,y,z0,roll0,pitch0)
        except ValueError: return None
        ikj.append(j5); wp.append((x,y))
    return {"n":n,"wp":wp,"uj":unwrap(ikj)}

def diagnose(b):
    """Return (first_violation_str, base_range_deg) for a built arc."""
    uj=b["uj"]; n=b["n"]
    # sanity
    for k in range(5):
        if abs(uj[0][k]-start[k])>SANITY:
            return ("SANITY: first wp joint %s=%.2f deg vs start %.2f deg (diff %.2f deg)"%(
                JOINT_NAMES[k], math.degrees(uj[0][k]), math.degrees(start[k]),
                math.degrees(abs(uj[0][k]-start[k]))), None)
    # step + limit
    for i in range(n):
        for k in range(5):
            v=uj[i][k]; lo,hi=JOINT_LIMITS[k]
            if not (lo+MARGIN<=v<=hi-MARGIN):
                return ("LIMIT at wp %d: %s=%.2f deg outside [%s..%s] deg"%(
                    i, JOINT_NAMES[k], math.degrees(v),
                    math.degrees(lo+MARGIN), math.degrees(hi-MARGIN)), None)
        if i>0:
            for k in range(5):
                d=abs(uj[i][k]-uj[i-1][k])
                if d>MAXSTEP:
                    return ("STEP at wp %d: %s delta %.2f deg > 5 deg"%(
                        i, JOINT_NAMES[k], math.degrees(d)), None)
    base=[uj[i][0] for i in range(n)]
    return None, (math.degrees(min(base)), math.degrees(max(base)))

print("home tip XY dist from base origin = %.1f mm" % math.hypot(x0,y0))
# Toward-base center is the only in-reach candidate.
for sign in (1,-1):
    cx,cy=197.9,7.5
    b=build(150.0,260.0,cx,cy,sign)
    if b is None:
        print("sign %+d center [197.9 7.5] 260deg: IK no solution" % sign); continue
    v,br=diagnose(b)
    print("sign %+d center [197.9 7.5] 260deg: n=%d  first violation: %s  base_range=%s"%(
        sign, b["n"], v, ("[%.1f..%.1f] deg"%br) if br else "n/a"))

# Max feasible sweep for the toward-base center, each sign.
print("\nMax feasible sweep from home (center [197.9 7.5], R=150):")
for sign in (1,-1):
    best=None; lo=10.0; hi=260.0
    # scan downward from 260 to find the largest feasible
    S=260.0
    while S>=10.0:
        b=build(150.0,S,197.9,7.5,sign)
        if b is not None:
            v,_=diagnose(b)
            if v is None:
                best=S; break
        S-=5.0
    if best is not None:
        b=build(150.0,best,197.9,7.5,sign)
        _,br=diagnose(b)
        path=sum(math.dist(b["wp"][i],b["wp"][i+1]) for i in range(b["n"]-1))
        print("  sign %+d: max feasible sweep = %.0f deg  (base range %s, path %.0f mm, n=%d)"%(
            sign, best, ("[%.1f..%.1f] deg"%br) if br else "n/a", path, b["n"]))
    else:
        print("  sign %+d: no feasible sweep >= 10 deg" % sign)
