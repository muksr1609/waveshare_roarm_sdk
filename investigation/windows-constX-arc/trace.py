import re
lines = open(r'investigation\windows-constX-arc\constX-console.log', encoding='utf-8').read().splitlines()
marks = [(i, l) for i, l in enumerate(lines)
         if any(k in l for k in ['executing forward leg', 'forward  aborted', 'reverse  aborted',
                                 'RESULT', 'homing (move_init', 'home reached', 'executing reverse',
                                 'validation passed'])]
for i, l in marks:
    print('line %4d: %s' % (i, l.strip()[:90]))
print('total lines', len(lines))
frames = []
for i, l in enumerate(lines):
    if not l.strip().startswith("{'T'"):
        continue
    mz = re.search(r"'z': (-?[\d.]+)", l)
    me = re.search(r"'e': (-?[\d.]+)", l)
    mt = re.search(r"'t': (-?[\d.]+)", l)
    ms = re.search(r"'s': (-?[\d.]+)", l)
    if mz and me:
        frames.append((i, float(mz.group(1)), float(me.group(1)),
                       float(mt.group(1)) if mt else 0.0,
                       float(ms.group(1)) if ms else 0.0))
print('raw frames:', len(frames))
print('first 3 (line, z, e, t, s):')
for f in frames[:3]:
    print('  line %4d  z=%6.1f e=%.3f t=%.3f s=%.3f' % f)
print('last 6 (line, z, e, t, s):')
for f in frames[-6:]:
    print('  line %4d  z=%6.1f e=%.3f t=%.3f s=%.3f' % f)
# where does z first drop below 195 (arm leaving home z=199.9)?
for f in frames:
    if f[1] < 195:
        print('first z<195 at line %d (z=%.1f)' % (f[0], f[1]))
        break
