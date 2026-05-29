import sys
from collections import Counter
PREFIX = "usb_avb_subsystem"
types = Counter()
seen = set()
def note(cell):
    if cell is None or cell.name in seen:
        return
    seen.add(cell.name)
    types[getattr(cell, "type", "?")] += 1
for nname, net in ctx.nets:
    if PREFIX not in nname:
        continue
    d = getattr(net, "driver", None)
    if d is not None:
        note(getattr(d, "cell", None))
    for u in getattr(net, "users", []):
        note(getattr(u, "cell", None))
print("[typeprobe] %d cells in usb-net set, by type:" % len(seen))
for t, c in types.most_common():
    print("   %6d  %s" % (c, t))
sys.exit(0)
