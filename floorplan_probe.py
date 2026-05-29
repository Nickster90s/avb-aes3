# nextpnr-xilinx --pre-place probe: dump the REAL cell/net name picture
# at floorplan time, so the net-based region matcher is built on facts
# not assumptions. Aborts after dumping (no P&R) via a hard exit.
#
# Run:
#   cd build/colorlight_i9plus/gateware
#   CHIPDB=/home/lisp/FPGA/demo-projects/chipdb \
#   PATH=/home/lisp/openxc7/bin:$PATH nextpnr-xilinx \
#     --json colorlight_i9plus.json --xdc colorlight_i9plus.xdc \
#     --chipdb $CHIPDB/xc7a50tfgg484.bin --pre-place ../../../floorplan_probe.py

import sys

PREFIX = "usb_avb_subsystem"

# ---- cells ----
total_cells = 0
cell_hit = 0
cell_names = []
for cname, cell in ctx.cells:
    total_cells += 1
    if PREFIX in cname:
        cell_hit += 1
        if len(cell_names) < 8:
            cell_names.append(cname)

# ---- nets ----
total_nets = 0
net_hit = 0
net_names = []
# Net→cell traversal probe: does a net expose driver + users?
net_api = []
first_hit_net = None
for nname, net in ctx.nets:
    total_nets += 1
    if PREFIX in nname:
        net_hit += 1
        if first_hit_net is None:
            first_hit_net = (nname, net)
        if len(net_names) < 8:
            net_names.append(nname)

print("=" * 60)
print("[probe] total cells = %d" % total_cells)
print("[probe] cells with '%s' in name = %d" % (PREFIX, cell_hit))
print("[probe] total nets  = %d" % total_nets)
print("[probe] nets with '%s' in name = %d" % (PREFIX, net_hit))
print("-" * 60)
print("[probe] sample cell names:")
for n in cell_names:
    print("   CELL  " + n)
print("[probe] sample net names:")
for n in net_names:
    print("   NET   " + n)
print("-" * 60)

# Probe the net object API for driver/users
if first_hit_net is not None:
    nname, net = first_hit_net
    print("[probe] net object attrs: " + ", ".join(
        a for a in dir(net) if not a.startswith("__")))
    # Common nextpnr-python: net.driver (PortRef), net.users (list of PortRef)
    for attr in ("driver", "users"):
        if hasattr(net, attr):
            v = getattr(net, attr)
            print("[probe]   net.%s = %r" % (attr, v))
            # PortRef.cell typical
            try:
                if attr == "driver" and hasattr(v, "cell") and v.cell is not None:
                    print("[probe]     driver.cell.name = %r" % v.cell.name)
                if attr == "users":
                    n_users = len(v)
                    print("[probe]     #users = %d" % n_users)
                    if n_users:
                        u0 = v[0]
                        print("[probe]     users[0] attrs: " + ", ".join(
                            a for a in dir(u0) if not a.startswith("__")))
                        if hasattr(u0, "cell") and u0.cell is not None:
                            print("[probe]     users[0].cell.name = %r" % u0.cell.name)
            except Exception as e:
                print("[probe]   (traversal err: %r)" % e)
print("=" * 60)
sys.exit(0)   # abort before P&R — this is a probe only
