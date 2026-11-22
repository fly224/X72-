# -*- coding: utf-8 -*-
"""按网络名 / 型号 / 位号反查网表，用于还原采样链路。

用法：
    python netlist_probe.py des R28 R29
    python netlist_probe.py net UI_P UI_N
    python netlist_probe.py part OPA
"""
import io
import json
import sys

PATH = "G:/FOC/X72/Hardware/Netlist_PG_X72_CAN_2026-11-07.enet"


def load():
    with io.open(PATH, encoding="utf-8") as f:
        return json.load(f)


D = load()


def comps():
    for uid, c in D["components"].items():
        p = c.get("props", {})
        yield p.get("Designator", ""), p, (c.get("pinInfoMap") or {})


def by_des(names):
    want = set(names)
    for des, p, pim in comps():
        if des not in want:
            continue
        print("=== %s  %s  %s" % (des, p.get("Manufacturer Part", ""),
                                  (p.get("LCSC Part Name", "") or "")[:36]))
        for pin in sorted(pim, key=lambda x: str(x)):
            info = pim[pin]
            if isinstance(info, dict):
                print("    pin %-4s -> %s" % (pin, info.get("net", "")))


def by_net(names):
    want = set(names)
    cur = None
    rows = []
    for des, p, pim in comps():
        for pin, info in pim.items():
            if isinstance(info, dict) and info.get("net") in want:
                rows.append((info["net"], des, pin,
                             p.get("Manufacturer Part", ""),
                             (p.get("LCSC Part Name", "") or "")))
    for net, des, pin, mp, nm in sorted(rows):
        if net != cur:
            print("--- net %s" % net)
            cur = net
        print("    %-6s pin %-3s  %-22s %s" % (des, pin, mp, nm[:32]))


def by_part(sub):
    seen = set()
    for des, p, pim in comps():
        mp = p.get("Manufacturer Part", "") or ""
        nm = p.get("LCSC Part Name", "") or ""
        if des in seen:
            continue
        if sub.lower() in (mp + " " + nm).lower():
            seen.add(des)
            print("%-6s %-24s %s" % (des, mp, nm[:40]))


if len(sys.argv) < 3:
    print(__doc__)
elif sys.argv[1] == "des":
    by_des(sys.argv[2:])
elif sys.argv[1] == "net":
    by_net(sys.argv[2:])
else:
    by_part(sys.argv[2])
