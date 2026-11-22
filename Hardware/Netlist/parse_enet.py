# -*- coding: utf-8 -*-
"""
parse_enet.py —— 解析嘉立创 EDA Pro 导出的 .enet 网表 (JSON 2.0.0)

用途: 从原理图网表中提取元件清单、引脚-网络映射, 作为 STM32CubeMX 外设/引脚配置的依据。

用法:
    python parse_enet.py                      # 探查模式: 打印结构 + MCU 引脚网络映射
    python parse_enet.py --dump-csv          # 导出 pin_map.csv / bom.csv (UTF-8 BOM)
    python parse_enet.py --net <关键字>      # 过滤含关键字的网络, 打印其所有连接点
    python parse_enet.py --enet <路径>       # 指定网表路径

参数:
    DEFAULT_ENET : 默认网表路径
    MCU_KEYS     : 用于识别主控元件的关键字 (Designator / Name / Device)
"""
import argparse
import csv
import json
import os
import re
import sys

DEFAULT_ENET = os.path.join(os.path.dirname(__file__), "..", "Netlist_PG_X72_CAN_2026-11-07.enet")
MCU_KEYS = ("STM32", "G474", "MCU", "U1")


def load(path):
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def comp_label(cid, comp):
    """元件的可读名称: 优先用 Manufacturer Part / Value / Name 的实际值"""
    props = comp.get("props", {}) or {}
    des = props.get("Designator", "")
    part = ""
    for key in ("Manufacturer Part", "Value", "Name", "Device", "Comment"):
        val = str(props.get(key, ""))
        if val and not val.startswith("="):
            part = val
            break
    return f"{des} [{part}]" if part else f"{des} [{cid}]"


def find_mcu(components):
    """主控 = 引脚数最多的元件, 并辅以型号关键字"""
    ranked = sorted(components.items(), key=lambda kv: -len(iter_pins(kv[1])))
    hits = []
    for cid, comp in ranked[:5]:
        label = comp_label(cid, comp)
        hits.append((cid, comp))
    # 关键字命中的也一并保留
    for cid, comp in components.items():
        label = comp_label(cid, comp)
        if any(k.lower() in label.lower() for k in MCU_KEYS) and (cid, comp) not in hits:
            hits.append((cid, comp))
    return hits


def iter_pins(comp):
    """兼容 pinInfoMap / pins, dict 或 list"""
    pins = comp.get("pinInfoMap", comp.get("pins", {})) or {}
    if isinstance(pins, dict):
        return list(pins.items())
    return [(str(i), p) for i, p in enumerate(pins)]


def pin_info(pid, pin):
    """返回 (引脚号, 引脚名, 网络名); 网表把网络名放在引脚 dict 的 net 字段"""
    if not isinstance(pin, dict):
        return (str(pid), "", "")
    num = pin.get("number", pin.get("pinNumber", pin.get("num", pid)))
    name = pin.get("name", pin.get("pinName", ""))
    net = pin.get("net", pin.get("netName", pin.get("netLabel", "")))
    if isinstance(net, dict):
        net = net.get("name", "") or net.get("net", "") or net.get("label", "")
    return (str(num), str(name), str(net))


def main():
    ap = argparse.ArgumentParser(description="解析 .enet 网表")
    ap.add_argument("--enet", default=DEFAULT_ENET)
    ap.add_argument("--dump-csv", action="store_true")
    ap.add_argument("--report", action="store_true", help="导出全部网络-连接点报告 net_report.md")
    ap.add_argument("--net", default="")
    args = ap.parse_args()

    path = os.path.abspath(args.enet)
    data = load(path)
    comps = data.get("components", {})
    print(f"网表: {path}")
    print(f"版本: {data.get('version')}  元件数: {len(comps)}")

    # 结构探查: 打印第一个元件的键与 MCU 候选
    first = next(iter(comps.values()), {})
    print(f"元件字段: {list(first.keys())}")
    print(f"元件 props 样例: {json.dumps(first.get('props', {}), ensure_ascii=False)[:500]}")
    sample = iter_pins(first)[:2]
    print(f"引脚样例: {json.dumps(sample, ensure_ascii=False)[:800]}")

    ranked = sorted(comps.items(), key=lambda kv: -len(iter_pins(kv[1])))
    print(f"\n引脚数最多的 8 个元件:")
    for cid, comp in ranked[:8]:
        print(f"  {len(iter_pins(comp)):>3}  {comp_label(cid, comp)}  props={json.dumps(comp.get('props', {}), ensure_ascii=False)[:220]}")

    mcu = find_mcu(comps)
    print(f"\nMCU 候选 ({len(mcu)}):")
    for cid, comp in mcu:
        print(f"  {cid}: {comp_label(cid, comp)}  引脚数={len(iter_pins(comp))}")

    # 全量引脚映射
    rows = []
    for cid, comp in comps.items():
        label = comp_label(cid, comp)
        for pid, pin in iter_pins(comp):
            num, name, net = pin_info(pid, pin)
            rows.append((label, num, name, net))

    connected = [r for r in rows if r[3]]
    print(f"\n总引脚记录: {len(rows)}  带网络名: {len(connected)}")

    if args.net:
        print(f"\n=== 网络含 '{args.net}' 的连接点 ===")
        for label, num, name, net in rows:
            if args.net.lower() in net.lower():
                print(f"  {net:<32} {label:<40} pin {num:<5} {name}")

    out_dir = os.path.dirname(os.path.abspath(__file__))
    if args.dump_csv:
        pin_csv = os.path.join(out_dir, "pin_map.csv")
        with open(pin_csv, "w", encoding="utf-8-sig", newline="") as f:
            w = csv.writer(f)
            w.writerow(["元件", "引脚号", "引脚名", "网络名"])
            w.writerows(rows)
        print(f"\n已导出: {pin_csv}")

    if args.report:
        from collections import defaultdict
        nets = defaultdict(list)
        for label, num, name, net in rows:
            if net:
                nets[net].append((label, num, name))
        md_path = os.path.join(out_dir, "net_report.md")
        with open(md_path, "w", encoding="utf-8-sig") as f:
            f.write("# PG_X72_CAN 原理图网络 - 连接点报告\n\n")
            f.write(f"来源: `{os.path.basename(path)}`  网络数: {len(nets)}\n\n")
            f.write("| 网络名 | 连接点 |\n|---|---|\n")
            for net in sorted(nets):
                pts = "; ".join(f"{lbl} pin {n} ({pn})" for lbl, n, pn in nets[net])
                f.write(f"| `{net}` | {pts} |\n")
        print(f"已导出: {md_path}  网络数={len(nets)}")

    # 打印 MCU 全部引脚网络
    for cid, comp in mcu:
        print(f"\n=== {comp_label(cid, comp)} 引脚网络 ===")
        for pid, pin in iter_pins(comp):
            num, name, net = pin_info(pid, pin)
            print(f"  {num:<5} {name:<24} {net}")


if __name__ == "__main__":
    main()
