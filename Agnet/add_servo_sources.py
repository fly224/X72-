# -*- coding: utf-8 -*-
"""
add_servo_sources.py —— 把 BSP / 注入 ISR / 三环控制律加入 MDK-ARM 工程（幂等）

用途:
    将 Board/bsp_servo.c、Board/servo_isr.c、simulink_code_gen/three_loop_deploy.c
    作为新文件组加入 X72_FOC.uvprojx，并追加 IncludePath。

用法:
    python Agnet/add_servo_sources.py            # 执行加入
    python Agnet/add_servo_sources.py --verify   # 只检查工程引用与文件存在性

注意: 只新增一个文件组，不改动原有 38 个源文件条目；重复运行不会重复添加。
"""
import argparse
import os
import re
import sys

sys.stdout.reconfigure(encoding="utf-8")
HERE = os.path.dirname(os.path.abspath(__file__))
MDK = os.path.normpath(os.path.join(
    HERE, "..", "Firmware", "FM_Servo_V3", "embedded_project",
    "STM32G474_App", "MDK-ARM"))
UVPROJX = os.path.join(MDK, "X72_FOC.uvprojx")

SOURCES = [
    ("bsp_servo.c", "../Board/bsp_servo.c"),
    ("servo_isr.c", "../Board/servo_isr.c"),
    ("telemetry_tx.c", "../Board/telemetry_tx.c"),
    ("three_loop_deploy.c", "../../../simulink_code_gen/three_loop_deploy.c"),
]
INCLUDE_EXTRA = ["../Board", "../../../simulink_code_gen"]
GROUP_NAME = "Application/User/Servo"


def build_group():
    lines = ["        <Group>", f"          <GroupName>{GROUP_NAME}</GroupName>",
             "          <Files>"]
    for name, path in SOURCES:
        lines += ["            <File>",
                  f"              <FileName>{name}</FileName>",
                  "              <FileType>1</FileType>",
                  f"              <FilePath>{path}</FilePath>",
                  "            </File>"]
    lines += ["          </Files>", "        </Group>", ""]
    return "\n".join(lines)


def verify(text):
    ok = True
    paths = re.findall(r"<FilePath>(.*?)</FilePath>", text, flags=re.S)
    missing = []
    for raw in paths:
        p = os.path.normpath(os.path.join(MDK, raw.strip().replace("\\", "/")))
        if not os.path.isfile(p):
            missing.append(raw.strip())
    print(f"工程源文件引用: {len(paths)} 个, 文件缺失: {len(missing)} 个")
    for m in missing[:20]:
        print("  - " + m)

    for inc in INCLUDE_EXTRA:
        p = os.path.normpath(os.path.join(MDK, inc))
        exist = os.path.isdir(p)
        print(f"包含路径 {inc}: {'存在' if exist else '缺失'}")
        ok &= exist

    m = re.search(r"<IncludePath>([^<]*\.\./Core/Inc[^<]*)</IncludePath>", text)
    inc_text = m.group(1) if m else ""
    for inc in INCLUDE_EXTRA:
        if inc not in inc_text:
            print(f"IncludePath 缺少: {inc}")
            ok = False

    if GROUP_NAME not in text:
        print("工程中缺少 Servo 文件组")
        ok = False
    ok &= not missing
    return ok


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--verify", action="store_true")
    args = ap.parse_args()

    with open(UVPROJX, "r", encoding="utf-8") as f:
        text = f.read()

    if args.verify:
        print(f"[核验] {UVPROJX}")
        ok = verify(text)
        print("[PASS] 工程引用完整" if ok else "[FAIL] 存在缺失项")
        return 0 if ok else 1

    changed = False
    if GROUP_NAME not in text:
        marker = "      </Groups>"
        if marker not in text:
            print("错误: 未找到 </Groups> 插入点")
            return 2
        text = text.replace(marker, build_group() + marker, 1)
        changed = True
        print(f"[加入] 新文件组 {GROUP_NAME}: {len(SOURCES)} 个 .c")
    else:
        # 文件组已存在（上次运行建的）：只补齐后续新增的源文件，幂等
        for name, path in SOURCES:
            if f"<FileName>{name}</FileName>" in text:
                continue
            gi = text.index(f"<GroupName>{GROUP_NAME}</GroupName>")
            fi = text.index("</Files>", gi)
            block = ("            <File>\n"
                     f"              <FileName>{name}</FileName>\n"
                     "              <FileType>1</FileType>\n"
                     f"              <FilePath>{path}</FilePath>\n"
                     "            </File>\n")
            text = text[:fi] + block + text[fi:]
            changed = True
            print(f"[加入] 文件组内补入 {name}")

    def add_include(m):
        global changed
        cur = m.group(1)
        add = [x for x in INCLUDE_EXTRA if x not in cur]
        if not add:
            return m.group(0)
        changed = True
        print(f"[加入] IncludePath: {', '.join(add)}")
        return "<IncludePath>" + cur + ";" + ";".join(add) + "</IncludePath>"

    text = re.sub(r"<IncludePath>([^<]*\.\./Core/Inc[^<]*)</IncludePath>",
                  add_include, text, count=1)

    if changed:
        with open(UVPROJX, "w", encoding="utf-8", newline="") as f:
            f.write(text)
    else:
        print("[跳过] 工程已包含全部 Servo 源文件与包含路径")

    print("\n[核验]")
    ok = verify(text)
    print("[PASS] 工程引用完整" if ok else "[FAIL] 存在缺失项")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
