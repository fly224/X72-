# -*- coding: utf-8 -*-
"""
prepare_mdk_project.py —— 适配并静态核验 X72_FOC 的 CubeMX 生成工程 (MDK-ARM)

背景:
  参考工程 FineMotionDriver.ioc 生成的 MDK-ARM 工程里, 除 CubeMX 源文件外还手工加入了
  99 个应用层 .c (app/ motor_control/ communication/ database/ board_public/
  common/ simulink_code_gen/ Board/)。X72 工作区这些目录目前为空, 直接编译会报文件缺失。
  本脚本把它裁剪为「纯净 CubeMX 初始化工程」, 保留 USER CODE BEGIN/END 标记。

用法:
    python Agnet/prepare_mdk_project.py            # 适配 + 核验
    python Agnet/prepare_mdk_project.py --verify   # 仅核验 (不改动文件)

输出:
    - 控制台统计
    - Agnet/verify_generated_code.csv (UTF-8 BOM): 每个工程源文件/include 的存在性判定
"""
import argparse
import csv
import os
import re
import sys

sys.stdout.reconfigure(encoding="utf-8")
HERE = os.path.dirname(os.path.abspath(__file__))
PROJ = os.path.normpath(os.path.join(
    HERE, "..", "Firmware", "FM_Servo_V3", "embedded_project", "STM32G474_App"))
MDK = os.path.join(PROJ, "MDK-ARM")
UVPROJX = os.path.join(MDK, "X72_FOC.uvprojx")
UVOPTX = os.path.join(MDK, "X72_FOC.uvoptx")
MAINC = os.path.join(PROJ, "Core", "Src", "main.c")

# 只保留 CubeMX 自身生成的源文件 (其余为手工加入的应用层源码)
KEEP_PREFIXES = ("../Core/", "../Drivers/", "..\\Core\\", "..\\Drivers\\", "startup_")
CUBEMX_INCLUDE_PATH = ("../Core/Inc;../Drivers/STM32G4xx_HAL_Driver/Inc;"
                       "../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy;"
                       "../Drivers/CMSIS/Device/ST/STM32G4xx/Include;"
                       "../Drivers/CMSIS/Include")

USER_INCLUDES_OLD = """/* USER CODE BEGIN Includes */
#include "board.h"
#include "app_scheduler.h"
#include "app_status_check.h"
/* USER CODE END Includes */"""
USER_INCLUDES_NEW = """/* USER CODE BEGIN Includes */
/* 应用层头文件待接入: board.h / app_scheduler.h / app_status_check.h */
/* USER CODE END Includes */"""

USER_INIT_OLD = """  /* USER CODE BEGIN 2 */
  RegisterSetAppCallback(set_app);
  RegisterCheckErrorCallback(app_get_check_error_val);
  RegisterCheckWarningCallback(app_get_check_warning_val);
  RegisterCheckStatusCallback(app_get_check_status_val);
  RegisterCheckIoInputsStatusCallback(app_get_check_di_io_val);
  RegisterSendCallback(mavlink_send_data);

  HalInit();
  BspInit();
  /* USER CODE END 2 */"""
USER_INIT_NEW = """  /* USER CODE BEGIN 2 */
  /* 应用层初始化待接入: Register*() 回调注册 / HalInit() / BspInit() */
  /* USER CODE END 2 */"""


def read(p):
    with open(p, "r", encoding="utf-8") as f:
        return f.read()


def write(p, s):
    with open(p, "w", encoding="utf-8", newline="") as f:
        f.write(s)


def adapt():
    changed = []

    # 1) .uvprojx: 删除非 CubeMX 源文件引用, 精简 IncludePath, 改工程名
    s = read(UVPROJX)
    blocks = re.findall(r"<File>.*?</File>", s, flags=re.S)
    removed = 0
    for b in blocks:
        m = re.search(r"<FilePath>(.*?)</FilePath>", b, flags=re.S)
        if not m:
            continue
        path = m.group(1).strip()
        if not path.startswith(KEEP_PREFIXES):
            s = s.replace(b, "")
            removed += 1
    s = re.sub(r"<IncludePath>[^<]*\.\./\.\./\.\./[^<]*</IncludePath>",
               "<IncludePath>" + CUBEMX_INCLUDE_PATH + "</IncludePath>", s)
    s = s.replace("FineMotionDriver", "X72_FOC")
    write(UVPROJX, s)
    changed.append(f".uvprojx: 移除 {removed} 个应用层源文件引用, IncludePath 精简, 工程名替换")

    # 2) .uvoptx: 工程名替换 (调试观察项里的旧路径仅作历史记录, 不影响编译)
    s = read(UVOPTX).replace("FineMotionDriver", "X72_FOC")
    write(UVOPTX, s)
    changed.append(".uvoptx: 工程名替换")

    # 3) main.c: 清空 USER CODE 中的应用层耦合, 保留标记
    s = read(MAINC)
    n = 0
    if USER_INCLUDES_OLD in s:
        s = s.replace(USER_INCLUDES_OLD, USER_INCLUDES_NEW)
        n += 1
    if USER_INIT_OLD in s:
        s = s.replace(USER_INIT_OLD, USER_INIT_NEW)
        n += 1
    write(MAINC, s)
    changed.append(f"main.c: 解耦 {n}/2 处 USER CODE 区 (应用层 include + 初始化调用)")

    for line in changed:
        print("  [适配] " + line)


def verify():
    rows = []
    missing = []

    # 工程引用的源文件是否存在
    s = read(UVPROJX)
    paths = re.findall(r"<FilePath>(.*?)</FilePath>", s, flags=re.S)
    for raw in paths:
        p = raw.strip().replace("\\", "/")
        full = os.path.normpath(os.path.join(MDK, p))
        ok = os.path.isfile(full)
        rows.append(["工程源文件", p, "存在" if ok else "缺失", "PASS" if ok else "FAIL"])
        if not ok:
            missing.append(p)

    # CubeMX 源码中 #include "..." 的本地头文件能否解析
    inc_dirs = [
        os.path.join(PROJ, "Core", "Inc"),
        os.path.join(PROJ, "Drivers", "STM32G4xx_HAL_Driver", "Inc"),
        os.path.join(PROJ, "Drivers", "CMSIS", "Include"),
        os.path.join(PROJ, "Drivers", "CMSIS", "Device", "ST", "STM32G4xx", "Include"),
    ]
    src_files = [os.path.join(PROJ, "Core", "Src", f)
                 for f in os.listdir(os.path.join(PROJ, "Core", "Src")) if f.endswith(".c")]
    src_files += [os.path.join(PROJ, "Core", "Inc", f)
                  for f in os.listdir(os.path.join(PROJ, "Core", "Inc")) if f.endswith(".h")]
    checked = 0
    for src in src_files:
        text = read(src)
        for hdr in re.findall(r'#include\s+"([^"]+)"', text):
            checked += 1
            ok = any(os.path.isfile(os.path.join(d, hdr)) for d in inc_dirs)
            rows.append(["本地 include", os.path.relpath(src, PROJ) + " -> " + hdr,
                         "可解析" if ok else "未找到", "PASS" if ok else "FAIL"])
            if not ok:
                missing.append(hdr)

    total = len(rows)
    fail = sum(1 for r in rows if r[3] == "FAIL")
    print(f"\n[核验] 工程源文件引用 {len(paths)} 个, 本地 include 检查 {checked} 处, "
          f"判定项 {total} 个, FAIL={fail}")
    if missing:
        print("[核验] 缺失项:")
        for m in sorted(set(missing))[:20]:
            print("   - " + m)
    else:
        print("[核验] 无缺失: 工程引用的源文件与所有本地 include 均可解析")

    out = os.path.join(HERE, "verify_generated_code.csv")
    with open(out, "w", encoding="utf-8-sig", newline="") as f:
        w = csv.writer(f)
        w.writerow(["类别", "对象", "结果", "判定"])
        w.writerows(rows)
    print("[核验] 已导出: " + out)
    return fail


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--verify", action="store_true", help="仅核验, 不改动文件")
    args = ap.parse_args()
    print(f"工程目录: {PROJ}")
    if not os.path.isfile(UVPROJX):
        print("错误: 未找到 X72_FOC.uvprojx, 请先复制生成代码")
        return 2
    if not args.verify:
        adapt()
    return 0 if verify() == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
