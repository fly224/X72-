# -*- coding: utf-8 -*-
"""build_x72.py —— X72_FOC 编译/烧录（UV4）

为什么要这个脚本：UV4 依赖 TMP/TEMP，而 Git Bash 的 /tmp 它不认，
直接调会报 "Failed to create temporary file name" 且**仍返回 0**、
日志里写 "Target not created" —— 看起来成功其实是假的。
这里把 TMP 指到工程内可写目录，并强制校验日志关键字。

2026-11-21 修正（重要）：UV4 的 `-f` 在本工程里**只做 Flash Download**，
不触发 build —— 日志里只有 "Load/Erase/Program"，返回码仍是 0。
于是"改完源码 → -f"会静默烧进上一次的 axf：实测连续 4 次 -f 之后
.axf 的时间戳还停在 1 小时前，而验证脚本跑的是旧固件，
现象是"新加的诊断槽读不到、n_int 还是旧值"。现在 -f 改成两步：
先 `-r` 全量编译（校验 0 Error/0 Warning），成功后再 `-f` 烧录（校验 Verify OK）。

用法:
    python build_x72.py            # 全量编译 (-r)
    python build_x72.py -f         # 全量编译 + 烧录（两步，且各自校验）
    python build_x72.py -r --src cog_ff.c param_store.c   # 额外校验这些文件被编译
"""
import os
import subprocess
import sys
import time

UV4 = r"E:\keil5c51\51\UV4\UV4.exe"
PROJ = (r"G:\FOC\X72\Firmware\FM_Servo_V3\embedded_project"
        r"\STM32G474_App\MDK-ARM\X72_FOC.uvprojx")
AXF = (r"G:\FOC\X72\Firmware\FM_Servo_V3\embedded_project"
       r"\STM32G474_App\MDK-ARM\X72_FOC\X72_FOC.axf")
BASE = os.path.dirname(os.path.abspath(__file__))
TMPD = os.path.join(BASE, ".build_tmp")
LOG = os.path.join(BASE, "build.log")
BUILD_LOG = os.path.join(BASE, "build_compile.log")


def run_uv4(mode, env, log=LOG):
    """跑一次 UV4，返回 (rc, 日志文本)。log 可分开，避免后一步覆盖前一步的证据。"""
    try:
        os.remove(log)
    except OSError:
        pass
    t0 = time.time()
    rc = subprocess.call([UV4, mode, PROJ, "-j0", "-o", log], env=env, timeout=420)
    dt = time.time() - t0
    txt = ""
    if os.path.exists(log):
        with open(log, "rb") as f:
            txt = f.read().decode("utf-8", "replace")
    print("---- %s RC=%d %.1fs ----" % (mode, rc, dt))
    print(txt.strip())
    return rc, txt


def main():
    argv = sys.argv[1:]
    mode = "-r"
    must = []
    if argv and argv[0] in ("-r", "-f", "-b"):
        mode = argv.pop(0)
    if "--src" in argv:
        i = argv.index("--src")
        must = argv[i + 1:]
    else:
        must = ["cog_ff.c", "param_store.c", "servo_cmd.c", "servo_isr.c"]

    os.makedirs(TMPD, exist_ok=True)
    env = dict(os.environ, TMP=TMPD, TEMP=TMPD)

    # -f 必须先编译：见文件头的说明。两步的日志各自校验，任一步失败即停。
    if mode == "-f":
        # 编译日志单独一份：烧录那一步会覆盖 build.log，
        # 覆盖后"0 Error(s), 0 Warning(s)"这条证据就没了（复盘时找不到依据）。
        rc, txt = run_uv4("-r", env, log=BUILD_LOG)
        bad = []
        if rc != 0:
            bad.append("build RC=%d" % rc)
        if "Target not created" in txt:
            bad.append("Target not created")
        if "Failed to create temporary file" in txt:
            bad.append("TMP 环境无效（未真正编译）")
        if " 0 Error(s), 0 Warning(s)" not in txt:
            bad.append("编译非 0 error/warning")
        for m in must:
            if ("compiling %s" % m) not in txt:
                bad.append("未编译 %s（可能是旧 .o）" % m)
        if bad:
            print("\n!! FAIL(build): " + "; ".join(bad))
            return 1
        axf_t = os.path.getmtime(AXF) if os.path.exists(AXF) else 0
        if (time.time() - axf_t) > 600:
            print("\n!! FAIL(build): axf 时间戳未更新（%.0f 分钟前）" % ((time.time() - axf_t) / 60.0))
            return 1
        print("\n[build] OK，开始烧录")

    mode2 = "-f" if mode == "-f" else mode
    rc, txt = run_uv4(mode2, env)
    bad = []
    if mode2 == "-r":
        if "Target not created" in txt:
            bad.append("Target not created")
        if "Failed to create temporary file" in txt:
            bad.append("TMP 环境无效（未真正编译）")
        if " 0 Error(s), 0 Warning(s)" not in txt:
            bad.append("非 0 error/warning")
        for m in must:
            if ("compiling %s" % m) not in txt:
                bad.append("未编译 %s（可能是旧 .o）" % m)
    else:
        if "Verify OK" not in txt:
            bad.append("烧录未 Verify OK")

    if bad:
        print("\n!! FAIL: " + "; ".join(bad))
        return 1
    print("\nOK")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
