#!/usr/bin/env python3
# ============================================================
# 已废弃（2026-11-15 18:37）：模型重新生成后已是 real32_T，且三环参数
# 改为顶层输入 par_c/par_w/par_p，本脚本的 4 条字面量替换规则不再匹配。
# 改参数请改 Board/servo_isr.c 的 model_params_init()。
# ============================================================
# -*- coding: utf-8 -*-
"""
patch_gen_code.py —— 「节拍适配」补丁（2026-11-15 起改为幂等核验，不再改文件）

历史
----
模型按 20 kHz（Ts_i=50us）设计，但本板 double 版单拍 8862~9282 cyc > 50us，
ADC2 每两次触发丢一次，控制环静默降到 10 kHz，导致 dt / N_w / N_p / gain_comp
四处失配。旧版本脚本用字面量替换给生成代码打补丁（见文档 §5.4）；
11-15 17:00 曾把参数源改为 Ts_i=100e-6 以匹配实机 10 kHz。

现状（2026-11-15 18:00 起）
------------------
1. 按用户要求把 `three_loop_params.m` 的 Ts_i 切回 50e-6（20 kHz）；
   `build_three_loop_deploy.m` 的 dt / N_w / N_p 与 gain_comp 全部由 Ts_i 派生：
   dt=5e-5、N_w=10（2 kHz）、N_p=20（1 kHz）、gain_comp=0.00105。
   ⚠ 实机要真跑 20 kHz 必须先 single 化（double 版 9282 cyc > 50us 会掉回 10 kHz）。
2. 参数不固化为 Constant：模型有 3 个顶层 Inport（par_c/par_w/par_p），
   生成代码从 `three_loop_deploy_U.par_*[]` 读取；默认值由
   `three_loop_tuning.c` 的 ThreeLoop_Tuning_Init() 填入。
3. 因此**重新生成的代码本身就带正确参数**，本脚本不再需要修改任何文件。

用法
----
  python patch_gen_code.py --check   # 只核验（推荐）
  python patch_gen_code.py           # 同样只核验；保持旧命令兼容

行为
----
  A. 检测到参数 Inport 版生成代码 -> 核验 4 项，输出「[模型侧已修正]」，退出 0
  B. 检测到旧固化常量版生成代码   -> 沿用原 4 条规则做幂等字面量替换（兼容历史流程）
  C. 两种特征都不匹配             -> 报异常并不改文件，退出 3
"""
import argparse
import io
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
TARGET = os.path.join(HERE, "three_loop_deploy.c")
TUNING = os.path.join(HERE, "three_loop_tuning.c")

# 旧版（固化常量）幂等补丁规则：(说明, 旧值, 新值, 期望次数)
# 注意：这是 10 kHz 阶段的历史规则；当前 20 kHz + 参数 Inport 模型不走此路径，
#       仅当检测到旧固化常量版生成代码时才作为兼容兜底。
LEGACY_RULES = [
    ("电流环积分步长 dt",
     "2378.18563877 * 5.0E-5", "2378.18563877 * 1.0E-4", 2),
    ("速度环分频 N_w (10 -> 5)",
     "three_loop_deploy_DW.cnt >= 10.0", "three_loop_deploy_DW.cnt >= 5.0", 1),
    ("位置环分频 N_p (20 -> 10)",
     "three_loop_deploy_DW.cnt_i >= 20.0", "three_loop_deploy_DW.cnt_i >= 10.0", 1),
    ("角度延时补偿 1.5*Ts_i*p",
     "+ 0.00105\n    * three_loop_deploy_U.omega_m",
     "+ 0.0021\n    * three_loop_deploy_U.omega_m", 1),
]

# 新版（参数 Inport）核验项：(说明, 需在 C 中出现的片段, 需在 tuning.c 中出现的片段)
MODERN_CHECKS = [
    ("电流环 dt 由 U.par_c[2] 提供（=5e-5）",
     "three_loop_deploy_U.par_c[2]", "#define TL_TS_I        (50.0e-6)"),
    ("速度环分频由 U.par_w[4] 提供（=10）",
     "three_loop_deploy_U.par_w[4]", "#define TL_N_W         (10.0)"),
    ("位置环分频由 U.par_p[3] 提供（=20）",
     "three_loop_deploy_U.par_p[3]", "#define TL_N_P         (20.0)"),
    ("角度延时补偿 = 14*1.5*50us = 0.00105",
     "0.00105", "#define TL_GAIN_COMP"),
]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--check", action="store_true", help="只核验，不修改（兼容旧用法）")
    args = ap.parse_args()

    if not os.path.isfile(TARGET):
        print("找不到 %s" % TARGET)
        return 2

    s = io.open(TARGET, encoding="utf-8").read()
    t = io.open(TUNING, encoding="utf-8").read() if os.path.isfile(TUNING) else ""
    print("目标: %s (%d 字节)" % (os.path.basename(TARGET), len(s)))
    print("-" * 76)

    modern = "three_loop_deploy_U.par_c[" in s
    if modern:
        ok = True
        for name, c_frag, t_frag in MODERN_CHECKS:
            c_ok = c_frag in s
            t_ok = (not t_frag) or (t_frag in t)
            tag = "模型侧已修正" if (c_ok and t_ok) else "异常"
            print("  [%s] %-42s C:%s tuning.c:%s" %
                  (tag, name, "OK" if c_ok else "MISS", "OK" if t_ok else "MISS"))
            if not (c_ok and t_ok):
                ok = False
        print("-" * 76)
        print("结论: 参数源已按 20 kHz 正规派生，PID 参数以 Inport 对外可调，")
        print("      本补丁脚本无需再做任何替换（生成 -> --check -> 编译 即可）。")
        return 0 if ok else 3

    # ---- 兼容旧固化常量版 ----
    pending, bad = 0, 0
    for name, old, new, want in LEGACY_RULES:
        n_old, n_new = s.count(old), s.count(new)
        if n_new == want and n_old == 0:
            print("  [已应用] %-42s (%d 处)" % (name, want))
        elif n_old == want and n_new == 0:
            print("  [待应用] %-42s (%d 处)" % (name, want))
            pending += 1
            if not args.check:
                s = s.replace(old, new)
        else:
            print("  [异常]   %-42s 旧 %d / 新 %d，期望 %d" % (name, n_old, n_new, want))
            bad += 1
    print("-" * 76)
    if bad:
        print("有 %d 项状态异常：模型结构已变，请人工确认后再动。" % bad)
        return 3
    if pending and not args.check:
        io.open(TARGET, "w", encoding="utf-8", newline="").write(s)
        print("已写入旧版字面量补丁 %d 项" % pending)
    if pending == 0:
        print("旧版生成代码：4 项全部已是新值，无需修改")
        return 0
    print("警告: 这是旧的固化参数版模型。请优先改 three_loop_params.m 后重新")
    print("      generate_three_loop_code，而不是长期依赖字面量补丁。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
