# -*- coding: utf-8 -*-
"""冷态重做 ψ 辨识（+），确认 Tref 记录与 ktc 一致性。

为什么现在必须重做：
    Flash 里的 psi=3.89 mWb 是 84 °C 温升测试时热态辨识的结果，
    而当时 Kt 补偿以固定 20 °C 为基准 —— 等于把热态值当冷态值再压一次。
    新固件会记录"辨识时温度"，但旧参数没有这个标记（Tref=old），
    必须在新温度下重测一次，ψ 与 Tref 才成对。
"""
import re
import sys
import time

sys.path.insert(0, r"G:\FOC\X72\Agnet")
from ident_lib import Link, BAUD  # noqa: E402
from thermal_soak import MSG_PERMISSION  # noqa: E402


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else "COM27"
    lk = Link(port)
    try:
        time.sleep(1.2)
        # 只发 permission 让 ASCII 通道可用；+ 自己会开 PWM，不需要 E
        st = lk.status()
        print("before: %s" % st.get("line", "(no status)"))

        # D 确保 MOE=0（+ 要求 armed=0）
        lk.s.reset_input_buffer()
        lk.write("D")
        time.sleep(0.6)

        print("温度: 见 30025 遥测；开始 + 辨识 ...", flush=True)
        txt, el, refused = lk.run("+", ["[CMD] FI Kt=", "[CMD] FI FAIL"], 120.0, "FI")
        print("耗时 %.1f s  refused=%s" % (el, refused))
        for line in txt.splitlines():
            if "[FI]" in line or "[CMD] FI" in line:
                print("  " + line.strip())

        m = re.search(r"psi=([\d.]+)uH|psi=([\d.]+)mWb", txt)
        st2 = lk.status()
        print("after : %s" % st2.get("line", "(no status)"))
        k = lk.query("K")
        print("K     : %s" % k)
    finally:
        try:
            lk.write("D")
            time.sleep(0.3)
        except Exception:
            pass
        lk.close()


if __name__ == "__main__":
    main()
