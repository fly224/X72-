st = lk.ctrl_status()
assert st["cal"] and st["z"], "先做 C(4) 和 Z(5)"
lk.set_controlword(0x06, halt=1); time.sleep(0.4)

assert lk.action(8).get("ok"), "^ 被拒"
r = wait_ident(7, 45)          # 固件内部超时 30 s，主机留 45 s
if r and r["ok"]:
    v = r["v"]
    print("turns=%.1f avg=%.1f rpm chip=%d ol=%d elapsed=%dms"
          % (v[0], v[1], int(v[4]), int(v[6]), int(v[5])))
    print("⚠ 现在给编码器断电 >6 s，再上电；参数才会被芯片加载")
else:
    print("^ 失败或超时（失败不推 30092）；查 30090 fault 与本机接线")
