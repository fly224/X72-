import time, struct, sys
sys.path.insert(0, r"G:\FOC\驱动开发\PG_X72_CAN\host")
from mavlink_link import SerialLink

lk = SerialLink(); lk.connect("COM27", 921600); time.sleep(1.2)

def wait_ident(kind, timeout):
    """等本次新产生的 30092（kind 匹配），返回 ok/n/v[]。"""
    t0 = time.time()
    while time.time() - t0 < timeout:
        m = lk.last_msg.get(30092)
        if m and m["ts"] >= t0:
            raw = lk._payload_of(m)
            if raw and len(raw) >= 40 and raw[0] == kind:
                return {"ok": raw[1], "n": raw[2] | (raw[3] << 8),
                        "v": list(struct.unpack_from("<8f", raw, 4))}
        time.sleep(0.02)
    return None

# --- 前置：权限 / 清故障 / 封锁 / C ---
lk.set_permission(0, 0)
st = lk.ctrl_status()
if st["fault"]:
    lk.action(3); time.sleep(0.6)
lk.set_controlword(0x06, halt=1); time.sleep(0.4)
if not st["cal"]:
    lk.action(4); time.sleep(0.5)

# --- Z ---
a = lk.action(5)
assert a.get("ok"), "Z 被拒（返回 3 说明还在闭环）"
r = wait_ident(8, 50)
if r and r["ok"]:
    v = r["v"]
    print("ratio=%.2f dir=%+d off=%.2f° zero=%.2f° resid=%.2f"
          % (v[0], int(v[1]), v[2], v[3], v[7]))
else:
    if r:
        v = r["v"]
        print("Z 失败: err=%d dmech=%.0f/%.0f/%.0f mdeg"
              % (int(v[0]), v[1], v[2], v[3]))
    else:
        print("Z 超时：50 s 内没收到 30092 kind 8")
