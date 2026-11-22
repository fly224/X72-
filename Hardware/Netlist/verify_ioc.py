# -*- coding: utf-8 -*-
"""
verify_ioc.py —— 核验 STM32CubeMX .ioc 引脚配置与 PG_X72_CAN 原理图网表的一致性

依据:
  1) Hardware/Netlist_PG_X72_CAN_2026-11-07.enet  (嘉立创 EDA 网表, JSON)
  2) Firmware/.../STM32G474_App/X72_FOC.ioc      (CubeMX 工程)

用法:
    python verify_ioc.py                 # 打印逐引脚核验表 + 统计
    python verify_ioc.py --csv          # 额外导出 verify_ioc.csv (UTF-8 BOM)
    python verify_ioc.py --ioc <路径> --enet <路径>

判定:
    PASS   .ioc Signal 包含全部期望信号
    WARN   原理图有连接但 .ioc 未显式配置 (如 SWD) / 预留脚
    FAIL   .ioc Signal 与原理图期望不符
    INFO   电源/复位/晶振等系统脚, 不参与一致率统计

注意: EXPECTED_SIGNAL 是人工依据原理图网络名 + STM32G474 数据手册 + MT6835 数据手册
      建立的期望表 (显式参数), 修改硬件或固件分配时必须同步更新本表。
"""
import argparse
import csv
import os
import sys
from collections import OrderedDict

sys.stdout.reconfigure(encoding="utf-8")
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from parse_enet import load, comp_label, iter_pins, pin_info  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_ENET = os.path.normpath(os.path.join(HERE, "..", "Netlist_PG_X72_CAN_2026-11-07.enet"))
DEFAULT_IOC = os.path.normpath(os.path.join(
    HERE, "..", "..", "Firmware", "FM_Servo_V3", "embedded_project",
    "STM32G474_App", "X72_FOC.ioc"))

# ---------------------------------------------------------------------------
# 显式参数表: 原理图网络名 -> (期望的 CubeMX Signal 关键字, 说明, 类别)
#   "expect" 内多个关键字需全部命中 (共享模拟脚会展开成 "OPAMPx_VOUT + ADCy_INz")
# ---------------------------------------------------------------------------
EXPECTED_SIGNAL = OrderedDict([
    ("LED_R",        (["GPIO_Output"], "RGB LED 红", "GPIO")),
    ("LED_G",        (["GPIO_Output"], "RGB LED 绿", "GPIO")),
    ("LED_B",        (["GPIO_Output"], "RGB LED 蓝", "GPIO")),
    ("OSC_IN",       (["RCC_OSC_IN"],  "HSE 晶振输入", "RCC")),
    ("OSC_OUT",      (["RCC_OSC_OUT"], "HSE 晶振输出", "RCC")),
    ("VBUS_ADC",     (["ADC1_IN1"],    "母线电压采样", "ADC")),
    ("OPA1_P",       (["OPAMP1_VINP"], "U 相电流运放 + 输入", "OPAMP")),
    ("OPA1_N",       (["OPAMP1_VINM"], "U 相电流运放 - 输入", "OPAMP")),
    ("OPA1_OUT",     (["OPAMP1_VOUT", "ADC1_IN3"], "U 相电流 运放输出→ADC1 注入", "OPAMP")),
    ("NTC_GAN",      (["ADC2_IN17"],   "GaN 板 NTC", "ADC")),
    ("OPA2_N",       (["OPAMP2_VINM"], "W 相电流运放 - 输入", "OPAMP")),
    ("OPA2_P",       (["OPAMP2_VINP"], "W 相电流运放 + 输入", "OPAMP")),
    ("OPA2_OUT",     (["OPAMP2_VOUT", "ADC2_IN3"], "W 相电流 运放输出→ADC2 注入", "OPAMP")),
    ("NTC_MOTOR",    (["ADC2_IN5"],    "电机 NTC", "ADC")),
    ("OPA3_P",       (["OPAMP3_VINP"], "第三路运放 + 输入 (板上悬空)", "OPAMP")),
    ("OPA3_N",       (["OPAMP3_VINM"], "第三路运放 - 输入 (板上悬空)", "OPAMP")),
    ("OPA3_OUT",     (["OPAMP3_VOUT", "ADC3_IN1"], "第三路运放输出→ADC3 注入 (板上悬空)", "OPAMP")),
    ("PB11_ADC1",    (["ADC1_IN14"],   "预留模拟输入 PB11", "ADC")),
    ("PB12_ADC2",    (["ADC1_IN11"],   "预留模拟输入 PB12", "ADC")),
    ("TIM1_CH1N",    (["TIM1_CH1N"],   "U 相下管 PWM", "TIM1")),
    ("TIM1_CH2N",    (["TIM1_CH2N"],   "V 相下管 PWM", "TIM1")),
    ("TIM1_CH3N",    (["TIM1_CH3N"],   "W 相下管 PWM", "TIM1")),
    ("TIM1_CH1",     (["TIM1_CH1"],    "U 相上管 PWM", "TIM1")),
    ("TIM1_CH2",     (["TIM1_CH2"],    "V 相上管 PWM", "TIM1")),
    ("TIM1_CH3",     (["TIM1_CH3"],    "W 相上管 PWM", "TIM1")),
    ("CAN1_RX",      (["FDCAN1_RX"],   "CANFD 收发器 SIT1042 RXD", "FDCAN")),
    ("CAN1_TX",      (["FDCAN1_TX"],   "CANFD 收发器 SIT1042 TXD", "FDCAN")),
    ("PA15",         (["TIM2_CH1"],    "刹车 PWM (BREAK_PWM)", "TIM2")),
    ("PC10",         (["GPIO_Output"], "预留 GPIO", "GPIO")),
    ("SPI_CS",       (["GPIO_Output"], "MT6835 CSN (软件片选)", "GPIO")),
    ("SPI_SCK",      (["SPI1_SCK"],    "MT6835 SPI 时钟", "SPI1")),
    ("SPI_MISO",     (["SPI1_MISO"],   "MT6835 SPI 数据输入", "SPI1")),
    ("SPI_MOSI",     (["SPI1_MOSI"],   "MT6835 SPI 数据输出", "SPI1")),
    ("UART_TX",      (["USART1_TX"],   "RS485/MAVLink 调试口", "USART1")),
    ("UART_RX",      (["USART1_RX"],   "RS485/MAVLink 调试口", "USART1")),
    ("CAL_EN",       (["GPIO_Output"], "MT6835 自动校准使能", "GPIO")),
    ("PB10",         ([],              "板上无外部连接, 应保持未配置", "NC")),
    ("PC6",          ([],              "板上无外部连接, 应保持未配置", "NC")),
    ("NRST",         ([],              "复位脚 (专用)", "SYS")),
    ("BOOT0",        ([],              "启动选择 (10k 下拉)", "SYS")),
    ("SWDIO",        (["SYS_JTMS-SWDIO"], "SWD 调试口 CN8 pin4", "SYS")),
    ("SWCLK",        (["SYS_JTCK-SWCLK"], "SWD 调试口 CN8 pin3", "SYS")),
    ("3V3",          ([],              "电源 VDD/VBAT", "PWR")),
    ("VCCA",         ([],              "模拟电源 VDDA", "PWR")),
    ("VCC3V3_REF",   ([],              "VREF+ 外部基准 REF3033", "PWR")),
    ("GND",          ([],              "地 / EP", "PWR")),
])


def parse_ioc(path):
    """解析 .ioc 为 dict; 展开 SharedAnalog_* 共享模拟脚"""
    kv = {}
    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#") or "=" not in line:
                continue
            k, v = line.split("=", 1)
            kv[k.strip()] = v.strip()

    shared = {}
    for k, v in kv.items():
        if k.startswith("SH.") and k.endswith(".ConfNb"):
            prefix = k[:-len(".ConfNb")]
            sigs = []
            i = 0
            while f"{prefix}.{i}" in kv:
                sigs.append(kv[f"{prefix}.{i}"].split(",")[0])
                i += 1
            shared[prefix.split(".", 1)[1]] = sigs

    pins = {}
    for k, v in kv.items():
        if k.endswith(".Signal"):
            pin = k[:-len(".Signal")]
            if pin.startswith("VP_"):
                continue
            if v in shared:
                v = " + ".join(shared[v])
            pins[pin] = v
    return kv, pins


def find_mcu(components):
    for cid, comp in components.items():
        if "STM32G474" in str(comp.get("props", {}).get("Manufacturer Part", "")):
            return cid, comp
    raise RuntimeError("网表中未找到 STM32G474")


def main():
    ap = argparse.ArgumentParser(description="核验 .ioc 与原理图网表一致性")
    ap.add_argument("--ioc", default=DEFAULT_IOC)
    ap.add_argument("--enet", default=DEFAULT_ENET)
    ap.add_argument("--csv", action="store_true")
    args = ap.parse_args()

    ioc_path, enet_path = os.path.abspath(args.ioc), os.path.abspath(args.enet)
    kv, ioc_pins = parse_ioc(ioc_path)
    data = load(enet_path)
    cid, mcu = find_mcu(data["components"])

    print(f"网表: {enet_path}")
    print(f"主控: {comp_label(cid, mcu)}  引脚数={len(iter_pins(mcu))}")
    print(f"ioc : {ioc_path}")
    print(f"      MCU={kv.get('Mcu.CPN')}  Package={kv.get('Mcu.Package')}  "
          f"CubeMX={kv.get('MxCube.Version')}  FW={kv.get('ProjectManager.FirmwarePackage')}")
    print("")
    print(f"{'脚号':<5}{'引脚':<18}{'原理图网络':<16}{'ioc Signal':<38}{'判定':<8}说明")
    print("-" * 118)

    rows, stat = [], {"PASS": 0, "WARN": 0, "FAIL": 0, "INFO": 0}
    for pid, pin in iter_pins(mcu):
        num, pname, net = pin_info(pid, pin)
        # .ioc 中键名与网表引脚名一致 (含 PC14-OSC32_IN / PB8-BOOT0 等)
        actual = ioc_pins.get(pname, "")
        label = kv.get(f"{pname}.GPIO_Label", "")
        expect, note, cat = EXPECTED_SIGNAL.get(net, ([], "未在期望表中登记", "UNKNOWN"))

        if cat in ("PWR", "SYS") and not expect:
            verdict = "INFO"
        elif cat == "SYS" and expect and not actual:
            verdict = "WARN"
        elif not expect:
            verdict = "PASS" if not actual else "WARN"   # NC 脚应保持未配置
        elif not actual:
            verdict = "FAIL"
        elif all(e in actual for e in expect):
            verdict = "PASS"
        else:
            verdict = "FAIL"

        stat[verdict] = stat.get(verdict, 0) + 1
        rows.append([num, pname, net, actual or "(未配置)", label,
                     "+".join(expect) or "(无)", verdict, note])
        print(f"{num:<5}{pname:<18}{net:<16}{actual or '(未配置)':<38}{verdict:<8}{note}")

    total = stat["PASS"] + stat["WARN"] + stat["FAIL"]
    print("-" * 118)
    print(f"核验脚数={len(rows)}  PASS={stat['PASS']}  WARN={stat['WARN']}  "
          f"FAIL={stat['FAIL']}  INFO(电源/系统)={stat['INFO']}")
    if total:
        print(f"一致率 (PASS/参与判定) = {stat['PASS']}/{total} = {100.0*stat['PASS']/total:.1f}%")

    if args.csv:
        out = os.path.join(HERE, "verify_ioc.csv")
        with open(out, "w", encoding="utf-8-sig", newline="") as f:
            w = csv.writer(f)
            w.writerow(["脚号", "引脚名", "原理图网络", "ioc Signal", "ioc Label",
                        "期望 Signal", "判定", "说明"])
            w.writerows(rows)
        print(f"已导出: {out}")

    return 0 if stat["FAIL"] == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
