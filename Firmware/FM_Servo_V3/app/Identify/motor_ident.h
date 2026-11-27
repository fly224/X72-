/*
 * motor_ident.h —— 电机参数辨识（极对数 / 方向 / 电角度零点），VESC 策略
 *
 * 为什么放在 Board 层：
 *   Keil 工程里 app/ motor_control/ communication/ 等都是 0 文件的空分组，
 *   IncludePath 也只有 Board、Core/Inc、Drivers、simulink_code_gen。
 *   本模块依赖 BSP（编码器、PWM、电流）与 servo_isr（虚拟角电流环），
 *   依赖方向是 命令层 -> 本模块 -> BSP，放进 Board 不会产生反向依赖。
 *   将来真的启用 app/Identify 分层时，整对文件平移即可，API 不用改。
 *
 * 算法来源（VESC mcpwm_foc_encoder_detect 的三段式，见
 *   Agnet/对比_三种辨识方案_FM_XC与VESC.md）：
 *     ① 120° 电角度步进，正反各 N 次 -> ratio（极对数）+ dir（方向）
 *        极对数用"角度差"而不是"计数差"：丢脉冲不会让结果跳变。
 *     ② 按 ratio 扫满一个机械圈（每电周期 3 点），正向一遍
 *     ③ 反向再扫一遍，与正向合并求平均 -> offset（电角度零点）
 *        两遍正反是为了抵消齿槽/摩擦造成的逼近滞后。
 *   所有角度平均都用圆形平均 atan2(Σsin, Σcos)，天然处理 0/2π 环绕，
 *   不需要 FM_XC 那种 "|b1-b0| >= π 则加 2π" 的特判。
 *
 * 安全约束（与 servo_cmd 的联锁一致）：
 *   - 启动前必须已完成电流零点标定（C）且无故障（X）；
 *   - 全程走虚拟电角度下的 dq 电流环（id 可配、iq=0），电流由 PI 限制；
 *   - 每 1 ms 检查故障位，任何故障立即封锁 MOE 并退出；
 *   - 总时长有硬超时（MI_TIMEOUT_MS），不会无限转下去。
 */
#ifndef MOTOR_IDENT_H
#define MOTOR_IDENT_H

#include <stdint.h>

typedef enum {
    MOTOR_IDENT_IDLE = 0,
    MOTOR_IDENT_RAMP,        /* 电流从 0 线性升到给定值 */
    MOTOR_IDENT_STEP,        /* ① 120° 步进：正 N 次 + 反 N 次 -> ratio / dir */
    MOTOR_IDENT_SWEEP_FWD,   /* ② 正向扫满一个机械圈 */
    MOTOR_IDENT_SWEEP_REV,   /* ③ 反向扫回，与正向合并 -> offset */
    MOTOR_IDENT_DONE,        /* 完成，结果有效，输出已封锁 */
    MOTOR_IDENT_FAIL,        /* 失败，原因见 MotorIdentResult.err */
} MotorIdentState;

typedef enum {
    MOTOR_IDENT_OK = 0,
    MOTOR_IDENT_ERR_BUSY = 1,      /* 已经在跑了 */
    MOTOR_IDENT_ERR_CAL = 2,       /* 电流零点未标定（先发 C） */
    MOTOR_IDENT_ERR_FAULT = 3,     /* 启动前已有故障（先发 X） */
    MOTOR_IDENT_ERR_PWM = 4,       /* MOE 打开失败 */
    MOTOR_IDENT_ERR_TIMEOUT = 5,   /* 总时长超限 */
    MOTOR_IDENT_ERR_STUCK = 6,     /* 转子几乎没动：电流不足 / 机械卡死 */
    MOTOR_IDENT_ERR_RATIO = 7,     /* 算出的极对数不合理（非整数或超范围） */
    MOTOR_IDENT_ERR_FAULT_RUN = 8, /* 运行中触发故障（多为过流） */
} MotorIdentErr;

typedef struct {
    float    ratio;         /* 极对数（实数，保留小数用于判可信度） */
    uint16_t ratio_int;     /* 取整后的极对数，最终应写入 pn 的那个值 */
    int8_t   dir;           /* +1：虚拟角增加方向 = 编码器增加方向 */
    float    offset_rad;    /* 电角度零点 φ ∈ [-π, π] */
    float    zero_mech;     /* 已写入 SetEncoderZero 的机械角（rad） */
    float    dmech_avg;     /* 每 120° 步进对应的平均机械角（rad） */
    float    dmech_min;     /* 步进角差最小/最大，看离散度判断是否打滑 */
    float    dmech_max;
    float    resid;         /* |ratio - round(ratio)|，越小越可信 */
    uint16_t n_step;        /* 步进有效样本数 */
    uint16_t n_sweep;       /* 扫描有效样本数 */
    uint16_t n_fail;        /* 被判为无效（丢步/卡住）而丢弃的样本数 */
    MotorIdentErr err;
} MotorIdentResult;

/* 启动辨识。id_ref_a：辨识期间注入的 d 轴电流（A），内部限幅 [0.5, 5.0]。
 * 返回 MOTOR_IDENT_OK 表示已受理；否则返回错误码，输出保持封锁。 */
MotorIdentErr MotorIdent_Start(float id_ref_a);

/* TIM6 1 ms 调用。返回非 0 表示本次调用中辨识刚结束（DONE 或 FAIL），
 * 调用者应当读取 MotorIdent_GetResult() 并回显。 */
uint8_t MotorIdent_Task1ms(void);

/* 立即中止：封锁输出、回到 IDLE。已算出的中间结果丢弃。 */
void MotorIdent_Abort(void);

uint8_t         MotorIdent_IsBusy(void);
MotorIdentState MotorIdent_GetState(void);
const char     *MotorIdent_StateStr(void);      /* 英文短名，用于串口回显 */
const char     *MotorIdent_ErrStr(MotorIdentErr e);
const MotorIdentResult *MotorIdent_GetResult(void);

/* 从 Flash 恢复上次辨识结果（只用于 I 命令回显与 persist 的内容来源，
 * 不改状态机、不碰 MOE）。ratio_int=0 表示无效，直接忽略。 */
void MotorIdent_SetCached(uint16_t ratio_int, float ratio, int8_t dir);

#endif /* MOTOR_IDENT_H */
