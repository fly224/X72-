#ifndef APP_PARAM_H
#define APP_PARAM_H

#include <stdint.h>

/* ---------------------------------------------------------------------------
 * 固件版本号（单一真源，见 docs/版本管理规范.md §2）
 *
 * 发版本时四个宏一起改，并同步：README.md 的"当前版本"、CHANGELOG.md 新条目、
 * 打对应 tag（tag 名带 v 前缀，宏里不带）。
 * 遥测出口（从总线读到版本）尚未实现，留待后续 MINOR。 */
#define FW_VERSION_MAJOR   (1)
#define FW_VERSION_MINOR   (1)
#define FW_VERSION_PATCH   (0)
#define FW_VERSION_STR     "1.1.0"

/* ---------------------------------------------------------------------------
 * app_param.h —— database 参数门卫层（2026-11-18）
 *
 * 依据《固件协议层_职责与编写指南.md》第 1/3 节：
 *   "所有参数读写必须走 database 的 get/set 回调，不允许协议层直接改应用变量。"
 * 本层是上位机（MAVLink）访问内部状态的**唯一通道**，负责：
 *   1. 权限总闸（Comm_control_authority，0=上位机接管）；
 *   2. 状态检查（使能态拒写额定参数、故障态拒使能走 ServoCmd_Arm 的联锁）；
 *   3. 范围检查（模式值、系统命令码）；
 *   4. 回读（set 后 get 返回固件真实值，写入被拒时不误报成功）。
 *
 * 与参考工程 kAxis/FlashDB 的差异：本工程控制量仍是 Board 层的 Servo_* API，
 * 故 set_app_* 内部调用 servo_cmd.c 暴露的受控入口（复用 E/D/X 联锁），
 * 而不是直接写 s_en/s_mode 等静态变量。这样命令行与 MAVLink 两条路径的联锁一致。
 * ------------------------------------------------------------------------- */

typedef enum {
    APP_PARAM_SUCCESS = 0U,
    APP_PARAM_READ_ONLY = 1U,        /* 只读参数 */
    APP_PARAM_OUT_OF_RANGE = 2U,     /* 超范围 */
    APP_PARAM_WRITE_STATE_ERROR = 3U,/* 状态错误（使能态/故障态拒写） */
    APP_PARAM_NO_AUTHORITY = 4U      /* 无上位机控制权 */
} AppParamStatus_t;

/* CiA402 运行模式（对应 30056 Modes_of_operation，参考工程 1~11；本板实际支持子集）
 * MIT=12 是本板自定义的阻抗控制模式（CiA402 标准无对应值，11 之后第一个空闲值），
 * 用来让上位机纯靠 MAVLink 指令就能切入 MIT，而不必再发私有 0x02 帧。 */
#define APP_OP_MODE_POS   (1)   /* 位置（profile position，映射手写位置环） */
#define APP_OP_MODE_VEL   (3)   /* 速度（profile velocity，映射速度闭环） */
#define APP_OP_MODE_TORQUE (4)  /* 力矩（profile torque，映射电流闭环） */
#define APP_OP_MODE_MIT   (12)  /* MIT 阻抗（本板自定义，映射 Servo_SetMitCmd） */

/* 编码器脉冲当量：MT6835 2^21 counts/rev，与 BspServo_GetPosSum() 同口径。
 * MAVLink 30019/30065 的 pos_tar_p 是 int64 脉冲（多圈），换算到本板 rad 机械角。 */
#define APP_ENC_COUNTS    (2097152.0f)
#define APP_TWO_PI        (6.283185307179586f)

/* 关节减速比（2026-11-24）：行星减速器 9:1。
 *
 * 位置模式（30019 给定 / 30100 回读）与 MIT（30065 给定 + 应答）的
 * **位置设定与回读对外一律用输出端角度**：
 *   · 位置环（2026-11-24）：内部坐标已改输出端口径（pos_coord = theta_multi/9），
 *     协议边界不再做 ×9/÷9，给定/回读直接就是输出端；
 *   · MIT：内部仍是电机端多圈角（theta_multi），协议边界 ×9（收）/÷9（发），
 *     kp/kd（力矩/电机端误差）语义不变。
 * 速度环/电流环全程保持电机端不变。换减速器时只改这一个宏。 */
#define ACT_GEAR_RATIO     (9.0f)

/* 系统命令（对应 30069 Sys_cmd，参考工程语义；首期实现 0/1/2） */
#define APP_SYS_CMD_NONE       (0U)
#define APP_SYS_CMD_SAVE_PARAM (1U)   /* 存参数到 Flash */
#define APP_SYS_CMD_RESET      (2U)   /* 软复位 */
#define APP_SYS_CMD_CLEAR_FAULT (3U)  /* 清故障（等价 X） */
/* 4..14 为校准/辨识动作，语义见 servo_cmd.h 的 SC_ACT_*。
 * 由 AppParam_ExecSysCmd() 转发到 ServoCmd_ExecAction()，
 * 与 ASCII 单字符命令共用同一实现（飞控协议化的过渡层）。 */

void AppParam_Init(void);

/* ---- 权限总闸（30068）----
 * Comm_control_authority: 0=上位机接管（可写），非0=本地/其他主体（拒写）。
 * Internal_control_authority 保留（本板无内部控制器，回读固定 0）。 */
AppParamStatus_t AppParam_SetAuthority(uint8_t comm_auth, uint8_t internal_auth);
uint8_t          AppParam_GetCommAuthority(void);
uint8_t          AppParam_GetInternalAuthority(void);

/* ---- 控制字（30054）----
 * Controlword bit0 = 使能开关（CiA402 简化：1=装配并使能，0=ServoCmd_Disarm）。
 * Halt_running_cmd !=0 = 立即封锁（等价 D）。
 * ★ bit0 由 0→1 时按 s_op_mode 装配"最近一次已就绪的给定"，而不是一律走
 *   do_enable() 的位置保持：先选速度/力矩、再使能的顺序才符合上位机直觉。
 *   若该模式还没有任何有效给定，才退回位置保持（与 E 命令一致）。 */
AppParamStatus_t AppParam_SetControlWord(uint16_t cw, uint8_t halt);
uint16_t         AppParam_GetControlWord(void);

/* ---- 运行模式（30056）---- */
AppParamStatus_t AppParam_SetOpMode(int8_t mode);
int8_t           AppParam_GetOpMode(void);

/* ---- 4 控制模式给定（2026-11-18）----
 * 所有给定经权限门卫后落到 servo_cmd 的受控装配入口（复用 E/W/M/P/MIT 联锁）。
 * 单位口径：
 *   Torque: iq 电流 A（±5 A，与电流环限幅一致）
 *   Speed : 机械角速度 rad/s（由 30019 speed_tar_p_s 脉冲/s 换算）
 *   Pos   : 机械角 rad（由 30019 pos_tar_p 脉冲换算，多圈）
 *   MIT   : p[rad] v[rad/s] kp[N·m/rad] kd[N·m/(rad/s)] tau[N·m]
 * 自动先封锁再切：任给定的模式与当前运行环不同，set 内部先 Disarm 再装配新环，
 * 保证 ISR 里不会出现两条环同时 s_en=1 的竞态。 */
AppParamStatus_t AppParam_SetTorqueIq(float iq_a);
AppParamStatus_t AppParam_SetSpeedW(float w_rad_s);
AppParamStatus_t AppParam_SetPosP(float p_rad);
/* 30019 专用：pos_tar_p 与 speed_tar_p_s 两个字段一次写入缓存，
 * 只按当前 s_op_mode 装配对应环；另一路留作切模式后由控制字使能直接装配。 */
AppParamStatus_t AppParam_SetPosSpeed(float p_rad, float w_rad_s);
AppParamStatus_t AppParam_SetMitCmd(float p, float v, float kp, float kd, float tau);
AppParamStatus_t AppParam_SetMitConfig(float kp, float kd, float kt);  /* 30066 配置 */

/* ---- 速度模式轨迹参数（2026-11-19）----
 * 加速度：30019 off32 acc_tar_p_ss（脉冲/s²）→ rad/s²，范围 5~20000，作用手写速度环斜坡。
 * 轨迹：新增 30089 VelProfileConfig，mode 0=梯形 1=S 曲线，j_max 0~200000 rad/s³。 */
AppParamStatus_t AppParam_SetSpeedAccel(float a_rad_s2);
AppParamStatus_t AppParam_SetSpeedProfile(uint8_t mode, float j_max);
float AppParam_GetSpeedAccel(void);
void  AppParam_GetSpeedProfile(uint8_t *mode, float *j_max);

float AppParam_GetTorqueIq(void);
float AppParam_GetSpeedW(void);
float AppParam_GetPosP(void);
void  AppParam_GetMit(float *p, float *v, float *kp, float *kd, float *tau);

/* 当前实际生效的模式（读 ISR 状态，而非缓存的上位机写入值） */
int8_t AppParam_GetActiveMode(void);

/* ---- 系统命令（30069，写后即时生效）---- */
AppParamStatus_t AppParam_ExecSysCmd(uint8_t cmd);

/* ---- 状态字/错误字（供 30055 AppStatusInfo 回读）----
 * statusword: bit0=使能就绪, bit1=故障, bit2=目标到达(位置环 DONE)
 * error_word: 直接映射 BspServo_GetFault() 低 16 位 */
uint32_t AppParam_GetStatusWord(void);
uint32_t AppParam_GetErrorWord(void);

#endif /* APP_PARAM_H */
