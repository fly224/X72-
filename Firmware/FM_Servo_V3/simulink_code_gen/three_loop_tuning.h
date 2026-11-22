/*
 * three_loop_tuning.h —— 三环 PID 参数对外接口（手工层，不随 codegen 被覆盖）
 *
 * 背景：原模型把 par_c/par_w/par_p 用 Constant 固化（InlineParams=on），
 *       改一次增益要重建模型重新生成代码。现模型已把三组参数改为顶层 Inport，
 *       生成代码里是 three_loop_deploy_U.par_c[11] / par_w[5] / par_p[4]。
 *
 * 本层职责：
 *   1) 定义「对外可调的 8 个参数」及默认值、范围、定点换算；
 *   2) 把周期/电机标定等固定字段按 Ts_i 派生填入 U，禁止上位机改写；
 *   3) 提供原子提交（影子结构体 -> 一次性展开到 U），避免半更新；
 *   4) 提供位置指令 wrap，解决 pos_ref 多圈 / theta_m 单圈坐标系不一致导致的过流。
 *
 * 与工程约定对齐（Agnet/固件协议层_职责与编写指南.md §5/§7）：
 *   - set/get_app_* 是唯一参数通道；本层不直接解析协议，只提供 ParamSet/ParamGet
 *     供 database 回调调用；
 *   - 使能态拒写、权限总闸由 database 层判定后传入 is_enabled / 自行把关；
 *   - 定点整数（int32）便于 MAVLink / CiA402 字典 ×1000 换算；
 *   - 解析与参数写入在 1 ms 非实时任务，绝不在控制 ISR 里做。
 *
 * 加入 MDK 工程：three_loop_tuning.c（+ 本头文件）；在 AppParamInit() 之后
 * 调一次 ThreeLoop_Tuning_Init()，再启动控制 ISR。
 */

#ifndef THREE_LOOP_TUNING_H
#define THREE_LOOP_TUNING_H

#include <stdint.h>
#include "three_loop_deploy.h"      /* real32_T, three_loop_deploy_U */

#ifdef __cplusplus
extern "C" {
#endif

/* ---------------- 对外可调参数（8 项） ---------------- */
typedef struct {
    real32_T kp_i;        /* 电流环 Kp [V/A]          */
    real32_T ki_i;        /* 电流环 Ki [V/(A*s)]      */
    real32_T kp_w;        /* 速度环 Kp [A/(rad/s)]    */
    real32_T ki_w;        /* 速度环 Ki [A/(rad/s)/s]  */
    real32_T kp_p;        /* 位置环 Kp [(rad/s)/rad]  */
    real32_T w_max;       /* 速度给定限幅 [rad/s]     */
    real32_T i_max;       /* q 轴电流限幅 [A]         */
    real32_T max_duty;    /* 最大调制深度 [0,1]       */
} ThreeLoop_Tuning_T;

/* 与 database 的 APP_PARAM_STATUS 对齐 */
typedef enum {
    THREE_LOOP_OK                = 0,
    THREE_LOOP_READ_ONLY         = 1,
    THREE_LOOP_OUT_OF_RANGE      = 2,
    THREE_LOOP_WRITE_STATE_ERROR = 3,   /* 使能态拒写 */
    THREE_LOOP_UNKNOWN_ID        = 4
} ThreeLoop_ParamStatus_T;

/* 参数 ID（可写 0x01..0x08；只读 0x81..0x85 用于上位机显示/自检） */
#define TL_PID_CUR_KP      0x01
#define TL_PID_CUR_KI      0x02
#define TL_PID_VEL_KP      0x03
#define TL_PID_VEL_KI      0x04
#define TL_PID_POS_KP      0x05
#define TL_PID_VEL_MAX     0x06
#define TL_PID_I_MAX       0x07
#define TL_PID_DUTY_MAX    0x08

#define TL_RO_CTRL_TS_US   0x81    /* 实际控制周期 [us]       */
#define TL_RO_N_W          0x82    /* 速度环分频              */
#define TL_RO_N_P          0x83    /* 位置环分频              */
#define TL_RO_GAIN_COMP    0x84    /* 角度延时补偿系数 ×1e6   */
#define TL_RO_POLE_PAIRS   0x85    /* 极对数                  */

/* 默认值：与 three_loop_params.m（Ts_i=50us / 20 kHz）逐项对齐 */
extern const ThreeLoop_Tuning_T THREE_LOOP_TUNING_DEFAULT;

/* 初始化：把默认值 + 固定字段写进 U.par_c/par_w/par_p。
   必须在 three_loop_deploy_initialize() 之后、启动控制 ISR 之前调用一次。 */
void ThreeLoop_Tuning_Init(void);

/* 范围校验（在写入影子结构体之前调用） */
ThreeLoop_ParamStatus_T ThreeLoop_Tuning_Validate(const ThreeLoop_Tuning_T *t);

/* 原子提交：一次性把完整结构体展开到 U.par_*（内部含临界区） */
void ThreeLoop_Tuning_Apply(const ThreeLoop_Tuning_T *t);

/* 回读当前生效值（供上位机回显 / Flash 保存） */
void ThreeLoop_Tuning_Get(ThreeLoop_Tuning_T *t);

/* database 回调入口：raw 为定点整数，按各参数 scale 换算。
   is_enabled != 0 时拒绝写入（APP_PARAM_WRITE_STATE_ERROR）。 */
ThreeLoop_ParamStatus_T ThreeLoop_ParamSet(uint8_t id, int32_t raw, uint8_t is_enabled);
ThreeLoop_ParamStatus_T ThreeLoop_ParamGet(uint8_t id, int32_t *raw);

/* 位置指令坐标系折叠：把多圈 pos_ref 折到 theta_m 所在的那一圈。
   返回可直接写入 U.pos_ref 的值；模型内 pos_ref - theta_m 即 wrap_pi(真实误差)。
   Ex 版本带 dir_sign（方向辨识得到的 ±1，见文档 §3.3 的 ds）。
   代价约 15 cyc，见 Agnet/电流环整定与三环频率修正.md §3.3。 */
real32_T ThreeLoop_WrapPosRefEx(real32_T pos_ref_multi, real32_T theta_m, real32_T dir_sign);
real32_T ThreeLoop_WrapPosRef(real32_T pos_ref_multi, real32_T theta_m);   /* dir_sign = +1 */

#ifdef __cplusplus
}
#endif
#endif /* THREE_LOOP_TUNING_H */
