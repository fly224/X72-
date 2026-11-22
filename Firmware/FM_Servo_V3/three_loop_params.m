function P = three_loop_params()
%THREE_LOOP_PARAMS  三环 FOC 部署控制器的唯一参数源
%
% 本文件是 build_three_loop_deploy.m 与 generate_three_loop_code.m 共同依赖的
% 参数入口。改这里 -> 重建模型 -> 重新生成代码，是唯一可持续的修改路径；
% 直接改 three_loop_deploy.c 会被下一次生成覆盖。
%
% 【2026-11-15 恢复说明】
%   原文件曾丢失（build_three_loop_deploy.m:29 引用它），本文件按当时 .slx 里
%   已固化的值逐项还原，并逐条与 three_loop_deploy.slx 的 par_c / par_w / par_p
%   / theta_off / gain_p_theta / gain_p_w / gain_comp 对齐验证过。
%
% 【控制周期：2026-11-15 18:00 切回 20 kHz】
%   设计目标 20 kHz（Ts_i = 50 us）。此前因 double 版单拍 8862~9282 cyc > 50us、
%   ADC2 每两次触发丢一次，实机静默降到 10 kHz，故一度把 Ts_i 写成 100e-6 以匹配
%   实际节拍。现按《电流环整定与三环频率修正.md》§6.2 切回设计值 50e-6：
%     dt = 5e-5，N_w = 10（2 kHz），N_p = 20（1 kHz），gain_comp = 0.00105，
%     全部由 Ts_i 自动派生，PID 默认值无需改动。
%
%   ⚠ 实机前提：本模型/生成代码仍是 double。M4F 只有单精度 FPU，double 版单拍
%   9282 cyc ≈ 54.6 us > 50 us，直接按 20 kHz 烧录会再次丢触发掉回 10 kHz。
%   必须先把模型 single 化（配置⑤实测 1813 cyc）或确认当前固件的单精度版本，
%   再让控制 ISR 真正跑 20 kHz。仿真不受此限制。

% ============ 采样周期 ============
% 电流环周期 = 控制 ISR 周期，必须等于板上实际节拍
P.Ts_i = 50e-6;         % [s] 20 kHz 设计值（实机需 single 化，见上方说明）
P.Ts_w = 500e-6;        % [s] 速度环 2 kHz
P.Ts_p = 1e-3;          % [s] 位置环 1 kHz

% ============ 电机参数 ============
P.p   = 14;             % 极对数（辨识实测 ratio=13.96~14.12，pn=14 一致）
P.Ld  = 113.5e-6;       % [H] d 轴电感
P.Lq  = 113.5e-6;       % [H] q 轴电感（表贴式，Ld=Lq）
P.psi = 7.59297995988e-3;   % [Wb] 永磁磁链

% 整定用的等效相电阻 [ohm]。
% 注意：万用表实测绕组约 0.42 ohm，但那个值含引线电阻、接触电阻与表笔误差；
% 这里取让 Ki = wc*R 与既有整定值吻合的等效值：
%   2378.18563877 / 6283.1853072 = 0.3785（恰好是整）
% 换电机时应重新辨识/测量，而不是沿用这个数。
P.R_eff = 0.3785;

% ============ 电流环（dq PI + 交叉耦合/BEMF 解耦）============
% 由带宽和电机参数派生，不再写死两个魔数（这是 VESC 的做法：
% 它的 foc_current_kp/ki 就是 detect 出 R、L 之后算出来的）。
%
%   Kp = wc * L       -> 带宽由 Kp/L 决定
%   Ki = wc * R       -> 零点 Ki/Kp = R/L，正好抵消对象极点
%
% 这样换电机只改 Ld 和 R_eff 两个物理量，不用重新整定。
P.wc_i = 2*pi*1000;          % 目标电流环带宽 1000 Hz
P.kp_i = P.wc_i * P.Ld;      % 0.713142 V/A
P.ki_i = P.wc_i * P.R_eff;   % 2378.186 V/(A*s)
P.dec_mode = 3;              % 1=交叉耦合 2=BEMF 3=两者全解耦

% 防漂移：公式化的结果必须与历史整定值一致，否则说明 Ld/R_eff 被改动过
assert(abs(P.kp_i - 0.713141532365) < 1e-9, ...
    'kp_i 与历史整定值不符：检查 Ld 或 wc_i');
assert(abs(P.ki_i - 2378.18563877) < 1e-6, ...
    'ki_i 与历史整定值不符：检查 R_eff 或 wc_i');

% ============ 速度环（PI）============
P.kp_w = 0.0729227202682;    % [A/(rad/s)]
P.ki_w = 10.1906891515;      % [A/(rad/s)/s]
P.w_max = 120;               % [rad/s] 位置环输出（速度给定）限幅

% ============ 位置环（纯 P）============
P.kp_p = 50.2654824574;      % [rad/s per rad]，约 8 Hz 带宽

% ============ 限幅与调制 ============
P.I_max     = 9;             % [A] iq 给定限幅
P.max_duty  = 0.95;          % 最大占空比
P.overmod   = 1;             % 过调制系数
P.mag_vd_max= 0.98;          % d 轴电压占可用圆的比例

% ============ 派生量 ============
% 角度变换的延时补偿：theta_e = p*theta_m + pi/2 + omega_e * T_angle_comp
% 1.5 拍 = 计算延时 1 拍 + 零阶保持 0.5 拍
% 注意：这里必须由 Ts_i 派生，不能写成常数，否则改采样率时补偿会失配
P.T_angle_comp = 1.5 * P.Ts_i;

% ============ 自洽性检查 ============
N_w = round(P.Ts_w / P.Ts_i);
N_p = round(P.Ts_p / P.Ts_i);
assert(N_w >= 1 && N_p >= N_w, 'Ts 分频关系不合理：Ts_p >= Ts_w >= Ts_i 不成立');
assert(abs(N_w*P.Ts_i - P.Ts_w) < 1e-12, 'Ts_w 必须是 Ts_i 的整数倍');
assert(abs(N_p*P.Ts_i - P.Ts_p) < 1e-12, 'Ts_p 必须是 Ts_i 的整数倍');
fprintf('[three_loop_params] Ts_i=%g s (%.1f kHz)  N_w=%d (%.2f kHz)  N_p=%d (%.2f kHz)\n', ...
    P.Ts_i, 1/P.Ts_i/1e3, N_w, 1/(N_w*P.Ts_i)/1e3, N_p, 1/(N_p*P.Ts_i)/1e3);
end
