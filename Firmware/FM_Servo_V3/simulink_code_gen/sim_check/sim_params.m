function S = sim_params()
%SIM_PARAMS  闭环 MIL 仿真参数：控制律与 three_loop_tuning.c 默认值逐项一致
%
% 控制节拍按设计 20 kHz（Ts_i=50us），分频 N_w=10 (2kHz) / N_p=20 (1kHz)。
% plant 为 Simulink 库 SPS PMSM，电气步长 10us。
% 电机参数来自截图宏（相间量/2、J、Kt->psi），与 11-14 闭环仿真一致。

S.codegen_dir = fileparts(fileparts(mfilename('fullpath')));   % simulink_code_gen/
addpath(S.codegen_dir);

% ---------------- 时序 ----------------
S.Ts_i     = 50e-6;       % 控制 ISR 20 kHz
S.Ts_w     = 500e-6;      % 速度环 2 kHz
S.Ts_p     = 1e-3;        % 位置环 1 kHz
S.Ts_plant = 10e-6;       % SPS 电气域
S.N_delay  = round(S.Ts_i / S.Ts_plant);   % PWM 更新延时 10 个电气步
S.sim_stop = 1.2;

% ---------------- 电机 / 母线 ----------------
S.Vbus = 48.0;
S.p    = 14;
S.Rs   = 0.3785;              % 相电阻 = 0.757/2
S.Ld   = 113.5e-6;           % 相电感 = 227uH/2
S.Lq   = 113.5e-6;
S.psi  = 7.59297995988e-3;
S.J    = 2.5725e-5;
S.B    = 1.0e-5;             % 假设
S.Kt   = 1.5*S.p*S.psi;     % 0.15945 N*m/A

% ---------------- 控制律参数（= THREE_LOOP_TUNING_DEFAULT + 固定项）----------------
S.par_c = [0.713141532365, 2378.18563877, S.Ts_i, S.Ld, S.Lq, S.psi, ...
           0.95, 1.0, 0.98, 3.0, 9.0];
S.par_w = [0.0729227202682, 10.1906891515, S.Ts_w, 9.0, ...
           round(S.Ts_w/S.Ts_i)];                       % N_w = 5
S.par_p = [50.2654824574, 120.0, S.Ts_p, ...
           round(S.Ts_p/S.Ts_i)];                      % N_p = 10

S.I_max     = 9.0;
S.w_max     = 120.0;
S.max_duty  = 0.95;
S.V_lim     = S.Vbus/sqrt(3)*S.max_duty;   % 26.327 V

% 角度变换
S.gain_comp = S.p * 1.5 * S.Ts_i;          % 0.00105
S.theta_off = pi/2;

% ---------------- 激励 ----------------
S.en_t   = 0.005;
S.pos_t1 = 0.050;   S.pos_a1 = pi/2;      % 场景1: 0 -> +90deg
S.pos_t2 = 0.600;                        % 场景1: 回零
S.tl_t   = 0.350;   S.tl_a   = 0.050;     % 负载扰动 0.05 N*m
S.vel_ref = 8.0;                         % 场景2: 连续旋转多圈指令 [rad/s]
end
