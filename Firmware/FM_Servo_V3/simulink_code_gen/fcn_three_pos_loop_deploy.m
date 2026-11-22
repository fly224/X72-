function [w_ref, tick_p] = fcn_three_pos_loop_deploy(pos_ref, theta_m, en, par)
%FCN_THREE_POS_LOOP_DEPLOY  位置环 P（部署版，单 50us ISR 内部分频到 1 kHz）
%
% MCU 部署模型运行在单一 20 kHz 节拍上；本函数每 N_div=20 拍真正计算一次，
% 其余拍保持上次输出（ZOH），因此等效执行频率 = 20 kHz / 20 = 1 kHz。
%
% par = [kp_p, w_max, Ts_p, N_div_p]
%   kp_p    [(rad/s)/rad]  = 50.265482
%   w_max   [rad/s]        位置环输出限速
%   Ts_p    [s]            1 ms（仅用于文档/核对，P 环不用 dt）
%   N_div_p 分频系数        = Ts_p/Ts_i = 20
%
% tick_p：实际执行次数（每计算一次 +1），用于在线核对 1 kHz 执行频率。

%#codegen
persistent cnt hold_w n_exec
if isempty(cnt)
    cnt    = 0.0;
    hold_w = 0.0;
    n_exec = 0.0;
end

kp    = par(1);
w_max = par(2);
n_div = par(4);

if en < 0.5
    % 关断立即清零输出，不等分频点（安全要求）
    hold_w = 0.0;
else
    cnt = cnt + 1.0;
    if cnt >= n_div
        cnt    = 0.0;
        n_exec = n_exec + 1.0;
        w = kp * (pos_ref - theta_m);
        if w > w_max
            w = w_max;
        elseif w < -w_max
            w = -w_max;
        end
        hold_w = w;
    end
end

w_ref  = hold_w;
tick_p = n_exec;
end
