function [iq_ref, tick_w] = fcn_three_speed_loop_deploy(w_ref, w_m, en, par)
%FCN_THREE_SPEED_LOOP_DEPLOY  速度环 PI（部署版，单 50us ISR 内部分频到 2 kHz）
%
% 每 N_div=10 拍真正计算一次（等效 2 kHz），其余拍保持输出。
% PI 积分步长用 Ts_w = 500 us，与仿真模型的 2 kHz 速度环完全一致。
%
% par = [kp_w, ki_w, Ts_w, i_max, N_div_w]
%   kp_w [A/(rad/s)] = 0.072923
%   ki_w [A/(rad/s)/s] = 10.190689
%   Ts_w = 500 us
%   i_max = 9 A
%   N_div_w = Ts_w/Ts_i = 10
%
% tick_w：实际执行次数，用于在线核对 2 kHz 执行频率。

%#codegen
persistent i_term cnt hold_iq n_exec
if isempty(i_term)
    i_term   = 0.0;
    cnt      = 0.0;
    hold_iq  = 0.0;
    n_exec   = 0.0;
end

kp    = par(1);
ki    = par(2);
ts    = par(3);
i_max = par(4);
n_div = par(5);

if en < 0.5
    i_term  = 0.0;
    hold_iq = 0.0;
else
    cnt = cnt + 1.0;
    if cnt >= n_div
        cnt    = 0.0;
        n_exec = n_exec + 1.0;

        err = w_ref - w_m;
        i_term = i_term + ki * err * ts;
        if i_term > i_max
            i_term = i_max;
        elseif i_term < -i_max
            i_term = -i_max;
        end

        out = kp * err + i_term;
        if out > i_max
            out = i_max;
        elseif out < -i_max
            out = -i_max;
        end
        hold_iq = out;
    end
end

iq_ref = hold_iq;
tick_w = n_exec;
end
