function [iq_ref, tick_w] = fcn_three_speed_loop_single(w_ref, w_m, en, par)
%FCN_THREE_SPEED_LOOP_SINGLE  速度环 PI（单精度 single 版，20 kHz ISR 内部分频 /10）
%
% 所有状态、常量、输出均为 single；积分步长 Ts_w=500us 不变。
% par = [kp_w, ki_w, Ts_w, i_max, N_div_w]

%#codegen
persistent i_term cnt hold_iq n_exec
if isempty(i_term)
    i_term   = single(0.0);
    cnt      = single(0.0);
    hold_iq  = single(0.0);
    n_exec   = single(0.0);
end

w_ref = single(w_ref);
w_m   = single(w_m);
en    = single(en);
par   = single(par);

kp    = par(1);
ki    = par(2);
ts    = par(3);
i_max = par(4);
n_div = par(5);

if en < single(0.5)
    i_term  = single(0.0);
    hold_iq = single(0.0);
else
    cnt = cnt + single(1.0);
    if cnt >= n_div
        cnt    = single(0.0);
        n_exec = n_exec + single(1.0);

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
