function [w_ref, tick_p] = fcn_three_pos_loop_single(pos_ref, theta_m, en, par)
%FCN_THREE_POS_LOOP_SINGLE  位置环 P（单精度 single 版，20 kHz ISR 内部分频 /20）
%
% 所有状态、常量、输出均为 single；ERT 生成 real32_T + 无 double 软件库调用。
% 输入在开头显式转 single，兼容闭环 MIL 仿真中 double 的 plant/激励。
% par = [kp_p, w_max, Ts_p, N_div_p]

%#codegen
persistent cnt hold_w n_exec
if isempty(cnt)
    cnt    = single(0.0);
    hold_w = single(0.0);
    n_exec = single(0.0);
end

pos_ref = single(pos_ref);
theta_m = single(theta_m);
en      = single(en);
par     = single(par);

kp    = par(1);
w_max = par(2);
n_div = par(4);

if en < single(0.5)
    hold_w = single(0.0);
else
    cnt = cnt + single(1.0);
    if cnt >= n_div
        cnt    = single(0.0);
        n_exec = n_exec + single(1.0);
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
