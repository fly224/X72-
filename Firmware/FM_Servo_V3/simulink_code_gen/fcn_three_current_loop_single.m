function [duty, iq_used, tick_i] = fcn_three_current_loop_single(ia, ib, ic, vbus, theta, omega_e, iq_ref, en, par)
%FCN_THREE_CURRENT_LOOP_SINGLE  有感 FOC 电流环（单精度 single 版，20 kHz 每拍）
%
% 与 double 版逐行一致，仅把所有状态/常量/输出改为 single。
% ERT 生成 real32_T 变量 + sinf/cosf/sqrtf，M4F 硬件 FPU 可直接执行。
% par = [kp ki dt Ld Lq psi max_duty overmod mag_vd_max dec imax]

%#codegen
persistent vd_int vq_int n_exec
if isempty(vd_int)
    vd_int = single(0.0);
    vq_int = single(0.0);
    n_exec = single(0.0);
end
n_exec = n_exec + single(1.0);
tick_i = n_exec;

ia      = single(ia);
ib      = single(ib);
ic      = single(ic);
vbus    = single(vbus);
theta   = single(theta);
omega_e = single(omega_e);
iq_ref  = single(iq_ref);
en      = single(en);
par     = single(par);

kp         = par(1);
ki         = par(2);
dt         = par(3);
Ld         = par(4);
Lq         = par(5);
psi        = par(6);
max_duty   = par(7);
overmod    = par(8);
mag_vd_max = par(9);
dec_mode   = par(10);
i_max      = par(11);

ONE_BY_SQRT3 = single(0.5773502691896258);
TWO_BY_SQRT3 = single(1.1547005383792515);

% ---- Clarke（幅值不变）----
i_alpha = ia;
i_beta  = ONE_BY_SQRT3 * ia + TWO_BY_SQRT3 * ib;

s = sin(theta);
c = cos(theta);

% ---- Park（标准）----
id = c * i_alpha + s * i_beta;
iq = c * i_beta  - s * i_alpha;

% ---- 电流给定限幅 ----
iq_cmd = iq_ref;
if iq_cmd > i_max
    iq_cmd = i_max;
elseif iq_cmd < -i_max
    iq_cmd = -i_max;
end
iq_used = iq_cmd;

vd        = single(0.0);
vq        = single(0.0);
mod_alpha = single(0.0);
mod_beta  = single(0.0);
sector    = 0.0;
duty      = single([0.5; 0.5; 0.5]);

if en < single(0.5)
    vd_int = single(0.0);
    vq_int = single(0.0);
else
    err_d = single(0.0) - id;
    err_q = iq_cmd - iq;

    % 积分先行，再比例
    vd_int = vd_int + err_d * ki * dt;
    vq_int = vq_int + err_q * ki * dt;

    vd = vd_int + err_d * kp;
    vq = vq_int + err_q * kp;

    % ---- 解耦前馈（dec=3：交叉耦合 + BEMF）----
    dec_vd   = single(0.0);
    dec_vq   = single(0.0);
    dec_bemf = single(0.0);
    if dec_mode == single(1.0) || dec_mode == single(3.0)
        dec_vd = iq * omega_e * Lq;
        dec_vq = id * omega_e * Ld;
    end
    if dec_mode == single(2.0) || dec_mode == single(3.0)
        dec_bemf = omega_e * psi;
    end
    vd = vd - dec_vd;
    vq = vq + dec_vq + dec_bemf;

    % ---- 电压圆限幅（d 轴优先，积分器同步钳位）----
    max_v_mag = ONE_BY_SQRT3 * max_duty * vbus * overmod;
    lim_d = max_v_mag * mag_vd_max;

    if vd > lim_d
        vd = lim_d;
    elseif vd < -lim_d
        vd = -lim_d;
    end
    if vd_int > lim_d
        vd_int = lim_d;
    elseif vd_int < -lim_d
        vd_int = -lim_d;
    end

    tmp = max_v_mag * max_v_mag - vd * vd;
    if tmp > single(0.0)
        max_vq = sqrt(tmp);
    else
        max_vq = single(0.0);
    end
    if vq > max_vq
        vq = max_vq;
    elseif vq < -max_vq
        vq = -max_vq;
    end
    if vq_int > max_vq
        vq_int = max_vq;
    elseif vq_int < -max_vq
        vq_int = -max_vq;
    end

    % ---- 归一化：mod = v * 1.5 / vbus ----
    vn    = single(1.5) / vbus;
    mod_d = vd * vn;
    mod_q = vq * vn;

    % ---- 反 Park（标准）----
    mod_alpha = c * mod_d - s * mod_q;
    mod_beta  = c * mod_q + s * mod_d;

    [duty, sector] = three_svm_single(mod_alpha, mod_beta, max_duty);
end
end

% =======================================================================
function [duty, sector] = three_svm_single(alpha, beta, max_mod)
%THREE_SVM_SINGLE  SVPWM 六扇区（single 版）

ONE_BY_SQRT3 = single(0.5773502691896258);
TWO_BY_SQRT3 = single(1.1547005383792515);

if beta >= single(0.0)
    if alpha >= single(0.0)
        if ONE_BY_SQRT3 * beta > alpha
            sector = 2;
        else
            sector = 1;
        end
    else
        if -ONE_BY_SQRT3 * beta > alpha
            sector = 3;
        else
            sector = 2;
        end
    end
else
    if alpha >= single(0.0)
        if -ONE_BY_SQRT3 * beta > alpha
            sector = 5;
        else
            sector = 6;
        end
    else
        if ONE_BY_SQRT3 * beta > alpha
            sector = 4;
        else
            sector = 5;
        end
    end
end

dA = single(0.5);
dB = single(0.5);
dC = single(0.5);

if sector == 1
    t1 = alpha - ONE_BY_SQRT3 * beta;
    t2 = TWO_BY_SQRT3 * beta;
    dA = (single(1.0) + t1 + t2) * single(0.5);
    dB = dA - t1;
    dC = dB - t2;
elseif sector == 2
    t2 = alpha + ONE_BY_SQRT3 * beta;
    t3 = -alpha + ONE_BY_SQRT3 * beta;
    dB = (single(1.0) + t2 + t3) * single(0.5);
    dA = dB - t3;
    dC = dA - t2;
elseif sector == 3
    t3 = TWO_BY_SQRT3 * beta;
    t4 = -alpha - ONE_BY_SQRT3 * beta;
    dB = (single(1.0) + t3 + t4) * single(0.5);
    dC = dB - t3;
    dA = dC - t4;
elseif sector == 4
    t4 = -alpha + ONE_BY_SQRT3 * beta;
    t5 = -TWO_BY_SQRT3 * beta;
    dC = (single(1.0) + t4 + t5) * single(0.5);
    dB = dC - t5;
    dA = dB - t4;
elseif sector == 5
    t5 = -alpha - ONE_BY_SQRT3 * beta;
    t6 = alpha - ONE_BY_SQRT3 * beta;
    dC = (single(1.0) + t5 + t6) * single(0.5);
    dA = dC - t5;
    dB = dA - t6;
else
    t6 = -TWO_BY_SQRT3 * beta;
    t1 = alpha + ONE_BY_SQRT3 * beta;
    dA = (single(1.0) + t1 + t6) * single(0.5);
    dC = dA - t1;
    dB = dC - t6;
end

d_max = (single(1.0) + max_mod) * single(0.5);
if dA > d_max
    dA = d_max;
elseif dA < single(0.0)
    dA = single(0.0);
end
if dB > d_max
    dB = d_max;
elseif dB < single(0.0)
    dB = single(0.0);
end
if dC > d_max
    dC = d_max;
elseif dC < single(0.0)
    dC = single(0.0);
end

duty = [dA; dB; dC];
end
