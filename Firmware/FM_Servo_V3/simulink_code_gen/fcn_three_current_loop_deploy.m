function [duty, iq_used, tick_i] = fcn_three_current_loop_deploy(ia, ib, ic, vbus, theta, omega_e, iq_ref, en, par)
%FCN_THREE_CURRENT_LOOP_DEPLOY  有感 FOC 电流环（部署版，20 kHz 每拍执行）
%
% 与仿真模型 three_loop_pmsm_sim 的电流环逐行一致：
%   Clarke(幅值不变) -> Park -> dq PI(积分先行) -> 交叉耦合/BEMF 全解耦
%   -> 电压圆限幅(d 优先) -> 归一化 -> 反 Park -> SVPWM 六扇区 -> 三相占空比
%
% par = [kp ki dt Ld Lq psi max_duty overmod mag_vd_max dec imax]
%   kp=0.713142 V/A, ki=2378.186 V/(A*s), dt=50us, Ld=Lq=113.5uH,
%   psi=7.593mWb, max_duty=0.95, overmod=1, mag_vd_max=0.98, dec=3, imax=9A
%
% 输出 duty 为 3x1 向量 [da db dc] ∈ [0,1]，可直接映射到 TIM1 CCR。
% deploy 版不输出 diagv 向量（避免 ERT 类型推导问题），仅保留 iq_used/tick_i。

%#codegen
persistent vd_int vq_int n_exec
if isempty(vd_int)
    vd_int = 0.0;
    vq_int = 0.0;
    n_exec = 0.0;
end
n_exec = n_exec + 1.0;
tick_i = n_exec;

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

ONE_BY_SQRT3 = 0.5773502691896258;
TWO_BY_SQRT3 = 1.1547005383792515;

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

vd        = 0.0;
vq        = 0.0;
mod_alpha = 0.0;
mod_beta  = 0.0;
sector    = 0.0;
duty      = [0.5; 0.5; 0.5];

if en < 0.5
    vd_int = 0.0;
    vq_int = 0.0;
else
    err_d = 0.0 - id;
    err_q = iq_cmd - iq;

    % 积分先行，再比例
    vd_int = vd_int + err_d * ki * dt;
    vq_int = vq_int + err_q * ki * dt;

    vd = vd_int + err_d * kp;
    vq = vq_int + err_q * kp;

    % ---- 解耦前馈（dec=3：交叉耦合 + BEMF）----
    dec_vd   = 0.0;
    dec_vq   = 0.0;
    dec_bemf = 0.0;
    if dec_mode == 1 || dec_mode == 3
        dec_vd = iq * omega_e * Lq;
        dec_vq = id * omega_e * Ld;
    end
    if dec_mode == 2 || dec_mode == 3
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
    if tmp > 0.0
        max_vq = sqrt(tmp);
    else
        max_vq = 0.0;
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
    vn    = 1.5 / vbus;
    mod_d = vd * vn;
    mod_q = vq * vn;

    % ---- 反 Park（标准）----
    mod_alpha = c * mod_d - s * mod_q;
    mod_beta  = c * mod_q + s * mod_d;

    [duty, sector] = three_svm_deploy(mod_alpha, mod_beta, max_duty);
end
end

% =======================================================================
function [duty, sector] = three_svm_deploy(alpha, beta, max_mod)
%THREE_SVM_DEPLOY  SVPWM 六扇区 + 归一化占空比

ONE_BY_SQRT3 = 0.5773502691896258;
TWO_BY_SQRT3 = 1.1547005383792515;

if beta >= 0.0
    if alpha >= 0.0
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
    if alpha >= 0.0
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

dA = 0.5;
dB = 0.5;
dC = 0.5;

if sector == 1
    t1 = alpha - ONE_BY_SQRT3 * beta;
    t2 = TWO_BY_SQRT3 * beta;
    dA = (1.0 + t1 + t2) * 0.5;
    dB = dA - t1;
    dC = dB - t2;
elseif sector == 2
    t2 = alpha + ONE_BY_SQRT3 * beta;
    t3 = -alpha + ONE_BY_SQRT3 * beta;
    dB = (1.0 + t2 + t3) * 0.5;
    dA = dB - t3;
    dC = dA - t2;
elseif sector == 3
    t3 = TWO_BY_SQRT3 * beta;
    t4 = -alpha - ONE_BY_SQRT3 * beta;
    dB = (1.0 + t3 + t4) * 0.5;
    dC = dB - t3;
    dA = dC - t4;
elseif sector == 4
    t4 = -alpha + ONE_BY_SQRT3 * beta;
    t5 = -TWO_BY_SQRT3 * beta;
    dC = (1.0 + t4 + t5) * 0.5;
    dB = dC - t5;
    dA = dB - t4;
elseif sector == 5
    t5 = -alpha - ONE_BY_SQRT3 * beta;
    t6 = alpha - ONE_BY_SQRT3 * beta;
    dC = (1.0 + t5 + t6) * 0.5;
    dA = dC - t5;
    dB = dA - t6;
else
    t6 = -TWO_BY_SQRT3 * beta;
    t1 = alpha + ONE_BY_SQRT3 * beta;
    dA = (1.0 + t1 + t6) * 0.5;
    dC = dA - t1;
    dB = dC - t6;
end

d_max = (1.0 + max_mod) * 0.5;
if dA > d_max
    dA = d_max;
elseif dA < 0.0
    dA = 0.0;
end
if dB > d_max
    dB = d_max;
elseif dB < 0.0
    dB = 0.0;
end
if dC > d_max
    dC = d_max;
elseif dC < 0.0
    dC = 0.0;
end

duty = [dA; dB; dC];
end
