function p = fcn_wrap_ref(pos_ref_raw, theta_m, wrap_en, dir_sign)
%FCN_WRAP_REF  位置指令坐标系折叠（与 three_loop_tuning.c 的 WrapPosRefEx 一致）
%  wrap_en = 0 时不折叠（复现文档 §3.3 的跨 2pi 过流 bug）
%  wrap_en = 1 时把多圈 pos_ref 折到 theta_m 所在圈，误差 = wrap_pi(真实误差)

%#codegen
TWO_PI = 6.28318530717958647692;

if wrap_en < 0.5
    p = pos_ref_raw;
else
    ds = dir_sign;
    if ds >= 0.0
        ds = 1.0;
    else
        ds = -1.0;
    end
    th = ds * theta_m;
    e  = ds * (pos_ref_raw - theta_m);
    e  = e - TWO_PI * floor(e * (1.0/TWO_PI) + 0.5);
    p  = th + e;
end
end
