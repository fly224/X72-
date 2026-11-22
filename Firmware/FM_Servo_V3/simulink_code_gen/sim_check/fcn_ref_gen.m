function pos_ref = fcn_ref_gen(t, scenario, par)
%FCN_REF_GEN  位置指令发生器
%  scenario = 1: 位置阶跃 0 -> +90deg @0.05s，0.60s 回零
%  scenario = 2: 连续旋转多圈累积指令 pos_ref = vel_ref * t（速度模式，用于跨 2pi 验证）
%  par = [pos_t1, pos_a1, pos_t2, vel_ref]

%#codegen
pos_ref = 0.0;

if scenario < 1.5
    if t >= par(1)
        pos_ref = pos_ref + par(2);
    end
    if t >= par(3)
        pos_ref = pos_ref - par(2);
    end
else
    pos_ref = par(4) * t;
end
end
