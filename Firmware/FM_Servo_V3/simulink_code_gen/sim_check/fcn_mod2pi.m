function th = fcn_mod2pi(theta_raw)
%FCN_MOD2PI  把 SPS PMSM 连续累积的机械角折成单圈 0..2pi，
% 模拟真实 MT6835 21bit 绝对编码器（BSP 层会把 theta 折到单圈）。
% p=14 为整数，14*theta 在跳变点相差 28pi = 14*2pi，电角连续，不影响 Park。

%#codegen
TWO_PI = 6.28318530717958647692;
th = mod(theta_raw, TWO_PI);
end
