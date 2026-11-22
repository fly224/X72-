function run_all_sim()
%RUN_ALL_SIM  一键：构建闭环 MIL 模型 -> 跑 5 个场景 -> 输出图/CSV/指标
% 用法：matlab -sd "<sim_check 目录>" -batch "run_all_sim"
here = fileparts(mfilename('fullpath'));
addpath(here);
res = fullfile(here, 'results');
if ~exist(res, 'dir'), mkdir(res); end

fprintf('########## 1/2 构建闭环模型 ##########\n');
build_closed_sim(here);

fprintf('\n########## 2/2 运行场景 ##########\n');
run_closed_sim(res);

fprintf('\n[DONE] 模型: %s\n', fullfile(here, 'three_loop_closed_sim.slx'));
fprintf('[DONE] 结果: %s\n', res);
end
