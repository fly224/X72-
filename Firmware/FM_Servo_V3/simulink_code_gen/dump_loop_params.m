function dump_loop_params()
%DUMP_LOOP_PARAMS 只读：打印 three_loop_deploy.slx 里被固化的参数块
%
% 这些 Constant/Gain 的值是 build_three_loop_deploy.m 在建模时写进去的，
% 生成代码时被 InlineParams=on 直接展开成字面量。
% 所以要改"生成代码里的常数"，源头就在这几个块里。
%
% 块在子系统内部（生成代码注释里是 '<S1>/par_c'），所以用 find_system 搜。

here = fileparts(mfilename('fullpath'));
mdl  = 'three_loop_deploy';

if isempty(here)
    here = pwd;
end
load_system(fullfile(here, mdl));

names = {'par_c','par_w','par_p', ...
         'theta_off','gain_p_theta','gain_p_w','gain_comp'};

fprintf('\n===== %s.slx fixed params =====\n', mdl);
for k = 1:numel(names)
    blks = find_system(mdl, 'LookUnderMasks', 'all', 'FollowLinks', 'on', ...
                       'SearchDepth', 5, 'Name', names{k});
    if isempty(blks)
        fprintf('%-14s : <not found>\n', names{k});
        continue;
    end
    b = blks{1};
    v = '';
    try
        v = get_param(b, 'Value');          % Constant
    catch
        try
            v = get_param(b, 'Gain');       % Gain
        catch
            v = '<neither Value nor Gain>';
        end
    end
    fprintf('%-14s @ %s\n', names{k}, strrep(b, [mdl '/'], ''));
    fprintf('               = %s\n', v);
end

fprintf('\n===== cycle-related settings =====\n');
fprintf('FixedStep     = %s\n', get_param(mdl, 'FixedStep'));
fprintf('Solver        = %s\n', get_param(mdl, 'Solver'));
fprintf('==============================\n\n');

close_system(mdl, 0);
end
