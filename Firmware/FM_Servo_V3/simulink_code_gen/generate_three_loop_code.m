function R = generate_three_loop_code(outdir)
%GENERATE_THREE_LOOP_CODE  为 STM32G474 生成 ERT C 代码并做交付核验
%
% 用法：
%   generate_three_loop_code
%   generate_three_loop_code('G:\FOC\X72\Firmware\FM_Servo_V3\simulink_code_gen')
%
% 流程：
%   1) build_three_loop_deploy 构建部署模型（单 50us ISR + 内部分频）
%   2) 复制模型/脚本/控制律到目标目录
%   3) 用 ert.tlc + ARM Cortex-M 目标 slbuild 生成代码（Generate code only）
%   4) 把核心 .c/.h 平铺到目标目录根（便于 Keil/MDK 直接添加）
%   5) 导出 SIL 比对用的输入/期望输出 CSV
%   6) 核验生成文件、接口字段、无 SPS 依赖，写 model_io_check.txt

if nargin < 1 || isempty(outdir)
    outdir = 'G:\FOC\X72\Firmware\FM_Servo_V3\simulink_code_gen';
end
here   = fileparts(mfilename('fullpath'));
parent = fileparts(here);
addpath(parent); addpath(here);
if ~exist(outdir, 'dir')
    mkdir(outdir);
end

mdl = 'three_loop_deploy';
P   = three_loop_params();

fprintf('=== 目标路径: %s ===\n', outdir);
% 清理交付目录中的旧模型副本：同名 .slx 在 MATLAB 路径上会遮蔽工作区模型
old_slx  = fullfile(outdir, [mdl '.slx']);
old_slxc = fullfile(outdir, [mdl '.slxc']);
if exist(old_slx,  'file'), delete(old_slx);  end
if exist(old_slxc, 'file'), delete(old_slxc); end

fprintf('--- 1/5 构建部署模型 ---\n');
build_three_loop_deploy(here);
slx = fullfile(here, [mdl '.slx']);

% ---- 先复制脚本（slx 等生成完成后再复制，避免 shadowing）----
srcs = {'build_three_loop_deploy.m', 'generate_three_loop_code.m', ...
        'fcn_three_pos_loop_deploy.m', 'fcn_three_speed_loop_deploy.m', ...
        'fcn_three_current_loop_deploy.m', ...
        'fcn_three_pos_loop_single.m', 'fcn_three_speed_loop_single.m', ...
        'fcn_three_current_loop_single.m'};
for k = 1:numel(srcs)
    s = fullfile(here, srcs{k});
    d = fullfile(outdir, srcs{k});
    if ~strcmp(s, d)          % 从交付目录自身运行时跳过同源复制
        copyfile(s, d, 'f');
    end
end
fprintf('[OK] 构建脚本与控制律已复制 (%d 个)\n', numel(srcs));

% ---- 代码生成 ----
fprintf('--- 2/5 slbuild (ert.tlc / ARM Cortex-M) ---\n');
if bdIsLoaded(mdl), close_system(mdl, 0); end
load_system(slx);
try
    set_param(mdl, 'GenCodeOnly', 'on');
catch
    fprintf('[WARN] GenCodeOnly 参数未生效，改由 slbuild 处理\n');
end
olddir = cd(outdir);
try
    try
        Simulink.fileGenControl('set', 'CodeGenFolder', outdir, 'createDir', true);
    catch
        fprintf('[WARN] fileGenControl 未生效，使用当前目录生成\n');
    end
    slbuild(mdl);
catch e
    cd(olddir);
    rethrow(e);
end
cd(olddir);

gendir = fullfile(outdir, [mdl '_ert_rtw']);
core_c = fullfile(gendir, [mdl '.c']);
core_h = fullfile(gendir, [mdl '.h']);
assert(exist(core_c, 'file') == 2 && exist(core_h, 'file') == 2, ...
       '代码生成未产出主文件: %s', gendir);
fprintf('[OK] 代码生成: %s\n', gendir);
% 生成完成后再复制模型副本到交付目录
if ~strcmp(slx, fullfile(outdir, [mdl '.slx']))
    copyfile(slx, fullfile(outdir, [mdl '.slx']), 'f');
end

% ---- 平铺核心源码（排除 ert_main.c 示例，避免与固件 main 冲突）----
fprintf('--- 3/5 平铺核心源码 ---\n');
cfiles = dir(fullfile(gendir, '*.c'));
hfiles = dir(fullfile(gendir, '*.h'));
nflat = 0;
allf = [cfiles; hfiles];
for k = 1:numel(allf)
    if strcmp(allf(k).name, 'ert_main.c')
        continue;
    end
    copyfile(fullfile(gendir, allf(k).name), fullfile(outdir, allf(k).name), 'f');
    nflat = nflat + 1;
end
fprintf('[OK] 平铺 %d 个 .c/.h 到目标根（已排除 ert_main.c）\n', nflat);

% ---- SIL 比对数据 ----
fprintf('--- 4/5 导出 SIL 输入/期望输出 ---\n');
try
    export_sil_data(mdl, P, outdir);
    fprintf('[OK] sil_input.csv / sil_expected.csv\n');
catch e
    fprintf('[WARN] SIL 数据导出跳过: %s\n', e.message);
end

% ---- 接口与依赖核验 ----
fprintf('--- 5/5 接口核验 ---\n');
R = check_generated(outdir, gendir, mdl, P, nflat);

fprintf('\n=== 完成 ===\n');
fprintf('模型   : %s\n', fullfile(outdir, [mdl '.slx']));
fprintf('源码   : %s (根目录平铺 %d 个 .c/.h)\n', outdir, nflat);
fprintf('完整包 : %s\n', gendir);
fprintf('核验   : %s\n', fullfile(outdir, 'model_io_check.txt'));

% SIL 仿真会改模型配置（ExternalInput 等）使其 dirty；用 bdclose 强制丢弃，
% 不保存（保存会把 SIL 配置带进部署模型），也避免 MATLAB 退出时报"无法关闭模型"。
if bdIsLoaded(mdl), bdclose(mdl); end
end

% =======================================================================
function export_sil_data(mdl, P, outdir)
% 用同一组确定性激励跑部署模型，导出 step() 输入与期望 duty，供 host C 程序比对
n = 3000;                              % 3000 拍；Ts_i=50us 时为 0.15 s
t = (0:n-1)' * P.Ts_i;
ia = 0.8*sin(2*pi*50*t);
ib = 0.8*sin(2*pi*50*t - 2*pi/3);
ic = -ia - ib;
th = 0.15*sin(2*pi*3*t);
wm = 0.15*2*pi*3*cos(2*pi*3*t);
u = [ia, ib, ic, 48.0*ones(n,1), th, wm, 1.0*ones(n,1), ones(n,1)];

% 三个参数向量 Inport 也由 ExternalInput 提供（向量端口按列展开）。
% 值与 three_loop_tuning.c 的 THREE_LOOP_TUNING_DEFAULT 必须完全一致。
par_c = [P.kp_i, P.ki_i, P.Ts_i, P.Ld, P.Lq, P.psi, ...
         P.max_duty, P.overmod, P.mag_vd_max, P.dec_mode, P.I_max];
par_w = [P.kp_w, P.ki_w, P.Ts_w, P.I_max, round(P.Ts_w/P.Ts_i)];
par_p = [P.kp_p, P.w_max, P.Ts_p, round(P.Ts_p/P.Ts_i)];
uext = [u, repmat(par_c,n,1), repmat(par_w,n,1), repmat(par_p,n,1)];
% 根 Inport 已改为 single：矩阵型 ExternalInput 会类型不匹配，改用 Dataset +
% single timeseries（按 Inport 名称/顺序绑定）。uext 仅用于说明列顺序。
in_names = {'ia','ib','ic','vbus','theta_m','omega_m','pos_ref','en', ...
            'par_c','par_w','par_p'};
in_cols  = {ia, ib, ic, 48.0*ones(n,1), th, wm, ones(n,1), ones(n,1), ...
            repmat(par_c,n,1), repmat(par_w,n,1), repmat(par_p,n,1)};
ds_ext = Simulink.SimulationData.Dataset();
for k = 1:numel(in_names)
    ds_ext = ds_ext.addElement( ...
        timeseries(single(in_cols{k}), t, 'Name', in_names{k}), in_names{k});
end
assignin('base', 'sil_ext', ds_ext);

set_param(mdl, ...
    'LoadExternalInput', 'on', ...
    'ExternalInput',     'sil_ext', ...
    'StopTime',          sprintf('%.12g', t(end)), ...
    'SaveOutput',        'on', ...
    'SaveTime',          'off');
out = sim(mdl);

% 提取 duty（第 1 个根输出端口，3x1）
d = nan(n,3);
try
    ds = out.yout;
    for k = 1:numel(ds)
        sig = ds{k}.Values.Data;
        if ndims(sig) == 3 && size(sig,1) == 3 && size(sig,3) >= n
            d = squeeze(sig(:,1,1:n)).';       % [3 x 1 x N] -> [N x 3]
            break
        elseif size(sig,2) == 3 && size(sig,1) >= n
            d = sig(1:n,:);
            break
        elseif size(sig,1) == 3 && size(sig,2) >= n
            d = sig(:,1:n).';
            break
        end
    end
catch
end
assert(all(isfinite(d(:))), 'SIL 期望 duty 提取失败');

fi = fopen(fullfile(outdir, 'sil_input.csv'), 'w');
fprintf(fi, '%.12g,%.12g,%.12g,%.12g,%.12g,%.12g,%.12g,%.12g\n', u');
fclose(fi);
fe = fopen(fullfile(outdir, 'sil_expected.csv'), 'w');
fprintf(fe, '%.12g,%.12g,%.12g\n', d');
fclose(fe);

% 复位为部署状态（避免下次仿真误用外部输入）
set_param(mdl, 'LoadExternalInput', 'off', 'StopTime', '1');
end

% =======================================================================
function R = check_generated(outdir, gendir, mdl, P, nflat)
h = fileread(fullfile(outdir, [mdl '.h']));
c = fileread(fullfile(outdir, [mdl '.c']));

% 1) 入口符号
R.has_init = ~isempty(regexp(h, [mdl '_initialize\s*\('], 'once'));
R.has_step = ~isempty(regexp(h, [mdl '_step\s*\('], 'once'));
R.has_U    = ~isempty(regexp(h, ['ExtU_' mdl '_T'], 'once'));
R.has_Y    = ~isempty(regexp(h, ['ExtY_' mdl '_T'], 'once'));

% 2) 输入/输出字段
in_names  = {'ia','ib','ic','vbus','theta_m','omega_m','pos_ref','en', ...
             'par_c','par_w','par_p'};
out_names = {'duty','iq_ref','w_ref','iq_used','ticks'};
R.in_found  = cellfun(@(s) ~isempty(regexp(h, ['real(32_)?T\s+' s '\s*[;\[]'], 'once')), in_names);
R.out_found = cellfun(@(s) ~isempty(regexp(h, ['real(32_)?T\s+' s '\s*[;\[]'], 'once')), out_names);
R.in_names  = in_names;
R.out_names = out_names;

% 3) 不含被控对象依赖
R.no_sps = isempty(regexpi(c, 'powergui|PermanentMagnet|pmsm|ControlledVoltage', 'once'));

% 4) 代码规模与状态
R.n_lines_c = numel(regexp(c, '\n', 'split'));
R.n_lines_h = numel(regexp(h, '\n', 'split'));
R.nflat     = nflat;

% 5) PID 参数以 Inport 对外可调（不再固化成字面量）
R.tunable = ~isempty(regexp(h, 'real(32_)?T\s+par_c\s*\[', 'once')) && ...
            ~isempty(regexp(h, 'real(32_)?T\s+par_w\s*\[', 'once')) && ...
            ~isempty(regexp(h, 'real(32_)?T\s+par_p\s*\[', 'once'));
R.fixed_pid_literals = ~isempty(regexp(c, '0\.713141532365|2378\.18563877', 'once'));
% 单精度核验：接口为 real32_T，且生成代码调用单精度数学函数
R.is_single   = ~isempty(regexp(h, 'real32_T\s+ia', 'once')) && ...
                isempty(regexp(h, 'real_T\s+ia', 'once'));
R.single_math = ~isempty(regexp(c, 'sinf\s*\(|cosf\s*\(|sqrtf\s*\(', 'once'));

% 6) 模型端口与生成结构体字段数量一致性
R.n_in  = numel(in_names);
R.n_out = numel(out_names);

f = fullfile(outdir, 'model_io_check.txt');
fid = fopen(f, 'w', 'n', 'UTF-8');
fwrite(fid, [239 187 191], 'uint8');
fprintf(fid, 'STM32G474 ERT 代码生成核验（%s）\n', datestr(now,'yyyy-mm-dd HH:MM:SS'));
fprintf(fid, '模型: %s.slx | 目标: ert.tlc / ARM Compatible->ARM Cortex-M\n', mdl);
fprintf(fid, '生成目录: %s\n', gendir);
fprintf(fid, '--------------------------------------------------------------\n');
fprintf(fid, '[%s] %s_initialize 声明\n', onoff(R.has_init), mdl);
fprintf(fid, '[%s] %s_step 声明\n',       onoff(R.has_step), mdl);
fprintf(fid, '[%s] ExtU_%s_T / ExtY_%s_T 结构体\n', onoff(R.has_U && R.has_Y), mdl, mdl);
for k = 1:numel(in_names)
    fprintf(fid, '[%s] 输入字段 U.%s\n', onoff(R.in_found(k)), in_names{k});
end
for k = 1:numel(out_names)
    fprintf(fid, '[%s] 输出字段 Y.%s\n', onoff(R.out_found(k)), out_names{k});
end
fprintf(fid, '[%s] 生成代码不含 SPS 被控对象依赖\n', onoff(R.no_sps));
fprintf(fid, '[%s] PID 参数以 Inport 对外可调 (U.par_c/par_w/par_p)\n', onoff(R.tunable));
fprintf(fid, '[%s] 生成 C 中已无固定 PID 字面量\n', onoff(~R.fixed_pid_literals));
fprintf(fid, '[%s] 接口为单精度 real32_T（非 double）\n', onoff(R.is_single));
fprintf(fid, '[%s] 生成代码使用 sinf/cosf/sqrtf（M4F 硬件 FPU）\n', onoff(R.single_math));
fprintf(fid, '--------------------------------------------------------------\n');
fprintf(fid, '主代码行数: %s.c = %d, %s.h = %d；平铺 .c/.h = %d 个\n', ...
        mdl, R.n_lines_c, mdl, R.n_lines_h, nflat);
fprintf(fid, '时序: step() @ %g us (%.0f Hz)；位置环 /%d -> %.0f Hz；速度环 /%d -> %.0f Hz\n', ...
        P.Ts_i*1e6, 1/P.Ts_i, round(P.Ts_p/P.Ts_i), 1/P.Ts_p, ...
        round(P.Ts_w/P.Ts_i), 1/P.Ts_w);
fprintf(fid, '角度: theta_e = p*theta_m + pi/2 + omega_e*1.5*Ts_i, p=%d, gain_comp=%.6g\n', ...
        P.p, P.p*P.T_angle_comp);
fprintf(fid, '数据类型: 单精度 real32_T（float）；M4F 硬件 FPU 可直接执行，实机单拍预算需 DWT 复测\n');
fclose(fid);

fprintf('init=%d step=%d U/Y=%d 输入=%d/%d 输出=%d/%d 无SPS=%d 可调=%d single=%d 单精度数学=%d\n', ...
    R.has_init, R.has_step, R.has_U && R.has_Y, ...
    sum(R.in_found), numel(in_names), sum(R.out_found), numel(out_names), ...
    R.no_sps, R.tunable, R.is_single, R.single_math);
end

function s = onoff(v)
if v, s = 'PASS'; else, s = 'FAIL'; end
end
