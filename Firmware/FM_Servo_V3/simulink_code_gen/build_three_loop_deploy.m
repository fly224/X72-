function build_three_loop_deploy(outdir)
%BUILD_THREE_LOOP_DEPLOY  构建 STM32G474 部署模型 three_loop_deploy.slx
%
% 生成边界：顶层 = Inport -> ThreeLoop_Controller(原子子系统) -> Outport
%   只含三环控制律，不含 SPS PMSM / 受控电压源 / 平均逆变器 / powergui /
%   激励源 / Scope / To Workspace（这些属于被控对象或仿真激励，不能进 MCU）。
%
% 时序策略（面向裸机单 ISR，避免 ERT 多速率代码生成问题）：
%   模型单速率 50 us（20 kHz），由 TIM1 更新中断调用 step()；
%   位置环 chart 内每 20 拍算一次 -> 等效 1 kHz；
%   速度环 chart 内每 10 拍算一次 -> 等效 2 kHz；
%   电流环每拍执行 -> 20 kHz。
%   三个 chart 各输出 tick 计数，可在实机在线核对执行频率。
%
% 输入（8）：ia, ib, ic [A], vbus [V], theta_m [rad 机械], omega_m [rad/s 机械],
%            pos_ref [rad 机械], en
% 输出（5）：duty[3] [0,1], iq_ref [A], w_ref [rad/s], iq_used [A], ticks[3]
%
% 角度变换在控制器内完成（真实 MCU 也在此处做）：
%   omega_e = p*omega_m
%   theta_e = p*theta_m + pi/2 + omega_e*1.5*Ts_i   （+90° 库模型对齐 + 延时补偿）

if nargin < 1 || isempty(outdir)
    outdir = fileparts(mfilename('fullpath'));
end
if ~exist(outdir, 'dir')
    mkdir(outdir);
end
addpath(fileparts(fileparts(mfilename('fullpath'))));   % three_loop_params.m

mdl = 'three_loop_deploy';
P   = three_loop_params();

if bdIsLoaded(mdl), close_system(mdl, 0); end
slx = fullfile(outdir, [mdl '.slx']);
if exist(slx, 'file'), delete(slx); end
new_system(mdl); load_system(mdl);

% ---------------- 求解器 / 目标 ----------------
set_param(mdl, ...
    'SolverType', 'Fixed-step', ...
    'Solver',     'FixedStepDiscrete', ...
    'FixedStep',  lit(P.Ts_i), ...
    'StopTime',   '1', ...
    'StartTime',  '0', ...
    'SaveOutput', 'on', ...
    'SaveTime',   'off', ...
    'SystemTargetFile', 'ert.tlc', ...
    'TargetLang',       'C', ...
    'GenerateReport',   'on', ...
    'InlineParams',     'on', ...
    'ProdHWDeviceType', 'ARM Compatible->ARM Cortex-M', ...
    'GenCodeOnly',      'on');

% 以下为可选 ERT 优化项，不同版本参数名可能不同，逐个 try
try_set(mdl, 'CodeInterfacePackaging', 'Nonreusable function');
try_set(mdl, 'SupportNonFinite',       'off');
try_set(mdl, 'SupportComplex',         'off');
try_set(mdl, 'SupportAbsoluteTime',    'off');
try_set(mdl, 'SupportVariableSizeSignals', 'off');
try_set(mdl, 'GenerateSampleERTMain',  'off');
try_set(mdl, 'GenerateMakefile',       'off');
try_set(mdl, 'MatFileLogging',         'off');
try_set(mdl, 'IncludeMdlTerminateFcn', 'off');
try_set(mdl, 'CombineOutputUpdateFcns','on');
try_set(mdl, 'SuppressErrorStatus',    'on');
try_set(mdl, 'ERTFilePackagingFormat', 'Modular');
try_set(mdl, 'PreserveBusStructures',  'off');
try_set(mdl, 'ProdEndianess',          'LittleEndian');
try_set(mdl, 'ProdWordSize',           '32');

% ---------------- 顶层 ----------------
hCtrl = [mdl '/ThreeLoop_Controller'];
add_block('built-in/Subsystem', hCtrl, 'Position', [220 20 720 620]);
set_param(hCtrl, 'TreatAsAtomicUnit', 'on');

in_names  = {'ia','ib','ic','vbus','theta_m','omega_m','pos_ref','en', ...
             'par_c','par_w','par_p'};   % 后 3 个 = PID 参数对外接口（运行时可调）
out_names = {'duty','iq_ref','w_ref','iq_used','ticks'};
for k = 1:numel(in_names)
    blk = [mdl '/' in_names{k}];
    add_block('simulink/Ports & Subsystems/In1', blk, ...
        'Position', [20 30+(k-1)*55 50 50+(k-1)*55]);
    set_param(blk, 'OutDataTypeStr', 'single');   % 单精度接口
end
% 参数向量端口必须显式给维度，否则 ERT 无法推导端口宽度
set_param([mdl '/par_c'], 'PortDimensions', '11');
set_param([mdl '/par_w'], 'PortDimensions', '5');
set_param([mdl '/par_p'], 'PortDimensions', '4');
for k = 1:numel(out_names)
    add_block('simulink/Ports & Subsystems/Out1', [mdl '/' out_names{k}], ...
        'Position', [780 30+(k-1)*70 810 50+(k-1)*70]);
end

build_controller(hCtrl, P);

L = @(src, dst) add_line(mdl, src, dst, 'autorouting', 'on');
for k = 1:numel(in_names)
    L(sprintf('%s/1', in_names{k}), sprintf('ThreeLoop_Controller/%d', k));
end
for k = 1:numel(out_names)
    L(sprintf('ThreeLoop_Controller/%d', k), sprintf('%s/1', out_names{k}));
end

save_system(mdl, slx);
fprintf('[OK] deploy model saved: %s\n', slx);
end

% =======================================================================
function build_controller(sys, P)
clear_default(sys);
ts = lit(P.Ts_i);

mk_in(sys, {'ia','ib','ic','vbus','theta_m','omega_m','pos_ref','en', ...
            'par_c','par_w','par_p'});
mk_out(sys, {'duty','iq_ref','w_ref','iq_used','ticks'});

% 参数端口维度，与 three_loop_tuning.h 的展开顺序一一对应
set_param([sys '/par_c'], 'PortDimensions', '11');
set_param([sys '/par_w'], 'PortDimensions', '5');
set_param([sys '/par_p'], 'PortDimensions', '4');

N_p = round(P.Ts_p / P.Ts_i);
N_w = round(P.Ts_w / P.Ts_i);
fprintf('[build] Ts_i=%g s -> N_w=%d (%.2f kHz), N_p=%d (%.2f kHz)\n', ...
        P.Ts_i, N_w, 1/(N_w*P.Ts_i)/1e3, N_p, 1/(N_p*P.Ts_i)/1e3);

% ---- 参数不再固化为 Constant：由顶层 Inport 提供，运行时经 database 可调 ----
%   par_c = [kp_i ki_i dt Ld Lq psi max_duty overmod mag_vd_max dec i_max]  (11)
%   par_w = [kp_w ki_w Ts_w i_max N_w]                                      (5)
%   par_p = [kp_p w_max Ts_p N_p]                                           (4)
%   默认值由 three_loop_tuning.c 的 ThreeLoop_Tuning_Init() 写入 U；
%   周期相关字段(dt/Ts_w/Ts_p/N_w/N_p)由 Init 按 Ts_i 派生，禁止上位机改写。

% ---- 编码器角度/速度换算（全部 single）----
mk([sys '/gain_p_theta'], 'simulink/Math Operations/Gain', [120 30 190 60], ...
    'Gain', sprintf('%.12g', P.p));
mk([sys '/gain_p_w'],     'simulink/Math Operations/Gain', [120 80 190 110], ...
    'Gain', sprintf('%.12g', P.p));
mk([sys '/gain_comp'],    'simulink/Math Operations/Gain', [120 130 190 160], ...
    'Gain', sprintf('%.17g', single(P.p * P.T_angle_comp)), ...
    'ParamDataTypeStr', 'single', 'OutDataTypeStr', 'single');  % 给可精确表示的 single 值，避免量化诊断
mk([sys '/theta_off'],    'simulink/Sources/Constant', [120 180 210 210], ...
    'Value', sprintf('%.17g', single(pi/2)), 'SampleTime', 'inf', ...
    'OutDataTypeStr', 'single');   % 直接给可精确表示的单精度值，避免量化诊断
add_block('simulink/Math Operations/Sum', [sys '/theta_sum'], ...
    'Position', [250 40 275 130], 'Inputs', '+++');
% Gain 强制 single 参数/输出，避免 double 常量把角度链提升成双精度
gb = {'gain_p_theta','gain_p_w','gain_comp'};
for k = 1:numel(gb)
    try_setb([sys '/' gb{k}], 'ParamDataTypeStr', 'single');
    try_setb([sys '/' gb{k}], 'OutDataTypeStr',   'single');
end

% ---- 三个环（single 版 chart，50 us 基节拍 + 内部分频）----
add_block('simulink/User-Defined Functions/MATLAB Function', [sys '/Position_Loop'], ...
    'Position', [340 30 480 130]);
set_chart([sys '/Position_Loop'], 'fcn_three_pos_loop_single.m', ts);

add_block('simulink/User-Defined Functions/MATLAB Function', [sys '/Speed_Loop'], ...
    'Position', [540 30 680 130]);
set_chart([sys '/Speed_Loop'], 'fcn_three_speed_loop_single.m', ts);

add_block('simulink/User-Defined Functions/MATLAB Function', [sys '/Current_Loop'], ...
    'Position', [740 160 900 330]);
set_chart([sys '/Current_Loop'], 'fcn_three_current_loop_single.m', ts);

mk([sys '/mux_ticks'], 'simulink/Signal Routing/Mux', [740 400 745 470], 'Inputs', '3');

L = @(src, dst) add_line(sys, src, dst, 'autorouting', 'on');

% 角度链
L('theta_m/1', 'gain_p_theta/1');
L('omega_m/1', 'gain_p_w/1');
L('omega_m/1', 'gain_comp/1');
L('gain_p_theta/1', 'theta_sum/1');
L('theta_off/1',    'theta_sum/2');
L('gain_comp/1',    'theta_sum/3');

% 位置环（1 kHz，N=20）
L('pos_ref/1', 'Position_Loop/1');
L('theta_m/1', 'Position_Loop/2');
L('en/1',      'Position_Loop/3');
L('par_p/1',   'Position_Loop/4');
L('Position_Loop/1', 'w_ref/1');
L('Position_Loop/1', 'Speed_Loop/1');
L('Position_Loop/2', 'mux_ticks/3');

% 速度环（2 kHz，N=10）
L('omega_m/1', 'Speed_Loop/2');
L('en/1',      'Speed_Loop/3');
L('par_w/1',   'Speed_Loop/4');
L('Speed_Loop/1', 'iq_ref/1');
L('Speed_Loop/1', 'Current_Loop/7');
L('Speed_Loop/2', 'mux_ticks/2');

% 电流环（20 kHz，每拍）
L('ia/1',       'Current_Loop/1');
L('ib/1',       'Current_Loop/2');
L('ic/1',       'Current_Loop/3');
L('vbus/1',     'Current_Loop/4');
L('theta_sum/1','Current_Loop/5');
L('gain_p_w/1', 'Current_Loop/6');
L('en/1',       'Current_Loop/8');
L('par_c/1',    'Current_Loop/9');
L('Current_Loop/1', 'duty/1');
L('Current_Loop/2', 'iq_used/1');
L('Current_Loop/3', 'mux_ticks/1');
L('mux_ticks/1',    'ticks/1');
end

% =======================================================================
function s = lit(x)
s = sprintf('%.12g', x);
end

function s = vec(v)
s = ['[' sprintf('%.12g ', v) ']'];
end

function mk(name, lib, pos, varargin)
add_block(lib, name, 'Position', pos, varargin{:});
end

function try_set(mdl, name, value)
try
    set_param(mdl, name, value);
catch
    fprintf('[WARN] 参数 %s 在当前版本不可用，已跳过\n', name);
end
end

function try_setb(blk, name, value)
% 块级参数：不同 Simulink 版本参数名可能不同，静默跳过
try
    set_param(blk, name, value);
catch
end
end

function clear_default(sys)
blks = get_param(sys, 'Blocks');
if any(strcmp(blks, 'In1')) && any(strcmp(blks, 'Out1'))
    try, delete_line(sys, 'In1/1', 'Out1/1'); catch, end
    delete_block([sys '/In1']); delete_block([sys '/Out1']);
end
end

function mk_in(sys, names)
for k = 1:numel(names)
    p = sprintf('%s/In%d', sys, k);
    add_block('simulink/Ports & Subsystems/In1', p, ...
        'Position', [20 20+(k-1)*46 50 40+(k-1)*46]);
    set_param(p, 'OutDataTypeStr', 'single');   % 先设类型（此时路径仍为 InN）
    set_param(p, 'Name', names{k});             % 再重命名，之后旧路径失效
end
end

function mk_out(sys, names)
for k = 1:numel(names)
    p = sprintf('%s/Out%d', sys, k);
    add_block('simulink/Ports & Subsystems/Out1', p, ...
        'Position', [1000 20+(k-1)*46 1030 40+(k-1)*46]);
    set_param(p, 'Name', names{k});
end
end

function set_chart(blk, fname, ts)
txt = fileread(fullfile(fileparts(mfilename('fullpath')), fname));
sf  = sfroot;
ch  = sf.find('-isa', 'Stateflow.EMChart', 'Path', blk);
assert(numel(ch) == 1, 'EMChart not found: %s', blk);
ch.Script = txt;
try
    set_param(blk, 'SystemSampleTime', ts);
catch
    fprintf('[WARN] SystemSampleTime 未生效: %s\n', blk);
end
end
