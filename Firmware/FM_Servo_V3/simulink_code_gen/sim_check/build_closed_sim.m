function build_closed_sim(outdir)
%BUILD_CLOSED_SIM  构建与部署代码同源的闭环 MIL 仿真 three_loop_closed_sim.slx
%
% 控制器 = 直接从 simulink_code_gen/fcn_three_*_deploy.m 载入的三环控制律
%          （20 kHz 基节拍 + 内部分频 /10、/20），参数由 tuning 默认值填入；
% 被控对象 = Simulink 库 SPS PMSM（电气域）+ 3 个受控电压源 + 平均逆变器；
% 编码器 = 把连续机械角 mod 2pi 成单圈，模拟 MT6835 真实输出；
% 位置指令 = 可切换「阶跃 / 多圈连续旋转」，并可开关 wrap（复现文档 §3.3）。
%
% 用法：build_closed_sim          % 输出到本文件所在目录

if nargin < 1 || isempty(outdir)
    outdir = fileparts(mfilename('fullpath'));
end
S = sim_params();

mdl = 'three_loop_closed_sim';
if bdIsLoaded(mdl), close_system(mdl, 0); end
slx = fullfile(outdir, [mdl '.slx']);
if exist(slx, 'file'), delete(slx); end
new_system(mdl); load_system(mdl);

set_param(mdl, ...
    'SolverType', 'Fixed-step', ...
    'Solver',     'FixedStepDiscrete', ...
    'FixedStep',  lit(S.Ts_plant), ...
    'StopTime',   lit(S.sim_stop), ...
    'StartTime',  '0', ...
    'SaveTime',   'on', ...
    'SaveOutput', 'off', ...
    'SignalLogging', 'off');
try, set_param(mdl, 'SingleTaskRateTransMsg', 'none'); catch, end

hHarness = [mdl '/Test_Harness'];
hCtrl    = [mdl '/Controller'];
hInv     = [mdl '/Inverter_Average'];
hPlant   = [mdl '/PMSM_Plant'];
add_block('built-in/Subsystem', hHarness, 'Position', [ 20  20 210 300]);
add_block('built-in/Subsystem', hCtrl,    'Position', [260  20 560 340]);
add_block('built-in/Subsystem', hInv,     'Position', [610  20 750 220]);
add_block('built-in/Subsystem', hPlant,   'Position', [800  20 990 340]);

add_block('powerlib/powergui', [mdl '/powergui'], 'Position', [1050 420 1160 480]);
set_param([mdl '/powergui'], 'SimulationMode', 'Discrete');
set_param([mdl '/powergui'], 'SampleTime', lit(S.Ts_plant));

build_harness(hHarness, S);
build_controller(hCtrl, S);
build_inverter(hInv, S);
build_plant(hPlant, S);

% ---- 顶层 wrap（位置指令折叠开关）----
add_block('simulink/User-Defined Functions/MATLAB Function', [mdl '/PosWrap'], ...
    'Position', [230 380 380 450]);
set_chart([mdl '/PosWrap'], fullfile(outdir, 'fcn_wrap_ref.m'), lit(S.Ts_plant));

L = @(a,b) add_line(mdl, a, b, 'autorouting', 'on');
L('Test_Harness/1',  'PosWrap/1');           % pos_ref_raw
L('PMSM_Plant/4',    'PosWrap/2');           % theta_m single
L('Test_Harness/9',  'PosWrap/3');           % wrap_en
L('Test_Harness/10', 'PosWrap/4');           % dir_sign
L('PosWrap/1',       'Controller/7');        % pos_ref wrapped

L('Test_Harness/2',  'PMSM_Plant/4');        % TL -> plant inport 4
L('Test_Harness/3',  'Controller/4');        % vbus
L('Test_Harness/3',  'Inverter_Average/2');
L('Test_Harness/4',  'Controller/8');        % en
L('Test_Harness/5',  'Controller/9');        % par_c
L('Test_Harness/6',  'Controller/10');       % par_w
L('Test_Harness/7',  'Controller/11');       % par_p

L('PMSM_Plant/1', 'Controller/1');           % ia
L('PMSM_Plant/2', 'Controller/2');           % ib
L('PMSM_Plant/3', 'Controller/3');           % ic
L('PMSM_Plant/5', 'Controller/6');           % omega_m
L('PMSM_Plant/4', 'Controller/5');           % theta_m single

L('Controller/1', 'Inverter_Average/1');     % duty
L('Inverter_Average/1', 'PMSM_Plant/1');     % va
L('Inverter_Average/2', 'PMSM_Plant/2');     % vb
L('Inverter_Average/3', 'PMSM_Plant/3');     % vc

build_logging(mdl, S);
save_system(mdl, slx);
fprintf('[OK] closed-loop model saved: %s\n', slx);
end

% =======================================================================
function s = lit(x), s = sprintf('%.12g', x); end
function s = vec(v), s = ['[' sprintf('%.12g ', v) ']']; end
function mk(n, lib, pos, varargin), add_block(lib, n, 'Position', pos, varargin{:}); end

function clear_default(sys)
b = get_param(sys, 'Blocks');
if any(strcmp(b,'In1')) && any(strcmp(b,'Out1'))
    try, delete_line(sys, 'In1/1', 'Out1/1'); catch, end
    delete_block([sys '/In1']); delete_block([sys '/Out1']);
end
end

function mk_in(sys, names)
for k = 1:numel(names)
    p = sprintf('%s/In%d', sys, k);
    add_block('simulink/Ports & Subsystems/In1', p, ...
        'Position', [20 20+(k-1)*42 50 40+(k-1)*42]);
    set_param(p, 'Name', names{k});
end
end

function mk_out(sys, names)
for k = 1:numel(names)
    p = sprintf('%s/Out%d', sys, k);
    add_block('simulink/Ports & Subsystems/Out1', p, ...
        'Position', [1080 20+(k-1)*44 1110 40+(k-1)*44]);
    set_param(p, 'Name', names{k});
end
end

function set_chart(blk, fname, ts)
txt = fileread(fname);
sf = sfroot; ch = sf.find('-isa', 'Stateflow.EMChart', 'Path', blk);
assert(numel(ch) == 1, 'EMChart not found: %s', blk);
ch.Script = txt;
try, set_param(blk, 'SystemSampleTime', ts); catch, end
end

% =======================================================================
function build_harness(sys, S)
clear_default(sys);
out = {'pos_ref_raw','TL','vbus','en','par_c','par_w','par_p','scenario','wrap_en','dir_sign'};
mk_out(sys, out);

mk([sys '/clock'], 'simulink/Sources/Digital Clock', [30 20 90 50], ...
    'SampleTime', lit(S.Ts_plant));
mk([sys '/scenario_c'], 'simulink/Sources/Constant', [30 75 100 105], ...
    'Value', '1', 'SampleTime', 'inf');
mk([sys '/ref_par'], 'simulink/Sources/Constant', [30 125 140 155], ...
    'Value', vec([S.pos_t1, S.pos_a1, S.pos_t2, S.vel_ref]), 'SampleTime', 'inf');
add_block('simulink/User-Defined Functions/MATLAB Function', [sys '/RefGen'], ...
    'Position', [180 20 330 110]);
set_chart([sys '/RefGen'], fullfile(fileparts(fileparts(mfilename('fullpath'))), ...
          'sim_check', 'fcn_ref_gen.m'), lit(S.Ts_plant));

mk([sys '/tl_step'], 'simulink/Sources/Step', [30 180 60 210], ...
    'Time', lit(S.tl_t), 'Before', '0', 'After', lit(S.tl_a), 'SampleTime', lit(S.Ts_plant));
mk([sys '/vbus_c'],  'simulink/Sources/Constant', [30 235 90 265], ...
    'Value', lit(S.Vbus), 'SampleTime', 'inf');
mk([sys '/en_step'], 'simulink/Sources/Step', [30 290 60 320], ...
    'Time', lit(S.en_t), 'Before', '0', 'After', '1', 'SampleTime', lit(S.Ts_i));
mk([sys '/par_c_c'],   'simulink/Sources/Constant', [30 345 220 380], ...
    'Value', vec(S.par_c), 'SampleTime', 'inf');
mk([sys '/par_w_c'],   'simulink/Sources/Constant', [30 395 220 425], ...
    'Value', vec(S.par_w), 'SampleTime', 'inf');
mk([sys '/par_p_c'],   'simulink/Sources/Constant', [30 440 220 470], ...
    'Value', vec(S.par_p), 'SampleTime', 'inf');
mk([sys '/wrap_en_c'], 'simulink/Sources/Constant', [30 490 120 520], ...
    'Value', '1', 'SampleTime', 'inf');
mk([sys '/dir_sign_c'],'simulink/Sources/Constant', [30 540 120 570], ...
    'Value', '1', 'SampleTime', 'inf');

L = @(a,b) add_line(sys, a, b, 'autorouting', 'on');
L('clock/1',    'RefGen/1');
L('scenario_c/1', 'RefGen/2');
L('ref_par/1',  'RefGen/3');
L('RefGen/1',   'pos_ref_raw/1');
L('tl_step/1',  'TL/1');
L('vbus_c/1',   'vbus/1');
L('en_step/1',  'en/1');
L('par_c_c/1',  'par_c/1');
L('par_w_c/1',  'par_w/1');
L('par_p_c/1',  'par_p/1');
L('scenario_c/1', 'scenario/1');
L('wrap_en_c/1',  'wrap_en/1');
L('dir_sign_c/1', 'dir_sign/1');
end

% =======================================================================
function build_controller(sys, S)
clear_default(sys);
ts = lit(S.Ts_i);
mk_in(sys, {'ia','ib','ic','vbus','theta_m','omega_m','pos_ref','en','par_c','par_w','par_p'});
mk_out(sys, {'duty','iq_ref','w_ref','iq_used','ticks','theta_e'});

% 角度变换（与部署模型一致）
mk([sys '/gain_p_theta'], 'simulink/Math Operations/Gain', [130 20 200 50], ...
    'Gain', sprintf('%.12g', S.p));
mk([sys '/gain_p_w'],     'simulink/Math Operations/Gain', [130 70 200 100], ...
    'Gain', sprintf('%.12g', S.p));
mk([sys '/gain_comp'],    'simulink/Math Operations/Gain', [130 120 200 150], ...
    'Gain', sprintf('%.12g', S.gain_comp));
mk([sys '/theta_off'],    'simulink/Sources/Constant', [130 170 210 200], ...
    'Value', lit(S.theta_off), 'SampleTime', 'inf');
add_block('simulink/Math Operations/Sum', [sys '/theta_sum'], ...
    'Position', [250 30 275 120], 'Inputs', '+++');

cg = S.codegen_dir;
add_block('simulink/User-Defined Functions/MATLAB Function', [sys '/Position_Loop'], ...
    'Position', [340 20 480 120]);
set_chart([sys '/Position_Loop'], fullfile(cg, 'fcn_three_pos_loop_deploy.m'), ts);
add_block('simulink/User-Defined Functions/MATLAB Function', [sys '/Speed_Loop'], ...
    'Position', [540 20 680 120]);
set_chart([sys '/Speed_Loop'], fullfile(cg, 'fcn_three_speed_loop_deploy.m'), ts);
add_block('simulink/User-Defined Functions/MATLAB Function', [sys '/Current_Loop'], ...
    'Position', [740 160 900 330]);
set_chart([sys '/Current_Loop'], fullfile(cg, 'fcn_three_current_loop_deploy.m'), ts);
mk([sys '/mux_ticks'], 'simulink/Signal Routing/Mux', [740 400 745 470], 'Inputs', '3');

L = @(a,b) add_line(sys, a, b, 'autorouting', 'on');
L('theta_m/1', 'gain_p_theta/1');
L('omega_m/1', 'gain_p_w/1');
L('omega_m/1', 'gain_comp/1');
L('gain_p_theta/1', 'theta_sum/1');
L('theta_off/1',    'theta_sum/2');
L('gain_comp/1',    'theta_sum/3');
L('theta_sum/1',    'theta_e/1');

L('pos_ref/1', 'Position_Loop/1');
L('theta_m/1', 'Position_Loop/2');
L('en/1',      'Position_Loop/3');
L('par_p/1',   'Position_Loop/4');
L('Position_Loop/1', 'w_ref/1');
L('Position_Loop/1', 'Speed_Loop/1');
L('Position_Loop/2', 'mux_ticks/3');

L('omega_m/1', 'Speed_Loop/2');
L('en/1',      'Speed_Loop/3');
L('par_w/1',   'Speed_Loop/4');
L('Speed_Loop/1', 'iq_ref/1');
L('Speed_Loop/1', 'Current_Loop/7');
L('Speed_Loop/2', 'mux_ticks/2');

L('ia/1',   'Current_Loop/1');
L('ib/1',   'Current_Loop/2');
L('ic/1',   'Current_Loop/3');
L('vbus/1', 'Current_Loop/4');
L('theta_sum/1', 'Current_Loop/5');
L('gain_p_w/1',  'Current_Loop/6');
L('en/1',        'Current_Loop/8');
L('par_c/1',     'Current_Loop/9');
L('Current_Loop/1', 'duty/1');
L('Current_Loop/2', 'iq_used/1');
L('Current_Loop/3', 'mux_ticks/1');
L('mux_ticks/1',    'ticks/1');
end

% =======================================================================
function build_inverter(sys, S)
clear_default(sys);
mk_in(sys, {'duty','vbus'});
mk_out(sys, {'va','vb','vc'});
mk([sys '/pwm_delay'], 'simulink/Discrete/Delay', [110 30 150 60], ...
    'DelayLength', lit(S.N_delay), 'SampleTime', lit(S.Ts_plant));
mk([sys '/demux_d'], 'simulink/Signal Routing/Demux', [190 20 195 90], 'Outputs', '3');
mk([sys '/sum_d'],   'simulink/Math Operations/Sum', [230 120 255 165], 'Inputs', '+++');
mk([sys '/mean_d'],  'simulink/Math Operations/Gain', [290 125 325 165], 'Gain', '1/3');
ph = {'a','b','c'};
for k = 1:3
    y0 = 60+(k-1)*150;
    mk([sys '/dsub_' ph{k}], 'simulink/Math Operations/Sum', [400 y0 425 y0+45], 'Inputs', '+-');
    mk([sys '/vbus_' ph{k}], 'simulink/Math Operations/Product', [470 y0 500 y0+40], 'Inputs', '2');
end
L = @(a,b) add_line(sys, a, b, 'autorouting', 'on');
L('duty/1','pwm_delay/1'); L('pwm_delay/1','demux_d/1');
L('demux_d/1','sum_d/1'); L('demux_d/2','sum_d/2'); L('demux_d/3','sum_d/3');
L('sum_d/1','mean_d/1');
for k = 1:3
    L(sprintf('demux_d/%d',k), sprintf('dsub_%s/1',ph{k}));
    L('mean_d/1', sprintf('dsub_%s/2',ph{k}));
    L(sprintf('dsub_%s/1',ph{k}), sprintf('vbus_%s/1',ph{k}));
    L('vbus/1', sprintf('vbus_%s/2',ph{k}));
    L(sprintf('vbus_%s/1',ph{k}), sprintf('v%s/1',ph{k}));
end
end

% =======================================================================
function build_plant(sys, S)
clear_default(sys);
mk_in(sys, {'va','vb','vc','TL'});
mk_out(sys, {'ia','ib','ic','theta_m','omega_m','Te','theta_raw'});
ts = lit(S.Ts_plant);

add_block('powerlib/Electrical Sources/Controlled Voltage Source', [sys '/vs_a'], 'Position', [40 40 110 100]);
add_block('powerlib/Electrical Sources/Controlled Voltage Source', [sys '/vs_b'], 'Position', [40 140 110 200]);
add_block('powerlib/Electrical Sources/Controlled Voltage Source', [sys '/vs_c'], 'Position', [40 240 110 300]);
add_block('powerlib/Measurements/Current Measurement', [sys '/im_a'], 'Position', [170 40 220 80]);
add_block('powerlib/Measurements/Current Measurement', [sys '/im_b'], 'Position', [170 140 220 180]);
add_block('powerlib/Measurements/Current Measurement', [sys '/im_c'], 'Position', [170 240 220 280]);
add_block('powerlib/Elements/Ground', [sys '/gnd'], 'Position', [40 380 80 410]);
add_block('powerlib/Machines/Permanent Magnet Synchronous Machine', [sys '/pmsm'], 'Position', [320 80 430 300]);
set_param([sys '/pmsm'], 'RotorType','Round', 'FluxDistribution','Sinusoidal', ...
    'MechanicalLoad','Torque Tm', 'MeasurementBus','on', ...
    'MachineConstant','Flux linkage established by magnets (V.s)');
set_param([sys '/pmsm'], 'Resistance', lit(S.Rs));
set_param([sys '/pmsm'], 'dqInductances', sprintf('[%.12g %.12g]', S.Ld, S.Lq));
set_param([sys '/pmsm'], 'Flux', lit(S.psi));
set_param([sys '/pmsm'], 'Mechanical', sprintf('[%.12g %.12g %.12g 0]', S.J, S.B, S.p));

add_block('simulink/Signal Routing/Bus Selector', [sys '/m_sel'], 'Position', [520 80 525 200]);
set_param([sys '/m_sel'], 'OutputSignals', 'w,theta,Te');
set_param([sys '/m_sel'], 'OutputAsBus', 'off');

% 连续机械角 -> 单圈（模拟真实绝对编码器）
add_block('simulink/User-Defined Functions/MATLAB Function', [sys '/mod2pi'], ...
    'Position', [620 140 760 200]);
set_chart([sys '/mod2pi'], fullfile(fileparts(fileparts(mfilename('fullpath'))), ...
          'sim_check', 'fcn_mod2pi.m'), ts);

mk([sys '/neg_a'], 'simulink/Math Operations/Gain', [300 40 330 70], 'Gain', '-1');
mk([sys '/neg_b'], 'simulink/Math Operations/Gain', [300 140 330 170], 'Gain', '-1');
mk([sys '/neg_c'], 'simulink/Math Operations/Gain', [300 240 330 270], 'Gain', '-1');

L = @(a,b) add_line(sys, a, b, 'autorouting', 'on');
L('va/1','vs_a/1'); L('vb/1','vs_b/1'); L('vc/1','vs_c/1');
L('vs_a/LConn1','im_a/LConn1'); L('vs_b/LConn1','im_b/LConn1'); L('vs_c/LConn1','im_c/LConn1');
L('im_a/RConn1','pmsm/LConn1'); L('im_b/RConn1','pmsm/LConn2'); L('im_c/RConn1','pmsm/LConn3');
L('vs_a/RConn1','gnd/LConn1'); L('vs_b/RConn1','gnd/LConn1'); L('vs_c/RConn1','gnd/LConn1');
L('TL/1','pmsm/1'); L('pmsm/1','m_sel/1');
L('im_a/1','neg_a/1'); L('im_b/1','neg_b/1'); L('im_c/1','neg_c/1');
L('neg_a/1','ia/1'); L('neg_b/1','ib/1'); L('neg_c/1','ic/1');
L('m_sel/1','omega_m/1');
L('m_sel/2','theta_raw/1');
L('m_sel/2','mod2pi/1');
L('mod2pi/1','theta_m/1');
L('m_sel/3','Te/1');
end

% =======================================================================
function build_logging(mdl, S)
ts = lit(S.Ts_plant);
mk([mdl '/mux_pos'],  'simulink/Signal Routing/Mux', [1030 20 1035 100], 'Inputs', '4');
mk([mdl '/mux_mech'], 'simulink/Signal Routing/Mux', [1030 120 1035 220], 'Inputs', '4');
mk([mdl '/mux_iabc'], 'simulink/Signal Routing/Mux', [1030 240 1035 320], 'Inputs', '3');
mk([mdl '/mux_cmd'],  'simulink/Signal Routing/Mux', [1030 340 1035 400], 'Inputs', '2');

L = @(a,b) add_line(mdl, a, b, 'autorouting', 'on');
L('Test_Harness/1', 'mux_pos/1');   % pos_ref_raw
L('PosWrap/1',      'mux_pos/2');   % pos_ref wrapped
L('PMSM_Plant/4',   'mux_pos/3');   % theta_m single
L('PMSM_Plant/7',   'mux_pos/4');   % theta_raw
L('PMSM_Plant/5',   'mux_mech/1');  % omega_m
L('Controller/3',   'mux_mech/2');  % w_ref
L('PMSM_Plant/6',   'mux_mech/3');  % Te
L('Test_Harness/2', 'mux_mech/4');  % TL
L('PMSM_Plant/1',   'mux_iabc/1');
L('PMSM_Plant/2',   'mux_iabc/2');
L('PMSM_Plant/3',   'mux_iabc/3');
L('Controller/2',   'mux_cmd/1');   % iq_ref
L('Controller/4',   'mux_cmd/2');   % iq_used

logv = {'log_pos','mux_pos/1',[1130 20 1250 60];
        'log_mech','mux_mech/1',[1130 90 1250 130];
        'log_iabc','mux_iabc/1',[1130 160 1250 200];
        'log_cmd','mux_cmd/1',[1130 230 1250 270];
        'log_duty','Controller/1',[1130 300 1250 340];
        'log_ticks','Controller/5',[1130 370 1250 410];
        'log_theta_e','Controller/6',[1130 440 1250 480]};
for k = 1:size(logv,1)
    add_block('simulink/Sinks/To Workspace', [mdl '/' logv{k,1}], ...
        'Position', logv{k,3}, 'VariableName', logv{k,1}, ...
        'SaveFormat','Timeseries','SampleTime',ts);
    L(logv{k,2}, [logv{k,1} '/1']);
end
end
