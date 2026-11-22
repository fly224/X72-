function R = run_closed_sim(outdir)
%RUN_CLOSED_SIM  跑三组闭环 MIL 场景，输出曲线、CSV 与量化指标
%
%  A  位置阶跃 +90deg / 回零 + 0.35s 负载扰动（基准）
%  B  参数/修正效果对比：正确 dt(5e-5) / 错误 dt(1e-4，10kHz 遗留) / Kp&Ki x2
%  C  跨 2pi 坐标系对比：连续旋转指令，wrap 关闭 / 开启
%
% 用法：run_closed_sim

if nargin < 1 || isempty(outdir)
    outdir = fullfile(fileparts(mfilename('fullpath')), 'results');
end
if ~exist(outdir, 'dir'), mkdir(outdir); end
here = fileparts(mfilename('fullpath'));
S = sim_params();
mdl = 'three_loop_closed_sim';
if ~bdIsLoaded(mdl)
    load_system(fullfile(here, [mdl '.slx']));
end

% 场景配置：{名称, scenario, wrap_en, par_c}
base = S.par_c;
p_bad = base;  p_bad(3) = 1e-4;                 % 10 kHz 遗留 dt 误用于 20 kHz（Ki_eff 加倍）
p_kp2 = base;  p_kp2(1) = 2*base(1); p_kp2(2) = 2*base(2);   % 对外接口调带宽 x2（Ki 同步）
cfg = {
  'A_base',    1, 1, base;
  'B_bad_dt',  1, 1, p_bad;
  'B_kp2',     1, 1, p_kp2;
  'C_no_wrap', 2, 0, base;
  'C_wrap',    2, 1, base };

fprintf('=== 闭环 MIL 仿真：%d 个场景，每个 %.2f s ===\n', size(cfg,1), S.sim_stop);
D = cell(size(cfg,1),1);
for k = 1:size(cfg,1)
    name = cfg{k,1};
    set_param([mdl '/Test_Harness/scenario_c'], 'Value', num2str(cfg{k,2}));
    set_param([mdl '/Test_Harness/wrap_en_c'],  'Value', num2str(cfg{k,3}));
    set_param([mdl '/Test_Harness/par_c_c'],   'Value', vec(cfg{k,4}));
    set_param(mdl, 'StopTime', sprintf('%.12g', S.sim_stop));
    t0 = tic;
    out = sim(mdl);
    D{k} = extract(out, S);
    D{k}.name = name;
    fprintf('  [%d/%d] %-12s  wall %.1f s, iq peak %.2f A, |v| peak %.2f V\n', ...
        k, size(cfg,1), name, toc(t0), D{k}.iq_peak, D{k}.v_peak);
end
R.D = D; R.S = S;

% ---------------- 指标 ----------------
A = D{1}; B1 = D{2}; B2 = D{3}; C0 = D{4}; C1 = D{5};
M = struct();
M.A.pos_err_deg_1 = mean(A.err_deg(A.t>=0.25 & A.t<=0.34));
M.A.pos_err_deg_2 = mean(A.err_deg(A.t>=0.95 & A.t<=S.sim_stop));
M.A.overshoot_deg = max(0, max(A.theta_m(A.t>=S.pos_t1 & A.t<S.tl_t)) - S.pos_a1)*180/pi;
M.A.ts_ms = settling(A.t, A.theta_m, S.pos_t1, S.pos_a1, 0.02*S.pos_a1, S.tl_t)*1000;
m3 = A.t>=S.tl_t & A.t<=S.tl_t+0.2;
M.A.dist_peak_deg = max(abs(A.theta_m(m3) - S.pos_a1))*180/pi;
M.A.dist_rec_ms = settling(A.t, A.theta_m, S.tl_t, S.pos_a1, 0.2*pi/180, S.tl_t+0.2)*1000;
M.A.w_peak = max(abs(A.w_m));
M.A.iq_peak = A.iq_peak;
M.A.Te_peak = max(abs(A.Te));
M.A.v_peak = A.v_peak;
M.A.duty_range = A.duty_range;
M.A.tick_end = A.ticks(end,:);
M.A.tick_ratio = A.ticks(end,:) ./ A.ticks(end,3);

% 电流环：0.05 s 阶跃附近 10-90% 上升时间 + 跟踪 RMS
for j = 1:3
    d = D{j};
    M.B(j).name = d.name;
    M.B(j).tr_ms = rise_time(d.t, d.iq, S.pos_t1, 0.08)*1000;
    M.B(j).rms_err = sqrt(mean((d.iq_ref(d.t>S.en_t+0.002) - d.iq(d.t>S.en_t+0.002)).^2));
    M.B(j).iq_peak = d.iq_peak;
end

% 跨 2pi
M.C.no_wrap.iq_peak = C0.iq_peak;
M.C.no_wrap.err_peak = max(abs(C0.err_deg));
M.C.no_wrap.sat_samples = sum(abs(C0.iq) >= S.I_max-1e-6);
M.C.wrap.iq_peak = C1.iq_peak;
M.C.wrap.err_peak = max(abs(C1.err_deg));
M.C.wrap.sat_samples = sum(abs(C1.iq) >= S.I_max-1e-6);
R.M = M;

% ---------------- 图形 ----------------
plot_A(A, S, outdir);
plot_B(D(1:3), S, outdir);
plot_C(C0, C1, S, outdir);
plot_extra(A, S, outdir);

% ---------------- CSV（场景 A，100us 抽稀）----------------
write_csv(A, outdir);

% ---------------- 指标文本 ----------------
write_metrics(M, S, outdir);
save(fullfile(outdir, 'sim_result.mat'), 'R');
fprintf('[DONE] results -> %s\n', outdir);
end

% =======================================================================
function d = extract(out, S)
d = struct();
p = pick(out,'log_pos');   d.t = p.Time;
d.pos_raw = p.Data(:,1); d.pos_wrapped = p.Data(:,2);
d.theta_m = p.Data(:,3); d.theta_raw = p.Data(:,4);
m = pick(out,'log_mech');
d.w_m = m.Data(:,1); d.w_ref = m.Data(:,2); d.Te = m.Data(:,3); d.TL = m.Data(:,4);
i3 = pick(out,'log_iabc'); d.ia = i3.Data(:,1); d.ib = i3.Data(:,2); d.ic = i3.Data(:,3);
c2 = pick(out,'log_cmd'); d.iq_ref = c2.Data(:,1); d.iq_used = c2.Data(:,2);
du = pick(out,'log_duty'); d.da = du.Data(:,1); d.db = du.Data(:,2); d.dc = du.Data(:,3);
tk = pick(out,'log_ticks'); d.ticks = tk.Data;
te = pick(out,'log_theta_e'); d.theta_e = te.Data;

% 从 duty 反算等效 dq 电压（与控制器 Park 同约定）
mean_d = (d.da + d.db + d.dc)/3;
va = (d.da - mean_d)*S.Vbus;
vb = (d.db - mean_d)*S.Vbus;
v_alpha = va;
v_beta  = (va + 2*vb)/sqrt(3);
c = cos(d.theta_e); s = sin(d.theta_e);
d.vd = c.*v_alpha + s.*v_beta;
d.vq = c.*v_beta  - s.*v_alpha;
d.v_mag = sqrt(d.vd.^2 + d.vq.^2);
d.id = c.*d.ia + s.*((d.ia + 2*d.ib)/sqrt(3));
d.iq = c.*((d.ia + 2*d.ib)/sqrt(3)) - s.*d.ia;
d.iq_peak = max(abs(d.iq));
d.v_peak = max(d.v_mag);
d.duty_range = [min([d.da;d.db;d.dc]), max([d.da;d.db;d.dc])];
d.err_deg = (d.pos_wrapped - d.theta_m)*180/pi;
end

function p = pick(out, name)
if isa(out,'Simulink.SimulationOutput') && ismember(name, out.who)
    ts = out.(name);
elseif evalin('base', sprintf('exist(''%s'',''var'')', name))
    ts = evalin('base', name);
else
    error('missing log %s', name);
end
D = squeeze(ts.Data);
if isvector(D), D = D(:);
elseif size(D,1) <= 16 && size(D,2) > 16, D = D.'; end
p.Data = D; p.Time = ts.Time(:);
end

function ts = settling(t, y, t0, target, band, t_end)
m = find(t>=t0 & t<=t_end);
if isempty(m), ts = NaN; return; end
out = find(abs(y(m)-target) > band, 1, 'last');
if isempty(out), ts = t(m(1))-t0;
elseif out+1 > numel(m), ts = NaN;
else, ts = t(m(out+1))-t0; end
end

function tr = rise_time(t, y, t0, win)
m = t>=t0 & t<=t0+win;
tt = t(m); yy = y(m);
if isempty(tt), tr = NaN; return; end
tgt = max(abs(yy));
if tgt < 1e-6, tr = NaN; return; end
i10 = find(abs(yy)>=0.1*tgt,1); i90 = find(abs(yy)>=0.9*tgt,1);
if isempty(i10)||isempty(i90), tr = NaN; else, tr = tt(i90)-tt(i10); end
end

% =======================================================================
function plot_A(A, S, outdir)
f = figure('Visible','off','Position',[100 100 1100 860],'Color','w');
subplot(4,1,1);
plot(A.t, A.pos_raw*180/pi,'k--', A.t, A.pos_wrapped*180/pi,'m:', ...
     A.t, A.theta_m*180/pi,'r','LineWidth',1.1); grid on;
ylabel('position (deg)'); legend({'pos ref raw','pos ref wrapped','theta m'},'Location','southeast');
title('A: 3-loop closed loop (20kHz/2kHz/1kHz) - position step +90deg, load step 0.35s');
subplot(4,1,2);
plot(A.t, A.w_ref,'k--', A.t, A.w_m,'b'); grid on;
ylabel('speed (rad/s)'); legend({'w ref','w m'},'Location','southeast');
subplot(4,1,3);
plot(A.t, A.iq_ref,'k--', A.t, A.iq,'r', A.t, A.id,'b'); grid on;
ylabel('current (A)'); legend({'iq ref','iq','id'},'Location','southeast');
subplot(4,1,4);
plot(A.t, A.vd,'b', A.t, A.vq,'r'); hold on;
yline(S.V_lim,'k--'); grid on;
ylabel('voltage (V)'); xlabel('time (s)');
legend({'vd','vq','Vlim'},'Location','southeast');
title(sprintf('|v| peak %.2f V / limit %.2f V', A.v_peak, S.V_lim));
print(f, fullfile(outdir,'scenario_A_response.png'),'-dpng','-r120'); close(f);
end

function plot_B(D, S, outdir)
f = figure('Visible','off','Position',[100 100 1000 620],'Color','w');
subplot(2,1,1);
cols = {'r','b','g'}; names = {'correct dt=5e-5','wrong dt=1e-4','Kp&Ki x2'};
hold on;
for j = 1:3
    d = D{j}; m = d.t>=0.045 & d.t<=0.18;
    plot(d.t(m)*1000, d.iq(m), cols{j}, 'LineWidth',1.1);
end
grid on; ylabel('iq (A)'); xlabel('time (ms)');
legend(names,'Location','northeast');
title('B: current-loop response vs parameter setting (position step at 50 ms)');
subplot(2,1,2);
hold on;
for j = 1:3
    d = D{j}; m = d.t>=0.045 & d.t<=0.18;
    plot(d.t(m)*1000, d.iq_ref(m)-d.iq(m), cols{j});
end
grid on; ylabel('iq ref - iq (A)'); xlabel('time (ms)');
legend(names,'Location','northeast'); title('tracking error');
print(f, fullfile(outdir,'scenario_B_param_compare.png'),'-dpng','-r120'); close(f);
end

function plot_C(C0, C1, S, outdir)
f = figure('Visible','off','Position',[100 100 1000 660],'Color','w');
subplot(2,1,1);
plot(C0.t, C0.iq, 'r', C1.t, C1.iq, 'b'); hold on;
yline(S.I_max,'k--'); yline(-S.I_max,'k--'); grid on;
ylabel('iq (A)'); legend({'wrap OFF (bug)','wrap ON (fixed)','Ilim'},'Location','northeast');
title('C: continuous multi-turn command (8 rad/s), crossing 2pi at ~0.785 s');
subplot(2,1,2);
plot(C0.t, C0.err_deg, 'r', C1.t, C1.err_deg, 'b'); grid on;
ylabel('position error (deg)'); xlabel('time (s)');
legend({'wrap OFF','wrap ON'},'Location','northeast');
title('position error jump without wrap');
print(f, fullfile(outdir,'scenario_C_wrap_compare.png'),'-dpng','-r120'); close(f);
end

function plot_extra(A, S, outdir)
f = figure('Visible','off','Position',[100 100 1000 640],'Color','w');
subplot(2,1,1);
plot(A.t, A.ia, A.t, A.ib, A.t, A.ic); grid on;
ylabel('phase current (A)'); legend({'ia','ib','ic'},'Location','northeast');
title('SPS PMSM phase currents');
subplot(2,1,2);
plot(A.t, A.da, A.t, A.db, A.t, A.dc); grid on;
ylabel('duty'); xlabel('time (s)'); title('SVPWM duty');
print(f, fullfile(outdir,'scenario_A_current_duty.png'),'-dpng','-r120'); close(f);
end

% =======================================================================
function write_csv(A, outdir)
k = 1:10:numel(A.t);
f = fullfile(outdir,'scenario_A.csv');
fid = fopen(f,'w','n','UTF-8'); fwrite(fid,[239 187 191],'uint8');
fprintf(fid,['time_s,pos_ref_raw_rad,pos_ref_wrapped_rad,theta_m_rad,theta_raw_rad,' ...
             'w_ref_rad_s,w_m_rad_s,iq_ref_A,iq_used_A,ia_A,ib_A,ic_A,id_A,iq_A,' ...
             'vd_V,vq_V,Te_Nm,TL_Nm,da,db,dc,tick_i,tick_w,tick_p\n']);
M = [A.t(k), A.pos_raw(k), A.pos_wrapped(k), A.theta_m(k), A.theta_raw(k), ...
     A.w_ref(k), A.w_m(k), A.iq_ref(k), A.iq_used(k), A.ia(k), A.ib(k), A.ic(k), ...
     A.id(k), A.iq(k), A.vd(k), A.vq(k), A.Te(k), A.TL(k), ...
     A.da(k), A.db(k), A.dc(k), A.ticks(k,1), A.ticks(k,2), A.ticks(k,3)];
fprintf(fid, [repmat('%.8f,',1,23) '%.8f\n'], M');
fclose(fid);
end

function s = vec(v)
s = ['[' sprintf('%.12g ', v) ']'];
end

function write_metrics(M, S, outdir)
f = fullfile(outdir,'sim_metrics.txt');
fid = fopen(f,'w','n','UTF-8'); fwrite(fid,[239 187 191],'uint8');
fprintf(fid,'三环闭环 MIL 仿真指标（%s）\n', datestr(now,'yyyy-mm-dd HH:MM:SS'));
fprintf(fid,'控制律: fcn_three_*_deploy.m（MIL double, 20kHz / N_w=10 / N_p=20），参数=tuning 默认值\n');
fprintf(fid,'plant : powerlib SPS PMSM, 10us, Rs=%.4f, L=%.1fuH, psi=%.4fmWb, p=%d, J=%.4g\n', ...
        S.Rs, S.Ld*1e6, S.psi*1e3, S.p, S.J);
fprintf(fid,'Vbus=%.0f V, 电压上限=%.3f V, 仿真 %.2f s\n\n', S.Vbus, S.V_lim, S.sim_stop);
fprintf(fid,'--- 场景 A 基准（位置阶跃 +90deg / 回零，0.35s 加 0.05N*m）---\n');
fprintf(fid,'位置稳态误差      : %.4f deg / 回零 %.4f deg\n', M.A.pos_err_deg_1, M.A.pos_err_deg_2);
fprintf(fid,'位置超调          : %.3f deg\n', M.A.overshoot_deg);
fprintf(fid,'调节时间(±2%%)     : %.1f ms\n', M.A.ts_ms);
fprintf(fid,'负载扰动峰值/恢复  : %.4f deg / %.1f ms\n', M.A.dist_peak_deg, M.A.dist_rec_ms);
fprintf(fid,'速度峰值          : %.2f rad/s\n', M.A.w_peak);
fprintf(fid,'iq 峰值 / Te 峰值 : %.3f A / %.3f N*m\n', M.A.iq_peak, M.A.Te_peak);
fprintf(fid,'|v| 峰值          : %.2f V (限幅 %.2f V)\n', M.A.v_peak, S.V_lim);
fprintf(fid,'占空比范围        : [%.3f %.3f]\n', M.A.duty_range);
fprintf(fid,'tick 末值         : %.0f / %.0f / %.0f  (比值 %.3f : %.3f : 1)\n', ...
        M.A.tick_end(1), M.A.tick_end(2), M.A.tick_end(3), ...
        M.A.tick_ratio(1), M.A.tick_ratio(2));
fprintf(fid,'\n--- 场景 B 参数 / 频率修正效果 ---\n');
for j = 1:3
    fprintf(fid,'%-14s : iq 上升 %.2f ms, 跟踪 RMS %.4f A, iq 峰值 %.3f A\n', ...
        M.B(j).name, M.B(j).tr_ms, M.B(j).rms_err, M.B(j).iq_peak);
end
fprintf(fid,'\n--- 场景 C 跨 2pi（连续旋转 8 rad/s）---\n');
fprintf(fid,'wrap OFF : iq 峰值 %.3f A, 位置误差峰值 %.1f deg, 限幅采样点 %d\n', ...
        M.C.no_wrap.iq_peak, M.C.no_wrap.err_peak, M.C.no_wrap.sat_samples);
fprintf(fid,'wrap ON  : iq 峰值 %.3f A, 位置误差峰值 %.1f deg, 限幅采样点 %d\n', ...
        M.C.wrap.iq_peak, M.C.wrap.err_peak, M.C.wrap.sat_samples);
fclose(fid);
fprintf('metrics: %s\n', f);
end
