function rotorResponse = write_strong_coupled_report(report_file, yn, dyn, ddyn, time, F_b_hist, loaded_count_hist, slip_hist, bearingStateHist, params, modelInfo)
%WRITE_STRONG_COUPLED_REPORT Save coupled bearing and rotor response report.
% Inputs:
%   report_file       - target txt path.
%   yn, dyn, ddyn     - displacement, velocity and acceleration histories.
%   time              - simulation time vector.
%   F_b_hist          - bearing force history [Fx1; Fy1; Fx2; Fy2...].
%   loaded_count_hist - loaded rolling element count.
%   slip_hist         - cage slip ratio trend.
%   bearingStateHist  - per-step bearing states.
%   params, modelInfo - model metadata.
% Output:
%   rotorResponse - compact last-five-cycle response data.

Fen = params.Fen;
n_end = numel(time);
idx_plot = max(1, n_end - 5*Fen + 1):n_end;
tt = time(idx_plot).';
tt = tt - tt(1);
caiyang = 1/(time(2) - time(1));

nb = numel(params.bearing);
num_rotor_nodes = modelInfo.num_rotor_nodes;
yn1 = zeros(numel(idx_plot), 2*num_rotor_nodes);
dyn1 = zeros(numel(idx_plot), 2*num_rotor_nodes);
ddyn1 = zeros(numel(idx_plot), 2*num_rotor_nodes);
for i = 1:num_rotor_nodes
    yn1(:,2*i-1) = yn(4*i-3, idx_plot).';
    yn1(:,2*i) = yn(4*i-2, idx_plot).';
    dyn1(:,2*i-1) = dyn(4*i-3, idx_plot).';
    dyn1(:,2*i) = dyn(4*i-2, idx_plot).';
    ddyn1(:,2*i-1) = ddyn(4*i-3, idx_plot).';
    ddyn1(:,2*i) = ddyn(4*i-2, idx_plot).';
end
F_b1 = F_b_hist(:, idx_plot).';

rotorResponse.idx_plot = idx_plot;
rotorResponse.tt = tt;
rotorResponse.caiyang = caiyang;
rotorResponse.yn1 = yn1;
rotorResponse.dyn1 = dyn1;
rotorResponse.ddyn1 = ddyn1;
rotorResponse.F_b1 = F_b1;

report_dir = fileparts(report_file);
if ~isempty(report_dir) && ~isfolder(report_dir)
    mkdir(report_dir);
end

fid = fopen(report_file, 'wt', 'n', 'UTF-8');
if fid < 0
    warning('Cannot open report file: %s', report_file);
    return;
end
cleanupObj = onCleanup(@() fclose(fid));

fprintf(fid,'\t\t\t\t\t\t************************\n');
fprintf(fid,'\t\t\t\t\t\t高速滚子轴承拟动力学分析 \n');
fprintf(fid,'\t\t\t\t\t\t************************\n\n');

fprintf(fid,'一、强耦合转子-轴承-机匣模型\n');
fprintf(fid,'模型来源: %s\n', get_model_source(modelInfo));
fprintf(fid,'转速: %.2f rpm, omega = %.6f rad/s\n', params.rpm, params.omega);
fprintf(fid,'采样频率: %.6f Hz\n', caiyang);
fprintf(fid,'计算步数: %d, 每周期步数 Fen = %d, 总周期数 n_Fen = %d\n', numel(time)-1, params.Fen, params.n_Fen);
fprintf(fid,'转子节点数: %d, 机匣节点数: %d, 总自由度: %d\n\n', ...
    modelInfo.num_rotor_nodes, modelInfo.num_case_nodes, size(yn,1));

fprintf(fid,'二、输入参数\n');
fprintf(fid,'不平衡节点: %s\n', mat2str(params.unbalance.nodes));
fprintf(fid,'不平衡等效质量(g): %s\n', mat2str(params.unbalance.mass_g));
fprintf(fid,'偏心半径(mm): %s\n', mat2str(params.unbalance.ecc_mm));
if isfield(params, 'static_load') && ~isempty(params.static_load)
    fprintf(fid,'静径向载荷节点: %s\n', mat2str(params.static_load.nodes));
    fprintf(fid,'静径向载荷 Fx(N): %s\n', mat2str(params.static_load.Fx));
    fprintf(fid,'静径向载荷 Fy(N): %s\n', mat2str(params.static_load.Fy));
end
for ib = 1:nb
    brg = params.bearing(ib);
    fprintf(fid,'轴承%d: %s, type=%s, rotor_node=%d, case_node=%d\n', ...
        ib, brg.name, brg.type, brg.rotor_node, brg.case_node);
end
fprintf(fid,'\n');

fprintf(fid,'三、滚子/滚动体拟动力学输出\n');
for ib = 1:nb
    Fxy = F_b_hist(2*ib-1:2*ib,:);
    Fmag = sqrt(sum(Fxy.^2,1));
    avg_loaded = sum(loaded_count_hist(ib,:))/numel(loaded_count_hist(ib,:));
    max_q = get_max_state_value(bearingStateHist, ib, 'Q');
    max_pv = get_max_state_value(bearingStateHist, ib, 'PV');
    min_h = get_min_positive_state_value(bearingStateHist, ib, 'h');
    fprintf(fid,'轴承%d [%s]\n', ib, params.bearing(ib).type);
    fprintf(fid,'  平均承载滚动体数 = %.3f\n', avg_loaded);
    fprintf(fid,'  最大合接触力 = %.6e N\n', max(Fmag));
    fprintf(fid,'  最大单滚动体载荷 Qmax = %.6e N\n', max_q);
    fprintf(fid,'  最小油膜厚度 hmin = %.6e m\n', min_h);
    fprintf(fid,'  最大 PV 值 = %.6e Pa*m/s\n', max_pv);
    fprintf(fid,'  最大保持架打滑率 = %.6f\n', max(abs(slip_hist(ib,:))));
end
fprintf(fid,'\n');

fprintf(fid,'四、转子系统动力学响应输出（最后五个周期）\n');
for ib = 1:nb
    brg = params.bearing(ib);
    ix = 4*brg.rotor_node - 3;
    iy = 4*brg.rotor_node - 2;
    ux = yn(ix, idx_plot);
    uy = yn(iy, idx_plot);
    vx = dyn(ix, idx_plot);
    vy = dyn(iy, idx_plot);
    ax = ddyn(ix, idx_plot);
    ay = ddyn(iy, idx_plot);
    clearance = get_bearing_clearance(params, ib);
    ratio_x = max(abs(ux))/clearance;
    ratio_y = max(abs(uy))/clearance;
    judge = judge_displacement(ratio_x, ratio_y);

    fprintf(fid,'轴承%d转子节点%d:\n', ib, brg.rotor_node);
    fprintf(fid,'  x峰值 = %.6f um | 比值 = %.6f\n', max(abs(ux))*1e6, ratio_x);
    fprintf(fid,'  y峰值 = %.6f um | 比值 = %.6f\n', max(abs(uy))*1e6, ratio_y);
    fprintf(fid,'  x峰峰值 = %.6f um, y峰峰值 = %.6f um\n', (max(ux)-min(ux))*1e6, (max(uy)-min(uy))*1e6);
    fprintf(fid,'  vx峰值 = %.6f mm/s, vy峰值 = %.6f mm/s\n', max(abs(vx))*1e3, max(abs(vy))*1e3);
    fprintf(fid,'  ax峰值 = %.6e m/s^2, ay峰值 = %.6e m/s^2\n', max(abs(ax)), max(abs(ay)));
    fprintf(fid,'  判断 = %s\n', judge);
end
fprintf(fid,'\n');

fprintf(fid,'五、保存变量说明\n');
fprintf(fid,'完整时域结果保存于 strong_coupled_result.mat。\n');
fprintf(fid,'其中 yn/dyn/ddyn 为全自由度位移、速度、加速度；F_b_hist 为轴承接触力；bearingStateHist 为滚动体接触状态。\n');
fprintf(fid,'rotorResponse.yn1/dyn1/ddyn1/F_b1/tt/caiyang 为按 mian.m 方式截取的最后五个周期转子响应数据。\n');

end

function source = get_model_source(modelInfo)
if isfield(modelInfo, 'source')
    source = modelInfo.source;
else
    source = 'internal fallback beam model';
end
end

function clearance = get_bearing_clearance(params, ib)
if isfield(params.bearing(ib), 'clearance0') && params.bearing(ib).clearance0 > 0
    clearance = params.bearing(ib).clearance0;
else
    clearance = 1e-6;
end
end

function judge = judge_displacement(ratio_x, ratio_y)
if ratio_x < 0.2 && ratio_y < 0.2
    judge = '合理（远小于游隙，未接触小振动）';
elseif ratio_x < 1 && ratio_y < 1
    judge = '基本合理（接近游隙，可能接触前状态）';
elseif ratio_x >= 1 || ratio_y >= 1
    judge = '可能不合理（位移达到/超过游隙，需检查接触或参数）';
else
    judge = '需进一步分析';
end
end

function val = get_max_state_value(bearingStateHist, ib, field)
val = 0;
for k = 1:numel(bearingStateHist)
    if isempty(bearingStateHist{k}) || numel(bearingStateHist{k}.bearings) < ib
        continue;
    end
    s = bearingStateHist{k}.bearings(ib);
    if isfield(s, field) && ~isempty(s.(field))
        data = s.(field);
        data = data(isfinite(data));
        if ~isempty(data)
            val = max(val, max(abs(data(:))));
        end
    end
end
end

function val = get_min_positive_state_value(bearingStateHist, ib, field)
val = inf;
for k = 1:numel(bearingStateHist)
    if isempty(bearingStateHist{k}) || numel(bearingStateHist{k}.bearings) < ib
        continue;
    end
    s = bearingStateHist{k}.bearings(ib);
    if isfield(s, field) && ~isempty(s.(field))
        data = s.(field);
        data = data(isfinite(data) & data > 0);
        if ~isempty(data)
            val = min(val, min(data(:)));
        end
    end
end
if isinf(val)
    val = 0;
end
end
