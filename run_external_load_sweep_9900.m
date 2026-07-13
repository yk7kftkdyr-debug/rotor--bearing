%% External load sweep at 9900 r/min for the rotor-bearing-casing Newmark model
% Single-factor study: only data(10), the equivalent radial bearing load
% entering F_bearing.m, is changed. Rotor speed, unbalance, bearing
% stiffness/damping, clearance, support stiffness, time step, integration
% method and observation node follow the original program.

clearvars
clc
close all
set(0, 'DefaultFigureVisible', 'off');

external_load_list = [0, 1000, 2500, 5000, 7500];  % N
case_labels = {'L0','L1','L2','L3','L4'};
rpm_fixed = 9900;
f_rot = rpm_fixed / 60;
harmonics = [1, 2, 3] * f_rot;

fig_dir = fullfile(pwd, 'external_load_sweep_9900_figures');
if ~exist(fig_dir, 'dir')
    mkdir(fig_dir);
end

mat_files = cell(numel(external_load_list), 1);
results = cell(numel(external_load_list), 1);

for ii = 1:numel(external_load_list)
    external_load_N = external_load_list(ii);
    case_id = case_labels{ii};
    mat_name = fullfile(pwd, sprintf('results_external_load_%gN.mat', external_load_N));
    mat_files{ii} = mat_name;

    fprintf('\n========== External load %g N (%s, %d/%d) ==========\n', ...
        external_load_N, case_id, ii, numel(external_load_list));

    result = run_single_external_load_case(external_load_N, case_id, fig_dir, f_rot, harmonics);
    results{ii} = result;
    save(mat_name, 'result', '-v7.3');

    if isfield(result, 'signals') && ~strcmp(result.simulation_status, 'failed')
        plot_case_figures(result, fig_dir);
    end
    close all
end

summary = build_summary_from_mat(mat_files);
writetable(summary, fullfile(pwd, 'external_load_sweep_9900_summary.xlsx'));
writetable(summary, fullfile(pwd, 'external_load_sweep_9900_summary.csv'));

plot_comparison_figures(results, fig_dir);
write_markdown_report(summary, fig_dir, fullfile(pwd, 'external_load_sweep_9900_report.md'));

fprintf('\nGenerated:\n');
fprintf('  external_load_sweep_9900_summary.xlsx\n');
fprintf('  external_load_sweep_9900_summary.csv\n');
fprintf('  external_load_sweep_9900_report.md\n');
fprintf('  results_external_load_*N.mat\n');
fprintf('  %s\n', fig_dir);


function result = run_single_external_load_case(external_load_N, case_id, fig_dir, f_rot, harmonics)
    suppress_main_plots = true; %#ok<NASGU>
    sweep_output_dir = fig_dir; %#ok<NASGU>
    sweep_case_id = case_id; %#ok<NASGU>

    result = init_result_shell(external_load_N, case_id, f_rot, harmonics);
    try
        run('mian.m');

        result = init_result_shell(external_load_N, sweep_case_id, 165, [165, 330, 495]);
        result.case_id = sweep_case_id;
        result.external_load_N = external_load_N;
        result.data10_value = data(10);
        result.rpm = 9900;
        result.wi = wi;
        result.f_rot = 165;
        result.harmonics = [165, 330, 495];
        result.U = U_current;
        result.U_base = U_base;
        result.Famp1 = Famp1;
        result.fai = fai;
        result.loca = loca;
        result.loc_rub = loc_rub;
        result.data = data;
        result.N = N;
        result.N_C = N_C;
        result.num_rotor = num_rotor;
        result.num_case = num_case;
        result.r1 = r1;
        result.r2 = r2;
        result.c1 = c1;
        result.c2 = c2;
        result.Fen = Fen;
        result.n_Fen = n_Fen;
        result.total_steps = total_steps;
        result.ht = ht;
        result.fs = 1 / ht;
        result.n_end = n_end;
        result.idx_plot = idx_plot;
        result.time_plot = time_plot;
        result.yn = yn;
        result.dyn = dyn;
        result.ddyn = ddyn;
        result.F_b_hist = F_b_hist;
        result.miu_hist = miu_hist;
        result.yn1 = yn1;
        result.dyn1 = dyn1;
        result.ddyn1 = ddyn1;
        result.tt = tt;
        result.F_b1 = F_b1;
        result.load_application_type = 'bearing_support_equivalent_radial_load';
        result.bearing_load_node_1 = r1;
        result.bearing_load_node_2 = r2;
        result.bearing_load_node_3 = NaN;
        result.actual_load_node_1 = r1;
        result.actual_load_node_2 = r2;
        result.actual_load_node_3 = NaN;
        result.actual_case_node_1 = c1;
        result.actual_case_node_2 = c2;
        result.actual_case_node_3 = NaN;
        result.load_direction = '+x radial direction; y component is 0';
        result.load_distribution_method = ...
            'F_bearing uses F_r=[data(10);0;data(10);0]; data(10) is applied at each of the two bearing supports, not split as a total load';

        if any(~isfinite(yn(:))) || any(~isfinite(dyn(:))) || any(~isfinite(ddyn(:)))
            result.simulation_status = 'failed';
        elseif n_end < total_steps
            result.simulation_status = 'abnormal';
        else
            result.simulation_status = 'completed';
        end

        result = compute_case_metrics(result);
    catch ME
        if ~exist('result', 'var') || ~isstruct(result) || ~isfield(result, 'load_application_type')
            if exist('sweep_case_id', 'var')
                failed_case_id = sweep_case_id;
            else
                failed_case_id = case_id;
            end
            result = init_result_shell(external_load_N, failed_case_id, 165, [165, 330, 495]);
        end
        result.simulation_status = 'failed';
        result.error_message = ME.message;
        result.metrics = failed_metrics(result);
        fprintf('Case %s failed during post-processing or simulation: %s\n', result.case_id, ME.message);
        for kk = 1:numel(ME.stack)
            fprintf('  at %s line %d\n', ME.stack(kk).name, ME.stack(kk).line);
        end
    end
end


function result = init_result_shell(external_load_N, case_id, f_rot, harmonics)
    result = struct();
    result.case_id = case_id;
    result.external_load_N = external_load_N;
    result.data10_value = external_load_N;
    result.rpm = 9900;
    result.f_rot = f_rot;
    result.harmonics = harmonics;
    result.load_application_type = 'bearing_support_equivalent_radial_load';
    result.bearing_load_node_1 = NaN;
    result.bearing_load_node_2 = NaN;
    result.bearing_load_node_3 = NaN;
    result.actual_load_node_1 = NaN;
    result.actual_load_node_2 = NaN;
    result.actual_load_node_3 = NaN;
    result.actual_case_node_1 = NaN;
    result.actual_case_node_2 = NaN;
    result.actual_case_node_3 = NaN;
    result.load_direction = '+x radial direction; y component is 0';
    result.load_distribution_method = 'data(10) original F_bearing.m path';
    result.simulation_status = 'not_run';
end


function summary = build_summary_from_mat(mat_files)
    summary = table();
    for ii = 1:numel(mat_files)
        data = load(mat_files{ii}, 'result');
        row = data.result.metrics;
        if isempty(summary)
            summary = row;
        else
            summary = [summary; row]; %#ok<AGROW>
        end
    end
end


function result = compute_case_metrics(result)
    idx = result.idx_plot;
    t = result.time_plot(:);
    fs = result.fs;
    rotor_node = result.loc_rub(1);
    case_node = result.c1;

    rotor_x_idx = 4 * rotor_node - 3;
    rotor_y_idx = 4 * rotor_node - 2;
    case_x_idx = result.num_rotor + 4 * case_node - 3;
    case_y_idx = result.num_rotor + 4 * case_node - 2;

    rx = result.yn(rotor_x_idx, idx).';
    ry = result.yn(rotor_y_idx, idx).';
    rvx = result.dyn(rotor_x_idx, idx).';
    rvy = result.dyn(rotor_y_idx, idx).';
    rax = result.ddyn(rotor_x_idx, idx).';
    ray = result.ddyn(rotor_y_idx, idx).';
    cx = result.yn(case_x_idx, idx).';
    cy = result.yn(case_y_idx, idx).';
    cvx = result.dyn(case_x_idx, idx).';
    cvy = result.dyn(case_y_idx, idx).';
    cax = result.ddyn(case_x_idx, idx).';
    cay = result.ddyn(case_y_idx, idx).';

    rx0 = rx - local_mean(rx);
    ry0 = ry - local_mean(ry);
    rvx0 = rvx - local_mean(rvx);
    rvy0 = rvy - local_mean(rvy);
    rax0 = rax - local_mean(rax);
    ray0 = ray - local_mean(ray);
    cx0 = cx - local_mean(cx);
    cy0 = cy - local_mean(cy);
    cax0 = cax - local_mean(cax);
    cay0 = cay - local_mean(cay);
    orbit_radius = sqrt(rx0.^2 + ry0.^2);

    [freq_vx, vel_amp_x] = single_sided_spectrum(rvx0, fs);
    [freq_vy, vel_amp_y] = single_sided_spectrum(rvy0, fs);
    [freq_ax, acc_amp_x] = single_sided_spectrum(rax0, fs);
    [freq_ay, acc_amp_y] = single_sided_spectrum(ray0, fs);

    amp_1X = harmonic_amp(freq_ax, acc_amp_x, 165);
    amp_2X = harmonic_amp(freq_ax, acc_amp_x, 330);
    amp_3X = harmonic_amp(freq_ax, acc_amp_x, 495);
    ratio_2X_1X = amp_2X / max(amp_1X, eps);
    ratio_3X_1X = amp_3X / max(amp_1X, eps);

    result.signals = struct();
    result.signals.t = t;
    result.signals.rotor_x = rx;
    result.signals.rotor_y = ry;
    result.signals.rotor_vx = rvx;
    result.signals.rotor_vy = rvy;
    result.signals.rotor_ax = rax;
    result.signals.rotor_ay = ray;
    result.signals.case_x = cx;
    result.signals.case_y = cy;
    result.signals.case_vx = cvx;
    result.signals.case_vy = cvy;
    result.signals.case_ax = cax;
    result.signals.case_ay = cay;
    result.signals.orbit_x = rx0;
    result.signals.orbit_y = ry0;
    result.signals.orbit_radius = orbit_radius;
    result.spectrum = struct('freq_vx', freq_vx, 'vel_amp_x', vel_amp_x, ...
        'freq_vy', freq_vy, 'vel_amp_y', vel_amp_y, ...
        'freq_ax', freq_ax, 'acc_amp_x', acc_amp_x, ...
        'freq_ay', freq_ay, 'acc_amp_y', acc_amp_y);

    disp_peak = max([abs(rx0); abs(ry0)]);
    disp_pp = max([local_peak2peak(rx0), local_peak2peak(ry0)]);
    vel_peak = max([abs(rvx0); abs(rvy0)]);
    vel_rms = sqrt(local_mean(rvx0.^2 + rvy0.^2));
    acc_peak = max([abs(rax0); abs(ray0)]);
    acc_rms = sqrt(local_mean(rax0.^2 + ray0.^2));
    case_disp_peak = max([abs(cx0); abs(cy0)]);
    case_acc_rms = sqrt(local_mean(cax0.^2 + cay0.^2));

    result.metrics = table( ...
        {result.case_id}, result.external_load_N, result.data10_value, ...
        {result.load_application_type}, result.bearing_load_node_1, result.bearing_load_node_2, result.bearing_load_node_3, ...
        {result.load_direction}, {result.load_distribution_method}, ...
        result.actual_load_node_1, result.actual_load_node_2, result.actual_load_node_3, ...
        result.actual_case_node_1, result.actual_case_node_2, result.actual_case_node_3, ...
        result.rpm, 165, ...
        disp_peak, disp_pp, vel_peak, vel_rms, acc_peak, acc_rms, ...
        max(orbit_radius), amp_1X, amp_2X, amp_3X, ratio_2X_1X, ratio_3X_1X, ...
        case_disp_peak, case_acc_rms, {result.simulation_status}, ...
        'VariableNames', {'case_id','external_load_N','data10_value', ...
        'load_application_type','bearing_load_node_1','bearing_load_node_2','bearing_load_node_3', ...
        'load_direction','load_distribution_method', ...
        'actual_load_node_1','actual_load_node_2','actual_load_node_3', ...
        'actual_case_node_1','actual_case_node_2','actual_case_node_3', ...
        'rpm','oneX_Hz', ...
        'disp_peak','disp_pp','vel_peak','vel_rms','acc_peak','acc_rms', ...
        'orbit_radius_max','amp_1X','amp_2X','amp_3X','ratio_2X_1X','ratio_3X_1X', ...
        'case_disp_peak','case_acc_rms','simulation_status'});
end


function metrics = failed_metrics(result)
    metrics = table( ...
        {result.case_id}, result.external_load_N, result.data10_value, ...
        {result.load_application_type}, result.bearing_load_node_1, result.bearing_load_node_2, result.bearing_load_node_3, ...
        {result.load_direction}, {result.load_distribution_method}, ...
        result.actual_load_node_1, result.actual_load_node_2, result.actual_load_node_3, ...
        result.actual_case_node_1, result.actual_case_node_2, result.actual_case_node_3, ...
        result.rpm, 165, ...
        NaN, NaN, NaN, NaN, NaN, NaN, NaN, NaN, NaN, NaN, NaN, NaN, NaN, NaN, ...
        {result.simulation_status}, ...
        'VariableNames', {'case_id','external_load_N','data10_value', ...
        'load_application_type','bearing_load_node_1','bearing_load_node_2','bearing_load_node_3', ...
        'load_direction','load_distribution_method', ...
        'actual_load_node_1','actual_load_node_2','actual_load_node_3', ...
        'actual_case_node_1','actual_case_node_2','actual_case_node_3', ...
        'rpm','oneX_Hz', ...
        'disp_peak','disp_pp','vel_peak','vel_rms','acc_peak','acc_rms', ...
        'orbit_radius_max','amp_1X','amp_2X','amp_3X','ratio_2X_1X','ratio_3X_1X', ...
        'case_disp_peak','case_acc_rms','simulation_status'});
end


function plot_case_figures(result, fig_dir)
    tag = sprintf('%gN', result.external_load_N);
    t = result.signals.t;

    fig = figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 1120 780]);
    subplot(2,2,1)
    plot(t, result.signals.rotor_x, 'k', t, result.signals.rotor_y, 'b', 'LineWidth', 1.0)
    xlabel('t / s'); ylabel('u / m'); title('Rotor displacement')
    legend({'x','y'}, 'Location', 'best'); grid on
    subplot(2,2,2)
    plot(t, result.signals.rotor_vx, 'k', t, result.signals.rotor_vy, 'b', 'LineWidth', 1.0)
    xlabel('t / s'); ylabel('v / (m/s)'); title('Rotor velocity')
    legend({'x','y'}, 'Location', 'best'); grid on
    subplot(2,2,3)
    plot(result.spectrum.freq_vx, result.spectrum.vel_amp_x, 'k', 'LineWidth', 1.0)
    hold on; mark_harmonics(); hold off
    xlim([0 1000]); xlabel('f / Hz'); ylabel('|V_x|'); title('Velocity spectrum')
    grid on
    subplot(2,2,4)
    plot(result.spectrum.freq_ax, result.spectrum.acc_amp_x, 'k', 'LineWidth', 1.0)
    hold on; mark_harmonics(); hold off
    xlim([0 1000]); xlabel('f / Hz'); ylabel('|A_x|'); title('Acceleration spectrum')
    grid on
    sgtitle(sprintf('External load %g N', result.external_load_N))
    saveas(fig, fullfile(fig_dir, sprintf('case_external_load_%s_four_panel.png', tag)));
    close(fig)

    fig = figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 680 560]);
    plot(result.signals.orbit_x, result.signals.orbit_y, 'k', 'LineWidth', 1.2)
    axis equal
    grid on
    xlabel('x / m'); ylabel('y / m')
    title(sprintf('Rotor orbit: external load %g N', result.external_load_N))
    saveas(fig, fullfile(fig_dir, sprintf('case_external_load_%s_orbit.png', tag)));
    close(fig)
end


function plot_comparison_figures(results, fig_dir)
    valid = cellfun(@(r) isfield(r, 'signals') && ~strcmp(r.simulation_status, 'failed'), results);
    valid_results = results(valid);
    if isempty(valid_results)
        return
    end
    loads = cellfun(@(r) r.external_load_N, valid_results);
    colors = lines(numel(valid_results));

    plot_xy_time_compare(valid_results, colors, 'rotor_x', 'rotor_y', 'u / m', ...
        'Rotor displacement comparison under external loads', fullfile(fig_dir, 'fig_5_7_displacement_compare.png'));
    plot_xy_time_compare(valid_results, colors, 'rotor_vx', 'rotor_vy', 'v / (m/s)', ...
        'Rotor velocity comparison under external loads', fullfile(fig_dir, 'fig_5_8_velocity_compare.png'));
    plot_xy_time_compare(valid_results, colors, 'rotor_ax', 'rotor_ay', 'a / (m/s^2)', ...
        'Rotor acceleration comparison under external loads', fullfile(fig_dir, 'fig_5_9_acceleration_compare.png'));

    fig = figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 760 620]);
    hold on
    for ii = 1:numel(valid_results)
        plot(valid_results{ii}.signals.orbit_x, valid_results{ii}.signals.orbit_y, ...
            'LineWidth', 1.1, 'Color', colors(ii,:))
    end
    hold off
    axis equal
    grid on
    xlabel('x / m'); ylabel('y / m')
    title('Rotor orbit comparison under external loads')
    legend(load_labels(loads), 'Location', 'best')
    saveas(fig, fullfile(fig_dir, 'fig_5_10_orbit_compare.png'));
    close(fig)

    fig = figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 780 560]);
    hold on
    for ii = 1:numel(valid_results)
        plot(valid_results{ii}.spectrum.freq_ax, valid_results{ii}.spectrum.acc_amp_x, ...
            'LineWidth', 1.1, 'Color', colors(ii,:))
    end
    mark_harmonics()
    hold off
    xlim([0 1000])
    grid on
    xlabel('f / Hz'); ylabel('|A_x|')
    title('Acceleration spectrum comparison under external loads')
    legend(load_labels(loads), 'Location', 'best')
    saveas(fig, fullfile(fig_dir, 'fig_5_11_acc_spectrum_compare.png'));
    close(fig)

    rows = cellfun(@(r) r.metrics, valid_results, 'UniformOutput', false);
    tbl = vertcat(rows{:});
    fig = figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 980 760]);
    subplot(2,3,1); plot(loads, tbl.disp_pp, '-o', 'LineWidth', 1.2); grid on; xlabel('external load / N'); ylabel('disp pp / m')
    subplot(2,3,2); plot(loads, tbl.vel_rms, '-o', 'LineWidth', 1.2); grid on; xlabel('external load / N'); ylabel('vel RMS / (m/s)')
    subplot(2,3,3); plot(loads, tbl.acc_rms, '-o', 'LineWidth', 1.2); grid on; xlabel('external load / N'); ylabel('acc RMS / (m/s^2)')
    subplot(2,3,4); plot(loads, tbl.orbit_radius_max, '-o', 'LineWidth', 1.2); grid on; xlabel('external load / N'); ylabel('orbit radius / m')
    subplot(2,3,5); plot(loads, tbl.amp_1X, '-o', 'LineWidth', 1.2); grid on; xlabel('external load / N'); ylabel('1X amp')
    subplot(2,3,6); plot(loads, tbl.ratio_2X_1X, '-o', 'LineWidth', 1.2); grid on; xlabel('external load / N'); ylabel('2X/1X')
    sgtitle('Influence of external load on main response indices')
    saveas(fig, fullfile(fig_dir, 'fig_5_12_indicator_compare.png'));
    close(fig)
end


function plot_xy_time_compare(results, colors, field_x, field_y, y_label, title_text, file_name)
    fig = figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 850 620]);
    subplot(2,1,1)
    hold on
    for ii = 1:numel(results)
        plot(results{ii}.signals.t, results{ii}.signals.(field_x), ...
            'LineWidth', 1.0, 'Color', colors(ii,:))
    end
    hold off
    grid on
    ylabel([field_x(end) ' ' y_label])
    title(title_text)
    loads = cellfun(@(r) r.external_load_N, results);
    legend(load_labels(loads), 'Location', 'best')

    subplot(2,1,2)
    hold on
    for ii = 1:numel(results)
        plot(results{ii}.signals.t, results{ii}.signals.(field_y), ...
            'LineWidth', 1.0, 'Color', colors(ii,:))
    end
    hold off
    grid on
    xlabel('t / s'); ylabel([field_y(end) ' ' y_label])
    saveas(fig, file_name);
    close(fig)
end


function write_markdown_report(summary, fig_dir, out_file)
    fid = fopen(out_file, 'w', 'n', 'UTF-8');
    fprintf(fid, '# 5.2 外加载荷对转子-轴承-机匣系统振动响应的影响\n\n');
    fprintf(fid, '本研究固定转速 9900 r/min，仅改变外加载荷。外加载荷具体取值为 0 N、1000 N、2500 N、5000 N 和 7500 N，其余参数保持不变。\n\n');
    fprintf(fid, '外加载荷沿用 F_bearing.m 中 data(10) 的原始施加方式：F_r=[data(10);0;data(10);0]，作用于转子轴承节点 2 和 10，并通过 newmark_newton_multi.m 将反力装配到机匣节点 3 和 10。因此 external_load_N 表示两个轴承支承位置各自承受的 x 向等效径向载荷，不是系统总载荷分配值。\n\n');
    fprintf(fid, '完整统计结果见 external_load_sweep_9900_summary.xlsx，各工况完整时域和频域结果见 results_external_load_*N.mat。\n\n');
    fprintf(fid, '## 表 5.4 不同外加载荷下系统振动响应指标\n\n');
    fprintf(fid, '|case_id|load_N|disp_peak|disp_pp|vel_peak|vel_rms|acc_peak|acc_rms|orbit_R|amp_1X|amp_2X|amp_3X|2X/1X|3X/1X|status|\n');
    fprintf(fid, '|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|\n');
    for ii = 1:height(summary)
        fprintf(fid, '|%s|%.5g|%.4e|%.4e|%.4e|%.4e|%.4e|%.4e|%.4e|%.4e|%.4e|%.4e|%.5g|%.5g|%s|\n', ...
            summary.case_id{ii}, summary.external_load_N(ii), summary.disp_peak(ii), summary.disp_pp(ii), ...
            summary.vel_peak(ii), summary.vel_rms(ii), summary.acc_peak(ii), summary.acc_rms(ii), ...
            summary.orbit_radius_max(ii), summary.amp_1X(ii), summary.amp_2X(ii), summary.amp_3X(ii), ...
            summary.ratio_2X_1X(ii), summary.ratio_3X_1X(ii), summary.simulation_status{ii});
    end
    fprintf(fid, '\n');
    figs = {'fig_5_7_displacement_compare.png','fig_5_8_velocity_compare.png','fig_5_9_acceleration_compare.png', ...
        'fig_5_10_orbit_compare.png','fig_5_11_acc_spectrum_compare.png','fig_5_12_indicator_compare.png'};
    captions = {'图 5.7 不同外加载荷下转子节点位移响应对比','图 5.8 不同外加载荷下转子节点速度响应对比', ...
        '图 5.9 不同外加载荷下转子节点加速度响应对比','图 5.10 不同外加载荷下轴心轨迹对比', ...
        '图 5.11 不同外加载荷下加速度频谱对比','图 5.12 外加载荷变化对主要响应指标的影响'};
    for ii = 1:numel(figs)
        fprintf(fid, '![%s](%s)\n\n', captions{ii}, slash_path(fullfile(fig_dir, figs{ii})));
    end
    fclose(fid);
end


function [freq, amp] = single_sided_spectrum(y, fs)
    y = y(:);
    n = numel(y);
    y = y - local_mean(y);
    Y = fft(y);
    p2 = abs(Y / n);
    n1 = floor(n / 2) + 1;
    amp = p2(1:n1);
    if n1 > 2
        amp(2:end-1) = 2 * amp(2:end-1);
    end
    freq = (0:n1-1).' * fs / n;
end


function amp = harmonic_amp(freq, spec, target)
    [~, idx] = min(abs(freq - target));
    amp = spec(idx);
end


function y = local_peak2peak(x)
    y = max(x) - min(x);
end


function m = local_mean(x)
    x = x(:);
    if isempty(x)
        m = NaN;
    else
        m = sum(x) / numel(x);
    end
end


function labels = load_labels(loads)
    labels = arrayfun(@(s) sprintf('%g N', s), loads, 'UniformOutput', false);
end


function mark_harmonics()
    hs = [165, 330, 495];
    names = {'1X', '2X', '3X'};
    yl = ylim;
    for kk = 1:numel(hs)
        plot([hs(kk) hs(kk)], yl, 'r--', 'LineWidth', 0.8)
        text(hs(kk), yl(2) * 0.92, names{kk}, 'Color', 'r', ...
            'HorizontalAlignment', 'center', 'VerticalAlignment', 'top')
    end
    ylim(yl)
end


function out = slash_path(path_in)
    out = strrep(path_in, '\', '/');
end
