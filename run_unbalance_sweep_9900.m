%% Unbalance sweep at 9900 r/min for the rotor-bearing-casing Newmark model
% This script keeps all structural, bearing, damping, clearance and load
% parameters unchanged. Only the four-disk unbalance vector is scaled.

clearvars
clc
close all

scale_list = [0, 0.5, 1, 2, 5, 8, 10];
rpm_fixed = 9900;
f_rot = rpm_fixed / 60;
harmonics = [1, 2, 3] * f_rot;

fig_dir = fullfile(pwd, 'unbalance_sweep_9900_figures');
if ~exist(fig_dir, 'dir')
    mkdir(fig_dir);
end

summary = table();
results = cell(numel(scale_list), 1);

for ii = 1:numel(scale_list)
    scale = scale_list(ii);
    mat_name = sprintf('results_unbalance_scale_%s.mat', scale_to_tag(scale));
    fprintf('\n========== Unbalance scale %.3g (%d/%d) ==========\n', scale, ii, numel(scale_list));
    result = run_single_unbalance_case(scale, fig_dir);
    result.case_id = ii;
    result.case_name = sprintf('scale_%s', scale_to_tag(scale));
    result.f_rot = f_rot;
    result.harmonics = harmonics;
    if ~isfield(result, 'metrics')
        result = compute_case_metrics(result);
    end
    if ~ismember('case_id', result.metrics.Properties.VariableNames)
        result.metrics.case_id = ii;
    else
        result.metrics.case_id(:) = ii;
    end
    results{ii} = result;
    save(mat_name, 'result', '-v7.3');

    plot_case_figures(result, fig_dir);
    if isempty(summary)
        summary = result.metrics;
    else
        summary = [summary; result.metrics]; %#ok<AGROW>
    end
    close all
end

summary = movevars(summary, 'case_id', 'Before', 1);
writetable(summary, 'unbalance_sweep_9900_summary.xlsx');
writetable(summary, 'unbalance_sweep_9900_summary.csv');

plot_comparison_figures(results, fig_dir);
write_markdown_report(summary, results, fig_dir, 'unbalance_sweep_9900_report.md');

fprintf('\nGenerated:\n');
fprintf('  unbalance_sweep_9900_summary.xlsx\n');
fprintf('  unbalance_sweep_9900_report.md\n');
fprintf('  results_unbalance_scale_*.mat\n');
fprintf('  %s\n', fig_dir);


function result = run_single_unbalance_case(scale, fig_dir)
    set(0, 'DefaultFigureVisible', 'off');
    unbalance_scale = scale; %#ok<NASGU>
    suppress_main_plots = true; %#ok<NASGU>
    sweep_output_dir = fig_dir; %#ok<NASGU>
    run('mian.m');

    result = struct();
    result.unbalance_scale = unbalance_scale_used;
    result.rpm = rpm_used;
    result.wi = wi;
    result.U_base = U_base;
    result.U = U_current;
    result.Famp1 = Famp1;
    result.fai = fai;
    result.data = data;
    result.loca = loca;
    result.loc_rub = loc_rub;
    result.N = N;
    result.N_C = N_C;
    result.num_rotor = num_rotor;
    result.c1 = c1;
    result.c2 = c2;
    result.r1 = r1;
    result.r2 = r2;
    result.Fen = Fen;
    result.n_Fen = n_Fen;
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
    result.sweep_output_dir = sweep_output_dir;
end


function result = compute_case_metrics(result)
    idx = result.idx_plot;
    t = result.time_plot(:);
    fs = result.fs;
    rotor_node = result.loc_rub(1);
    rear_rotor_node = result.loc_rub(2);
    disk_node = result.loca(3);
    case_node = result.c1;

    rotor_x_idx = 4 * rotor_node - 3;
    rotor_y_idx = 4 * rotor_node - 2;
    rear_rotor_x_idx = 4 * rear_rotor_node - 3;
    rear_rotor_y_idx = 4 * rear_rotor_node - 2;
    disk_x_idx = 4 * disk_node - 3;
    disk_y_idx = 4 * disk_node - 2;
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

    rear_x = result.yn(rear_rotor_x_idx, idx).';
    rear_y = result.yn(rear_rotor_y_idx, idx).';
    disk_x = result.yn(disk_x_idx, idx).';
    disk_y = result.yn(disk_y_idx, idx).';

    rx0 = rx - local_mean(rx);
    ry0 = ry - local_mean(ry);
    orbit_radius = sqrt(rx0.^2 + ry0.^2);
    [freq, acc_amp] = single_sided_spectrum(rax - local_mean(rax), fs);
    [freq_case, case_acc_amp] = single_sided_spectrum(cax - local_mean(cax), fs);
    [~, vel_amp] = single_sided_spectrum(rvx - local_mean(rvx), fs);

    amp_1X = harmonic_amp(freq, acc_amp, 165);
    amp_2X = harmonic_amp(freq, acc_amp, 330);
    amp_3X = harmonic_amp(freq, acc_amp, 495);
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
    result.signals.rear_rotor_x = rear_x;
    result.signals.rear_rotor_y = rear_y;
    result.signals.disk_x = disk_x;
    result.signals.disk_y = disk_y;
    result.spectrum = struct('freq', freq, 'acc_amp', acc_amp, ...
        'freq_case', freq_case, 'case_acc_amp', case_acc_amp, 'vel_amp', vel_amp);

    result.metrics = table( ...
        result.unbalance_scale, ...
        result.U(1), result.U(2), result.U(3), result.U(4), ...
        result.Famp1(1), result.Famp1(2), result.Famp1(3), result.Famp1(4), ...
        max(abs(rx0)), peak2peak(rx0), ...
        max(abs(rvx - local_mean(rvx))), rms_local(rvx - local_mean(rvx)), ...
        max(abs(rax - local_mean(rax))), rms_local(rax - local_mean(rax)), ...
        max(orbit_radius), amp_1X, amp_2X, amp_3X, ratio_2X_1X, ratio_3X_1X, ...
        max(abs(cx - local_mean(cx))), rms_local(cax - local_mean(cax)), ...
        'VariableNames', {'unbalance_scale', ...
        'U1','U2','U3','U4', ...
        'Famp1_1','Famp1_2','Famp1_3','Famp1_4', ...
        'disp_peak','disp_pp','vel_peak','vel_rms','acc_peak','acc_rms', ...
        'orbit_radius_max','amp_1X','amp_2X','amp_3X','ratio_2X_1X','ratio_3X_1X', ...
        'case_disp_peak','case_acc_rms'});
end


function plot_case_figures(result, fig_dir)
    tag = scale_to_tag(result.unbalance_scale);
    t = result.signals.t;
    freq = result.spectrum.freq;
    freq_case = result.spectrum.freq_case;

    fig = figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 1100 760]);
    subplot(2,2,1)
    plot(t, result.signals.rotor_ax, 'k', 'LineWidth', 1.0)
    xlabel('t / s'); ylabel('a_x / (m/s^2)'); title('转子节点加速度时域')
    grid on
    subplot(2,2,2)
    plot(t, result.signals.case_ax, 'k', 'LineWidth', 1.0)
    xlabel('t / s'); ylabel('a_x / (m/s^2)'); title('机匣节点加速度时域')
    grid on
    subplot(2,2,3)
    plot(freq, result.spectrum.acc_amp, 'k', 'LineWidth', 1.0)
    hold on; mark_harmonics(); hold off
    xlim([0 1000]); xlabel('f / Hz'); ylabel('|A|'); title('转子节点加速度频谱')
    grid on
    subplot(2,2,4)
    plot(freq_case, result.spectrum.case_acc_amp, 'k', 'LineWidth', 1.0)
    hold on; mark_harmonics(); hold off
    xlim([0 1000]); xlabel('f / Hz'); ylabel('|A|'); title('机匣节点加速度频谱')
    grid on
    sgtitle(sprintf('不平衡放大系数 %.3g', result.unbalance_scale))
    saveas(fig, fullfile(fig_dir, sprintf('case_scale_%s_four_panel.png', tag)));
    close(fig)

    fig = figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 680 560]);
    x = result.signals.rotor_x - local_mean(result.signals.rotor_x);
    y = result.signals.rotor_y - local_mean(result.signals.rotor_y);
    plot(x, y, 'k', 'LineWidth', 1.2)
    axis equal
    grid on
    xlabel('x / m'); ylabel('y / m')
    title(sprintf('轴心轨迹：unbalance\\_scale = %.3g', result.unbalance_scale))
    saveas(fig, fullfile(fig_dir, sprintf('case_scale_%s_orbit.png', tag)));
    close(fig)
end


function plot_comparison_figures(results, fig_dir)
    scales = cellfun(@(r) r.unbalance_scale, results);
    colors = lines(numel(results));

    plot_time_compare(results, colors, 'rotor_x', 'x / m', ...
        '不同不平衡量下转子节点位移响应对比', fullfile(fig_dir, 'fig_5_1_displacement_compare.png'));
    plot_time_compare(results, colors, 'rotor_vx', 'v_x / (m/s)', ...
        '不同不平衡量下转子节点速度响应对比', fullfile(fig_dir, 'fig_5_2_velocity_compare.png'));
    plot_time_compare(results, colors, 'rotor_ax', 'a_x / (m/s^2)', ...
        '不同不平衡量下转子节点加速度响应对比', fullfile(fig_dir, 'fig_5_3_acceleration_compare.png'));

    fig = figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 760 620]);
    hold on
    for ii = 1:numel(results)
        x = results{ii}.signals.rotor_x - local_mean(results{ii}.signals.rotor_x);
        y = results{ii}.signals.rotor_y - local_mean(results{ii}.signals.rotor_y);
        plot(x, y, 'LineWidth', 1.1, 'Color', colors(ii,:))
    end
    hold off
    axis equal
    grid on
    xlabel('x / m'); ylabel('y / m')
    title('不同不平衡量下轴心轨迹对比')
    legend(scale_labels(scales), 'Location', 'best')
    saveas(fig, fullfile(fig_dir, 'fig_5_4_orbit_compare.png'));
    close(fig)

    fig = figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 780 560]);
    hold on
    for ii = 1:numel(results)
        plot(results{ii}.spectrum.freq, results{ii}.spectrum.acc_amp, ...
            'LineWidth', 1.1, 'Color', colors(ii,:))
    end
    mark_harmonics()
    hold off
    xlim([0 1000])
    grid on
    xlabel('f / Hz'); ylabel('|A|')
    title('不同不平衡量下加速度频谱对比')
    legend(scale_labels(scales), 'Location', 'best')
    saveas(fig, fullfile(fig_dir, 'fig_5_5_acc_spectrum_compare.png'));
    close(fig)

    rows = cellfun(@(r) r.metrics, results, 'UniformOutput', false);
    tbl = vertcat(rows{:});
    fig = figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 980 760]);
    subplot(2,3,1); plot(scales, tbl.disp_pp, '-o', 'LineWidth', 1.2); grid on; xlabel('unbalance\_scale'); ylabel('disp pp / m')
    subplot(2,3,2); plot(scales, tbl.vel_rms, '-o', 'LineWidth', 1.2); grid on; xlabel('unbalance\_scale'); ylabel('vel RMS / (m/s)')
    subplot(2,3,3); plot(scales, tbl.acc_rms, '-o', 'LineWidth', 1.2); grid on; xlabel('unbalance\_scale'); ylabel('acc RMS / (m/s^2)')
    subplot(2,3,4); plot(scales, tbl.amp_1X, '-o', 'LineWidth', 1.2); grid on; xlabel('unbalance\_scale'); ylabel('1X amp')
    subplot(2,3,5); plot(scales, tbl.ratio_2X_1X, '-o', 'LineWidth', 1.2); grid on; xlabel('unbalance\_scale'); ylabel('2X/1X')
    subplot(2,3,6); plot(scales, tbl.ratio_3X_1X, '-o', 'LineWidth', 1.2); grid on; xlabel('unbalance\_scale'); ylabel('3X/1X')
    sgtitle('不平衡量变化对系统响应指标的影响')
    saveas(fig, fullfile(fig_dir, 'fig_5_6_indicator_compare.png'));
    close(fig)
end


function plot_time_compare(results, colors, field_name, y_label, title_text, file_name)
    fig = figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 780 520]);
    hold on
    for ii = 1:numel(results)
        plot(results{ii}.signals.t, results{ii}.signals.(field_name), ...
            'LineWidth', 1.1, 'Color', colors(ii,:))
    end
    hold off
    grid on
    xlabel('t / s'); ylabel(y_label)
    title(title_text)
    scales = cellfun(@(r) r.unbalance_scale, results);
    legend(scale_labels(scales), 'Location', 'best')
    saveas(fig, file_name);
    close(fig)
end


function write_markdown_report(summary, results, fig_dir, out_file)
    scales = summary.unbalance_scale;
    disp_growth = pct_change(summary.disp_pp(1), summary.disp_pp(end));
    vel_growth = pct_change(summary.vel_rms(1), summary.vel_rms(end));
    acc_growth = pct_change(summary.acc_rms(1), summary.acc_rms(end));
    one_x_dominant = all(summary.amp_1X >= summary.amp_2X & summary.amp_1X >= summary.amp_3X);
    ratio2_growth = summary.ratio_2X_1X(end) - summary.ratio_2X_1X(1);
    case_growth = pct_change(summary.case_acc_rms(1), summary.case_acc_rms(end));

    fid = fopen(out_file, 'w', 'n', 'UTF-8');
    fprintf(fid, '# 不平衡激励对转子-轴承-机匣系统振动响应的影响\n\n');
    fprintf(fid, '本文固定转速为 9900 r/min，转频 1X=165 Hz，保持轴承刚度、阻尼、外载荷、径向游隙、基础支承刚度和其他结构参数不变，仅改变四个轮盘的不平衡量。四个不平衡量按相同比例系数整体放大或减小，比例系数为 `%s`。\n\n', mat2str(scales.'));
    fprintf(fid, '不平衡量与离心力幅值的计算关系为：`U_i = m_i e_i`，`F_i = U_i * wi^2`。由于本节固定转速，工况间不平衡力幅值变化完全由 `U_i` 的比例放大引起。\n\n');
    fprintf(fid, '从统计结果看，位移峰峰值由 %.4e m 变化至 %.4e m，变化幅度为 %.2f%%；速度 RMS 由 %.4e m/s 变化至 %.4e m/s，变化幅度为 %.2f%%；加速度 RMS 由 %.4e m/s^2 变化至 %.4e m/s^2，变化幅度为 %.2f%%。\n\n', ...
        summary.disp_pp(1), summary.disp_pp(end), disp_growth, summary.vel_rms(1), summary.vel_rms(end), vel_growth, summary.acc_rms(1), summary.acc_rms(end), acc_growth);
    if one_x_dominant
        fprintf(fid, '频谱中 1X 成分在各工况下均高于 2X 与 3X 成分，说明系统响应主要受同步不平衡激励控制。');
    else
        fprintf(fid, '部分工况中 2X 或 3X 成分接近或超过 1X 成分，说明响应中已经出现更明显的非同步或非线性成分。');
    end
    fprintf(fid, '2X/1X 幅值比从 %.4g 变化至 %.4g，变化量为 %.4g。机匣节点加速度 RMS 从 %.4e m/s^2 变化至 %.4e m/s^2，变化幅度为 %.2f%%，反映转子侧振动能量经轴承支承路径向机匣传递。\n\n', ...
        summary.ratio_2X_1X(1), summary.ratio_2X_1X(end), ratio2_growth, summary.case_acc_rms(1), summary.case_acc_rms(end), case_growth);
    fprintf(fid, '## 主要图件\n\n');
    fprintf(fid, '![图5.1 不同不平衡量下转子节点位移响应对比](%s)\n\n', slash_path(fullfile(fig_dir, 'fig_5_1_displacement_compare.png')));
    fprintf(fid, '![图5.2 不同不平衡量下转子节点速度响应对比](%s)\n\n', slash_path(fullfile(fig_dir, 'fig_5_2_velocity_compare.png')));
    fprintf(fid, '![图5.3 不同不平衡量下转子节点加速度响应对比](%s)\n\n', slash_path(fullfile(fig_dir, 'fig_5_3_acceleration_compare.png')));
    fprintf(fid, '![图5.4 不同不平衡量下轴心轨迹对比](%s)\n\n', slash_path(fullfile(fig_dir, 'fig_5_4_orbit_compare.png')));
    fprintf(fid, '![图5.5 不同不平衡量下加速度频谱对比](%s)\n\n', slash_path(fullfile(fig_dir, 'fig_5_5_acc_spectrum_compare.png')));
    fprintf(fid, '![图5.6 不平衡量变化对系统响应指标的影响](%s)\n\n', slash_path(fullfile(fig_dir, 'fig_5_6_indicator_compare.png')));
    fprintf(fid, '## 指标表\n\n');
    fprintf(fid, '指标汇总见 `unbalance_sweep_9900_summary.xlsx`。\n');
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


function y = rms_local(x)
    x = x(:);
    y = sqrt(local_mean(x.^2));
end


function y = peak2peak(x)
    y = max(x) - min(x);
end


function tag = scale_to_tag(scale)
    tag = strrep(sprintf('%g', scale), '.', 'p');
end


function labels = scale_labels(scales)
    labels = arrayfun(@(s) sprintf('scale=%g', s), scales, 'UniformOutput', false);
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


function value = pct_change(a, b)
    if abs(a) < eps
        value = NaN;
    else
        value = (b - a) / abs(a) * 100;
    end
end


function out = slash_path(path_in)
    out = strrep(path_in, '\', '/');
end


function m = local_mean(x)
    x = x(:);
    if isempty(x)
        m = NaN;
    else
        m = sum(x) / numel(x);
    end
end
