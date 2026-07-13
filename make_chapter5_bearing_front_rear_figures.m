%% Rebuild Chapter 5.3 figures with front/rear bearing comparison.
% Reads existing bearing-stiffness MAT outputs only; does not rerun dynamics.
% Figure filenames are kept the same as the report so the Word generator can
% reuse the old figure slots.

clearvars
close all
set(0, 'DefaultFigureVisible', 'off');
set(0, 'DefaultAxesFontName', 'Microsoft YaHei');
set(0, 'DefaultTextFontName', 'Microsoft YaHei');

fig_dir = fullfile(pwd, 'bearing_stiffness_sweep_9900_figures');
case_ids = {'K3','K4','K6','K7','K8'};
colors = chapter_palette();

results = cell(numel(case_ids), 1);
for ii = 1:numel(case_ids)
    data_in = load(fullfile(pwd, sprintf('results_bearing_stiffness_%s.mat', case_ids{ii})), 'result');
    results{ii} = add_front_rear_signals(data_in.result);
end

K_levels = cellfun(@(r) r.K_level, results);
labels = arrayfun(@(v) sprintf('%.0e N/m', v), K_levels, 'UniformOutput', false);
summary = build_front_rear_summary(results);
writetable(summary, fullfile(pwd, 'bearing_stiffness_front_rear_summary.csv'));
writetable(summary, fullfile(pwd, 'bearing_stiffness_front_rear_summary.xlsx'));

plot_time_xy_front_rear(results, colors, labels, 'x', 1e6, '位移 / μm', ...
    '不同轴承支承刚度下前后轴承处位移响应对比', ...
    fullfile(fig_dir, 'fig_5_13_displacement_compare.png'));

plot_time_xy_front_rear(results, colors, labels, 'v', 1e3, '速度 / (mm/s)', ...
    '不同轴承支承刚度下前后轴承处速度响应对比', ...
    fullfile(fig_dir, 'fig_5_14_velocity_compare.png'));

plot_time_xy_front_rear(results, colors, labels, 'a', 1, '加速度 / (m/s^2)', ...
    '不同轴承支承刚度下前后轴承处加速度响应对比', ...
    fullfile(fig_dir, 'fig_5_15_acceleration_compare.png'));

plot_orbit_front_rear(results, colors, labels, fullfile(fig_dir, 'fig_5_16_orbit_compare.png'));
plot_spectrum_front_rear(results, colors, labels, fullfile(fig_dir, 'fig_5_17_acc_spectrum_compare.png'));
plot_harmonics_front_rear(summary, fullfile(fig_dir, 'fig_5_19_harmonic_compare.png'));
plot_indicator_front_rear(summary, fullfile(fig_dir, 'fig_5_18_indicator_compare.png'));

fprintf('Chapter 5.3 front/rear bearing figures rebuilt.\n');
fprintf('Generated summary: bearing_stiffness_front_rear_summary.csv\n');


function colors = chapter_palette()
    colors = [
        0.0000 0.4470 0.7410
        0.8500 0.3250 0.0980
        0.9290 0.6940 0.1250
        0.4940 0.1840 0.5560
        0.4660 0.6740 0.1880
    ];
end


function result = add_front_rear_signals(result)
    idx = result.idx_plot;
    fs = result.fs;
    nodes = result.loc_rub(:).';
    result.front_node = nodes(1);
    result.rear_node = nodes(2);
    for ii = 1:2
        node = nodes(ii);
        prefix = ternary(ii == 1, 'front', 'rear');
        x_idx = 4 * node - 3;
        y_idx = 4 * node - 2;
        x = result.yn(x_idx, idx).';
        y = result.yn(y_idx, idx).';
        vx = result.dyn(x_idx, idx).';
        vy = result.dyn(y_idx, idx).';
        ax = result.ddyn(x_idx, idx).';
        ay = result.ddyn(y_idx, idx).';
        x0 = x - local_mean(x);
        y0 = y - local_mean(y);
        vx0 = vx - local_mean(vx);
        vy0 = vy - local_mean(vy);
        ax0 = ax - local_mean(ax);
        ay0 = ay - local_mean(ay);
        [freq_ax, acc_amp_x] = single_sided_spectrum(ax0, fs);

        result.frontrear.(prefix).node = node;
        result.frontrear.(prefix).x = x;
        result.frontrear.(prefix).y = y;
        result.frontrear.(prefix).vx = vx;
        result.frontrear.(prefix).vy = vy;
        result.frontrear.(prefix).ax = ax;
        result.frontrear.(prefix).ay = ay;
        result.frontrear.(prefix).orbit_x = x0;
        result.frontrear.(prefix).orbit_y = y0;
        result.frontrear.(prefix).orbit_radius = sqrt(x0.^2 + y0.^2);
        result.frontrear.(prefix).freq_ax = freq_ax;
        result.frontrear.(prefix).acc_amp_x = acc_amp_x;
        result.frontrear.(prefix).disp_pp = max([local_peak2peak(x0), local_peak2peak(y0)]);
        result.frontrear.(prefix).vel_rms = sqrt(local_mean(vx0.^2 + vy0.^2));
        result.frontrear.(prefix).acc_rms = sqrt(local_mean(ax0.^2 + ay0.^2));
        result.frontrear.(prefix).orbit_radius_max = max(result.frontrear.(prefix).orbit_radius);
        result.frontrear.(prefix).amp_1X = harmonic_amp(freq_ax, acc_amp_x, 165);
        result.frontrear.(prefix).amp_2X = harmonic_amp(freq_ax, acc_amp_x, 330);
        result.frontrear.(prefix).amp_3X = harmonic_amp(freq_ax, acc_amp_x, 495);
    end
end


function summary = build_front_rear_summary(results)
    summary = table();
    for ii = 1:numel(results)
        r = results{ii};
        f = r.frontrear.front;
        b = r.frontrear.rear;
        row = table({r.case_id}, r.K_level, f.node, b.node, ...
            f.disp_pp, b.disp_pp, rel_diff(b.disp_pp, f.disp_pp), ...
            f.vel_rms, b.vel_rms, rel_diff(b.vel_rms, f.vel_rms), ...
            f.acc_rms, b.acc_rms, rel_diff(b.acc_rms, f.acc_rms), ...
            f.orbit_radius_max, b.orbit_radius_max, rel_diff(b.orbit_radius_max, f.orbit_radius_max), ...
            f.amp_1X, b.amp_1X, rel_diff(b.amp_1X, f.amp_1X), ...
            f.amp_2X, b.amp_2X, f.amp_3X, b.amp_3X, ...
            'VariableNames', {'case_id','K_level','front_node','rear_node', ...
            'front_disp_pp','rear_disp_pp','disp_rel_diff', ...
            'front_vel_rms','rear_vel_rms','vel_rel_diff', ...
            'front_acc_rms','rear_acc_rms','acc_rel_diff', ...
            'front_orbit_radius_max','rear_orbit_radius_max','orbit_rel_diff', ...
            'front_amp_1X','rear_amp_1X','amp_1X_rel_diff', ...
            'front_amp_2X','rear_amp_2X','front_amp_3X','rear_amp_3X'});
        summary = [summary; row]; %#ok<AGROW>
    end
end


function plot_time_xy_front_rear(results, colors, labels, kind, scale, y_label, title_text, out_file)
    fig = figure('Visible', 'off', 'Color', 'w', 'Position', [80 80 1120 760]);
    tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
    field_map = struct('x', {{'x','y'}}, 'v', {{'vx','vy'}}, 'a', {{'ax','ay'}});
    fields = field_map.(kind);
    dirs = {'x 向', 'y 向'};
    for tile = 1:2
        nexttile
        hold on
        for ii = 1:numel(results)
            plot(results{ii}.signals.t, scale * results{ii}.frontrear.front.(fields{tile}), ...
                '-', 'LineWidth', 1.1, 'Color', colors(ii,:));
            plot(results{ii}.signals.t, scale * results{ii}.frontrear.rear.(fields{tile}), ...
                '--', 'LineWidth', 1.1, 'Color', colors(ii,:));
        end
        hold off
        grid on
        ylabel([dirs{tile} y_label])
        if tile == 1
            title(title_text)
            legend(build_legend(labels), 'Location', 'best', 'NumColumns', 2)
        else
            xlabel('时间 / s')
        end
    end
    exportgraphics(fig, out_file, 'Resolution', 220);
    close(fig)
end


function plot_orbit_front_rear(results, colors, labels, out_file)
    fig = figure('Visible', 'off', 'Color', 'w', 'Position', [80 80 900 740]);
    hold on
    for ii = 1:numel(results)
        plot(1e6 * results{ii}.frontrear.front.orbit_x, 1e6 * results{ii}.frontrear.front.orbit_y, ...
            '-', 'LineWidth', 1.1, 'Color', colors(ii,:));
        plot(1e6 * results{ii}.frontrear.rear.orbit_x, 1e6 * results{ii}.frontrear.rear.orbit_y, ...
            '--', 'LineWidth', 1.1, 'Color', colors(ii,:));
    end
    hold off
    axis equal
    grid on
    xlabel('x / μm')
    ylabel('y / μm')
    title('不同轴承支承刚度下前后轴承处轴心轨迹对比')
    legend(build_legend(labels), 'Location', 'best', 'NumColumns', 2)
    exportgraphics(fig, out_file, 'Resolution', 220);
    close(fig)
end


function plot_spectrum_front_rear(results, colors, labels, out_file)
    fig = figure('Visible', 'off', 'Color', 'w', 'Position', [80 80 1000 660]);
    hold on
    for ii = 1:numel(results)
        plot(results{ii}.frontrear.front.freq_ax, results{ii}.frontrear.front.acc_amp_x, ...
            '-', 'LineWidth', 1.1, 'Color', colors(ii,:));
        plot(results{ii}.frontrear.rear.freq_ax, results{ii}.frontrear.rear.acc_amp_x, ...
            '--', 'LineWidth', 1.1, 'Color', colors(ii,:));
    end
    mark_harmonics();
    hold off
    xlim([0 800])
    grid on
    xlabel('频率 / Hz')
    ylabel('加速度幅值 / (m/s^2)')
    title('不同轴承支承刚度下前后轴承处加速度频谱对比')
    legend(build_legend(labels), 'Location', 'best', 'NumColumns', 2)
    exportgraphics(fig, out_file, 'Resolution', 220);
    close(fig)
end


function plot_indicator_front_rear(summary, out_file)
    fig = figure('Visible', 'off', 'Color', 'w', 'Position', [80 80 1200 820]);
    tiledlayout(2, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
    metric_tile(summary.K_level, summary.front_disp_pp * 1e6, summary.rear_disp_pp * 1e6, '位移峰峰值 / μm', '位移峰峰值');
    metric_tile(summary.K_level, summary.front_vel_rms * 1e3, summary.rear_vel_rms * 1e3, '速度 RMS / (mm/s)', '速度 RMS');
    metric_tile(summary.K_level, summary.front_acc_rms, summary.rear_acc_rms, '加速度 RMS / (m/s^2)', '加速度 RMS');
    metric_tile(summary.K_level, summary.front_orbit_radius_max * 1e6, summary.rear_orbit_radius_max * 1e6, '轨迹半径 / μm', '轴心轨迹最大半径');
    metric_tile(summary.K_level, summary.front_amp_1X, summary.rear_amp_1X, '1X 加速度幅值 / (m/s^2)', '1X 加速度分量');
    metric_tile(summary.K_level, 100 * summary.vel_rel_diff, 100 * summary.acc_rel_diff, '相对差异 / %', '后轴承相对前轴承差异');
    sgtitle('轴承支承刚度变化对前后轴承响应指标的影响')
    exportgraphics(fig, out_file, 'Resolution', 220);
    close(fig)
end


function plot_harmonics_front_rear(summary, out_file)
    fig = figure('Visible', 'off', 'Color', 'w', 'Position', [80 80 1120 640]);
    tiledlayout(1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    nexttile
    hold on
    semilogx(summary.K_level, summary.front_amp_1X, '-o', 'LineWidth', 1.2);
    semilogx(summary.K_level, summary.rear_amp_1X, '--o', 'LineWidth', 1.2);
    semilogx(summary.K_level, summary.front_amp_2X, '-s', 'LineWidth', 1.2);
    semilogx(summary.K_level, summary.rear_amp_2X, '--s', 'LineWidth', 1.2);
    semilogx(summary.K_level, summary.front_amp_3X, '-^', 'LineWidth', 1.2);
    semilogx(summary.K_level, summary.rear_amp_3X, '--^', 'LineWidth', 1.2);
    hold off
    grid on
    xlabel('轴承支承刚度 / (N/m)')
    ylabel('加速度幅值 / (m/s^2)')
    title('前后轴承 1X、2X、3X 分量')
    legend({'前1X','后1X','前2X','后2X','前3X','后3X'}, 'Location', 'best')
    nexttile
    hold on
    semilogx(summary.K_level, summary.rear_amp_1X ./ max(summary.front_amp_1X, eps), '-o', 'LineWidth', 1.2);
    semilogx(summary.K_level, summary.rear_vel_rms ./ max(summary.front_vel_rms, eps), '-s', 'LineWidth', 1.2);
    semilogx(summary.K_level, summary.rear_acc_rms ./ max(summary.front_acc_rms, eps), '-^', 'LineWidth', 1.2);
    hold off
    grid on
    xlabel('轴承支承刚度 / (N/m)')
    ylabel('后/前幅值比')
    title('后轴承相对前轴承的响应比')
    legend({'1X幅值比','速度RMS比','加速度RMS比'}, 'Location', 'best')
    sgtitle('轴承支承刚度变化对前后轴承倍频分量及幅值比的影响')
    exportgraphics(fig, out_file, 'Resolution', 220);
    close(fig)
end


function metric_tile(x, front_y, rear_y, y_label, title_text)
    nexttile
    semilogx(x, front_y, '-o', 'LineWidth', 1.2, 'MarkerFaceColor', [0.00 0.45 0.74]);
    hold on
    semilogx(x, rear_y, '--s', 'LineWidth', 1.2, 'MarkerFaceColor', [0.85 0.33 0.10]);
    hold off
    grid on
    xlabel('轴承支承刚度 / (N/m)')
    ylabel(y_label)
    title(title_text)
    legend({'前轴承','后轴承'}, 'Location', 'best')
end


function labels_out = build_legend(labels)
    labels_out = cell(1, 2 * numel(labels));
    for ii = 1:numel(labels)
        labels_out{2 * ii - 1} = [labels{ii} ' 前'];
        labels_out{2 * ii} = [labels{ii} ' 后'];
    end
end


function r = rel_diff(rear_value, front_value)
    r = (rear_value - front_value) / max(abs(front_value), eps);
end


function s = ternary(cond, a, b)
    if cond
        s = a;
    else
        s = b;
    end
end


function [freq, amp] = single_sided_spectrum(x, fs)
    x = x(:);
    x = x - local_mean(x);
    n = numel(x);
    if n < 4
        freq = 0;
        amp = 0;
        return
    end
    w = hann(n);
    xw = x .* w;
    y = fft(xw);
    p2 = abs(y / sum(w) * 2);
    p1 = p2(1:floor(n/2)+1);
    p1(1) = p1(1) / 2;
    freq = fs * (0:floor(n/2)).' / n;
    amp = p1(:);
end


function amp = harmonic_amp(freq, spec, target)
    [~, idx] = min(abs(freq - target));
    amp = spec(idx);
end


function p = local_peak2peak(x)
    x = x(:);
    p = max(x) - min(x);
end


function m = local_mean(x)
    x = x(:);
    m = sum(x) / numel(x);
end


function mark_harmonics()
    hs = [165, 330, 495];
    names = {'1X', '2X', '3X'};
    yl = ylim;
    for kk = 1:numel(hs)
        plot([hs(kk) hs(kk)], yl, 'r--', 'LineWidth', 0.85)
        text(hs(kk), yl(2) * 0.92, names{kk}, 'Color', 'r', ...
            'HorizontalAlignment', 'center', 'VerticalAlignment', 'top', 'FontWeight', 'bold')
    end
    ylim(yl)
end
