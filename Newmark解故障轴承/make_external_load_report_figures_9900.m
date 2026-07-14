%% Rebuild external-load comparison figures from saved simulation data
% This script does not rerun the dynamic solver. It only reads the existing
% results_external_load_*N.mat files and external_load_sweep_9900_summary.xlsx.

clearvars
clc
close all

set(0, 'DefaultFigureVisible', 'off');
set(groot, 'defaultAxesFontName', 'Microsoft YaHei');
set(groot, 'defaultTextFontName', 'Microsoft YaHei');

loads = [0, 1000, 2500, 5000, 7500];
mat_files = arrayfun(@(v) sprintf('results_external_load_%gN.mat', v), loads, 'UniformOutput', false);
fig_dir = fullfile(pwd, 'external_load_sweep_9900_report_figures');
if ~exist(fig_dir, 'dir')
    mkdir(fig_dir);
end

results = cell(numel(mat_files), 1);
for i = 1:numel(mat_files)
    s = load(mat_files{i}, 'result');
    results{i} = s.result;
end

summary = readtable('external_load_sweep_9900_summary.xlsx');
valid = strcmp(summary.simulation_status, 'completed') | strcmp(summary.simulation_status, 'abnormal');
summary = summary(valid, :);
results = results(valid);
loads = loads(valid);

colors = lines(numel(results));
labels = arrayfun(@(v) sprintf('%g N', v), loads, 'UniformOutput', false);

plot_time_xy(results, colors, labels, 'rotor_x', 'rotor_y', 1e6, '位移 / um', ...
    '不同外加载荷下转子观测节点位移响应对比', fullfile(fig_dir, 'fig_5_7_displacement_compare.png'));
plot_time_xy(results, colors, labels, 'rotor_vx', 'rotor_vy', 1e3, '速度 / (mm/s)', ...
    '不同外加载荷下转子观测节点速度响应对比', fullfile(fig_dir, 'fig_5_8_velocity_compare.png'));
plot_time_xy(results, colors, labels, 'rotor_ax', 'rotor_ay', 1, '加速度 / (m/s^2)', ...
    '不同外加载荷下转子观测节点加速度响应对比', fullfile(fig_dir, 'fig_5_9_acceleration_compare.png'));
plot_orbits(results, colors, labels, fullfile(fig_dir, 'fig_5_10_orbit_compare.png'));
plot_acc_spectrum(results, colors, labels, fullfile(fig_dir, 'fig_5_11_acc_spectrum_compare.png'));
plot_metrics(summary, fullfile(fig_dir, 'fig_5_12_indicator_compare.png'));
plot_harmonics(summary, fullfile(fig_dir, 'fig_5_13_harmonic_compare.png'));

fprintf('Rebuilt report figures in: %s\n', fig_dir);


function plot_time_xy(results, colors, labels, field_x, field_y, scale, y_label, title_text, out_file)
    fig = figure('Visible', 'off', 'Color', 'w', 'Position', [80 80 1100 720]);
    tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

    nexttile
    hold on
    for i = 1:numel(results)
        t = results{i}.signals.t;
        plot(t, scale * results{i}.signals.(field_x), 'LineWidth', 1.15, 'Color', colors(i,:));
    end
    hold off
    grid on
    ylabel(['x 向' y_label])
    title(title_text)
    legend(labels, 'Location', 'best', 'NumColumns', 3)

    nexttile
    hold on
    for i = 1:numel(results)
        t = results{i}.signals.t;
        plot(t, scale * results{i}.signals.(field_y), 'LineWidth', 1.15, 'Color', colors(i,:));
    end
    hold off
    grid on
    xlabel('时间 / s')
    ylabel(['y 向' y_label])

    exportgraphics(fig, out_file, 'Resolution', 220);
    close(fig)
end


function plot_orbits(results, colors, labels, out_file)
    fig = figure('Visible', 'off', 'Color', 'w', 'Position', [80 80 900 760]);
    hold on
    for i = 1:numel(results)
        x = 1e6 * results{i}.signals.orbit_x;
        y = 1e6 * results{i}.signals.orbit_y;
        plot(x, y, 'LineWidth', 1.2, 'Color', colors(i,:));
    end
    hold off
    axis equal
    grid on
    xlabel('x 向振动位移 / um')
    ylabel('y 向振动位移 / um')
    title('不同外加载荷下轴心轨迹对比')
    legend(labels, 'Location', 'best')
    exportgraphics(fig, out_file, 'Resolution', 220);
    close(fig)
end


function plot_acc_spectrum(results, colors, labels, out_file)
    fig = figure('Visible', 'off', 'Color', 'w', 'Position', [80 80 1000 680]);
    hold on
    for i = 1:numel(results)
        plot(results{i}.spectrum.freq_ax, results{i}.spectrum.acc_amp_x, ...
            'LineWidth', 1.15, 'Color', colors(i,:));
    end
    mark_harmonics();
    hold off
    xlim([0 800])
    grid on
    xlabel('频率 / Hz')
    ylabel('加速度幅值 / (m/s^2)')
    title('不同外加载荷下转子 x 向加速度频谱对比')
    legend(labels, 'Location', 'northeast')
    exportgraphics(fig, out_file, 'Resolution', 220);
    close(fig)
end


function plot_metrics(summary, out_file)
    load_N = summary.external_load_N;
    fig = figure('Visible', 'off', 'Color', 'w', 'Position', [80 80 1220 820]);
    tiledlayout(2, 3, 'TileSpacing', 'compact', 'Padding', 'compact');

    plot_metric_tile(load_N, summary.disp_pp * 1e6, '位移峰峰值 / um', '位移峰峰值');
    plot_metric_tile(load_N, summary.vel_rms * 1e3, '速度 RMS / (mm/s)', '速度 RMS');
    plot_metric_tile(load_N, summary.acc_rms, '加速度 RMS / (m/s^2)', '加速度 RMS');
    plot_metric_tile(load_N, summary.orbit_radius_max * 1e6, '轴心轨迹半径 / um', '轴心轨迹最大半径');
    plot_metric_tile(load_N, summary.amp_1X, '1X 加速度幅值', '1X 分量');
    plot_metric_tile(load_N, summary.ratio_2X_1X, '2X/1X', '倍频占比');

    sgtitle('外加载荷变化对主要振动响应指标的影响')
    exportgraphics(fig, out_file, 'Resolution', 220);
    close(fig)
end


function plot_harmonics(summary, out_file)
    load_N = summary.external_load_N;
    fig = figure('Visible', 'off', 'Color', 'w', 'Position', [80 80 1120 620]);
    tiledlayout(1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

    nexttile
    hold on
    plot(load_N, summary.amp_1X, '-o', 'LineWidth', 1.2, 'MarkerFaceColor', [0.00 0.45 0.74]);
    plot(load_N, summary.amp_2X, '-s', 'LineWidth', 1.2, 'MarkerFaceColor', [0.85 0.33 0.10]);
    plot(load_N, summary.amp_3X, '-^', 'LineWidth', 1.2, 'MarkerFaceColor', [0.47 0.67 0.19]);
    hold off
    grid on
    xlabel('外加载荷 / N')
    ylabel('加速度谱幅值')
    title('1X、2X、3X 幅值随外加载荷变化')
    legend({'1X=165 Hz','2X=330 Hz','3X=495 Hz'}, 'Location', 'best')

    nexttile
    hold on
    plot(load_N, summary.ratio_2X_1X, '-o', 'LineWidth', 1.2, 'MarkerFaceColor', [0.85 0.33 0.10]);
    plot(load_N, summary.ratio_3X_1X, '-s', 'LineWidth', 1.2, 'MarkerFaceColor', [0.47 0.67 0.19]);
    hold off
    grid on
    xlabel('外加载荷 / N')
    ylabel('幅值比')
    title('倍频分量相对 1X 的变化')
    legend({'2X/1X','3X/1X'}, 'Location', 'best')

    exportgraphics(fig, out_file, 'Resolution', 220);
    close(fig)
end


function plot_metric_tile(x, y, y_label, title_text)
    nexttile
    plot(x, y, '-o', 'Color', [0.10 0.35 0.70], 'LineWidth', 1.25, ...
        'MarkerFaceColor', [0.10 0.35 0.70], 'MarkerSize', 5.5);
    grid on
    xlabel('外加载荷 / N')
    ylabel(y_label)
    title(title_text)
    for i = 1:numel(x)
        text(x(i), y(i), sprintf(' %.3g', y(i)), 'FontSize', 8, ...
            'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'left');
    end
end


function mark_harmonics()
    hs = [165, 330, 495];
    names = {'1X', '2X', '3X'};
    yl = ylim;
    for k = 1:numel(hs)
        plot([hs(k), hs(k)], yl, 'k--', 'LineWidth', 0.85)
        text(hs(k), yl(2) * 0.92, names{k}, 'Color', 'k', ...
            'HorizontalAlignment', 'center', 'VerticalAlignment', 'top', ...
            'FontWeight', 'bold');
    end
    ylim(yl)
end
