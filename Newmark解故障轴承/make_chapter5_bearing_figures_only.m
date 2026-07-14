%% Rebuild only Chapter 5 bearing-stiffness figures.
% This script uses existing real MAT/CSV outputs and does not rerun dynamics.
% K=1e7 N/m is a recorded abnormal boundary case, so it is excluded from
% the main comparison figures to keep engineering-range curves readable.

clearvars
close all
set(0, 'DefaultFigureVisible', 'off');
set(0, 'DefaultAxesFontName', 'Microsoft YaHei');
set(0, 'DefaultTextFontName', 'Microsoft YaHei');

fig_dir = fullfile(pwd, 'bearing_stiffness_sweep_9900_figures');
summary_all = readtable(fullfile(pwd, 'bearing_stiffness_sweep_9900_summary.csv'));

use_mask = strcmp(string(summary_all.simulation_status), "completed") ...
    & summary_all.disp_pp < 1e-3 ...
    & summary_all.vel_rms < 10 ...
    & summary_all.acc_rms < 1e5;
summary = summary_all(use_mask, :);

results = cell(height(summary), 1);
for ii = 1:height(summary)
    data_in = load(fullfile(pwd, sprintf('results_bearing_stiffness_%s.mat', summary.case_id{ii})), 'result');
    results{ii} = data_in.result;
end

colors = lines(numel(results));
K_levels = summary.K_level;
labels = arrayfun(@(v) sprintf('%.0e N/m', v), K_levels, 'UniformOutput', false);

plot_time_xy(results, colors, labels, 'rotor_x', 'rotor_y', 1e6, '位移 / μm', ...
    '不同轴承支承刚度下转子轴承处位移响应对比', ...
    fullfile(fig_dir, 'fig_5_13_displacement_compare.png'));

plot_time_xy(results, colors, labels, 'rotor_vx', 'rotor_vy', 1e3, '速度 / (mm/s)', ...
    '不同轴承支承刚度下转子轴承处速度响应对比', ...
    fullfile(fig_dir, 'fig_5_14_velocity_compare.png'));

plot_time_xy(results, colors, labels, 'rotor_ax', 'rotor_ay', 1, '加速度 / (m/s^2)', ...
    '不同轴承支承刚度下转子轴承处加速度响应对比', ...
    fullfile(fig_dir, 'fig_5_15_acceleration_compare.png'));

plot_orbit(results, colors, labels, fullfile(fig_dir, 'fig_5_16_orbit_compare.png'), ...
    '不同轴承支承刚度下转子轴承处轴心轨迹对比');

plot_acc_spectrum(results, colors, labels, fullfile(fig_dir, 'fig_5_17_acc_spectrum_compare.png'), ...
    '不同轴承支承刚度下转子轴承处加速度频谱对比');

plot_indicator(K_levels, summary, fullfile(fig_dir, 'fig_5_18_indicator_compare.png'));

plot_harmonics(K_levels, summary, fullfile(fig_dir, 'fig_5_19_harmonic_compare.png'));

fprintf('Chapter 5 bearing-stiffness figures rebuilt from existing real outputs.\n');
fprintf('Included engineering-range cases: %s\n', strjoin(labels, ', '));


function plot_time_xy(results, colors, labels, field_x, field_y, scale, y_label, title_text, out_file)
    fig = figure('Visible', 'off', 'Color', 'w', 'Position', [80 80 1100 720]);
    tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

    nexttile
    hold on
    for ii = 1:numel(results)
        plot(results{ii}.signals.t, scale * results{ii}.signals.(field_x), ...
            'LineWidth', 1.1, 'Color', colors(ii,:));
    end
    hold off
    grid on
    ylabel(['x 向' y_label])
    title(title_text)
    legend(labels, 'Location', 'best', 'NumColumns', 3)

    nexttile
    hold on
    for ii = 1:numel(results)
        plot(results{ii}.signals.t, scale * results{ii}.signals.(field_y), ...
            'LineWidth', 1.1, 'Color', colors(ii,:));
    end
    hold off
    grid on
    xlabel('时间 / s')
    ylabel(['y 向' y_label])
    exportgraphics(fig, out_file, 'Resolution', 220);
    close(fig)
end


function plot_orbit(results, colors, labels, out_file, title_text)
    fig = figure('Visible', 'off', 'Color', 'w', 'Position', [80 80 860 720]);
    hold on
    for ii = 1:numel(results)
        x = results{ii}.signals.orbit_x;
        y = results{ii}.signals.orbit_y;
        plot(1e6 * x, 1e6 * y, 'LineWidth', 1.15, 'Color', colors(ii,:));
    end
    hold off
    axis equal
    grid on
    xlabel('x / μm')
    ylabel('y / μm')
    title(title_text)
    legend(labels, 'Location', 'best')
    exportgraphics(fig, out_file, 'Resolution', 220);
    close(fig)
end


function plot_acc_spectrum(results, colors, labels, out_file, title_text)
    fig = figure('Visible', 'off', 'Color', 'w', 'Position', [80 80 980 640]);
    hold on
    for ii = 1:numel(results)
        plot(results{ii}.spectrum.freq_ax, results{ii}.spectrum.acc_amp_x, ...
            'LineWidth', 1.1, 'Color', colors(ii,:));
    end
    mark_harmonics();
    hold off
    xlim([0 800])
    grid on
    xlabel('频率 / Hz')
    ylabel('加速度幅值 / (m/s^2)')
    title(title_text)
    legend(labels, 'Location', 'best')
    exportgraphics(fig, out_file, 'Resolution', 220);
    close(fig)
end


function plot_indicator(x, summary, out_file)
    fig = figure('Visible', 'off', 'Color', 'w', 'Position', [80 80 1200 800]);
    tiledlayout(2, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
    metric_tile(x, summary.disp_pp * 1e6, '位移峰峰值 / μm', '位移峰峰值');
    metric_tile(x, summary.vel_rms * 1e3, '速度 RMS / (mm/s)', '速度 RMS');
    metric_tile(x, summary.acc_rms, '加速度 RMS / (m/s^2)', '加速度 RMS');
    metric_tile(x, summary.orbit_radius_max * 1e6, '轨迹半径 / μm', '轴心轨迹最大半径');
    metric_tile(x, summary.amp_1X, '1X 加速度幅值 / (m/s^2)', '1X 加速度分量');
    metric_tile(x, summary.ratio_2X_1X, '2X/1X', '2X/1X 幅值比');
    sgtitle('轴承支承刚度变化对转子响应指标的影响')
    exportgraphics(fig, out_file, 'Resolution', 220);
    close(fig)
end


function plot_harmonics(x, summary, out_file)
    fig = figure('Visible', 'off', 'Color', 'w', 'Position', [80 80 1120 620]);
    tiledlayout(1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

    nexttile
    hold on
    semilogx(x, summary.amp_1X, '-o', 'LineWidth', 1.2, 'MarkerFaceColor', [0.00 0.45 0.74]);
    semilogx(x, summary.amp_2X, '-s', 'LineWidth', 1.2, 'MarkerFaceColor', [0.85 0.33 0.10]);
    semilogx(x, summary.amp_3X, '-^', 'LineWidth', 1.2, 'MarkerFaceColor', [0.47 0.67 0.19]);
    hold off
    grid on
    xlabel('轴承支承刚度 / (N/m)')
    ylabel('加速度幅值 / (m/s^2)')
    title('1X、2X、3X 分量')
    legend({'1X=165 Hz','2X=330 Hz','3X=495 Hz'}, 'Location', 'best')

    nexttile
    hold on
    semilogx(x, summary.ratio_2X_1X, '-o', 'LineWidth', 1.2, 'MarkerFaceColor', [0.85 0.33 0.10]);
    semilogx(x, summary.ratio_3X_1X, '-s', 'LineWidth', 1.2, 'MarkerFaceColor', [0.47 0.67 0.19]);
    hold off
    grid on
    xlabel('轴承支承刚度 / (N/m)')
    ylabel('幅值比')
    title('倍频分量相对 1X 的变化')
    legend({'2X/1X','3X/1X'}, 'Location', 'best')
    sgtitle('轴承支承刚度变化对倍频分量及幅值比的影响')
    exportgraphics(fig, out_file, 'Resolution', 220);
    close(fig)
end


function metric_tile(x, y, y_label, title_text)
    nexttile
    semilogx(x, y, '-o', 'LineWidth', 1.2, 'MarkerFaceColor', [0.10 0.35 0.70]);
    grid on
    xlabel('轴承支承刚度 / (N/m)')
    ylabel(y_label)
    title(title_text)
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
