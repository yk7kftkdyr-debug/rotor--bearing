%% Rebuild Chapter 5 figures with selected cases only.
% This script only reloads existing real MAT/CSV results and redraws figures.
% It does not rerun dynamics or alter numerical output files.

clearvars
close all
set(0, 'DefaultFigureVisible', 'off');
set(0, 'DefaultAxesFontName', 'Microsoft YaHei');
set(0, 'DefaultTextFontName', 'Microsoft YaHei');

palette = chapter_palette();
rebuild_unbalance_figures(palette);
rebuild_external_load_figures(palette);
rebuild_bearing_stiffness_figures(palette);

fprintf('Chapter 5 filtered figures rebuilt from existing real outputs.\n');


function colors = chapter_palette()
    colors = [
        0.0000 0.4470 0.7410
        0.8500 0.3250 0.0980
        0.9290 0.6940 0.1250
        0.4940 0.1840 0.5560
        0.4660 0.6740 0.1880
        0.3010 0.7450 0.9330
        0.6350 0.0780 0.1840
        0.2500 0.2500 0.2500
    ];
end


function rebuild_unbalance_figures(palette)
    fig_dir = fullfile(pwd, 'unbalance_sweep_9900_figures');
    scales = [1, 2, 5, 8];
    tags = {'1', '2', '5', '8'};
    results = cell(numel(scales), 1);
    for ii = 1:numel(scales)
        data_in = load(fullfile(pwd, sprintf('results_unbalance_scale_%s.mat', tags{ii})), 'result');
        results{ii} = data_in.result;
    end
    summary_all = readtable(fullfile(pwd, 'unbalance_sweep_9900_summary.csv'));
    summary = summary_all(ismember(summary_all.unbalance_scale, scales), :);
    colors = palette(1:numel(results), :);
    labels = arrayfun(@(v) sprintf('scale=%g', v), scales, 'UniformOutput', false);

    plot_time_xy(results, colors, labels, 'rotor_x', 'rotor_y', 1e6, '位移 / μm', ...
        '不同不平衡量下转子轴承处位移响应对比', fullfile(fig_dir, 'fig_5_1_displacement_compare.png'));
    plot_time_xy(results, colors, labels, 'rotor_vx', 'rotor_vy', 1e3, '速度 / (mm/s)', ...
        '不同不平衡量下转子轴承处速度响应对比', fullfile(fig_dir, 'fig_5_2_velocity_compare.png'));
    plot_time_xy(results, colors, labels, 'rotor_ax', 'rotor_ay', 1, '加速度 / (m/s^2)', ...
        '不同不平衡量下转子轴承处加速度响应对比', fullfile(fig_dir, 'fig_5_3_acceleration_compare.png'));
    plot_orbit(results, colors, labels, fullfile(fig_dir, 'fig_5_4_orbit_compare.png'), ...
        '不同不平衡量下转子轴承处轴心轨迹对比');
    plot_acc_spectrum(results, colors, labels, fullfile(fig_dir, 'fig_5_5_acc_spectrum_compare.png'), ...
        '不同不平衡量下转子轴承处加速度频谱对比');
    plot_indicator(summary.unbalance_scale, summary, '不平衡量比例系数', ...
        '不平衡量变化对转子响应指标的影响', fullfile(fig_dir, 'fig_5_6_indicator_compare.png'), false);
end


function rebuild_external_load_figures(palette)
    fig_dir = fullfile(pwd, 'external_load_sweep_9900_report_figures');
    loads = [0, 1000, 2500, 5000, 7500];
    results = cell(numel(loads), 1);
    for ii = 1:numel(loads)
        data_in = load(fullfile(pwd, sprintf('results_external_load_%gN.mat', loads(ii))), 'result');
        results{ii} = data_in.result;
    end
    summary = readtable(fullfile(pwd, 'external_load_sweep_9900_summary.csv'));
    colors = palette(1:numel(results), :);
    labels = arrayfun(@(v) sprintf('%g N', v), loads, 'UniformOutput', false);

    plot_time_xy(results, colors, labels, 'rotor_x', 'rotor_y', 1e6, '位移 / μm', ...
        '不同外加载荷下转子轴承处位移响应对比', fullfile(fig_dir, 'fig_5_7_displacement_compare.png'));
    plot_time_xy(results, colors, labels, 'rotor_vx', 'rotor_vy', 1e3, '速度 / (mm/s)', ...
        '不同外加载荷下转子轴承处速度响应对比', fullfile(fig_dir, 'fig_5_8_velocity_compare.png'));
    plot_time_xy(results, colors, labels, 'rotor_ax', 'rotor_ay', 1, '加速度 / (m/s^2)', ...
        '不同外加载荷下转子轴承处加速度响应对比', fullfile(fig_dir, 'fig_5_9_acceleration_compare.png'));
    plot_orbit(results, colors, labels, fullfile(fig_dir, 'fig_5_10_orbit_compare.png'), ...
        '不同外加载荷下转子轴承处轴心轨迹对比');
    plot_acc_spectrum(results, colors, labels, fullfile(fig_dir, 'fig_5_11_acc_spectrum_compare.png'), ...
        '不同外加载荷下转子轴承处加速度频谱对比');
    plot_harmonics(summary.external_load_N, summary, '外加载荷 / N', ...
        '外加载荷变化对倍频分量及幅值比的影响', fullfile(fig_dir, 'fig_5_13_harmonic_compare.png'), false);
    plot_indicator(summary.external_load_N, summary, '外加载荷 / N', ...
        '外加载荷变化对转子响应指标的影响', fullfile(fig_dir, 'fig_5_12_indicator_compare.png'), false);
end


function rebuild_bearing_stiffness_figures(palette)
    fig_dir = fullfile(pwd, 'bearing_stiffness_sweep_9900_figures');
    keep_ids = {'K3','K4','K6','K7','K8'};
    summary_all = readtable(fullfile(pwd, 'bearing_stiffness_sweep_9900_summary.csv'));
    summary = summary_all(ismember(summary_all.case_id, keep_ids), :);
    results = cell(height(summary), 1);
    for ii = 1:height(summary)
        data_in = load(fullfile(pwd, sprintf('results_bearing_stiffness_%s.mat', summary.case_id{ii})), 'result');
        results{ii} = data_in.result;
    end
    colors = palette(1:numel(results), :);
    labels = arrayfun(@(v) sprintf('%.0e N/m', v), summary.K_level, 'UniformOutput', false);

    plot_time_xy(results, colors, labels, 'rotor_x', 'rotor_y', 1e6, '位移 / μm', ...
        '不同轴承支承刚度下转子轴承处位移响应对比', fullfile(fig_dir, 'fig_5_13_displacement_compare.png'));
    plot_time_xy(results, colors, labels, 'rotor_vx', 'rotor_vy', 1e3, '速度 / (mm/s)', ...
        '不同轴承支承刚度下转子轴承处速度响应对比', fullfile(fig_dir, 'fig_5_14_velocity_compare.png'));
    plot_time_xy(results, colors, labels, 'rotor_ax', 'rotor_ay', 1, '加速度 / (m/s^2)', ...
        '不同轴承支承刚度下转子轴承处加速度响应对比', fullfile(fig_dir, 'fig_5_15_acceleration_compare.png'));
    plot_orbit(results, colors, labels, fullfile(fig_dir, 'fig_5_16_orbit_compare.png'), ...
        '不同轴承支承刚度下转子轴承处轴心轨迹对比');
    plot_acc_spectrum(results, colors, labels, fullfile(fig_dir, 'fig_5_17_acc_spectrum_compare.png'), ...
        '不同轴承支承刚度下转子轴承处加速度频谱对比');
    plot_harmonics(summary.K_level, summary, '轴承支承刚度 / (N/m)', ...
        '轴承支承刚度变化对倍频分量及幅值比的影响', fullfile(fig_dir, 'fig_5_19_harmonic_compare.png'), true);
    plot_indicator(summary.K_level, summary, '轴承支承刚度 / (N/m)', ...
        '轴承支承刚度变化对转子响应指标的影响', fullfile(fig_dir, 'fig_5_18_indicator_compare.png'), true);
end


function plot_time_xy(results, colors, labels, field_x, field_y, scale, y_label, title_text, out_file)
    fig = figure('Visible', 'off', 'Color', 'w', 'Position', [80 80 1100 720]);
    tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

    nexttile
    hold on
    for ii = 1:numel(results)
        plot(results{ii}.signals.t, scale * results{ii}.signals.(field_x), 'LineWidth', 1.1, 'Color', colors(ii,:));
    end
    hold off
    grid on
    ylabel(['x 向' y_label])
    title(title_text)
    legend(labels, 'Location', 'best', 'NumColumns', 3)

    nexttile
    hold on
    for ii = 1:numel(results)
        plot(results{ii}.signals.t, scale * results{ii}.signals.(field_y), 'LineWidth', 1.1, 'Color', colors(ii,:));
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
        if isfield(results{ii}.signals, 'orbit_x')
            x = results{ii}.signals.orbit_x;
            y = results{ii}.signals.orbit_y;
        else
            x = results{ii}.signals.rotor_x - local_mean(results{ii}.signals.rotor_x);
            y = results{ii}.signals.rotor_y - local_mean(results{ii}.signals.rotor_y);
        end
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
        if isfield(results{ii}.spectrum, 'freq_ax')
            freq = results{ii}.spectrum.freq_ax;
            amp = results{ii}.spectrum.acc_amp_x;
        else
            freq = results{ii}.spectrum.freq;
            amp = results{ii}.spectrum.acc_amp;
        end
        plot(freq, amp, 'LineWidth', 1.1, 'Color', colors(ii,:));
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


function plot_indicator(x, summary, x_label, title_text, out_file, log_x)
    fig = figure('Visible', 'off', 'Color', 'w', 'Position', [80 80 1200 800]);
    tiledlayout(2, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
    metric_tile(x, summary.disp_pp * 1e6, x_label, '位移峰峰值 / μm', '位移峰峰值', log_x);
    metric_tile(x, summary.vel_rms * 1e3, x_label, '速度 RMS / (mm/s)', '速度 RMS', log_x);
    metric_tile(x, summary.acc_rms, x_label, '加速度 RMS / (m/s^2)', '加速度 RMS', log_x);
    metric_tile(x, summary.orbit_radius_max * 1e6, x_label, '轨迹半径 / μm', '轴心轨迹最大半径', log_x);
    metric_tile(x, summary.amp_1X, x_label, '1X 加速度幅值 / (m/s^2)', '1X 加速度分量', log_x);
    metric_tile(x, summary.ratio_2X_1X, x_label, '2X/1X', '2X/1X 幅值比', log_x);
    sgtitle(title_text)
    exportgraphics(fig, out_file, 'Resolution', 220);
    close(fig)
end


function plot_harmonics(x, summary, x_label, title_text, out_file, log_x)
    fig = figure('Visible', 'off', 'Color', 'w', 'Position', [80 80 1120 620]);
    tiledlayout(1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

    nexttile
    hold on
    plot_series(x, summary.amp_1X, '-o', log_x);
    plot_series(x, summary.amp_2X, '-s', log_x);
    plot_series(x, summary.amp_3X, '-^', log_x);
    hold off
    grid on
    xlabel(x_label)
    ylabel('加速度幅值 / (m/s^2)')
    title('1X、2X、3X 分量')
    legend({'1X=165 Hz','2X=330 Hz','3X=495 Hz'}, 'Location', 'best')

    nexttile
    hold on
    plot_series(x, summary.ratio_2X_1X, '-o', log_x);
    plot_series(x, summary.ratio_3X_1X, '-s', log_x);
    hold off
    grid on
    xlabel(x_label)
    ylabel('幅值比')
    title('倍频分量相对 1X 的变化')
    legend({'2X/1X','3X/1X'}, 'Location', 'best')
    sgtitle(title_text)
    exportgraphics(fig, out_file, 'Resolution', 220);
    close(fig)
end


function metric_tile(x, y, x_label, y_label, title_text, log_x)
    nexttile
    plot_series(x, y, '-o', log_x);
    grid on
    xlabel(x_label)
    ylabel(y_label)
    title(title_text)
end


function plot_series(x, y, style, log_x)
    if log_x
        semilogx(x, y, style, 'LineWidth', 1.2, 'MarkerFaceColor', [0.10 0.35 0.70]);
    else
        plot(x, y, style, 'LineWidth', 1.2, 'MarkerFaceColor', [0.10 0.35 0.70]);
    end
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


function m = local_mean(x)
    x = x(:);
    if isempty(x)
        m = NaN;
    else
        m = sum(x) / numel(x);
    end
end
