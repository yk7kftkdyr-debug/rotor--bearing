%% Rebuild chapter 5 figures for unbalance and external-load studies.
% This script only reloads completed MAT/CSV results and redraws figures with
% consistent Chinese titles and engineering units. It does not rerun dynamics.

clearvars
close all
set(0, 'DefaultFigureVisible', 'off');
set(0, 'DefaultAxesFontName', 'Microsoft YaHei');
set(0, 'DefaultTextFontName', 'Microsoft YaHei');

rebuild_unbalance_figures();
rebuild_external_load_figures();

fprintf('Chapter 5 unbalance and external-load figures rebuilt.\n');


function rebuild_unbalance_figures()
    fig_dir = fullfile(pwd, 'unbalance_sweep_9900_figures');
    scales = [0, 0.5, 1, 2, 5, 8, 10];
    tags = {'0', '0p5', '1', '2', '5', '8', '10'};
    results = cell(numel(scales), 1);
    for ii = 1:numel(scales)
        data_in = load(fullfile(pwd, sprintf('results_unbalance_scale_%s.mat', tags{ii})), 'result');
        results{ii} = data_in.result;
    end
    summary = readtable(fullfile(pwd, 'unbalance_sweep_9900_summary.csv'));
    colors = lines(numel(results));
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
        '不平衡量变化对转子响应指标的影响', fullfile(fig_dir, 'fig_5_6_indicator_compare.png'));
end


function rebuild_external_load_figures()
    fig_dir = fullfile(pwd, 'external_load_sweep_9900_report_figures');
    loads = [0, 1000, 2500, 5000, 7500];
    results = cell(numel(loads), 1);
    for ii = 1:numel(loads)
        data_in = load(fullfile(pwd, sprintf('results_external_load_%gN.mat', loads(ii))), 'result');
        results{ii} = data_in.result;
    end
    summary = readtable(fullfile(pwd, 'external_load_sweep_9900_summary.csv'));
    colors = lines(numel(results));
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
        '外加载荷变化对倍频分量及幅值比的影响', fullfile(fig_dir, 'fig_5_13_harmonic_compare.png'));
    plot_indicator(summary.external_load_N, summary, '外加载荷 / N', ...
        '外加载荷变化对转子响应指标的影响', fullfile(fig_dir, 'fig_5_12_indicator_compare.png'));
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
    ylabel(['x 向 ' y_label])
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
    ylabel(['y 向 ' y_label])
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
        plot(1e6 * x, 1e6 * y, ...
            'LineWidth', 1.15, 'Color', colors(ii,:));
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


function m = local_mean(x)
    x = x(:);
    if isempty(x)
        m = NaN;
    else
        m = sum(x) / numel(x);
    end
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


function plot_indicator(x, summary, x_label, title_text, out_file)
    fig = figure('Visible', 'off', 'Color', 'w', 'Position', [80 80 1200 800]);
    tiledlayout(2, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
    metric_tile(x, summary.disp_pp * 1e6, x_label, '位移峰峰值 / μm', '位移峰峰值');
    metric_tile(x, summary.vel_rms * 1e3, x_label, '速度 RMS / (mm/s)', '速度 RMS');
    metric_tile(x, summary.acc_rms, x_label, '加速度 RMS / (m/s^2)', '加速度 RMS');
    metric_tile(x, summary.orbit_radius_max * 1e6, x_label, '轨迹半径 / μm', '轴心轨迹最大半径');
    metric_tile(x, summary.amp_1X, x_label, '1X 加速度幅值 / (m/s^2)', '1X 加速度分量');
    metric_tile(x, summary.ratio_2X_1X, x_label, '2X/1X', '2X/1X 幅值比');
    sgtitle(title_text)
    exportgraphics(fig, out_file, 'Resolution', 220);
    close(fig)
end


function plot_harmonics(x, summary, x_label, title_text, out_file)
    fig = figure('Visible', 'off', 'Color', 'w', 'Position', [80 80 1120 620]);
    tiledlayout(1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    nexttile
    hold on
    plot(x, summary.amp_1X, '-o', 'LineWidth', 1.2, 'MarkerFaceColor', [0.00 0.45 0.74]);
    plot(x, summary.amp_2X, '-s', 'LineWidth', 1.2, 'MarkerFaceColor', [0.85 0.33 0.10]);
    plot(x, summary.amp_3X, '-^', 'LineWidth', 1.2, 'MarkerFaceColor', [0.47 0.67 0.19]);
    hold off
    grid on
    xlabel(x_label)
    ylabel('加速度幅值 / (m/s^2)')
    title('1X、2X、3X 分量')
    legend({'1X=165 Hz','2X=330 Hz','3X=495 Hz'}, 'Location', 'best')
    nexttile
    hold on
    plot(x, summary.ratio_2X_1X, '-o', 'LineWidth', 1.2, 'MarkerFaceColor', [0.85 0.33 0.10]);
    plot(x, summary.ratio_3X_1X, '-s', 'LineWidth', 1.2, 'MarkerFaceColor', [0.47 0.67 0.19]);
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


function metric_tile(x, y, x_label, y_label, title_text)
    nexttile
    plot(x, y, '-o', 'LineWidth', 1.2, 'MarkerFaceColor', [0.10 0.35 0.70]);
    grid on
    xlabel(x_label)
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
