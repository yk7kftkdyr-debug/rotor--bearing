%% Build completed-case zoom figures for bearing stiffness report revision
% This script reads existing results_bearing_stiffness_K3/K4/K5.mat files.
% It does not rerun the dynamic solver and does not alter simulation data.

clearvars
clc
close all

set(0, 'DefaultFigureVisible', 'off');
fig_dir = fullfile(pwd, 'bearing_stiffness_sweep_9900_figures');
if ~exist(fig_dir, 'dir')
    mkdir(fig_dir);
end

case_ids = {'K3','K4','K5'};
labels = {'K3, 1e8 N/m','K4, 1e9 N/m','K5, 1e10 N/m'};
results = cell(numel(case_ids), 1);
for ii = 1:numel(case_ids)
    loaded = load(sprintf('results_bearing_stiffness_%s.mat', case_ids{ii}), 'result');
    results{ii} = loaded.result;
    if ~strcmp(results{ii}.simulation_status, 'completed')
        error('%s is not completed and must not be included in completed-case zoom figures.', case_ids{ii});
    end
end

colors = lines(numel(results));
plot_time_xy(results, colors, labels, 'rotor_x', 'rotor_y', 1e6, 'displacement / um', ...
    'Completed cases K3-K5: rotor displacement zoom', fullfile(fig_dir, 'fig_5_20_completed_displacement_zoom.png'));
plot_time_xy(results, colors, labels, 'rotor_vx', 'rotor_vy', 1e3, 'velocity / (mm/s)', ...
    'Completed cases K3-K5: rotor velocity zoom', fullfile(fig_dir, 'fig_5_21_completed_velocity_zoom.png'));
plot_time_xy(results, colors, labels, 'rotor_ax', 'rotor_ay', 1, 'acceleration / (m/s^2)', ...
    'Completed cases K3-K5: rotor acceleration zoom', fullfile(fig_dir, 'fig_5_22_completed_acceleration_zoom.png'));
plot_orbit_zoom(results, colors, labels, fullfile(fig_dir, 'fig_5_23_completed_orbit_zoom.png'));
plot_acc_spectrum_zoom(results, colors, labels, fullfile(fig_dir, 'fig_5_24_completed_acc_spectrum_zoom.png'));

fprintf('Generated completed-case zoom figures in %s\n', fig_dir);


function plot_time_xy(results, colors, labels, field_x, field_y, scale, y_label, title_text, out_file)
    fig = figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 1080 700]);
    tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

    nexttile
    hold on
    for ii = 1:numel(results)
        plot(results{ii}.signals.t, scale * results{ii}.signals.(field_x), ...
            'LineWidth', 1.1, 'Color', colors(ii,:));
    end
    hold off
    grid on
    ylabel(['x ' y_label])
    title(title_text)
    legend(labels, 'Location', 'best')

    nexttile
    hold on
    for ii = 1:numel(results)
        plot(results{ii}.signals.t, scale * results{ii}.signals.(field_y), ...
            'LineWidth', 1.1, 'Color', colors(ii,:));
    end
    hold off
    grid on
    xlabel('t / s')
    ylabel(['y ' y_label])

    exportgraphics(fig, out_file, 'Resolution', 220);
    close(fig)
end


function plot_orbit_zoom(results, colors, labels, out_file)
    fig = figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 820 680]);
    hold on
    for ii = 1:numel(results)
        plot(1e6 * results{ii}.signals.orbit_x, 1e6 * results{ii}.signals.orbit_y, ...
            'LineWidth', 1.15, 'Color', colors(ii,:));
    end
    hold off
    axis equal
    grid on
    xlabel('x displacement / um')
    ylabel('y displacement / um')
    title('Completed cases K3-K5: rotor orbit zoom')
    legend(labels, 'Location', 'best')
    exportgraphics(fig, out_file, 'Resolution', 220);
    close(fig)
end


function plot_acc_spectrum_zoom(results, colors, labels, out_file)
    fig = figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 920 620]);
    hold on
    for ii = 1:numel(results)
        plot(results{ii}.spectrum.freq_ax, results{ii}.spectrum.acc_amp_x, ...
            'LineWidth', 1.15, 'Color', colors(ii,:));
    end
    mark_harmonics()
    hold off
    xlim([0 700])
    grid on
    xlabel('frequency / Hz')
    ylabel('acceleration amplitude / (m/s^2)')
    title('Completed cases K3-K5: acceleration spectrum zoom')
    legend(labels, 'Location', 'best')
    exportgraphics(fig, out_file, 'Resolution', 220);
    close(fig)
end


function mark_harmonics()
    hs = [165, 330, 495];
    names = {'1X = 165 Hz', '2X = 330 Hz', '3X = 495 Hz'};
    yl = ylim;
    for kk = 1:numel(hs)
        plot([hs(kk), hs(kk)], yl, 'k--', 'LineWidth', 0.85);
        text(hs(kk), yl(2) * 0.92, names{kk}, 'Color', 'k', ...
            'HorizontalAlignment', 'center', 'VerticalAlignment', 'top', ...
            'FontSize', 8, 'FontWeight', 'bold');
    end
    ylim(yl)
end
