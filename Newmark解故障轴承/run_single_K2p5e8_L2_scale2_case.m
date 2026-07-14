%% Single combined case:
% Bearing support stiffness K = 2.5e8 N/m, external load L2 = 2500 N,
% unbalance scale = 2. This script runs the real model once and extracts
% rotor bearing-node responses at loc_rub(1) and loc_rub(2).

clearvars
clc
close all
set(0, 'DefaultFigureVisible', 'off');
set(0, 'DefaultAxesFontName', 'Microsoft YaHei');
set(0, 'DefaultTextFontName', 'Microsoft YaHei');

case_id = 'K2p5e8_L2_scale2';
K_level = 2.5e8;
external_load_N = 2500;
unbalance_scale = 2;
rpm_fixed = 9900;
f_rot = rpm_fixed / 60;
harmonics = [1, 2, 3] * f_rot;
fig_dir = fullfile(pwd, 'single_case_K2p5e8_L2_scale2_figures');
if ~exist(fig_dir, 'dir')
    mkdir(fig_dir);
end

fprintf('\n========== Single combined case: %s ==========\n', case_id);
fprintf('K = %.3e N/m, external load = %.0f N, unbalance scale = %.3g\n', ...
    K_level, external_load_N, unbalance_scale);

suppress_main_plots = true; %#ok<NASGU>
sweep_output_dir = fig_dir; %#ok<NASGU>
sweep_case_id = case_id; %#ok<NASGU>
bearing_stiffness_level = K_level; %#ok<NASGU>
kb1_case = K_level; %#ok<NASGU>
kb2_case = K_level; %#ok<NASGU>

result = struct();
try
    run('mian.m');

    result.case_id = case_id;
    result.K_level = K_level;
    result.kb1_case = kb1;
    result.kb2_case = kb2;
    result.external_load_N = external_load_N;
    result.data10_value = data(10);
    result.unbalance_scale = unbalance_scale_used;
    result.rpm = rpm_fixed;
    result.f_rot = f_rot;
    result.harmonics = harmonics;
    result.wi = wi;
    result.U_base = U_base;
    result.U = U_current;
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

    if any(~isfinite(yn(:))) || any(~isfinite(dyn(:))) || any(~isfinite(ddyn(:)))
        result.simulation_status = 'failed';
    elseif n_end < total_steps
        result.simulation_status = 'abnormal';
    else
        result.simulation_status = 'completed';
    end

    result = extract_bearing_node_metrics(result);
    result.sweep_output_dir = sweep_output_dir;
    plot_single_case_figures(result, sweep_output_dir);

catch ME
    result.case_id = case_id;
    result.K_level = K_level;
    result.external_load_N = external_load_N;
    result.unbalance_scale = unbalance_scale;
    result.simulation_status = 'failed';
    result.error_message = ME.message;
    result.metrics = failed_metrics(case_id, K_level, external_load_N, unbalance_scale);
    fprintf('Case failed: %s\n', ME.message);
    for kk = 1:numel(ME.stack)
        fprintf('  at %s line %d\n', ME.stack(kk).name, ME.stack(kk).line);
    end
end

mat_file = fullfile(pwd, 'results_single_K2p5e8_L2_scale2.mat');
csv_file = fullfile(pwd, 'single_K2p5e8_L2_scale2_summary.csv');
xlsx_file = fullfile(pwd, 'single_K2p5e8_L2_scale2_summary.xlsx');
save(mat_file, 'result', '-v7.3');
writetable(result.metrics, csv_file);
writetable(result.metrics, xlsx_file);

disp(result.metrics);
fprintf('\nGenerated:\n');
fprintf('  %s\n', mat_file);
fprintf('  %s\n', csv_file);
fprintf('  %s\n', xlsx_file);
fprintf('  %s\n', result.sweep_output_dir);


function result = extract_bearing_node_metrics(result)
    idx = result.idx_plot;
    fs = result.fs;
    nodes = result.loc_rub(:).';
    positions = {'front_bearing', 'rear_bearing'};
    metrics = table();

    result.signals = struct();
    result.spectrum = struct();
    result.signals.t = result.time_plot(:);

    for ii = 1:numel(nodes)
        node = nodes(ii);
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
        orbit_radius = sqrt(x0.^2 + y0.^2);

        [freq_ax, acc_amp_x] = single_sided_spectrum(ax0, fs);
        amp_1X = harmonic_amp(freq_ax, acc_amp_x, result.harmonics(1));
        amp_2X = harmonic_amp(freq_ax, acc_amp_x, result.harmonics(2));
        amp_3X = harmonic_amp(freq_ax, acc_amp_x, result.harmonics(3));

        row = table( ...
            {result.case_id}, {positions{ii}}, node, ...
            result.K_level, result.kb1_case, result.kb2_case, ...
            result.external_load_N, result.data10_value, result.unbalance_scale, ...
            result.U(1), result.U(2), result.U(3), result.U(4), ...
            result.Famp1(1), result.Famp1(2), result.Famp1(3), result.Famp1(4), ...
            result.rpm, result.f_rot, ...
            max([abs(x0); abs(y0)]), max([local_peak2peak(x0), local_peak2peak(y0)]), ...
            max([abs(vx0); abs(vy0)]), sqrt(local_mean(vx0.^2 + vy0.^2)), ...
            max([abs(ax0); abs(ay0)]), sqrt(local_mean(ax0.^2 + ay0.^2)), ...
            max(orbit_radius), amp_1X, amp_2X, amp_3X, ...
            amp_2X / max(amp_1X, eps), amp_3X / max(amp_1X, eps), ...
            {result.simulation_status}, ...
            'VariableNames', {'case_id','bearing_position','rotor_node', ...
            'K_level','kb1_case','kb2_case','external_load_N','data10_value','unbalance_scale', ...
            'U1','U2','U3','U4','Famp1_1','Famp1_2','Famp1_3','Famp1_4', ...
            'rpm','oneX_Hz','disp_peak','disp_pp','vel_peak','vel_rms','acc_peak','acc_rms', ...
            'orbit_radius_max','amp_1X','amp_2X','amp_3X','ratio_2X_1X','ratio_3X_1X', ...
            'simulation_status'});

        metrics = [metrics; row]; %#ok<AGROW>
        result.signals.(sprintf('node%d_x', node)) = x;
        result.signals.(sprintf('node%d_y', node)) = y;
        result.signals.(sprintf('node%d_vx', node)) = vx;
        result.signals.(sprintf('node%d_vy', node)) = vy;
        result.signals.(sprintf('node%d_ax', node)) = ax;
        result.signals.(sprintf('node%d_ay', node)) = ay;
        result.signals.(sprintf('node%d_orbit_x', node)) = x0;
        result.signals.(sprintf('node%d_orbit_y', node)) = y0;
        result.spectrum.(sprintf('node%d_freq_ax', node)) = freq_ax;
        result.spectrum.(sprintf('node%d_acc_amp_x', node)) = acc_amp_x;
    end

    result.metrics = metrics;
end


function metrics = failed_metrics(case_id, K_level, external_load_N, unbalance_scale)
    metrics = table({case_id}, {'front_bearing'}, NaN, K_level, K_level, K_level, ...
        external_load_N, external_load_N, unbalance_scale, ...
        NaN, NaN, NaN, NaN, NaN, NaN, NaN, NaN, 9900, 165, ...
        NaN, NaN, NaN, NaN, NaN, NaN, NaN, NaN, NaN, NaN, NaN, NaN, {'failed'}, ...
        'VariableNames', {'case_id','bearing_position','rotor_node', ...
        'K_level','kb1_case','kb2_case','external_load_N','data10_value','unbalance_scale', ...
        'U1','U2','U3','U4','Famp1_1','Famp1_2','Famp1_3','Famp1_4', ...
        'rpm','oneX_Hz','disp_peak','disp_pp','vel_peak','vel_rms','acc_peak','acc_rms', ...
        'orbit_radius_max','amp_1X','amp_2X','amp_3X','ratio_2X_1X','ratio_3X_1X', ...
        'simulation_status'});
end


function plot_single_case_figures(result, fig_dir)
    if ~isfield(result, 'signals') || ~strcmp(result.simulation_status, 'completed')
        return
    end
    t = result.signals.t;
    nodes = result.loc_rub(:).';

    fig = figure('Visible', 'off', 'Color', 'w', 'Position', [80 80 1180 780]);
    tiledlayout(3, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    for ii = 1:numel(nodes)
        node = nodes(ii);
        nexttile(ii)
        plot(t, 1e6 * result.signals.(sprintf('node%d_x', node)), 'LineWidth', 1.1)
        hold on
        plot(t, 1e6 * result.signals.(sprintf('node%d_y', node)), 'LineWidth', 1.1)
        hold off
        grid on
        xlabel('时间 / s'); ylabel('位移 / μm')
        title(sprintf('转子轴承节点 %d 位移', node))
        legend({'x向','y向'}, 'Location', 'best')

        nexttile(ii + 2)
        plot(t, 1e3 * result.signals.(sprintf('node%d_vx', node)), 'LineWidth', 1.1)
        hold on
        plot(t, 1e3 * result.signals.(sprintf('node%d_vy', node)), 'LineWidth', 1.1)
        hold off
        grid on
        xlabel('时间 / s'); ylabel('速度 / (mm/s)')
        title(sprintf('转子轴承节点 %d 速度', node))
        legend({'x向','y向'}, 'Location', 'best')

        nexttile(ii + 4)
        plot(t, result.signals.(sprintf('node%d_ax', node)), 'LineWidth', 1.1)
        hold on
        plot(t, result.signals.(sprintf('node%d_ay', node)), 'LineWidth', 1.1)
        hold off
        grid on
        xlabel('时间 / s'); ylabel('加速度 / (m/s^2)')
        title(sprintf('转子轴承节点 %d 加速度', node))
        legend({'x向','y向'}, 'Location', 'best')
    end
    sgtitle('K=2.5e8 N/m、L2=2500 N、scale=2 响应时域')
    exportgraphics(fig, fullfile(fig_dir, 'single_case_time_response.png'), 'Resolution', 220);
    close(fig)

    fig = figure('Visible', 'off', 'Color', 'w', 'Position', [80 80 1120 520]);
    tiledlayout(1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    for ii = 1:numel(nodes)
        node = nodes(ii);
        nexttile
        plot(1e6 * result.signals.(sprintf('node%d_orbit_x', node)), ...
            1e6 * result.signals.(sprintf('node%d_orbit_y', node)), 'LineWidth', 1.1)
        axis equal
        grid on
        xlabel('x / μm'); ylabel('y / μm')
        title(sprintf('转子轴承节点 %d 轴心轨迹', node))
    end
    exportgraphics(fig, fullfile(fig_dir, 'single_case_orbit.png'), 'Resolution', 220);
    close(fig)

    fig = figure('Visible', 'off', 'Color', 'w', 'Position', [80 80 980 620]);
    hold on
    for ii = 1:numel(nodes)
        node = nodes(ii);
        plot(result.spectrum.(sprintf('node%d_freq_ax', node)), ...
            result.spectrum.(sprintf('node%d_acc_amp_x', node)), 'LineWidth', 1.1)
    end
    mark_harmonics();
    hold off
    xlim([0 800])
    grid on
    xlabel('频率 / Hz'); ylabel('加速度幅值 / (m/s^2)')
    title('转子轴承处节点 x 向加速度频谱')
    legend({'节点2','节点10'}, 'Location', 'best')
    exportgraphics(fig, fullfile(fig_dir, 'single_case_acc_spectrum.png'), 'Resolution', 220);
    close(fig)
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
