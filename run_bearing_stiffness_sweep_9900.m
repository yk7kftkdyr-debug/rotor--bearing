%% Bearing support stiffness sweep at 9900 r/min
% Single-factor study: only the coupled bearing support stiffness entries
% kb1 and kb2 in mian.m are changed. They are assembled directly into the
% global rotor-casing stiffness matrix KK in both x and y directions. The
% Hertzian contact stiffness C_b=data(6), clearance data(9), external load
% data(10), bearing damping cb, casing base stiffness, time step, Newmark
% parameters, unbalance locations and observation nodes follow the original
% program settings.

clearvars
clc
close all
set(0, 'DefaultFigureVisible', 'off');

K_levels = [1e7, 2e7, 5e7, 8e7, 1e8, 2e8, 5e8, 1e9];
case_ids = {'K1','K2','K3','K4','K5','K6','K7','K8'};
rpm_fixed = 9900;
f_rot = rpm_fixed / 60;
harmonics = [1, 2, 3] * f_rot;
unbalance_amplification = 1;

fig_dir = fullfile(pwd, 'bearing_stiffness_sweep_9900_figures');
if ~exist(fig_dir, 'dir')
    mkdir(fig_dir);
end

mat_files = cell(numel(K_levels), 1);
results = cell(numel(K_levels), 1);

for ii = 1:numel(K_levels)
    K_level = K_levels(ii);
    case_id = case_ids{ii};
    mat_name = fullfile(pwd, sprintf('results_bearing_stiffness_%s.mat', case_id));
    mat_files{ii} = mat_name;

    fprintf('\n========== Bearing support stiffness %s: %.3e N/m (%d/%d) ==========\n', ...
        case_id, K_level, ii, numel(K_levels));

    result = run_single_bearing_stiffness_case(K_level, case_id, fig_dir, ...
        rpm_fixed, f_rot, harmonics, unbalance_amplification);
    results{ii} = result;
    save(mat_name, 'result', '-v7.3');

    if isfield(result, 'signals') && ~strcmp(result.simulation_status, 'failed')
        plot_case_figures(result, fig_dir);
    end
    close all
end

summary = build_summary_from_mat(mat_files);
writetable(summary, fullfile(pwd, 'bearing_stiffness_sweep_9900_summary.xlsx'));
writetable(summary, fullfile(pwd, 'bearing_stiffness_sweep_9900_summary.csv'));

plot_comparison_figures(results, fig_dir);
write_markdown_report(summary, fig_dir, fullfile(pwd, 'bearing_stiffness_sweep_9900_report.md'));

fprintf('\nGenerated:\n');
fprintf('  bearing_stiffness_sweep_9900_summary.xlsx\n');
fprintf('  bearing_stiffness_sweep_9900_summary.csv\n');
fprintf('  bearing_stiffness_sweep_9900_report.md\n');
fprintf('  results_bearing_stiffness_K*.mat\n');
fprintf('  %s\n', fig_dir);


function result = run_single_bearing_stiffness_case(K_level, case_id, fig_dir, rpm_fixed, f_rot, harmonics, unbalance_amplification)
    suppress_main_plots = true; %#ok<NASGU>
    sweep_output_dir = fig_dir; %#ok<NASGU>
    sweep_case_id = case_id; %#ok<NASGU>
    bearing_stiffness_level = K_level; %#ok<NASGU>
    kb1_case = K_level; %#ok<NASGU>
    kb2_case = K_level; %#ok<NASGU>
    unbalance_scale = unbalance_amplification; %#ok<NASGU>

    result = init_result_shell(K_level, case_id, rpm_fixed, f_rot, harmonics, unbalance_amplification);
    try
        run('mian.m');

        data(5) = 0;
        data(7) = rpm_fixed;
        data(10) = data(10); %#ok<NASGU>

        result.case_id = case_id;
        result.K_level = K_level;
        result.kb1_case = kb1;
        result.kb2_case = kb2;
        result.kb3_case = NaN;
        result.Kzx_case = NaN;
        result.Kzy_case = NaN;
        result.rpm = rpm_fixed;
        result.wi = wi;
        result.f_rot = f_rot;
        result.harmonics = harmonics;
        result.unbalance_amplification = unbalance_scale_used;
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
        result.stiffness_entry_description = ...
            'kb1 and kb2 are assembled into KK as rotor-casing coupling stiffness in x/y directions at r1-c1 and r2-c2; kb3, K_b, bearing Kzx/Kzy entries do not exist in the active coupled model.';

        if any(~isfinite(yn(:))) || any(~isfinite(dyn(:))) || any(~isfinite(ddyn(:)))
            result.simulation_status = 'failed';
        elseif n_end < total_steps
            result.simulation_status = 'abnormal';
        else
            result.simulation_status = 'completed';
        end

        result = compute_case_metrics(result);
    catch ME
        if ~exist('result', 'var') || ~isstruct(result) || ~isfield(result, 'case_id')
            result = init_result_shell(K_level, case_id, rpm_fixed, f_rot, harmonics, unbalance_amplification);
        end
        result.simulation_status = 'failed';
        result.error_message = ME.message;
        result.metrics = failed_metrics(result);
        fprintf('Case %s failed during simulation or post-processing: %s\n', result.case_id, ME.message);
        for kk = 1:numel(ME.stack)
            fprintf('  at %s line %d\n', ME.stack(kk).name, ME.stack(kk).line);
        end
    end
end


function result = init_result_shell(K_level, case_id, rpm_fixed, f_rot, harmonics, unbalance_amplification)
    result = struct();
    result.case_id = case_id;
    result.K_level = K_level;
    result.kb1_case = K_level;
    result.kb2_case = K_level;
    result.kb3_case = NaN;
    result.Kzx_case = NaN;
    result.Kzy_case = NaN;
    result.rpm = rpm_fixed;
    result.f_rot = f_rot;
    result.harmonics = harmonics;
    result.unbalance_amplification = unbalance_amplification;
    result.U = [NaN NaN NaN NaN];
    result.Famp1 = [NaN NaN NaN NaN];
    result.simulation_status = 'not_run';
end


function summary = build_summary_from_mat(mat_files)
    summary = table();
    for ii = 1:numel(mat_files)
        data_in = load(mat_files{ii}, 'result');
        row = data_in.result.metrics;
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
    cvx0 = cvx - local_mean(cvx);
    cvy0 = cvy - local_mean(cvy);
    cax0 = cax - local_mean(cax);
    cay0 = cay - local_mean(cay);
    orbit_radius = sqrt(rx0.^2 + ry0.^2);

    [freq_vx, vel_amp_x] = single_sided_spectrum(rvx0, fs);
    [freq_vy, vel_amp_y] = single_sided_spectrum(rvy0, fs);
    [freq_ax, acc_amp_x] = single_sided_spectrum(rax0, fs);
    [freq_ay, acc_amp_y] = single_sided_spectrum(ray0, fs);
    [freq_case_ax, case_acc_amp_x] = single_sided_spectrum(cax0, fs);

    amp_1X = harmonic_amp(freq_ax, acc_amp_x, result.harmonics(1));
    amp_2X = harmonic_amp(freq_ax, acc_amp_x, result.harmonics(2));
    amp_3X = harmonic_amp(freq_ax, acc_amp_x, result.harmonics(3));
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
        'freq_ay', freq_ay, 'acc_amp_y', acc_amp_y, ...
        'freq_case_ax', freq_case_ax, 'case_acc_amp_x', case_acc_amp_x);

    disp_peak = max([abs(rx0); abs(ry0)]);
    disp_pp = max([local_peak2peak(rx0), local_peak2peak(ry0)]);
    vel_peak = max([abs(rvx0); abs(rvy0)]);
    vel_rms = sqrt(local_mean(rvx0.^2 + rvy0.^2));
    acc_peak = max([abs(rax0); abs(ray0)]);
    acc_rms = sqrt(local_mean(rax0.^2 + ray0.^2));
    case_disp_peak = max([abs(cx0); abs(cy0)]);
    case_vel_rms = sqrt(local_mean(cvx0.^2 + cvy0.^2));
    case_acc_rms = sqrt(local_mean(cax0.^2 + cay0.^2));

    result.metrics = table( ...
        {result.case_id}, result.K_level, result.kb1_case, result.kb2_case, result.kb3_case, result.Kzx_case, result.Kzy_case, ...
        result.rpm, result.f_rot, result.unbalance_amplification, ...
        result.U(1), result.U(2), result.U(3), result.U(4), ...
        result.Famp1(1), result.Famp1(2), result.Famp1(3), result.Famp1(4), ...
        disp_peak, disp_pp, vel_peak, vel_rms, acc_peak, acc_rms, ...
        max(orbit_radius), amp_1X, amp_2X, amp_3X, ratio_2X_1X, ratio_3X_1X, ...
        case_disp_peak, case_vel_rms, case_acc_rms, {result.simulation_status}, ...
        'VariableNames', {'case_id','K_level','kb1_case','kb2_case','kb3_case','Kzx_case','Kzy_case', ...
        'rpm','oneX_Hz','unbalance_amplification', ...
        'U1_used','U2_used','U3_used','U4_used', ...
        'Famp1_1_used','Famp1_2_used','Famp1_3_used','Famp1_4_used', ...
        'disp_peak','disp_pp','vel_peak','vel_rms','acc_peak','acc_rms', ...
        'orbit_radius_max','amp_1X','amp_2X','amp_3X','ratio_2X_1X','ratio_3X_1X', ...
        'case_disp_peak','case_vel_rms','case_acc_rms','simulation_status'});
end


function metrics = failed_metrics(result)
    metrics = table( ...
        {result.case_id}, result.K_level, result.kb1_case, result.kb2_case, result.kb3_case, result.Kzx_case, result.Kzy_case, ...
        result.rpm, result.f_rot, result.unbalance_amplification, ...
        result.U(1), result.U(2), result.U(3), result.U(4), ...
        result.Famp1(1), result.Famp1(2), result.Famp1(3), result.Famp1(4), ...
        NaN, NaN, NaN, NaN, NaN, NaN, NaN, NaN, NaN, NaN, NaN, NaN, ...
        NaN, NaN, NaN, {result.simulation_status}, ...
        'VariableNames', {'case_id','K_level','kb1_case','kb2_case','kb3_case','Kzx_case','Kzy_case', ...
        'rpm','oneX_Hz','unbalance_amplification', ...
        'U1_used','U2_used','U3_used','U4_used', ...
        'Famp1_1_used','Famp1_2_used','Famp1_3_used','Famp1_4_used', ...
        'disp_peak','disp_pp','vel_peak','vel_rms','acc_peak','acc_rms', ...
        'orbit_radius_max','amp_1X','amp_2X','amp_3X','ratio_2X_1X','ratio_3X_1X', ...
        'case_disp_peak','case_vel_rms','case_acc_rms','simulation_status'});
end


function plot_case_figures(result, fig_dir)
    t = result.signals.t;
    fig = figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 1120 780]);
    subplot(2,2,1)
    plot(t, result.signals.rotor_x * 1e6, 'k', t, result.signals.rotor_y * 1e6, 'b', 'LineWidth', 1.0)
    xlabel('时间 / s'); ylabel('位移 / μm'); title('转子轴承处位移时域')
    legend({'x 向','y 向'}, 'Location', 'best'); grid on
    subplot(2,2,2)
    plot(t, result.signals.rotor_vx * 1e3, 'k', t, result.signals.rotor_vy * 1e3, 'b', 'LineWidth', 1.0)
    xlabel('时间 / s'); ylabel('速度 / (mm/s)'); title('转子轴承处速度时域')
    legend({'x 向','y 向'}, 'Location', 'best'); grid on
    subplot(2,2,3)
    plot(result.spectrum.freq_vx, result.spectrum.vel_amp_x * 1e3, 'k', 'LineWidth', 1.0)
    hold on; mark_harmonics(); hold off
    xlim([0 1000]); xlabel('频率 / Hz'); ylabel('速度幅值 / (mm/s)'); title('转子轴承处 x 向速度频谱')
    grid on
    subplot(2,2,4)
    plot(result.spectrum.freq_ax, result.spectrum.acc_amp_x, 'k', 'LineWidth', 1.0)
    hold on; mark_harmonics(); hold off
    xlim([0 1000]); xlabel('频率 / Hz'); ylabel('加速度幅值 / (m/s^2)'); title('转子轴承处 x 向加速度频谱')
    grid on
    sgtitle(sprintf('%s，K=%.1e N/m', result.case_id, result.K_level))
    exportgraphics(fig, fullfile(fig_dir, sprintf('case_bearing_stiffness_%s_four_panel.png', result.case_id)), 'Resolution', 220);
    close(fig)

    fig = figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 680 560]);
    plot(result.signals.orbit_x * 1e6, result.signals.orbit_y * 1e6, 'k', 'LineWidth', 1.2)
    axis equal
    grid on
    xlabel('x / μm'); ylabel('y / μm')
    title(sprintf('转子轴承处轴心轨迹：%s，K=%.1e N/m', result.case_id, result.K_level))
    exportgraphics(fig, fullfile(fig_dir, sprintf('case_bearing_stiffness_%s_orbit.png', result.case_id)), 'Resolution', 220);
    close(fig)
end


function plot_comparison_figures(results, fig_dir)
    valid = cellfun(@(r) isfield(r, 'signals') && ~strcmp(r.simulation_status, 'failed'), results);
    valid_results = results(valid);
    if isempty(valid_results)
        return
    end
    K_levels = cellfun(@(r) r.K_level, valid_results);
    colors = lines(numel(valid_results));
    labels = stiffness_labels(K_levels);

    plot_xy_time_compare(valid_results, colors, labels, 'rotor_x', 'rotor_y', 1e6, '位移 / μm', ...
        '不同轴承支承刚度下转子轴承处位移响应对比', fullfile(fig_dir, 'fig_5_13_displacement_compare.png'));
    plot_xy_time_compare(valid_results, colors, labels, 'rotor_vx', 'rotor_vy', 1e3, 'v / (mm/s)', ...
        '不同轴承支承刚度下转子轴承处速度响应对比', fullfile(fig_dir, 'fig_5_14_velocity_compare.png'));
    plot_xy_time_compare(valid_results, colors, labels, 'rotor_ax', 'rotor_ay', 1, 'a / (m/s^2)', ...
        '不同轴承支承刚度下转子轴承处加速度响应对比', fullfile(fig_dir, 'fig_5_15_acceleration_compare.png'));

    fig = figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 820 700]);
    hold on
    for ii = 1:numel(valid_results)
        plot(valid_results{ii}.signals.orbit_x * 1e6, valid_results{ii}.signals.orbit_y * 1e6, ...
            'LineWidth', 1.1, 'Color', colors(ii,:))
    end
    hold off
    axis equal
    grid on
    xlabel('x / μm'); ylabel('y / μm')
    title('不同轴承支承刚度下转子轴承处轴心轨迹对比')
    legend(labels, 'Location', 'best')
    exportgraphics(fig, fullfile(fig_dir, 'fig_5_16_orbit_compare.png'), 'Resolution', 220);
    close(fig)

    fig = figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 920 620]);
    hold on
    for ii = 1:numel(valid_results)
        plot(valid_results{ii}.spectrum.freq_ax, valid_results{ii}.spectrum.acc_amp_x, ...
            'LineWidth', 1.1, 'Color', colors(ii,:))
    end
    mark_harmonics()
    hold off
    xlim([0 1000])
    grid on
    xlabel('频率 / Hz'); ylabel('加速度幅值 / (m/s^2)')
    title('不同轴承支承刚度下转子轴承处加速度频谱对比')
    legend(labels, 'Location', 'best')
    exportgraphics(fig, fullfile(fig_dir, 'fig_5_17_acc_spectrum_compare.png'), 'Resolution', 220);
    close(fig)

    rows = cellfun(@(r) r.metrics, valid_results, 'UniformOutput', false);
    tbl = vertcat(rows{:});
    fig = figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 1160 800]);
    tiledlayout(2, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
    metric_tile(K_levels, tbl.disp_pp * 1e6, '位移峰峰值 / μm', '位移峰峰值');
    metric_tile(K_levels, tbl.vel_rms * 1e3, '速度 RMS / (mm/s)', '速度 RMS');
    metric_tile(K_levels, tbl.acc_rms, '加速度 RMS / (m/s^2)', '加速度 RMS');
    metric_tile(K_levels, tbl.orbit_radius_max * 1e6, '轨迹半径 / μm', '轴心轨迹最大半径');
    metric_tile(K_levels, tbl.amp_1X, '1X 加速度幅值 / (m/s^2)', '1X 加速度分量');
    metric_tile(K_levels, tbl.ratio_2X_1X, '2X/1X', '2X/1X 幅值比');
    sgtitle('轴承支承刚度变化对转子响应指标的影响')
    exportgraphics(fig, fullfile(fig_dir, 'fig_5_18_indicator_compare.png'), 'Resolution', 220);
    close(fig)

    fig = figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 1120 620]);
    tiledlayout(1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    nexttile
    hold on
    semilogx(K_levels, tbl.amp_1X, '-o', 'LineWidth', 1.2);
    semilogx(K_levels, tbl.amp_2X, '-s', 'LineWidth', 1.2);
    semilogx(K_levels, tbl.amp_3X, '-^', 'LineWidth', 1.2);
    hold off
    grid on
    xlabel('轴承支承刚度 / (N/m)'); ylabel('加速度幅值 / (m/s^2)')
    title('1X、2X、3X 分量')
    legend({'1X=165 Hz','2X=330 Hz','3X=495 Hz'}, 'Location', 'best')
    nexttile
    hold on
    semilogx(K_levels, tbl.ratio_2X_1X, '-o', 'LineWidth', 1.2);
    semilogx(K_levels, tbl.ratio_3X_1X, '-s', 'LineWidth', 1.2);
    hold off
    grid on
    xlabel('轴承支承刚度 / (N/m)'); ylabel('幅值比')
    title('倍频分量相对 1X 的变化')
    legend({'2X/1X','3X/1X'}, 'Location', 'best')
    sgtitle('轴承支承刚度变化对倍频分量及幅值比的影响')
    exportgraphics(fig, fullfile(fig_dir, 'fig_5_19_harmonic_compare.png'), 'Resolution', 220);
    close(fig)
end


function plot_xy_time_compare(results, colors, labels, field_x, field_y, scale, y_label, title_text, file_name)
    fig = figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 1040 700]);
    tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
    nexttile
    hold on
    for ii = 1:numel(results)
        plot(results{ii}.signals.t, scale * results{ii}.signals.(field_x), ...
            'LineWidth', 1.0, 'Color', colors(ii,:))
    end
    hold off
    grid on
    ylabel(['x 向 ' y_label])
    title(title_text)
    legend(labels, 'Location', 'best', 'NumColumns', 2)
    nexttile
    hold on
    for ii = 1:numel(results)
        plot(results{ii}.signals.t, scale * results{ii}.signals.(field_y), ...
            'LineWidth', 1.0, 'Color', colors(ii,:))
    end
    hold off
    grid on
    xlabel('时间 / s'); ylabel(['y 向 ' y_label])
    exportgraphics(fig, file_name, 'Resolution', 220);
    close(fig)
end


function metric_tile(x, y, y_label, title_text)
    nexttile
    semilogx(x, y, '-o', 'LineWidth', 1.2, 'MarkerFaceColor', [0.10 0.35 0.70]);
    grid on
    xlabel('轴承支承刚度 / (N/m)')
    ylabel(y_label)
    title(title_text)
    for ii = 1:numel(x)
        text(x(ii), y(ii), sprintf(' %.3g', y(ii)), 'FontSize', 8, ...
            'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'left');
    end
end


function write_markdown_report(summary, fig_dir, out_file)
    fid = fopen(out_file, 'w', 'n', 'UTF-8');
    fprintf(fid, '# 5.3 轴承支承刚度对转子-轴承-机匣系统振动响应的影响\n\n');
    fprintf(fid, '本研究固定转速为 9900 r/min，1X=165 Hz，仅改变 `mian.m` 中实际装配进整体刚度矩阵 `KK` 的 `kb1` 和 `kb2`。五组工况的 `K_level` 分别为 1.0e6、1.0e7、1.0e8、1.0e9、1.0e10 N/m；`kb3`、轴承支承形式的 `Kzx/Kzy` 和 `K_b` 在当前耦合模型中不存在，因此汇总表对应列填 NaN。其余参数保持原程序设置不变，且 `data(5)=0`。\n\n');
    fprintf(fid, '完整统计结果见 `bearing_stiffness_sweep_9900_summary.xlsx`，各工况完整时域数据、频谱数据和指标结构体保存在 `results_bearing_stiffness_K*.mat`。\n\n');
    fprintf(fid, '|case_id|K_level|kb1_case|kb2_case|disp_pp|vel_rms|acc_rms|orbit_radius_max|amp_1X|amp_2X|amp_3X|2X/1X|3X/1X|status|\n');
    fprintf(fid, '|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|\n');
    for ii = 1:height(summary)
        fprintf(fid, '|%s|%.4e|%.4e|%.4e|%.4e|%.4e|%.4e|%.4e|%.4e|%.4e|%.4e|%.5g|%.5g|%s|\n', ...
            summary.case_id{ii}, summary.K_level(ii), summary.kb1_case(ii), summary.kb2_case(ii), ...
            summary.disp_pp(ii), summary.vel_rms(ii), summary.acc_rms(ii), summary.orbit_radius_max(ii), ...
            summary.amp_1X(ii), summary.amp_2X(ii), summary.amp_3X(ii), ...
            summary.ratio_2X_1X(ii), summary.ratio_3X_1X(ii), summary.simulation_status{ii});
    end
    fprintf(fid, '\n');
    figs = {'fig_5_13_displacement_compare.png','fig_5_14_velocity_compare.png','fig_5_15_acceleration_compare.png', ...
        'fig_5_16_orbit_compare.png','fig_5_17_acc_spectrum_compare.png','fig_5_18_indicator_compare.png','fig_5_19_harmonic_compare.png'};
    captions = {'图 5.13 不同轴承支承刚度下转子节点位移响应对比', ...
        '图 5.14 不同轴承支承刚度下转子节点速度响应对比', ...
        '图 5.15 不同轴承支承刚度下转子节点加速度响应对比', ...
        '图 5.16 不同轴承支承刚度下轴心轨迹对比', ...
        '图 5.17 不同轴承支承刚度下加速度频谱对比', ...
        '图 5.18 轴承支承刚度变化对主要响应指标的影响', ...
        '图 5.19 轴承支承刚度变化对倍频分量及幅值比的影响'};
    for ii = 1:numel(figs)
        fprintf(fid, '![%s](%s)\n\n', captions{ii}, slash_path(fullfile(fig_dir, figs{ii})));
    end
    fclose(fid);
end


function [freq, amp] = single_sided_spectrum(y, fs)
    y = y(:);
    n_sig = numel(y);
    y = y - local_mean(y);
    Y = fft(y);
    p2 = abs(Y / n_sig);
    n1 = floor(n_sig / 2) + 1;
    amp = p2(1:n1);
    if n1 > 2
        amp(2:end-1) = 2 * amp(2:end-1);
    end
    freq = (0:n1-1).' * fs / n_sig;
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


function labels = stiffness_labels(K_levels)
    labels = arrayfun(@(s) sprintf('%.0e N/m', s), K_levels, 'UniformOutput', false);
end


function mark_harmonics()
    hs = [165, 330, 495];
    names = {'1X', '2X', '3X'};
    yl = ylim;
    for kk = 1:numel(hs)
        plot([hs(kk) hs(kk)], yl, 'r--', 'LineWidth', 0.85)
        text(hs(kk), yl(2) * 0.92, names{kk}, 'Color', 'r', ...
            'HorizontalAlignment', 'center', 'VerticalAlignment', 'top', ...
            'FontWeight', 'bold')
    end
    ylim(yl)
end


function out = slash_path(path_in)
    out = strrep(path_in, '\', '/');
end
