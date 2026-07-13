function summary = run_timestep_convergence_check(Fen_list, n_Fen_override, output_tag)
%RUN_TIMESTEP_CONVERGENCE_CHECK Independent Fen convergence diagnostic.
% This diagnostic reruns the existing model with Fen = 1024, 2048 and 4096.
% It does not change the default main-program parameters and does not alter
% the Newmark solver or bearing contact-force formula.

if nargin < 1 || isempty(Fen_list)
    Fen_list = [1024 2048 4096];
end
if nargin < 2
    n_Fen_override = [];
end
if nargin < 3 || isempty(output_tag)
    output_tag = 'timestep_convergence';
end
base_params = initial_conditions();
base_output_dir = fullfile(base_params.output_dir, 'results', 'diagnostics', output_tag);
if ~isfolder(base_output_dir)
    mkdir(base_output_dir);
end

rows = struct([]);
for iFen = 1:numel(Fen_list)
    Fen = Fen_list(iFen);
    params = initial_conditions();
    params.Fen = Fen;
    if ~isempty(n_Fen_override)
        params.n_Fen = n_Fen_override;
    end
    params.postprocess_in_separate_matlab = false;
    params.output_dir = fullfile(base_output_dir, sprintf('Fen_%d', Fen));
    params.figure_dir = fullfile(params.output_dir, 'figures');
    params.report_file = fullfile(params.output_dir, sprintf('report_Fen_%d.txt', Fen));
    params.manual_file = fullfile(params.output_dir, sprintf('manual_Fen_%d.docx', Fen));
    params.result_mat_file = fullfile(params.output_dir, sprintf('result_Fen_%d.mat', Fen));
    params.solver_checkpoint_file = fullfile(params.output_dir, sprintf('solver_Fen_%d.mat', Fen));
    ensure_dir(params.output_dir);
    ensure_dir(params.figure_dir);

    for ib = 1:numel(params.bearing)
        params.bearing(ib).kx = [];
        params.bearing(ib).ky = [];
        params.bearing(ib).cx = [];
        params.bearing(ib).cy = [];
    end
    [bearingQD{1}, params.bearing(1)] = bearing_quasi_dynamic_ball(params.bearing(1), params); %#ok<AGROW>
    [bearingQD{2}, params.bearing(2)] = bearing_quasi_dynamic_roller(params.bearing(2), params); %#ok<AGROW>
    for ib = 1:numel(params.bearing)
        [params.bearing(ib), bearingQD{ib}] = apply_support_compliance_local(params.bearing(ib), bearingQD{ib});
        params.bearing(ib).operating_offset_x = bearingQD{ib}.operating_offset_x;
        params.bearing(ib).operating_offset_y = bearingQD{ib}.operating_offset_y;
        params.bearing(ib).subtract_preload_baseline = true;
    end

    sim = newmark_newton_multi(params);
    post = post_process(sim, params, bearingQD);
    save(params.result_mat_file, 'params', 'bearingQD', 'sim', 'post', '-v7.3');

    for ib = 1:numel(params.bearing)
        rows = append_metric_row(rows, params, bearingQD{ib}, sim, post, ib); %#ok<AGROW>
    end
end

summary = rows;
csv_file = fullfile(base_output_dir, 'timestep_convergence_summary.csv');
txt_file = fullfile(base_output_dir, 'timestep_convergence_summary.txt');
write_summary_csv(csv_file, rows);
write_summary_txt(txt_file, rows);
plot_timestep_convergence(base_output_dir, rows);
fprintf('Timestep convergence summary saved to:\n  %s\n  %s\n', csv_file, txt_file);
end

function rows = append_metric_row(rows, params, qd, sim, post, ib)
s = post.bearing(ib).summary;
[pressure, ~] = bearing_pressure_diagnostic(params.bearing(ib), qd);
row.Fen = params.Fen;
row.bearing_id = ib;
row.dt = sim.solver.dt;
row.mean_loaded_count = s.loaded_mean;
row.max_slip_ratio = s.slip_max;
row.max_contact_force = s.Fmax;
row.max_contact_pressure_diagnostic = max(pressure);
row.min_oil_film = min_positive(qd.oil_film);
row.max_PV = max(qd.PV);
row.max_displacement_x = s.xmax;
row.max_displacement_y = s.ymax;
row.max_velocity_x = s.vxmax;
row.max_velocity_y = s.vymax;
row.max_acceleration_x = s.axmax;
row.max_acceleration_y = s.aymax;
row.max_bearing_force_resultant = s.Fmax;
row.converged = sim.solver.unconverged_steps == 0;
row.avg_iter = sim.solver.avg_iter;
row.max_iter = sim.solver.max_used_iter;
row.unconverged_steps = sim.solver.unconverged_steps;
rows = [rows; row];
end

function [pressure, area] = bearing_pressure_diagnostic(b, qd)
if strcmpi(b.type, 'ball')
    area = max(pi*(0.20*b.Db)^2, 1e-12);
else
    area = max(b.L*b.Dw, 1e-12);
end
pressure = qd.Q/area;
end

function write_summary_csv(file_name, rows)
fid = fopen(file_name, 'wt');
if fid < 0
    error('Cannot write %s', file_name);
end
cleanupObj = onCleanup(@() fclose(fid)); %#ok<NASGU>
fields = metric_fields();
fprintf(fid, '%s\n', strjoin(fields, ','));
for i = 1:numel(rows)
    fprintf(fid, '%d,%d,%.12e,%.12e,%.12e,%.12e,%.12e,%.12e,%.12e,%.12e,%.12e,%.12e,%.12e,%.12e,%.12e,%d,%.12e,%d,%d\n', ...
        rows(i).Fen, rows(i).bearing_id, rows(i).dt, rows(i).mean_loaded_count, rows(i).max_slip_ratio, ...
        rows(i).max_contact_force, rows(i).max_contact_pressure_diagnostic, rows(i).min_oil_film, rows(i).max_PV, ...
        rows(i).max_displacement_x, rows(i).max_displacement_y, rows(i).max_velocity_x, rows(i).max_velocity_y, ...
        rows(i).max_acceleration_x, rows(i).max_acceleration_y, rows(i).converged, rows(i).avg_iter, ...
        rows(i).max_iter, rows(i).unconverged_steps);
end
end

function write_summary_txt(file_name, rows)
fid = fopen(file_name, 'wt', 'n', 'UTF-8');
if fid < 0
    error('Cannot write %s', file_name);
end
cleanupObj = onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid, 'Timestep convergence diagnostic for the bearing-rotor-case baseline\n');
fprintf(fid, 'Fen cases: %s. The default main program remains Fen=1024.\n\n', mat2str(sort(unique([rows.Fen]))));
fields = convergence_fields();
for ib = unique([rows.bearing_id])
    fprintf(fid, 'Bearing %d\n', ib);
    Fen_cases = sort(unique([rows([rows.bearing_id] == ib).Fen]));
    if numel(Fen_cases) < 2
        fprintf(fid, '  At least two Fen cases are required for a relative convergence judgement.\n\n');
        continue;
    end
    Fen_ref = Fen_cases(end-1);
    Fen_fine = Fen_cases(end);
    r2048 = find_row(rows, Fen_ref, ib);
    r4096 = find_row(rows, Fen_fine, ib);
    max_change = 0;
    for k = 1:numel(fields)
        v1 = r2048.(fields{k});
        v2 = r4096.(fields{k});
        chg = relative_change(v1, v2);
        max_change = max(max_change, chg);
        fprintf(fid, '  %s: Fen%d=%.6e, Fen%d=%.6e, relative change=%.3f%%\n', fields{k}, Fen_ref, v1, Fen_fine, v2, 100*chg);
    end
    if max_change < 0.05
        judgement = 'time step is basically converged';
        cn = '时间步基本收敛';
    elseif max_change <= 0.10
        judgement = 'usable for overall trend analysis, but high-frequency contact details should be interpreted cautiously';
        cn = '结果可用于整体趋势分析，但高频接触细节仍需谨慎';
    else
        judgement = 'time step may be insufficient; increasing Fen or reducing dt is recommended';
        cn = '当前时间步可能不足，需要提高 Fen 或减小 dt';
    end
    fprintf(fid, '  Judgement: %s.\n', judgement);
    fprintf(fid, '  判断：%s。\n\n', cn);
end
end

function plot_timestep_convergence(out_dir, rows)
for ib = unique([rows.bearing_id])
    idx = [rows.bearing_id] == ib;
    rr = rows(idx);
    Fen = [rr.Fen];
    fig = figure('Visible','off','Color','w');
    subplot(2,2,1);
    plot(Fen, [rr.max_contact_force], '-o', 'LineWidth', 1.4); grid on; box on;
    xlabel('Fen'); ylabel('Max contact force / N'); title('Contact force');
    subplot(2,2,2);
    disp_peak = max([[rr.max_displacement_x]; [rr.max_displacement_y]], [], 1);
    plot(Fen, disp_peak, '-o', 'LineWidth', 1.4); grid on; box on;
    xlabel('Fen'); ylabel('Max displacement / m'); title('Displacement');
    subplot(2,2,3);
    acc_peak = max([[rr.max_acceleration_x]; [rr.max_acceleration_y]], [], 1);
    plot(Fen, acc_peak, '-o', 'LineWidth', 1.4); grid on; box on;
    xlabel('Fen'); ylabel('Max acceleration / (m/s^2)'); title('Acceleration');
    subplot(2,2,4);
    plot(Fen, [rr.max_slip_ratio], '-o', 'LineWidth', 1.4); grid on; box on;
    xlabel('Fen'); ylabel('Max slip ratio'); title('Cage slip');
    set(findall(fig, '-property', 'FontName'), 'FontName', 'Times New Roman');
    set(findall(fig, '-property', 'FontSize'), 'FontSize', 11);
    print(fig, fullfile(out_dir, sprintf('timestep_convergence_bearing%d.png', ib)), '-dpng', '-r300');
    close(fig);
end
end

function fields = metric_fields()
fields = {'Fen','bearing_id','dt','mean_loaded_count','max_slip_ratio','max_contact_force', ...
    'max_contact_pressure_diagnostic','min_oil_film','max_PV','max_displacement_x', ...
    'max_displacement_y','max_velocity_x','max_velocity_y','max_acceleration_x', ...
    'max_acceleration_y','converged','avg_iter','max_iter','unconverged_steps'};
end

function fields = convergence_fields()
fields = {'mean_loaded_count','max_slip_ratio','max_contact_force','max_contact_pressure_diagnostic', ...
    'min_oil_film','max_PV','max_displacement_x','max_displacement_y','max_velocity_x', ...
    'max_velocity_y','max_acceleration_x','max_acceleration_y','max_bearing_force_resultant'};
end

function r = find_row(rows, Fen, ib)
idx = find([rows.Fen] == Fen & [rows.bearing_id] == ib, 1);
if isempty(idx)
    error('Missing Fen=%d bearing=%d convergence row.', Fen, ib);
end
r = rows(idx);
end

function chg = relative_change(a, b)
den = max(max(abs(a), abs(b)), eps);
chg = abs(b - a)/den;
end

function v = min_positive(x)
x = x(isfinite(x) & x > 0);
if isempty(x)
    v = NaN;
else
    v = min(x);
end
end

function ensure_dir(d)
if ~isfolder(d)
    mkdir(d);
end
end

function [bearing, qd] = apply_support_compliance_local(bearing, qd)
if isfield(bearing, 'support_series_kx') && bearing.support_series_kx > 0
    bearing.kx = 1/(1/max(bearing.kx, eps) + 1/bearing.support_series_kx);
end
if isfield(bearing, 'support_series_ky') && bearing.support_series_ky > 0
    bearing.ky = 1/(1/max(bearing.ky, eps) + 1/bearing.support_series_ky);
end
qd.kx = bearing.kx;
qd.ky = bearing.ky;
qd.cx = bearing.cx;
qd.cy = bearing.cy;
end
