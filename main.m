function main()
%MAIN Official baseline bearing-rotor-case strong-coupled simulation.
% Workflow:
%   parameters -> ball/roller quasi dynamics -> Newmark coupling ->
%   figures/statistics -> main report -> Word manual.

clearvars -except ans;
clc;
close all;

params = initial_conditions();
validate_official_case_config(params, 'main baseline');
ensure_output_dirs(params);

fprintf('Strong-coupled bearing-rotor-case simulation\n');
fprintf('Official case: %s\n', get_field_default(params.case_definition, 'name', 'custom'));
fprintf('Speed: %.3f r/min, omega = %.6f rad/s\n', params.rpm, params.omega);
fprintf('Bearing 1: %s, rotor node %d, case node %d\n', params.bearing(1).type, params.bearing(1).rotor_node, params.bearing(1).case_node);
fprintf('Bearing 2: %s, rotor node %d, case node %d\n', params.bearing(2).type, params.bearing(2).rotor_node, params.bearing(2).case_node);
fprintf('Quasi preload only: bearing1 %.1f kN, bearing2 %.1f kN\n', ...
    params.bearing(1).preload_z/1000, params.bearing(2).preload_z/1000);
fprintf('Rotor-node static_load Fy: %s N\n', mat2str(params.static_load.Fy));

% 1) Bearing quasi-dynamic calculations. Preloads are used here only.
for ib = 1:numel(params.bearing)
    params.bearing(ib).kx = [];
    params.bearing(ib).ky = [];
    params.bearing(ib).cx = [];
    params.bearing(ib).cy = [];
end
[bearingQD{1}, params.bearing(1)] = bearing_quasi_dynamic_ball(params.bearing(1), params);
[bearingQD{2}, params.bearing(2)] = bearing_quasi_dynamic_roller(params.bearing(2), params);
for ib = 1:numel(params.bearing)
    [params.bearing(ib), bearingQD{ib}] = apply_support_compliance(params.bearing(ib), bearingQD{ib});
    params.bearing(ib).operating_offset_x = bearingQD{ib}.operating_offset_x;
    params.bearing(ib).operating_offset_y = bearingQD{ib}.operating_offset_y;
    params.bearing(ib).subtract_preload_baseline = true;
end
bearingTxtFiles = write_bearing_quasi_txt_reports(params, bearingQD);

fprintf('Quasi dynamic bearing parameters:\n');
for ib = 1:numel(params.bearing)
    fprintf('  Bearing %d: kx=%.3e N/m, ky=%.3e N/m, cx=%.3e Ns/m, cy=%.3e Ns/m\n', ...
        ib, params.bearing(ib).kx, params.bearing(ib).ky, params.bearing(ib).cx, params.bearing(ib).cy);
end
fprintf('Bearing quasi-dynamic TXT reports:\n');
for ib = 1:numel(bearingTxtFiles)
    fprintf('  %s\n', bearingTxtFiles{ib});
end

% 2) Strong coupled rotor-case response.
sim = newmark_newton_multi(params);
if isfield(sim, 'modelInfo')
    params.modelInfo = sim.modelInfo;
end
save(params.solver_checkpoint_file, 'params', 'bearingQD', 'bearingTxtFiles', 'sim', '-v7.3');

if get_field_default(params, 'postprocess_in_separate_matlab', false)
    status = run_postprocess_in_separate_matlab(params.solver_checkpoint_file);
    if status == 0 && exist(params.result_mat_file, 'file')
        S = load(params.result_mat_file, 'params', 'bearingQD', 'post');
        print_console_summary(S.params, S.bearingQD, S.post);
        close all force;
        return;
    else
        warning('Separate postprocess failed; falling back to in-process postprocess.');
    end
end

% 3) Figures and numeric response output.
post = post_process(sim, params, bearingQD);

% 4) Main report and Word manual.
report_generator(params, bearingQD, sim, post);
generate_word_manual(params.manual_file);

save(params.result_mat_file, 'params', 'bearingQD', 'bearingTxtFiles', 'sim', 'post', '-v7.3');

print_console_summary(params, bearingQD, post);
close all force;
end

function status = run_postprocess_in_separate_matlab(checkpoint_file)
runner = fullfile(pwd, 'postprocess_checkpoint_job.m');
matlab_exe = fullfile(matlabroot, 'bin', 'matlab');
cmd = sprintf('"%s" -batch "cd(''%s''); postprocess_checkpoint_job(''%s'')"', ...
    matlab_exe, pwd, checkpoint_file);
fprintf('Postprocess checkpoint in separate MATLAB process:\n  %s\n', checkpoint_file);
status = system(cmd);
if ~exist(runner, 'file')
    status = 1;
end
end

function v = get_field_default(s, field, default_value)
if isfield(s, field) && ~isempty(s.(field))
    v = s.(field);
else
    v = default_value;
end
end

function ensure_output_dirs(params)
if ~isfolder(params.output_dir)
    mkdir(params.output_dir);
end
if ~isfolder(params.figure_dir)
    mkdir(params.figure_dir);
end
if isfield(params, 'bearing_txt_dir') && ~isfolder(params.bearing_txt_dir)
    mkdir(params.bearing_txt_dir);
end
end

function [bearing, qd] = apply_support_compliance(bearing, qd)
% The contact stiffness from quasi dynamics is placed in series with the
% bearing seat/case local compliance. This keeps contact physics while
% avoiding an unrealistically rigid support in the rotor response.
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

function print_console_summary(params, bearingQD, post)
fprintf('\nSimulation finished.\n');
fprintf('Report: %s\n', params.report_file);
fprintf('Manual: %s\n', params.manual_file);
fprintf('Figures: %s\n', params.figure_dir);
fprintf('MAT result: %s\n', params.result_mat_file);
for ib = 1:numel(params.bearing)
    s = post.bearing(ib).summary;
    fprintf('Bearing %d: avg loaded elements %.3f, max force %.3e N, max slip %.3f\n', ...
        ib, s.loaded_mean, s.Fmax, s.slip_max);
    fprintf('Bearing %d response: xmax %.3e m, ymax %.3e m, vxmax %.3e m/s, axmax %.3e m/s^2\n', ...
        ib, s.xmax, s.ymax, s.vxmax, s.axmax);
    fprintf('Bearing %d quasi: clearance %.3e m, max Q %.3e N, min film %.3e m\n', ...
        ib, bearingQD{ib}.clearance_work, bearingQD{ib}.max_contact_load, bearingQD{ib}.min_oil_film);
end
end
