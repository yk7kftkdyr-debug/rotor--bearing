function result = run_thermal_coupling_validation()
%RUN_THERMAL_COUPLING_VALIDATION Verify the local thermal-to-system chain.

root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
addpath(root);
report_name = fullfile(root, 'thermal_coupling_validation.txt');
if exist(report_name, 'file') == 2
    delete(report_name);
end

baseline = run_microphysics_interface_validation(false);
assert(baseline.b0.pass && baseline.b1.pass, ...
    'Thermal-off Stage4 B0/B1 baseline must be exact before coupling validation.');

[params_off, cfg_off] = b1_parameters();
params_off.microphysics = cfg_off;
t_off = tic;
evalc('sim_off = newmark_newton_multi(params_off);');
time_off_s = toc(t_off);

params_on = params_off;
params_on.microphysics.thermal.enabled = true;
assert(identical_b1_case(params_off, params_on), ...
    'Thermal ON/OFF B1 cases differ outside thermal.enabled.');
t_on = tic;
evalc('sim_on = newmark_newton_multi(params_on);');
time_on_s = toc(t_on);

assert(isequaln(sim_off.q0, sim_on.q0), 'Thermal ON/OFF q0 differs.');
assert(numel(sim_off.time) == 65 && isequaln(sim_off.time, sim_on.time), ...
    'Thermal ON/OFF B1 time grids differ from the required 64-step case.');

off_contact = contact_history(sim_off, sim_off.params);
on_contact = contact_history(sim_on, sim_on.params);
metrics = compare_cases(sim_off, sim_on, off_contact, on_contact, sim_on.params);
thermal = summarize_thermal(on_contact);
pass = metrics.force_history_max_abs > 0 && ...
    (metrics.Kb_max_abs > 0 || metrics.Cb_max_abs > 0) && ...
    metrics.rotor_response_max_abs > 0 && metrics.case_response_max_abs > 0 && ...
    sim_on.solver.unconverged_steps == 0 && all(thermal.nonconverged_count == 0);

if ~pass
    error('run_thermal_coupling_validation:PhysicalTransmissionMissing', ...
        'Thermal system transmission acceptance criteria were not met.');
end
result = struct('baseline', baseline, 'metrics', metrics, 'thermal', thermal, ...
    'time_off_s', time_off_s, 'time_on_s', time_on_s, 'pass', pass);
write_report(report_name, result, sim_off, sim_on, root);
end

function [params, cfg] = b1_parameters()
params = initial_conditions();
static = solve_static_equilibrium(params, false);
params.static_equilibrium_result = static;
cfg = microphysics_config();
cfg.thermal.enabled = false; cfg.roughness.enabled = false; cfg.impurity.enabled = false;
params.stage4B.enable = true;
params.stage4B.mode = 'B1_microphysics_interface_regression';
params.stage4B.linearized_bearing = false;
params.stage4B.excitation = struct('type', 'transverse_x', 'amplitude', 1, ...
    'node', 16, 'omega', params.omega, 'phase', 0);
params.unbalance.nodes = []; params.unbalance.mass_g = [];
params.unbalance.ecc_mm = []; params.unbalance.phase = [];
params.Fen = 64; params.n_Fen = 1;
end

function same = identical_b1_case(off, on)
same = isequaln(off.static_equilibrium_result.q0, on.static_equilibrium_result.q0) && ...
    off.Fen == on.Fen && off.n_Fen == on.n_Fen && off.omega == on.omega && ...
    isequaln(off.stage4B, rmfield(on.stage4B, {})) && ...
    isequaln(off.stage4B.excitation, on.stage4B.excitation) && ...
    isequaln(off.unbalance, on.unbalance);
end

function out = contact_history(sim, params)
nb = numel(params.bearing); nt = numel(sim.time); N = size(sim.u, 1);
out.f5 = zeros(5, nb, nt); out.loaded_count = zeros(nb, nt);
out.Qmax = zeros(nb, nt); out.T_final_C = NaN(nb, nt);
out.Q_fric_W = NaN(nb, nt); out.Q_cool_W = NaN(nb, nt);
out.eta = NaN(nb, nt); out.alpha_p = NaN(nb, nt); out.c_work = NaN(nb, nt);
out.h_min = NaN(nb, nt); out.iterations = NaN(nb, nt);
out.evaluations = NaN(nb, nt); out.converged = false(nb, nt);
for it = 1:nt
    force_params = params; force_params.current_time = sim.time(it);
    [~, state] = F_bearing(sim.yn(:, it), sim.dyn(:, it), force_params, N);
    for ib = 1:nb
        b = state.bearings(ib);
        out.f5(:, ib, it) = [b.Fx; b.Fy; b.Fz; b.Mx; b.My];
        out.loaded_count(ib, it) = b.loaded_count;
        out.Qmax(ib, it) = max(b.raw_contact.Q, [], 'all');
        t = b.microphysics_state.temperature;
        if t.enabled
            out.T_final_C(ib, it) = t.T_final_C;
            out.Q_fric_W(ib, it) = t.Q_fric_W;
            out.Q_cool_W(ib, it) = t.Q_cool_W;
            out.eta(ib, it) = t.viscosity_Pa_s;
            out.alpha_p(ib, it) = t.pressure_viscosity_Pa_inv;
            out.c_work(ib, it) = t.working_clearance_m;
            out.h_min(ib, it) = t.film_thickness_min_m;
            out.iterations(ib, it) = t.iterations;
            out.evaluations(ib, it) = t.local_contact_evaluations;
            out.converged(ib, it) = t.converged;
        end
    end
end
end

function metrics = compare_cases(off, on, off_contact, on_contact, params)
metrics.force_history_max_abs = max(abs(on_contact.f5 - off_contact.f5), [], 'all');
metrics.loaded_count_max_abs = max(abs(on_contact.loaded_count - off_contact.loaded_count), [], 'all');
metrics.Qmax_max_abs = max(abs(on_contact.Qmax - off_contact.Qmax), [], 'all');
metrics.Kb_max_abs = cell_max_abs_difference(on.solver.bearing_Kb_last, off.solver.bearing_Kb_last);
metrics.Cb_max_abs = cell_max_abs_difference(on.solver.bearing_Cb_last, off.solver.bearing_Cb_last);
[rotor_dof, case_dof] = response_dofs(params, on.modelInfo);
metrics.rotor_u_max_abs = max(abs(on.u(rotor_dof,:) - off.u(rotor_dof,:)), [], 'all');
metrics.rotor_v_max_abs = max(abs(on.dyn(rotor_dof,:) - off.dyn(rotor_dof,:)), [], 'all');
metrics.rotor_a_max_abs = max(abs(on.ddyn(rotor_dof,:) - off.ddyn(rotor_dof,:)), [], 'all');
metrics.case_u_max_abs = max(abs(on.u(case_dof,:) - off.u(case_dof,:)), [], 'all');
metrics.case_v_max_abs = max(abs(on.dyn(case_dof,:) - off.dyn(case_dof,:)), [], 'all');
metrics.case_a_max_abs = max(abs(on.ddyn(case_dof,:) - off.ddyn(case_dof,:)), [], 'all');
metrics.rotor_response_max_abs = max([metrics.rotor_u_max_abs, metrics.rotor_v_max_abs, metrics.rotor_a_max_abs]);
metrics.case_response_max_abs = max([metrics.case_u_max_abs, metrics.case_v_max_abs, metrics.case_a_max_abs]);
end

function value = cell_max_abs_difference(a, b)
value = 0;
for k = 1:numel(a)
    value = max(value, max(abs(a{k} - b{k}), [], 'all'));
end
end

function [rotor_dof, case_dof] = response_dofs(params, modelInfo)
rotor_dof = []; case_dof = [];
for ib = 1:numel(params.bearing)
    b = params.bearing(ib);
    rotor_dof = [rotor_dof, 6*b.rotor_node + (-5:-1)]; %#ok<AGROW>
    case_dof = [case_dof, modelInfo.num_rotor_dof + 6*b.case_node + (-5:-1)]; %#ok<AGROW>
end
end

function summary = summarize_thermal(history)
final = size(history.T_final_C, 2);
summary.final_T_C = history.T_final_C(:, final);
summary.final_Q_fric_W = history.Q_fric_W(:, final);
summary.final_Q_cool_W = history.Q_cool_W(:, final);
summary.final_eta = history.eta(:, final); summary.final_alpha_p = history.alpha_p(:, final);
summary.final_c_work = history.c_work(:, final); summary.final_h_min = history.h_min(:, final);
summary.final_loaded_count = history.loaded_count(:, final); summary.final_Qmax = history.Qmax(:, final);
summary.mean_iterations = mean(history.iterations, 2, 'omitnan');
summary.max_iterations = max(history.iterations, [], 2);
summary.contact_evaluations = sum(history.evaluations, 2, 'omitnan');
summary.nonconverged_count = sum(~history.converged, 2);
end

function write_report(file_name, result, off, on, root)
fid = fopen(file_name, 'wt');
assert(fid >= 0, 'Cannot create thermal_coupling_validation.txt.');
cleanup = onCleanup(@() fclose(fid));
fprintf(fid, 'Closed-loop thermal contact to rotor/casing coupling validation\n\n');
fprintf(fid, 'branch: %s\n', git_text(root, 'branch --show-current'));
fprintf(fid, 'baseline commit: 1afcebe\nthermal configuration commit: c8bf0a1\n');
fprintf(fid, 'subsequent commits: 8c8349f, 88ea909, ef9dbcf\n');
fprintf(fid, 'B0 strict: max|u|=%.16e max|DeltaFb|=%.16e max|Bu|=%.16e\n', ...
    result.baseline.b0.max_u, result.baseline.b0.max_delta_fb, result.baseline.b0.max_bu);
fprintf(fid, 'B1 frozen OFF errors: u=%.16e v=%.16e a=%.16e F5=%.16e loaded=%.16e Kb=%.16e Cb=%.16e Newton=%.16e unconverged-diff=%d\n', ...
    result.baseline.b1.u.max_abs_error, result.baseline.b1.v.max_abs_error, result.baseline.b1.a.max_abs_error, result.baseline.b1.bearing_force.max_abs_error, result.baseline.b1.contact_body_count.max_abs_error, result.baseline.b1.Kb_last.max_abs_error, result.baseline.b1.Cb_last.max_abs_error, result.baseline.b1.newton_iterations.max_abs_error, result.baseline.b1.unconverged_step_difference);
write_properties(fid);
fprintf(fid, 'B1 ON/OFF case equality: q0=%d time-grid=%d steps=%d excitation=[node=%d type=%s amp=%.16e omega=%.16e phase=%.16e]\n', ...
    isequaln(off.q0,on.q0), isequaln(off.time,on.time), numel(off.time)-1, on.params.stage4B.excitation.node, on.params.stage4B.excitation.type, on.params.stage4B.excitation.amplitude, on.params.stage4B.excitation.omega, on.params.stage4B.excitation.phase);
fprintf(fid, 'F5 full-history max|ON-OFF|=%.16e; loaded_count max diff=%.16e; Qmax max diff=%.16e\n', result.metrics.force_history_max_abs, result.metrics.loaded_count_max_abs, result.metrics.Qmax_max_abs);
fprintf(fid, 'Kb final max diff=%.16e; Cb final max diff=%.16e\n', result.metrics.Kb_max_abs, result.metrics.Cb_max_abs);
fprintf(fid, 'rotor nearby u/v/a max diff=[%.16e %.16e %.16e]\n', result.metrics.rotor_u_max_abs, result.metrics.rotor_v_max_abs, result.metrics.rotor_a_max_abs);
fprintf(fid, 'case nearby u/v/a max diff=[%.16e %.16e %.16e]\n', result.metrics.case_u_max_abs, result.metrics.case_v_max_abs, result.metrics.case_a_max_abs);
fprintf(fid, 'runtime OFF=%.6f s ON=%.6f s; Newton unconverged OFF=%d ON=%d\n', result.time_off_s, result.time_on_s, off.solver.unconverged_steps, on.solver.unconverged_steps);
for ib = 1:2
    fprintf(fid, 'bearing %d final: T=%.16e Qfric=%.16e Qcool=%.16e residual=%.16e eta=%.16e alpha_p=%.16e c_work=%.16e h_min=%.16e loaded=%d Qmax=%.16e mean/max thermal iterations=%.16e/%.16e local evaluations=%d thermal nonconverged=%d\n', ...
        ib, result.thermal.final_T_C(ib), result.thermal.final_Q_fric_W(ib), result.thermal.final_Q_cool_W(ib), abs(result.thermal.final_Q_fric_W(ib)-result.thermal.final_Q_cool_W(ib))/on.params.microphysics.thermal.H_W_per_K, result.thermal.final_eta(ib), result.thermal.final_alpha_p(ib), result.thermal.final_c_work(ib), result.thermal.final_h_min(ib), result.thermal.final_loaded_count(ib), result.thermal.final_Qmax(ib), result.thermal.mean_iterations(ib), result.thermal.max_iterations(ib), result.thermal.contact_evaluations(ib), result.thermal.nonconverged_count(ib));
end
fprintf(fid, 'protected files unchanged since 1afcebe: %d\n', protected_unchanged(root));
fprintf(fid, 'FINAL_%s\n', ternary(result.pass, 'PASS', 'FAIL'));
end

function write_properties(fid)
p = thermal_lubricant_properties([20 50 80 100]);
for k = 1:numel(p.temperature_C)
    fprintf(fid, 'properties T=%.0f C: nu=%.16e cSt eta=%.16e Pa s alpha_p=%.16e Pa^-1\n', p.temperature_C(k), p.nu_cSt(k), p.eta_Pa_s(k), p.alpha_p_Pa_inv(k));
end
end

function ok = protected_unchanged(root)
files = {'newmark_newton_multi.m', 'build_rotor_case_model.m', 'assemble_bearing_force.m', 'microphysics_interface_validation.txt'};
args = strjoin(files, ' ');
[status, ~] = system(sprintf('cd "%s" && git diff --quiet 1afcebe -- %s', root, args));
ok = status == 0;
end

function out = git_text(root, command)
[status, out] = system(sprintf('cd "%s" && git %s', root, command));
assert(status == 0, 'Unable to query git metadata.');
out = strtrim(out);
end

function value = ternary(condition, yes_value, no_value)
if condition, value = yes_value; else, value = no_value; end
end
