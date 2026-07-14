function result = run_microphysics_interface_validation()
%RUN_MICROPHYSICS_INTERFACE_VALIDATION B0 then one frozen/new B1 regression.

root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
addpath(root);
cfg = microphysics_config();
result.switches = struct('thermal', cfg.thermal.enabled, ...
    'roughness', cfg.roughness.enabled, 'impurity', cfg.impurity.enabled);

result.b0 = run_b0(cfg);
assert(result.b0.pass, 'B0 strict zero acceptance failed before B1 regression.');

params = b1_parameters(cfg);
result.b1.frozen = run_frozen_chain(params, root);
result.b1.interface = newmark_newton_multi(params);
assert_raw_contact_contract(params);
result.production_force_entry = which('nonlinear_bearing_force', '-all');
assert(strcmp(result.production_force_entry{1}, fullfile(root, 'nonlinear_bearing_force.m')), ...
    'Frozen validation path leaked into production resolution.');
result.b1 = compare_b1(result.b1.frozen, result.b1.interface, params, result.b1);
assert(result.b1.pass, 'B1 frozen/interface regression exceeded tolerance.');

result.static_boundary_pass = static_boundary_check(root);
assert(result.static_boundary_pass, 'Microphysics module boundary scan failed.');
write_validation_report(fullfile(root, 'microphysics_interface_validation.txt'), result);
end

function assert_raw_contact_contract(params)
q0 = params.static_equilibrium_result.q0;
[~, ~, ~, params.modelInfo] = build_rotor_case_model(params);
[~, bearing_state] = F_bearing(q0, zeros(size(q0)), params, numel(q0));
state = bearing_state.bearings(1);
assert(isfield(state, 'raw_contact'), 'Raw contact result is not exposed in bearing state.');
required = {'f5', 'Q', 'delta', 'delta_raw', 'loaded', 'loaded_count', ...
    'element_angle', 'contact_angle', 'normal_direction', ...
    'contact_position', 'slice_z', 'state'};
assert(all(isfield(state.raw_contact, required)), ...
    'Raw contact result contract is incomplete.');
end

function out = run_b0(cfg)
params = initial_conditions();
static = solve_static_equilibrium(params, false);
params.static_equilibrium_result = static;
params.microphysics = cfg;
params.stage4B.enable = true;
params.stage4B.mode = 'B0_zero_dynamic_load';
params.unbalance.nodes = []; params.unbalance.mass_g = [];
params.unbalance.ecc_mm = []; params.unbalance.phase = [];
sim = newmark_newton_multi(params);
out.max_u = max(abs(sim.u), [], 'all');
out.max_delta_fb = max(abs(sim.Fb_increment_hist), [], 'all');
out.max_bu = max(abs(sim.solver.axial_constraint_error_hist));
out.pass = out.max_u == 0 && out.max_delta_fb == 0 && out.max_bu == 0;
end

function params = b1_parameters(cfg)
params = initial_conditions();
static = solve_static_equilibrium(params, false);
params.static_equilibrium_result = static;
params.microphysics = cfg;
params.stage4B.enable = true;
params.stage4B.mode = 'B1_microphysics_interface_regression';
params.stage4B.linearized_bearing = false;
params.stage4B.excitation = struct('type', 'transverse_x', 'amplitude', 1, ...
    'node', 16, 'omega', params.omega, 'phase', 0);
params.unbalance.nodes = []; params.unbalance.mass_g = [];
params.unbalance.ecc_mm = []; params.unbalance.phase = [];
params.Fen = 64; params.n_Fen = 1;
end

function sim = run_frozen_chain(params, root)
shim_path = fullfile(root, 'bearing_microphysics', 'validation', 'frozen_chain');
original_dir = pwd;
cd(shim_path);
clear F_bearing;
cleanup = onCleanup(@() restore_interface_path(original_dir, shim_path));
frozen_entry = which('F_bearing');
sim = newmark_newton_multi(params);
sim.microphysics_validation_frozen_entry = frozen_entry;
end

function restore_interface_path(original_dir, shim_path)
clear F_bearing;
cd(original_dir);
if contains(path, shim_path), rmpath(shim_path); end
end

function out = compare_b1(frozen, interface, params, out)
abs_tol = 1e-12;
rel_tol = 1e-10;
out.tolerance = struct('absolute', abs_tol, 'relative', rel_tol, ...
    'fallback_relative', 1e-8, 'fallback_used', false);
out.u = compare_array(interface.u, frozen.u, abs_tol, rel_tol);
out.v = compare_array(interface.dyn, frozen.dyn, abs_tol, rel_tol);
out.a = compare_array(interface.ddyn, frozen.ddyn, abs_tol, rel_tol);
out.bearing_force = compare_array(bearing_force_history(interface, params), ...
    bearing_force_history(frozen, params), abs_tol, rel_tol);
out.contact_body_count = compare_array(interface.loaded_count_hist, ...
    frozen.loaded_count_hist, 0, 0);
out.Kb_last = compare_cell_matrix(interface.solver.bearing_Kb_last, ...
    frozen.solver.bearing_Kb_last, abs_tol, rel_tol);
out.Cb_last = compare_cell_matrix(interface.solver.bearing_Cb_last, ...
    frozen.solver.bearing_Cb_last, abs_tol, rel_tol);
out.newton_iterations = compare_array(interface.solver.iter_hist, ...
    frozen.solver.iter_hist, 0, 0);
out.unconverged_step_difference = interface.solver.unconverged_steps - ...
    frozen.solver.unconverged_steps;
out.pass = out.u.pass && out.v.pass && out.a.pass && out.bearing_force.pass && ...
    out.contact_body_count.pass && out.Kb_last.pass && out.Cb_last.pass && ...
    out.newton_iterations.pass && out.unconverged_step_difference == 0;
end

function history = bearing_force_history(sim, params)
nb = numel(params.bearing);
history = zeros(5, nb, size(sim.Fb_global_hist, 2));
for ib = 1:nb
    index = 6 * params.bearing(ib).rotor_node + (-5:-1);
    history(:, ib, :) = reshape(sim.Fb_global_hist(index, :), 5, 1, []);
end
end

function result = compare_cell_matrix(actual, expected, abs_tol, rel_tol)
actual_values = cellfun(@(x) x(:), actual, 'UniformOutput', false);
expected_values = cellfun(@(x) x(:), expected, 'UniformOutput', false);
result = compare_array(vertcat(actual_values{:}), vertcat(expected_values{:}), abs_tol, rel_tol);
end

function result = compare_array(actual, expected, abs_tol, rel_tol)
delta = abs(actual - expected);
limit = abs_tol + rel_tol * abs(expected);
result.max_abs_error = max(delta, [], 'all');
result.max_allowed_error = max(limit, [], 'all');
result.max_relative_error = max(delta ./ max(abs(expected), eps), [], 'all');
result.pass = all(delta <= limit, 'all');
end

function pass = static_boundary_check(root)
module_dirs = {fullfile(root, 'bearing_microphysics', 'thermal'), ...
    fullfile(root, 'bearing_microphysics', 'roughness'), ...
    fullfile(root, 'bearing_microphysics', 'impurity')};
pattern = 'newmark_newton_multi\\(|solve_static_equilibrium\\(|assemble_bearing_force\\(|params\\.modelInfo|num_rotor_dof|\\<global\\>|\\<persistent\\>';
pass = true;
for k = 1:numel(module_dirs)
    files = dir(fullfile(module_dirs{k}, '*.m'));
    for j = 1:numel(files)
        text = fileread(fullfile(files(j).folder, files(j).name));
        pass = pass && isempty(regexp(text, pattern, 'once'));
    end
end
end

function write_validation_report(file_name, result)
fid = fopen(file_name, 'wt');
assert(fid >= 0, 'Cannot create microphysics_interface_validation.txt.');
cleanup = onCleanup(@() fclose(fid));
fprintf(fid, 'Stage4-B1 microphysics interface validation\n\n');
fprintf(fid, 'switches: thermal=%d roughness=%d impurity=%d\n', ...
    result.switches.thermal, result.switches.roughness, result.switches.impurity);
fprintf(fid, 'B0: max|u|=%.16e max|DeltaFb|=%.16e max|Bu|=%.16e strict_zero_pass=%d\n', ...
    result.b0.max_u, result.b0.max_delta_fb, result.b0.max_bu, result.b0.pass);
fprintf(fid, ['B1 regression: frozen validation-only F_bearing shim -> ', ...
    'nonlinear_bearing_force_stage4B1_frozen; production F_bearing -> ', ...
    'nonlinear_bearing_force -> build_base_contact_state -> ', ...
    'apply_microphysics -> solve_contact_force.\n']);
fprintf(fid, 'frozen chain F_bearing resolved to: %s\n', result.b1.frozen.microphysics_validation_frozen_entry);
fprintf(fid, 'production nonlinear_bearing_force resolved to: %s\n', result.production_force_entry{1});
fprintf(fid, 'B1 tolerance: abs(new-frozen) <= %.1e + %.1e*abs(frozen); fallback relative %.1e used=%d\n', ...
    result.b1.tolerance.absolute, result.b1.tolerance.relative, ...
    result.b1.tolerance.fallback_relative, result.b1.tolerance.fallback_used);
write_metric(fid, 'u', result.b1.u); write_metric(fid, 'v', result.b1.v); write_metric(fid, 'a', result.b1.a);
write_metric(fid, 'two-bearing [Fx Fy Fz Mx My] history', result.b1.bearing_force);
write_metric(fid, 'contact-body count history', result.b1.contact_body_count);
write_metric(fid, 'Kb last local matrices', result.b1.Kb_last); write_metric(fid, 'Cb last local matrices', result.b1.Cb_last);
write_metric(fid, 'Newton iteration history', result.b1.newton_iterations);
fprintf(fid, 'unconverged-step difference=%d\n', result.b1.unconverged_step_difference);
fprintf(fid, 'B1 pass=%d (64 steps are interface regression only, not time-step convergence validation)\n', result.b1.pass);
fprintf(fid, 'module boundary pass=%d\n', result.static_boundary_pass);
fprintf(fid, 'thermal allowed fields: viscosity, pressure_viscosity, working_clearance, film_thickness\n');
fprintf(fid, 'roughness allowed fields: surface_height, effective_deformation, asperity_contact_ratio, contact_stiffness, contact_damping\n');
fprintf(fid, 'impurity allowed fields: characteristic_displacement, effective_deformation, contact_stiffness\n');
fprintf(fid, 'protected files: newmark_newton_multi.m; static KKT logic; structural M/C/K construction; assemble_bearing_force.m\n');
end

function write_metric(fid, name, metric)
fprintf(fid, '%s: max_abs=%.16e max_rel=%.16e max_allowed=%.16e pass=%d\n', ...
    name, metric.max_abs_error, metric.max_relative_error, ...
    metric.max_allowed_error, metric.pass);
end
