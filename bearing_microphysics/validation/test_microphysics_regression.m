function test_microphysics_regression
%TEST_MICROPHYSICS_REGRESSION Acceptance test for the B0/B1 interface record.

report_file = fullfile(fileparts(mfilename('fullpath')), '..', '..', ...
    'microphysics_interface_validation.txt');
before = dir(report_file);
result = run_microphysics_interface_validation(false);
after = dir(report_file);
assert(result.b0.pass, 'B0 must retain its strict zero-dynamic-load acceptance.');
assert(result.b1.pass, 'B1 interface regression exceeded its mixed tolerance.');
assert(result.switches.thermal == false && result.switches.roughness == false && ...
    result.switches.impurity == false, 'All microphysics switches must remain disabled.');
assert(strcmp(result.source_branch, 'brunch-2-micro'), ...
    'The frozen source branch must be recorded in the validation result.');
assert(strcmp(result.source_commit, '1afcebecef812481dc05a2f054e5a3766d0cbf70'), ...
    'The frozen source commit must be recorded in the validation result.');
assert(isequaln(before.datenum, after.datenum), ...
    'write_report=false must not update the validation report file.');
test_frozen_thermal_static_entry();
end

function test_frozen_thermal_static_entry
params = initial_conditions();
thermal_state = frozen_thermal_state(20);
zero = params;
zero.static_equilibrium.gravity = 0;
zero.static_load.Fx(:) = 0; zero.static_load.Fy(:) = 0; zero.static_load.Fz(:) = 0;
zero_result = solve_static_equilibrium_frozen_thermal(zero, zero.bearing, thermal_state, []);
assert(norm(zero_result.q) <= 1e-12 && norm(zero_result.bearing_force) <= 1e-12, ...
    'Zero gravity and zero steady load must retain the zero static state.');
result = solve_static_equilibrium_frozen_thermal(params, params.bearing, thermal_state, []);
required = {'q','residual','bearing_force','contact_state','converged','iterations'};
assert(all(isfield(result, required)), 'Frozen-thermal static result contract is incomplete.');
assert(result.converged, 'Frozen-thermal static solve did not converge.');
assert(all(isfinite(result.q)) && all(isfinite(result.residual)), ...
    'Frozen-thermal static solution must remain finite.');
assert(abs(result.constraint.front_axial_gap_m) <= 1e-8, ...
    'Front axial location constraint was not enforced.');
assert(abs(result.constraint.numerical_axial_gauge_m) <= 1e-8, ...
    'Numerical axial gauge constraint was not enforced.');
assert(result.action_reaction_error <= 1e-10, ...
    'Bearing action-reaction assembly must remain exact.');
end

function thermal_state = frozen_thermal_state(T_C)
entry = struct('T_oil', T_C, 'T_final', T_C, 'T_film', T_C, ...
    'T_inner', T_C, 'T_outer', T_C, 'T_element', T_C, ...
    'eta', NaN, 'alpha_p', NaN, 'working_clearance', NaN, ...
    'film_thickness', NaN, 'thermal_preload', false);
thermal_state = struct('ball', entry, 'roller', entry);
end
