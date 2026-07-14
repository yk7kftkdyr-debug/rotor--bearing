function test_microphysics_interface
%TEST_MICROPHYSICS_INTERFACE Contract test for the transparent gateway.

cfg = microphysics_config();
assert(~cfg.thermal.enabled && ~cfg.roughness.enabled && ~cfg.impurity.enabled, ...
    'All microphysics switches must default to false.');

contact_base = struct('contact_stiffness', 10, 'working_clearance', 2e-6, ...
    'effective_deformation', 3e-6);
[contact_mod, micro_state] = apply_microphysics(contact_base, struct(), struct(), cfg);

assert(isequaln(contact_mod, contact_base), ...
    'Disabled microphysics must preserve the complete contact record.');
assert(all(isfield(micro_state, {'temperature', 'roughness_phase', ...
    'impurity_state', 'history'})), ...
    'The microphysics state must expose the required persistent fields.');

source = fileread('nonlinear_bearing_force.m');
assert(numel(strfind(source, 'apply_microphysics(')) == 1, ...
    'The bearing entry must contain exactly one microphysics gateway call.');

module_dirs = {'bearing_microphysics/thermal', ...
    'bearing_microphysics/roughness', 'bearing_microphysics/impurity'};
for k = 1:numel(module_dirs)
    files = dir(fullfile(module_dirs{k}, '*.m'));
    for j = 1:numel(files)
        module_source = fileread(fullfile(files(j).folder, files(j).name));
        assert(isempty(regexp(module_source, ...
            'newmark_newton_multi|solve_static_equilibrium|\\<MM\\>|\\<KK\\>|\\<KKT\\>|\\<192\\>|assemble_bearing_force|rotor_node|case_node|num_rotor_dof', ...
            'once')), 'A microphysics module references a protected structural symbol.');
    end
end
end
