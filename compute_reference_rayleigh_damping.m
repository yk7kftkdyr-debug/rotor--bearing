function reference = compute_reference_rayleigh_damping(params_frozen, stiffness_results, damping_results, cfg)
%COMPUTE_REFERENCE_RAYLEIGH_DAMPING Build the fixed 20 C transverse reference model.

if nargin < 4 || isempty(cfg), cfg = struct(); end
reference = empty_reference();
cfg = fixed_config(cfg);
validate_inputs(params_frozen, stiffness_results, damping_results);

[M, ~, K_structure, modelInfo] = build_rotor_case_model(params_frozen);
n = size(M, 1);
validate_model_matrices(M, K_structure, modelInfo, n);
[K_bearing_global, C_ehl_global] = assemble_bearing_terms(params_frozen, stiffness_results, damping_results, modelInfo, n);
K_workpoint_20 = K_structure + K_bearing_global;
transverse_dofs = transverse_dof_indices(modelInfo);
M_t = M(transverse_dofs, transverse_dofs);
K_structure_t = K_structure(transverse_dofs, transverse_dofs);
K_bearing_t = K_bearing_global(transverse_dofs, transverse_dofs);
K_workpoint_t = K_workpoint_20(transverse_dofs, transverse_dofs);
C_foundation_t = modelInfo.C_foundation(transverse_dofs, transverse_dofs);
C_ehl_t = C_ehl_global(transverse_dofs, transverse_dofs);
G_t = modelInfo.GG(transverse_dofs, transverse_dofs);
checks = matrix_checks(M_t, K_structure_t, K_bearing_t, K_workpoint_t, C_foundation_t, C_ehl_t, G_t);
if ~checks.pass
    error('Stage8A1:ReferenceMatrixCheck', 'The 128-DOF transverse reference matrices did not pass symmetry, PSD, skew-symmetry, finite-value or size checks.');
end

[omega_rad_s, mode_shapes_full] = undamped_positive_modes(K_workpoint_t, M_t);
if numel(omega_rad_s) < 3
    error('Stage8A1:InsufficientModes', 'At least three finite positive transverse modes are required.');
end
reference_mode_indices = [1; 3];
omega1 = omega_rad_s(reference_mode_indices(1));
omega3 = omega_rad_s(reference_mode_indices(2));
if ~(isfinite(omega1) && isfinite(omega3) && omega1 > 0 && omega3 > omega1)
    error('Stage8A1:ReferenceModes', 'The first and third positive transverse modal frequencies are invalid.');
end
zeta = cfg.target_damping_ratio;
alpha_R = 2*zeta*omega1*omega3/(omega1 + omega3);
beta_R = 2*zeta/(omega1 + omega3);
C_rayleigh_reference_t = alpha_R*M_t + beta_R*K_workpoint_t;
recovered_damping_ratio = alpha_R./(2*[omega1; omega3]) + beta_R*[omega1; omega3]/2;
rayleigh_pass = all(isfinite([alpha_R; beta_R; recovered_damping_ratio])) && alpha_R >= 0 && beta_R >= 0 && ...
    all(abs(recovered_damping_ratio - zeta) <= cfg.recovered_damping_tolerance);
if ~rayleigh_pass
    error('Stage8A1:RayleighRecovery', 'The fixed Rayleigh coefficients did not recover the required 1%% damping ratio.');
end

reference.temperature_C = frozen_temperature_C(params_frozen);
reference.transverse_dofs = transverse_dofs;
reference.transverse_dof_count = numel(transverse_dofs);
reference.M_t = M_t;
reference.K_structure_t = K_structure_t;
reference.K_bearing_t = K_bearing_t;
reference.K_workpoint_t = K_workpoint_t;
reference.C_foundation_t = C_foundation_t;
reference.C_ehl_t = C_ehl_t;
reference.G_t = G_t;
reference.rotation_speed_rad_s = params_frozen.omega;
reference.mode_shapes_full = mode_shapes_full;
reference.omega_rad_s = omega_rad_s;
reference.frequency_Hz = omega_rad_s/(2*pi);
reference.reference_mode_indices = reference_mode_indices;
reference.target_damping_ratio = zeta;
reference.alpha_R = alpha_R;
reference.beta_R = beta_R;
reference.C_rayleigh_reference_t = C_rayleigh_reference_t;
reference.recovered_damping_ratio = recovered_damping_ratio;
reference.symmetry_checks = checks;
reference.pass = checks.pass && rayleigh_pass && reference.transverse_dof_count == 128 && ...
    reference.frequency_Hz(1) > 0 && reference.frequency_Hz(3) > reference.frequency_Hz(1);
end

function reference = empty_reference()
reference = struct('temperature_C', NaN, 'transverse_dofs', zeros(0,1), 'transverse_dof_count', 0, ...
    'M_t', zeros(0,0), 'K_structure_t', zeros(0,0), 'K_bearing_t', zeros(0,0), ...
    'K_workpoint_t', zeros(0,0), 'C_foundation_t', zeros(0,0), 'C_ehl_t', zeros(0,0), ...
    'G_t', zeros(0,0), 'rotation_speed_rad_s', NaN, 'mode_shapes_full', zeros(0,0), ...
    'omega_rad_s', zeros(0,1), 'frequency_Hz', zeros(0,1), 'reference_mode_indices', zeros(0,1), ...
    'target_damping_ratio', NaN, 'alpha_R', NaN, 'beta_R', NaN, ...
    'C_rayleigh_reference_t', zeros(0,0), 'recovered_damping_ratio', zeros(0,1), ...
    'symmetry_checks', empty_checks(), 'pass', false);
end

function cfg = fixed_config(cfg)
defaults = struct('target_damping_ratio', 0.01, 'recovered_damping_tolerance', 1e-12);
names = fieldnames(defaults);
for k = 1:numel(names)
    name = names{k};
    if ~isfield(cfg, name) || isempty(cfg.(name)), cfg.(name) = defaults.(name); end
    if ~isequal(cfg.(name), defaults.(name))
        error('Stage8A1:FixedConfiguration', 'Stage8A-1 requires cfg.%s = %.16g.', name, defaults.(name));
    end
end
end

function validate_inputs(params, stiffness, damping)
if ~isfield(params, 'bearing') || numel(params.bearing) ~= 2 || ...
        ~strcmpi(params.bearing(1).type, 'ball') || ~strcmpi(params.bearing(2).type, 'roller')
    error('Stage8A1:BearingOrder', 'Stage8A-1 requires front ball and rear roller bearing records.');
end
if ~isfield(params, 'omega') || ~isscalar(params.omega) || ~isfinite(params.omega)
    error('Stage8A1:RotationSpeed', 'params_frozen.omega must be one finite scalar.');
end
if ~isequal(numel(stiffness), 2) || ~isequal(numel(damping), 2)
    error('Stage8A1:ResultsCount', 'Exactly two stiffness and two damping workpoint results are required.');
end
for ib = 1:2
    if ~isfield(stiffness(ib), 'pass') || ~stiffness(ib).pass || ~isfield(stiffness(ib), 'K_local') || ...
            ~isequal(size(stiffness(ib).K_local), [5 5]) || any(~isfinite(stiffness(ib).K_local), 'all')
        error('Stage8A1:StiffnessGate', 'Each Stage6A K_local must be a finite passing 5-by-5 matrix.');
    end
    if ~isfield(damping(ib), 'pass') || ~damping(ib).pass || ~isfield(damping(ib), 'C_local') || ...
            ~isequal(size(damping(ib).C_local), [5 5]) || any(~isfinite(damping(ib).C_local), 'all')
        error('Stage8A1:DampingGate', 'Each Stage6B C_local must be a finite passing 5-by-5 matrix.');
    end
end
T_oil_C = frozen_temperature_C(params);
if abs(T_oil_C - 20) > 100*eps(max(1, abs(T_oil_C)))
    error('Stage8A1:TemperatureGate', 'Stage8A-1 is fixed to the 20 C thermal-consistent workpoint.');
end
end

function validate_model_matrices(M, K_structure, modelInfo, n)
required = {'num_rotor_dof','num_case_dof','num_rotor_nodes','num_case_nodes','C_foundation','GG'};
for k = 1:numel(required)
    if ~isfield(modelInfo, required{k}), error('Stage8A1:ModelInfoField', 'modelInfo.%s is required.', required{k}); end
end
if n ~= 192 || ~isequal(size(M), [n n]) || ~isequal(size(K_structure), [n n]) || ...
        modelInfo.num_rotor_dof + modelInfo.num_case_dof ~= n || ...
        ~isequal(size(modelInfo.C_foundation), [n n]) || ~isequal(size(modelInfo.GG), [n n])
    error('Stage8A1:ModelDimensions', 'Stage8A-1 requires a consistent 192-DOF model and damping/gyroscopic matrices.');
end
if modelInfo.num_rotor_nodes ~= 19 || modelInfo.num_case_nodes ~= 13
    error('Stage8A1:ModelNodes', 'Stage8A-1 requires 19 rotor and 13 casing nodes.');
end
end

function [K_global, C_global] = assemble_bearing_terms(params, stiffness, damping, modelInfo, n)
K_global = zeros(n,n); C_global = zeros(n,n);
for ib = 1:2
    B = bearing_relative_map(params.bearing(ib), modelInfo, n);
    K_global = K_global + B'*stiffness(ib).K_local*B;
    C_global = C_global + B'*damping(ib).C_local*B;
end
end

function B = bearing_relative_map(bearing, modelInfo, n)
ir = 6*bearing.rotor_node + (-5:-1);
ic = modelInfo.num_rotor_dof + 6*bearing.case_node + (-5:-1);
if any(ir < 1 | ir > n) || any(ic < 1 | ic > n)
    error('Stage8A1:LocalMap', 'A bearing 5-DOF local map is outside the 192-DOF state.');
end
B = sparse([1:5 1:5], [ir ic], [ones(1,5) -ones(1,5)], 5, n);
end

function dofs = transverse_dof_indices(modelInfo)
rotor_nodes = 1:modelInfo.num_rotor_nodes;
case_nodes = 1:modelInfo.num_case_nodes;
rotor_dofs = reshape(6*rotor_nodes + [-5; -4; -2; -1], [], 1);
case_dofs = modelInfo.num_rotor_dof + reshape(6*case_nodes + [-5; -4; -2; -1], [], 1);
dofs = [rotor_dofs; case_dofs];
if numel(dofs) ~= 4*(modelInfo.num_rotor_nodes + modelInfo.num_case_nodes) || ...
        numel(dofs) ~= 128 || numel(unique(dofs)) ~= 128
    error('Stage8A1:TransverseDofs', 'The transverse [ux uy theta_x theta_y] subspace must contain exactly 128 unique DOFs.');
end
end

function checks = matrix_checks(M, K_structure, K_bearing, K_workpoint, C_foundation, C_ehl, G)
checks = empty_checks();
matrices = {M, K_structure, K_bearing, K_workpoint, C_foundation, C_ehl, G};
if any(cellfun(@(A) ~isequal(size(A), [128 128]) || any(~isfinite(A), 'all'), matrices))
    return;
end
checks.M_symmetry_error = relative_symmetry_error(M);
checks.K_structure_symmetry_error = relative_symmetry_error(K_structure);
checks.K_bearing_symmetry_error = relative_symmetry_error(K_bearing);
checks.K_workpoint_symmetry_error = relative_symmetry_error(K_workpoint);
checks.C_foundation_symmetry_error = relative_symmetry_error(C_foundation);
checks.C_ehl_symmetry_error = relative_symmetry_error(C_ehl);
checks.G_skew_symmetry_error = norm(G + G', 'fro')/max(norm(G, 'fro'), 1);
[checks.C_foundation_min_eigenvalue, checks.C_foundation_max_eigenvalue] = symmetric_extrema(C_foundation);
[checks.C_ehl_min_eigenvalue, checks.C_ehl_max_eigenvalue] = symmetric_extrema(C_ehl);
symmetry_tolerance = 1e-10;
psd_foundation = checks.C_foundation_min_eigenvalue >= -1e-10*max(1, checks.C_foundation_max_eigenvalue);
psd_ehl = checks.C_ehl_min_eigenvalue >= -1e-10*max(1, checks.C_ehl_max_eigenvalue);
checks.pass = checks.M_symmetry_error <= symmetry_tolerance && ...
    checks.K_structure_symmetry_error <= symmetry_tolerance && ...
    checks.K_bearing_symmetry_error <= symmetry_tolerance && ...
    checks.K_workpoint_symmetry_error <= symmetry_tolerance && ...
    checks.C_foundation_symmetry_error <= symmetry_tolerance && ...
    checks.C_ehl_symmetry_error <= symmetry_tolerance && ...
    checks.G_skew_symmetry_error <= symmetry_tolerance && psd_foundation && psd_ehl;
end

function checks = empty_checks()
checks = struct('M_symmetry_error', NaN, 'K_structure_symmetry_error', NaN, ...
    'K_bearing_symmetry_error', NaN, 'K_workpoint_symmetry_error', NaN, ...
    'C_foundation_symmetry_error', NaN, 'C_ehl_symmetry_error', NaN, ...
    'G_skew_symmetry_error', NaN, 'C_foundation_min_eigenvalue', NaN, ...
    'C_foundation_max_eigenvalue', NaN, 'C_ehl_min_eigenvalue', NaN, ...
    'C_ehl_max_eigenvalue', NaN, 'pass', false);
end

function error_value = relative_symmetry_error(A)
error_value = norm(A-A', 'fro')/max(norm(A, 'fro'), 1);
end

function [minimum, maximum] = symmetric_extrema(A)
values = eig(0.5*(A + A'));
minimum = min(values); maximum = max(values);
end

function [omega, modes] = undamped_positive_modes(K, M)
[vectors, values] = eig(K, M, 'vector');
if any(abs(imag(values)) > 1e-10*max(1, max(abs(values)))) || any(abs(imag(vectors(:))) > 1e-10*max(1, max(abs(vectors(:)))))
    error('Stage8A1:ComplexModes', 'The symmetric undamped transverse reference problem returned non-negligible complex modes.');
end
values = real(values); vectors = real(vectors);
finite = isfinite(values) & all(isfinite(vectors), 1).';
scale = max(1, max(abs(values(finite))));
positive = finite & values > 100*eps(scale);
values = values(positive); vectors = vectors(:,positive);
[values, order] = sort(values, 'ascend'); vectors = vectors(:,order);
omega = sqrt(values);
modes = zeros(size(vectors));
for k = 1:numel(omega)
    normalization = sqrt(vectors(:,k)'*M*vectors(:,k));
    if ~isfinite(normalization) || normalization <= 0
        error('Stage8A1:ModeNormalization', 'A transverse mode cannot be M-normalized.');
    end
    modes(:,k) = vectors(:,k)/normalization;
end
end

function T_oil_C = frozen_temperature_C(params)
if ~isfield(params, 'microphysics') || ~isfield(params.microphysics, 'thermal') || ...
        ~params.microphysics.thermal.enabled || ~strcmp(params.microphysics.thermal.mode, 'frozen_external') || ...
        ~isfield(params.microphysics, 'thermal_state')
    error('Stage8A1:FrozenThermalState', 'Stage8A-1 requires enabled frozen_external thermal state inputs.');
end
state = params.microphysics.thermal_state;
if ~isfield(state, 'ball') || ~isfield(state, 'roller') || ~isfield(state.ball, 'T_oil') || ~isfield(state.roller, 'T_oil') || ...
        ~isscalar(state.ball.T_oil) || ~isscalar(state.roller.T_oil) || ~isfinite(state.ball.T_oil) || ~isfinite(state.roller.T_oil) || ...
        abs(state.ball.T_oil-state.roller.T_oil) > 100*eps(max(1, abs(state.ball.T_oil)))
    error('Stage8A1:FrozenOilTemperature', 'The ball and roller frozen thermal states require one common finite oil temperature.');
end
T_oil_C = state.ball.T_oil;
end
