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

[omega_rad_s, mode_shapes_full, modal_checks, singular_mass_checks] = undamped_positive_modes(K_workpoint_t, M_t);
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
reference.singular_mass_checks = singular_mass_checks;
reference.modal_checks = modal_checks;
reference.pass = checks.pass && rayleigh_pass && reference.transverse_dof_count == 128 && ...
    reference.frequency_Hz(1) > 0 && reference.frequency_Hz(3) > reference.frequency_Hz(1) && ...
    singular_mass_checks.pass && modal_checks.pass;
end

function reference = empty_reference()
reference = struct('temperature_C', NaN, 'transverse_dofs', zeros(0,1), 'transverse_dof_count', 0, ...
    'M_t', zeros(0,0), 'K_structure_t', zeros(0,0), 'K_bearing_t', zeros(0,0), ...
    'K_workpoint_t', zeros(0,0), 'C_foundation_t', zeros(0,0), 'C_ehl_t', zeros(0,0), ...
    'G_t', zeros(0,0), 'rotation_speed_rad_s', NaN, 'mode_shapes_full', zeros(0,0), ...
    'omega_rad_s', zeros(0,1), 'frequency_Hz', zeros(0,1), 'reference_mode_indices', zeros(0,1), ...
    'target_damping_ratio', NaN, 'alpha_R', NaN, 'beta_R', NaN, ...
    'C_rayleigh_reference_t', zeros(0,0), 'recovered_damping_ratio', zeros(0,1), ...
    'symmetry_checks', empty_checks(), 'singular_mass_checks', empty_singular_mass_checks(), ...
    'modal_checks', empty_modal_checks(), 'pass', false);
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

function checks = empty_singular_mass_checks()
checks = struct('mass_rank', 0, 'mass_nullity', 0, 'mass_tolerance', NaN, ...
    'minimum_mass_eigenvalue', NaN, 'maximum_mass_eigenvalue', NaN, ...
    'K00_minimum_eigenvalue', NaN, 'condensation_residual', NaN, ...
    'algebraic_residual', NaN, 'condensed_mass_cholesky_pass', false, ...
    'positive_finite_mode_count', 0, 'pass', false);
end

function checks = empty_modal_checks()
checks = struct('mass_diagonal_error', NaN, 'mass_cross_error', NaN, 'mass_fro_error', NaN, ...
    'stiffness_diagonalization_error', NaN, 'certified_mode_count', 0, ...
    'backward_error_all_finite', zeros(0,1), 'response_scaled_residual_all_finite', zeros(0,1), ...
    'maximum_backward_error_certified', NaN, 'maximum_backward_error_all_finite', NaN, ...
    'maximum_positive_subspace_error_certified', NaN, 'maximum_zero_subspace_error_certified', NaN, ...
    'maximum_response_scaled_residual_certified', NaN, ...
    'maximum_response_scaled_residual_all_finite', NaN, ...
    'response_scaled_residual_is_diagnostic', true, 'transformed_symmetry_error', NaN, 'pass', false);
end

function [omega_positive, Phi_positive, modal_checks, singular_checks] = undamped_positive_modes(K, M)
singular_checks = empty_singular_mass_checks();
modal_checks = empty_modal_checks();
validate_modal_matrices(M, K);
M_s = 0.5*(M+M');
K_s = 0.5*(K+K');
[U_M, mu] = eig(M_s, 'vector');
if any(~isfinite(mu)) || any(~isfinite(U_M), 'all') || ...
        any(abs(imag(mu)) > 1e-10*max(1, max(abs(mu)))) || ...
        any(abs(imag(U_M(:))) > 1e-10*max(1, max(abs(U_M(:)))))
    error('Stage8A1R2:MassSpectrum', 'The symmetric mass eigensystem must be finite and real to machine precision.');
end
[mu, order] = sort(real(mu), 'ascend');
U_M = real(U_M(:,order));
mu_scale = max(abs(mu));
if ~isfinite(mu_scale) || mu_scale <= 0
    error('Stage8A1R2:MassScale', 'The transverse mass spectrum must have one finite positive scale.');
end
tol_M = max(size(M_s))*eps(mu_scale);
negative_mass = mu < -tol_M;
if any(negative_mass)
    error('Stage8A1R2:NegativeMass', 'The transverse mass matrix has a negative eigenvalue: %.16g.', min(mu));
end
zero_mass = abs(mu) <= tol_M;
positive_mass = mu > tol_M;
U0 = U_M(:,zero_mass);
Up = U_M(:,positive_mass);
mass_rank = size(Up,2);
mass_nullity = size(U0,2);
if mass_rank < 30 || mass_rank + mass_nullity ~= size(M_s,1)
    error('Stage8A1R2:MassSubspaceDimension', 'Mass-positive and zero-mass subspaces must partition the 128-DOF transverse space with rank at least 30.');
end
if norm(Up'*Up-eye(mass_rank), 'fro') > 1e-10 || norm(U0'*U0-eye(mass_nullity), 'fro') > 1e-10 || ...
        norm(Up'*U0, 'fro') > 1e-10
    error('Stage8A1R2:MassSubspaceOrthogonality', 'Mass eigenvector subspaces are not Euclidean-orthogonal within 1e-10.');
end
singular_checks.mass_rank = mass_rank;
singular_checks.mass_nullity = mass_nullity;
singular_checks.mass_tolerance = tol_M;
singular_checks.minimum_mass_eigenvalue = min(mu);
singular_checks.maximum_mass_eigenvalue = max(mu);

Kpp = Up'*K_s*Up;
Kp0 = Up'*K_s*U0;
K0p = Kp0';
K00 = U0'*K_s*U0;
K00_s = 0.5*(K00+K00');
if relative_symmetry_error(K00) > 1e-10
    error('Stage8A1R2:K00Symmetry', 'The zero-mass stiffness block K00 must be symmetric within 1e-10.');
end
[L0, p0] = chol(K00_s, 'lower');
[K00_minimum, K00_maximum] = symmetric_extrema(K00_s);
singular_checks.K00_minimum_eigenvalue = K00_minimum;
if p0 ~= 0
    error('Stage8A1R2:K00PositiveDefinite', ...
        'chol(K00_s,''lower'') failed with p=%d; min=%g, max=%g, nullity=%d.', ...
        p0, K00_minimum, K00_maximum, mass_nullity);
end
X0 = -(L0'\(L0\K0p));
singular_checks.condensation_residual = norm(K00_s*X0+K0p, 'fro')/max(norm(K0p, 'fro'), 1);
if singular_checks.condensation_residual > 1e-10
    error('Stage8A1R2:CondensationResidual', 'The zero-mass static condensation residual exceeds 1e-10.');
end
T = Up + U0*X0;
M_c = T'*M_s*T;
K_c = T'*K_s*T;
K_c_blocks = Kpp + Kp0*X0 + X0'*K0p + X0'*K00_s*X0;
if norm(K_c-K_c_blocks, 'fro')/max(norm(K_c, 'fro'), 1) > 1e-10
    error('Stage8A1R2:CondensedStiffnessConsistency', 'T''*K_s*T and the block-condensed stiffness are inconsistent.');
end
if any(~isfinite(M_c), 'all') || any(~isfinite(K_c), 'all') || ...
        relative_symmetry_error(M_c) > 1e-10 || relative_symmetry_error(K_c) > 1e-10
    error('Stage8A1R2:CondensedMatrices', 'Condensed finite-mass M_c/K_c must be finite and symmetric within 1e-10.');
end
M_c_s = 0.5*(M_c+M_c');
K_c_s = 0.5*(K_c+K_c');
[Lm, pm] = chol(M_c_s, 'lower');
singular_checks.condensed_mass_cholesky_pass = pm == 0;
if pm ~= 0
    [Mc_minimum, ~] = symmetric_extrema(M_c_s);
    error('Stage8A1R2:CondensedMassPositiveDefinite', 'chol(M_c_s,''lower'') failed with p=%d; min=%g.', pm, Mc_minimum);
end

A = Lm\K_c_s/Lm';
modal_checks.transformed_symmetry_error = relative_symmetry_error(A);
if modal_checks.transformed_symmetry_error > 1e-10
    error('Stage8A1R2:TransformedMatrixSymmetry', 'The condensed symmetric eigenproblem has A symmetry error above 1e-10.');
end
A_s = 0.5*(A+A');
[V, lambda] = eig(A_s, 'vector');
if any(~isfinite(lambda)) || any(~isfinite(V), 'all') || ...
        any(abs(imag(lambda)) > 1e-10*max(1, max(abs(lambda)))) || ...
        any(abs(imag(V(:))) > 1e-10*max(1, max(abs(V(:)))))
    error('Stage8A1R2:FiniteModes', 'The condensed symmetric eigenproblem must return finite real modes.');
end
[lambda, order] = sort(real(lambda), 'ascend');
V = real(V(:,order));
Psi = Lm'\V;
Phi_full = T*Psi;
Phi_full = canonicalize_modal_signs(Phi_full);
lambda_scale = max(1, max(abs(lambda)));
tol_lambda = max(size(A_s))*eps(lambda_scale);
unstable_negative = lambda < -tol_lambda;
if any(unstable_negative)
    error('Stage8A1R2:NegativeFiniteMode', 'The condensed finite system has an unstable eigenvalue: %.16g.', min(lambda));
end
positive_finite = lambda > tol_lambda;
lambda_positive = lambda(positive_finite);
Phi_positive = Phi_full(:,positive_finite);
if numel(lambda_positive) < 30 || numel(lambda_positive) > mass_rank
    error('Stage8A1R2:FiniteModeCount', 'The finite positive mode count must lie between 30 and mass_rank.');
end
omega_positive = sqrt(lambda_positive);
modal_checks = full_modal_checks(M, K, Phi_positive, lambda_positive, Up, U0, modal_checks);
singular_checks.positive_finite_mode_count = numel(lambda_positive);
singular_checks.algebraic_residual = norm(U0'*K*Phi_positive, 'fro')/max(norm(K*Phi_positive, 'fro'), 1);
if singular_checks.algebraic_residual > 1e-10
    error('Stage8A1R2:AlgebraicResidual', 'The zero-mass algebraic equilibrium residual exceeds 1e-10.');
end
singular_checks.pass = singular_checks.mass_rank >= 30 && ...
    singular_checks.mass_rank + singular_checks.mass_nullity == size(M,1) && ...
    singular_checks.condensation_residual <= 1e-10 && singular_checks.algebraic_residual <= 1e-10 && ...
    singular_checks.condensed_mass_cholesky_pass && singular_checks.positive_finite_mode_count >= 30 && ...
    singular_checks.positive_finite_mode_count <= singular_checks.mass_rank;
if ~singular_checks.pass || ~modal_checks.pass
    error('Stage8A1R2:ModalVerification', 'The singular-mass finite-modal verification gate failed.');
end
end

function validate_modal_matrices(M, K)
if ~isequal(size(M), [128 128]) || ~isequal(size(K), [128 128]) || any(~isfinite(M), 'all') || any(~isfinite(K), 'all')
    error('Stage8A1R2:ModalDimensions', 'The finite modal reference requires finite 128-by-128 M and K matrices.');
end
if relative_symmetry_error(M) > 1e-12 || relative_symmetry_error(K) > 1e-12
    error('Stage8A1R2:ModalMatrixSymmetry', 'Raw transverse M/K symmetry errors must not exceed 1e-12.');
end
end

function checks = full_modal_checks(M, K, Phi, lambda, Up, U0, checks)
certified_mode_count = 30;
if numel(lambda) < certified_mode_count
    error('Stage8A1R2M:CertifiedModeCount', 'At least %d finite positive modes are required for certification.', certified_mode_count);
end
certified = 1:certified_mode_count;
Phi_certified = Phi(:, certified);
lambda_certified = lambda(certified);
G_M = Phi_certified'*M*Phi_certified;
G_K = Phi_certified'*K*Phi_certified;
identity = eye(certified_mode_count);
mass_off_diagonal = G_M-diag(diag(G_M));
checks.mass_diagonal_error = max(abs(diag(G_M)-1));
checks.mass_cross_error = max(abs(mass_off_diagonal), [], 'all');
checks.mass_fro_error = norm(G_M-identity, 'fro')/max(norm(identity, 'fro'), 1);
checks.stiffness_diagonalization_error = norm(G_K-diag(lambda_certified), 'fro')/max(norm(diag(lambda_certified), 'fro'), 1);
norm_K_2 = norm(K, 2); norm_M_2 = norm(M, 2);
if ~(isfinite(norm_K_2) && isfinite(norm_M_2) && norm_K_2 > 0 && norm_M_2 > 0)
    error('Stage8A1R2M:ResidualScale', 'Original transverse K and M require finite positive 2-norms for modal residual certification.');
end
mode_count = numel(lambda);
response_scaled = zeros(mode_count,1); backward_error = zeros(mode_count,1);
positive_subspace_error = zeros(certified_mode_count,1); zero_subspace_error = zeros(certified_mode_count,1);
for k = 1:mode_count
    phi = Phi(:,k);
    residual = K*phi-lambda(k)*M*phi;
    phi_norm = norm(phi, 2);
    response_denominator = max(norm(K*phi, 2) + abs(lambda(k))*norm(M*phi, 2), realmin);
    backward_denominator = max((norm_K_2 + abs(lambda(k))*norm_M_2)*phi_norm, realmin);
    response_scaled(k) = norm(residual, 2)/response_denominator;
    backward_error(k) = norm(residual, 2)/backward_denominator;
    if k <= certified_mode_count
        positive_subspace_error(k) = norm(Up.'*residual, 2)/backward_denominator;
        zero_subspace_error(k) = norm(U0.'*residual, 2)/max(norm_K_2*phi_norm, realmin);
    end
end
if any(~isfinite(response_scaled)) || any(~isfinite(backward_error)) || ...
        any(~isfinite(positive_subspace_error)) || any(~isfinite(zero_subspace_error))
    error('Stage8A1R2M:NonFiniteResidual', 'Finite-mode residual certification produced a non-finite value.');
end
checks.certified_mode_count = certified_mode_count;
checks.backward_error_all_finite = backward_error;
checks.response_scaled_residual_all_finite = response_scaled;
checks.maximum_backward_error_certified = max(backward_error(certified));
checks.maximum_backward_error_all_finite = max(backward_error);
checks.maximum_positive_subspace_error_certified = max(positive_subspace_error);
checks.maximum_zero_subspace_error_certified = max(zero_subspace_error);
checks.maximum_response_scaled_residual_certified = max(response_scaled(certified));
checks.maximum_response_scaled_residual_all_finite = max(response_scaled);
if checks.maximum_backward_error_certified > 1e-9
    [~, worst_index] = max(backward_error(certified));
    error('Stage8A1R2M:CertifiedBackwardError', ...
        ['Certified mode %d at %.16g Hz has backward=%0.16g, positive=%0.16g, ', ...
        'zero=%0.16g, response=%0.16g.'], worst_index, sqrt(lambda(worst_index))/(2*pi), ...
        backward_error(worst_index), positive_subspace_error(worst_index), ...
        zero_subspace_error(worst_index), response_scaled(worst_index));
end
checks.pass = checks.mass_diagonal_error <= 1e-10 && checks.mass_cross_error <= 1e-10 && ...
    checks.mass_fro_error <= 1e-10 && checks.stiffness_diagonalization_error <= 1e-8 && ...
    checks.certified_mode_count == certified_mode_count && ...
    checks.maximum_backward_error_certified <= 1e-9 && ...
    checks.maximum_positive_subspace_error_certified <= 1e-9 && ...
    checks.maximum_zero_subspace_error_certified <= 1e-10;
end

function Phi = canonicalize_modal_signs(Phi)
for k = 1:size(Phi,2)
    [~, index] = max(abs(Phi(:,k)));
    if Phi(index,k) < 0, Phi(:,k) = -Phi(:,k); end
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
