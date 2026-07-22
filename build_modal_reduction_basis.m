function basis = build_modal_reduction_basis(reference, cfg)
%BUILD_MODAL_REDUCTION_BASIS Select the minimum certified modal prefix.

if nargin < 2 || isempty(cfg), cfg = struct(); end
cfg = fixed_config(cfg);
basis = empty_basis();
validate_reference(reference, cfg);

n_certified = reference.modal_checks.certified_mode_count;
Phi30 = reference.mode_shapes_full(:, 1:n_certified);
frequency_certified_Hz = reference.frequency_Hz(1:n_certified);
[r_x, r_y] = participation_vectors(reference.transverse_dofs, reference.transverse_dof_count);
[Gamma_x, Gamma_y, meff_x, meff_y, mtotal_x, mtotal_y, ratio_x, ratio_y] = ...
    effective_mass_metrics(Phi30, reference.M_t, r_x, r_y);
[M30, K30, CR30, CF30, CE30, G30] = project_once(Phi30, reference);
[err_M30, err_K30] = upstream_errors(M30, K30);
validate_current_upstream(err_M30, err_K30, cfg);
frequency_tolerance = min(1e-4, max(1e-6, reference.modal_checks.maximum_response_scaled_residual_certified));

basis.temperature_C = reference.temperature_C;
basis.transverse_dofs = reference.transverse_dofs;
basis.certified_mode_count = n_certified;
basis.participation_vector_x = r_x;
basis.participation_vector_y = r_y;
basis.participation_factor_x = Gamma_x;
basis.participation_factor_y = Gamma_y;
basis.effective_mass_x = meff_x;
basis.effective_mass_y = meff_y;
basis.total_mass_x = mtotal_x;
basis.total_mass_y = mtotal_y;
basis.cumulative_ratio_x = ratio_x;
basis.cumulative_ratio_y = ratio_y;

[n_retained, candidate] = select_prefix(M30, K30, frequency_certified_Hz, ratio_x, ratio_y, frequency_tolerance, cfg);
if isempty(n_retained), return; end
Phi_t = Phi30(:, 1:n_retained);
M_r = M30(1:n_retained, 1:n_retained); K_r = K30(1:n_retained, 1:n_retained);
C_rayleigh_r = CR30(1:n_retained, 1:n_retained); C_foundation_r = CF30(1:n_retained, 1:n_retained);
C_ehl_r = CE30(1:n_retained, 1:n_retained); C_dissipative_r = C_rayleigh_r + C_foundation_r + C_ehl_r;
G_r = G30(1:n_retained, 1:n_retained);
checks = reduced_matrix_checks(M_r, K_r, C_rayleigh_r, C_foundation_r, C_ehl_r, C_dissipative_r, G_r, cfg);

basis.retained_mode_indices = (1:n_retained).';
basis.retained_mode_count = n_retained;
basis.Phi_t = Phi_t;
basis.frequency_Hz = candidate.frequency_Hz;
basis.omega_rad_s = 2*pi*candidate.frequency_Hz;
basis.final_ratio_x = ratio_x(n_retained);
basis.final_ratio_y = ratio_y(n_retained);
basis.M_r = M_r;
basis.K_r = K_r;
basis.C_rayleigh_r = C_rayleigh_r;
basis.C_foundation_r = C_foundation_r;
basis.C_ehl_r = C_ehl_r;
basis.C_dissipative_r = C_dissipative_r;
basis.G_r = G_r;
basis.mass_orthogonality_error = candidate.mass_error;
basis.stiffness_diagonalization_error = candidate.stiffness_error;
basis.frequency_consistency_error = candidate.frequency_consistency_error;
basis.frequency_tolerance = frequency_tolerance;
basis.maximum_ritz_backward_error = candidate.maximum_ritz_backward_error;
basis.ritz_mass_orthogonality_error = candidate.ritz_mass_orthogonality_error;
basis.matrix_checks = checks;
basis.pass = candidate.pass && checks.pass;
fprintf('retained_mode_count=%d\nfinal_ratio_x=%.16g\nfinal_ratio_y=%.16g\nmass_error=%.16g\nstiffness_error=%.16g\nfrequency_set_error=%.16g\nfrequency_tolerance=%.16g\nmaximum_ritz_backward_error=%.16g\nRitz_M_orthogonality_error=%.16g\n', ...
    n_retained, basis.final_ratio_x, basis.final_ratio_y, basis.mass_orthogonality_error, basis.stiffness_diagonalization_error, ...
    basis.frequency_consistency_error, basis.frequency_tolerance, basis.maximum_ritz_backward_error, basis.ritz_mass_orthogonality_error);
end

function cfg = fixed_config(cfg)
defaults = struct('minimum_mode_count', 12, 'preferred_maximum_mode_count', 20, ...
    'absolute_maximum_mode_count', 30, 'effective_mass_target', 0.95, ...
    'mass_orthogonality_tolerance', 1e-10, 'stiffness_diagonalization_tolerance', 1e-8, ...
    'matrix_tolerance', 1e-10);
names = fieldnames(defaults);
for k = 1:numel(names)
    name = names{k};
    if ~isfield(cfg, name) || isempty(cfg.(name)), cfg.(name) = defaults.(name); end
    if ~isequal(cfg.(name), defaults.(name))
        error('Stage8A2:FixedConfiguration', 'Stage8A-2 requires cfg.%s = %.16g.', name, defaults.(name));
    end
end
end

function basis = empty_basis()
basis = struct('temperature_C', NaN, 'transverse_dofs', zeros(0,1), 'certified_mode_count', 0, ...
    'retained_mode_indices', zeros(0,1), 'retained_mode_count', 0, 'Phi_t', zeros(0,0), ...
    'frequency_Hz', zeros(0,1), 'omega_rad_s', zeros(0,1), ...
    'participation_vector_x', zeros(0,1), 'participation_vector_y', zeros(0,1), ...
    'participation_factor_x', zeros(0,1), 'participation_factor_y', zeros(0,1), ...
    'effective_mass_x', zeros(0,1), 'effective_mass_y', zeros(0,1), ...
    'total_mass_x', NaN, 'total_mass_y', NaN, 'cumulative_ratio_x', zeros(0,1), ...
    'cumulative_ratio_y', zeros(0,1), 'final_ratio_x', NaN, 'final_ratio_y', NaN, ...
    'M_r', zeros(0,0), 'K_r', zeros(0,0), 'C_rayleigh_r', zeros(0,0), ...
    'C_foundation_r', zeros(0,0), 'C_ehl_r', zeros(0,0), 'C_dissipative_r', zeros(0,0), ...
    'G_r', zeros(0,0), 'mass_orthogonality_error', NaN, ...
    'stiffness_diagonalization_error', NaN, 'frequency_consistency_error', NaN, 'frequency_tolerance', NaN, ...
    'maximum_ritz_backward_error', NaN, 'ritz_mass_orthogonality_error', NaN, ...
    'matrix_checks', empty_matrix_checks(), 'pass', false);
end

function checks = empty_matrix_checks()
checks = struct('M_r_symmetry_error', NaN, 'K_r_symmetry_error', NaN, ...
    'C_rayleigh_r_symmetry_error', NaN, 'C_foundation_r_symmetry_error', NaN, ...
    'C_ehl_r_symmetry_error', NaN, 'C_dissipative_r_symmetry_error', NaN, ...
    'G_r_skew_symmetry_error', NaN, 'M_r_minimum_eigenvalue', NaN, ...
    'M_r_maximum_eigenvalue', NaN, 'K_r_minimum_eigenvalue', NaN, ...
    'K_r_maximum_eigenvalue', NaN, 'C_rayleigh_r_minimum_eigenvalue', NaN, ...
    'C_rayleigh_r_maximum_eigenvalue', NaN, 'C_foundation_r_minimum_eigenvalue', NaN, ...
    'C_foundation_r_maximum_eigenvalue', NaN, 'C_ehl_r_minimum_eigenvalue', NaN, ...
    'C_ehl_r_maximum_eigenvalue', NaN, 'C_dissipative_r_minimum_eigenvalue', NaN, ...
    'C_dissipative_r_maximum_eigenvalue', NaN, 'pass', false);
end

function validate_reference(reference, cfg)
required = {'pass','temperature_C','transverse_dofs','transverse_dof_count','M_t','K_workpoint_t', ...
    'C_rayleigh_reference_t','C_foundation_t','C_ehl_t','G_t','mode_shapes_full','frequency_Hz', ...
    'omega_rad_s','modal_checks'};
if ~isstruct(reference) || ~all(isfield(reference, required))
    error('Stage8A2:ReferenceFields', 'A complete Stage8A-1R2M reference is required.');
end
modal = reference.modal_checks;
modal_fields = {'certified_mode_count','maximum_backward_error_certified', ...
    'maximum_positive_subspace_error_certified','maximum_zero_subspace_error_certified', ...
    'maximum_response_scaled_residual_certified'};
if ~all(isfield(modal, modal_fields)) || ~reference.pass || reference.transverse_dof_count ~= 128 || ...
        modal.certified_mode_count ~= cfg.absolute_maximum_mode_count || ...
        size(reference.mode_shapes_full,1) ~= 128 || size(reference.mode_shapes_full,2) < cfg.absolute_maximum_mode_count
    error('Stage8A2:ReferenceGate', 'Stage8A-2 requires a passing 128-DOF reference with 30 certified modes.');
end
if modal.maximum_backward_error_certified > 1e-9 || modal.maximum_positive_subspace_error_certified > 1e-9 || ...
        modal.maximum_zero_subspace_error_certified > 1e-10
    error('Stage8A2:ReferenceCertification', 'The current reference did not pass finite-mode certification.');
end
matrices = {reference.M_t, reference.K_workpoint_t, reference.C_rayleigh_reference_t, reference.C_foundation_t, reference.C_ehl_t, reference.G_t};
if any(cellfun(@(A) ~isequal(size(A), [128 128]) || any(~isfinite(A), 'all'), matrices)) || ...
        any(~isfinite(reference.mode_shapes_full(:,1:30)), 'all') || any(~isfinite(reference.frequency_Hz(1:30))) || ...
        any(~isfinite(reference.omega_rad_s(1:30)))
    error('Stage8A2:ReferenceDimensions', 'Reference matrices and certified modes must be finite and 128-DOF consistent.');
end
end

function [r_x, r_y] = participation_vectors(transverse_dofs, n)
local_dof = mod(transverse_dofs(:)-1, 6) + 1;
r_x = double(local_dof == 1); r_y = double(local_dof == 2);
if numel(r_x) ~= n || nnz(r_x) == 0 || nnz(r_y) == 0 || any((r_x+r_y) > 1)
    error('Stage8A2:ParticipationVectors', 'The transverse map must provide disjoint ux and uy participation vectors.');
end
end

function [Gamma_x, Gamma_y, meff_x, meff_y, mtotal_x, mtotal_y, ratio_x, ratio_y] = effective_mass_metrics(Phi, M, r_x, r_y)
Gamma_x = Phi.'*M*r_x; Gamma_y = Phi.'*M*r_y;
meff_x = Gamma_x.^2; meff_y = Gamma_y.^2;
mtotal_x = r_x.'*M*r_x; mtotal_y = r_y.'*M*r_y;
if ~(isfinite(mtotal_x) && isfinite(mtotal_y) && mtotal_x > 0 && mtotal_y > 0)
    error('Stage8A2:TotalMass', 'Participation total masses must be finite and positive.');
end
ratio_x = cumsum(meff_x)/mtotal_x; ratio_y = cumsum(meff_y)/mtotal_y;
if any(~isfinite([Gamma_x; Gamma_y; meff_x; meff_y; ratio_x; ratio_y])) || ...
        any(diff(ratio_x) < -1e-13) || any(diff(ratio_y) < -1e-13)
    error('Stage8A2:EffectiveMass', 'Effective-mass ratios must be finite and nondecreasing within 1e-13.');
end
end

function [err_M, err_K] = upstream_errors(M30, K30)
err_M = norm(M30-eye(30), 'fro')/norm(eye(30), 'fro');
err_K = norm(K30-diag(diag(K30)), 'fro')/max(norm(K30, 'fro'), 1);
end

function validate_current_upstream(err_M30, err_K30, cfg)
if err_M30 > cfg.mass_orthogonality_tolerance || err_K30 > cfg.stiffness_diagonalization_tolerance
    error('Stage8A2:CurrentReferenceInconsistency', 'The current reference fails its required 30-mode M/K certification.');
end
end

function [n_retained, candidate] = select_prefix(M30, K30, frequency_certified_Hz, ratio_x, ratio_y, frequency_tolerance, cfg)
n_retained = []; candidate = empty_candidate();
for n = cfg.minimum_mode_count:cfg.absolute_maximum_mode_count
    candidate_n = candidate_metrics(n, M30(1:n,1:n), K30(1:n,1:n), ratio_x(n), ratio_y(n), ...
        frequency_certified_Hz(1:n), frequency_tolerance, cfg);
    if candidate_n.pass, n_retained = n; candidate = candidate_n; return; end
end
end

function candidate = empty_candidate()
candidate = struct('ratio_x', NaN, 'ratio_y', NaN, 'mass_error', NaN, 'stiffness_error', NaN, ...
    'frequency_consistency_error', NaN, 'frequency_Hz', zeros(0,1), ...
    'maximum_ritz_backward_error', NaN, 'ritz_mass_orthogonality_error', NaN, 'pass', false);
end

function candidate = candidate_metrics(n, M_n, K_n, ratio_x, ratio_y, reference_frequency_Hz, frequency_tolerance, cfg)
candidate = empty_candidate();
candidate.ratio_x = ratio_x; candidate.ratio_y = ratio_y;
candidate.mass_error = norm(M_n-eye(n), 'fro')/norm(eye(n), 'fro');
candidate.stiffness_error = norm(K_n-diag(diag(K_n)), 'fro')/max(norm(K_n, 'fro'), 1);
cheap_pass = ratio_x >= cfg.effective_mass_target && ratio_y >= cfg.effective_mass_target && ...
    candidate.mass_error <= cfg.mass_orthogonality_tolerance && ...
    candidate.stiffness_error <= cfg.stiffness_diagonalization_tolerance && finite_symmetric_pd(M_n, cfg) && finite_symmetric_pd(K_n, cfg);
if ~cheap_pass, return; end
[candidate.frequency_Hz, candidate.frequency_consistency_error, candidate.maximum_ritz_backward_error, candidate.ritz_mass_orthogonality_error] = ...
    ritz_metrics(M_n, K_n, reference_frequency_Hz);
candidate.pass = candidate.frequency_consistency_error <= frequency_tolerance && ...
    candidate.maximum_ritz_backward_error <= 1e-10 && candidate.ritz_mass_orthogonality_error <= 1e-10;
end

function [M30, K30, CR30, CF30, CE30, G30] = project_once(Phi30, reference)
M30 = Phi30.'*reference.M_t*Phi30;
K30 = Phi30.'*reference.K_workpoint_t*Phi30;
CR30 = Phi30.'*reference.C_rayleigh_reference_t*Phi30;
CF30 = Phi30.'*reference.C_foundation_t*Phi30;
CE30 = Phi30.'*reference.C_ehl_t*Phi30;
G30 = Phi30.'*reference.G_t*Phi30;
end

function checks = reduced_matrix_checks(M_r, K_r, C_rayleigh_r, C_foundation_r, C_ehl_r, C_dissipative_r, G_r, cfg)
checks = empty_matrix_checks();
matrices = {M_r, K_r, C_rayleigh_r, C_foundation_r, C_ehl_r, C_dissipative_r, G_r};
if any(cellfun(@(A) size(A,1) ~= size(A,2) || any(~isfinite(A), 'all'), matrices)), return; end
checks.M_r_symmetry_error = symmetry_error(M_r); checks.K_r_symmetry_error = symmetry_error(K_r);
checks.C_rayleigh_r_symmetry_error = symmetry_error(C_rayleigh_r); checks.C_foundation_r_symmetry_error = symmetry_error(C_foundation_r);
checks.C_ehl_r_symmetry_error = symmetry_error(C_ehl_r); checks.C_dissipative_r_symmetry_error = symmetry_error(C_dissipative_r);
checks.G_r_skew_symmetry_error = norm(G_r+G_r.', 'fro')/max(norm(G_r, 'fro'), 1);
[checks.M_r_minimum_eigenvalue, checks.M_r_maximum_eigenvalue] = spectrum_limits(M_r);
[checks.K_r_minimum_eigenvalue, checks.K_r_maximum_eigenvalue] = spectrum_limits(K_r);
[checks.C_rayleigh_r_minimum_eigenvalue, checks.C_rayleigh_r_maximum_eigenvalue] = spectrum_limits(C_rayleigh_r);
[checks.C_foundation_r_minimum_eigenvalue, checks.C_foundation_r_maximum_eigenvalue] = spectrum_limits(C_foundation_r);
[checks.C_ehl_r_minimum_eigenvalue, checks.C_ehl_r_maximum_eigenvalue] = spectrum_limits(C_ehl_r);
[checks.C_dissipative_r_minimum_eigenvalue, checks.C_dissipative_r_maximum_eigenvalue] = spectrum_limits(C_dissipative_r);
symmetry_pass = checks.M_r_symmetry_error <= cfg.matrix_tolerance && checks.K_r_symmetry_error <= cfg.matrix_tolerance && ...
    checks.C_rayleigh_r_symmetry_error <= cfg.matrix_tolerance && checks.C_foundation_r_symmetry_error <= cfg.matrix_tolerance && ...
    checks.C_ehl_r_symmetry_error <= cfg.matrix_tolerance && checks.C_dissipative_r_symmetry_error <= cfg.matrix_tolerance && ...
    checks.G_r_skew_symmetry_error <= cfg.matrix_tolerance;
positive_definite = checks.M_r_minimum_eigenvalue > 0 && checks.K_r_minimum_eigenvalue > 0;
psd = is_psd(checks.C_rayleigh_r_minimum_eigenvalue, checks.C_rayleigh_r_maximum_eigenvalue) && ...
    is_psd(checks.C_foundation_r_minimum_eigenvalue, checks.C_foundation_r_maximum_eigenvalue) && ...
    is_psd(checks.C_ehl_r_minimum_eigenvalue, checks.C_ehl_r_maximum_eigenvalue) && ...
    is_psd(checks.C_dissipative_r_minimum_eigenvalue, checks.C_dissipative_r_maximum_eigenvalue);
checks.pass = symmetry_pass && positive_definite && psd;
end

function value = symmetry_error(A)
value = norm(A-A.', 'fro')/max(norm(A, 'fro'), 1);
end

function [minimum, maximum] = spectrum_limits(A)
values = eig(0.5*(A+A.'));
minimum = min(real(values)); maximum = max(real(values));
end

function pass = is_psd(minimum, maximum)
pass = minimum >= -1e-10*max(1, maximum);
end

function pass = finite_symmetric_pd(A, cfg)
if any(~isfinite(A), 'all') || symmetry_error(A) > cfg.matrix_tolerance, pass = false; return; end
[~, p] = chol(0.5*(A+A.'), 'lower'); pass = p == 0;
end

function [frequency_Hz, frequency_set_error, maximum_backward_error, mass_orthogonality_error] = ritz_metrics(M_n, K_n, reference_frequency_Hz)
M_s = 0.5*(M_n+M_n.'); K_s = 0.5*(K_n+K_n.');
[L, p] = chol(M_s, 'lower');
if p ~= 0, error('Stage8A2:RitzMass', 'The candidate reduced mass matrix is not positive definite.'); end
A = L\K_s/L.'; A = 0.5*(A+A.');
[V, lambda] = eig(A, 'vector'); [lambda, order] = sort(real(lambda), 'ascend'); V = real(V(:,order));
if any(~isfinite(lambda)) || any(lambda <= 0), error('Stage8A2:RitzEigenvalues', 'Ritz eigenvalues must be finite and positive.'); end
V_r = L.'\V; frequency_Hz = sqrt(lambda)/(2*pi);
frequency_set_error = max(abs(frequency_Hz-reference_frequency_Hz)./max(reference_frequency_Hz, realmin));
residuals = K_n*V_r - M_n*V_r.*lambda.';
denominator = (norm(K_n,2) + abs(lambda).*norm(M_n,2)).*vecnorm(V_r,2,1).';
maximum_backward_error = max(vecnorm(residuals,2,1).'./max(denominator, realmin));
mass_orthogonality_error = norm(V_r.'*M_n*V_r-eye(size(M_n,1)), 'fro')/norm(eye(size(M_n,1)), 'fro');
end
