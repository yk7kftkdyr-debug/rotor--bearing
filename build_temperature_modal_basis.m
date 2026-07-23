function modal = build_temperature_modal_basis(model_cache, stiffness_results, damping_results, shared_reference20, cfg)
%BUILD_TEMPERATURE_MODAL_BASIS Project a temperature workpoint into the 20 C 30-mode subspace.

modal = empty_modal();
validate_inputs(model_cache, stiffness_results, damping_results, shared_reference20, cfg);
[M_t, K_t, C_ehl_t, C_foundation_t, G_t, C_ehl_ball_t, C_ehl_roller_t] = current_transverse_matrices(model_cache, stiffness_results, damping_results);
Phi30 = shared_reference20.Phi_reference30;
[Phi_current30, frequency30_Hz, lambda30, ~, ~, ritz_error, ritz_mass_error] = ritz_modes(Phi30, M_t, K_t);
[ratio_x, ratio_y] = effective_mass_ratios(Phi_current30, M_t, model_cache.transverse_dofs);
full_error = fullspace_backward_error(Phi_current30, lambda30, M_t, K_t);
frequency_tolerance = min(cfg.modal.frequency_tolerance_cap, max(cfg.modal.frequency_tolerance_floor, ...
    shared_reference20.reference20.modal_checks.maximum_response_scaled_residual_certified));
[candidate, retained] = select_candidate(Phi_current30, M_t, K_t, C_ehl_t, C_foundation_t, ...
    shared_reference20.C_rayleigh_reference_t, G_t, ratio_x, ratio_y, full_error, ritz_error, ritz_mass_error, cfg);
if isempty(retained), return; end

Phi_t = Phi_current30(:,1:retained);
modal.temperature_C = shared_reference20.current_temperature_C;
modal.strategy_used = cfg.modal.strategy;
modal.full_modal_fallback_used = false;
modal.full_modal_solve_count = 0;
modal.reference_subspace_count = cfg.modal.certified_reference_count;
modal.retained_mode_count = retained;
modal.retained_mode_indices = (1:retained).';
modal.Phi_t = Phi_t;
modal.mode_shapes_full = Phi_current30;
modal.frequency_Hz = frequency30_Hz(1:retained);
modal.frequency_full_Hz = frequency30_Hz;
modal.omega_rad_s = 2*pi*modal.frequency_Hz;
modal.M_r = Phi_t'*M_t*Phi_t;
modal.K_r = Phi_t'*K_t*Phi_t;
modal.C_rayleigh_r = Phi_t'*shared_reference20.C_rayleigh_reference_t*Phi_t;
modal.C_foundation_r = Phi_t'*C_foundation_t*Phi_t;
modal.C_ehl_r = Phi_t'*C_ehl_t*Phi_t;
modal.C_ehl_ball_r = Phi_t'*C_ehl_ball_t*Phi_t;
modal.C_ehl_roller_r = Phi_t'*C_ehl_roller_t*Phi_t;
if norm(modal.C_ehl_r-modal.C_ehl_ball_r-modal.C_ehl_roller_r, 'fro')/max(1, norm(modal.C_ehl_r, 'fro')) > 1e-12
    error('SEPARATE_EHL_COMPONENTS_NOT_AVAILABLE_AT_MAPPING', ...
        'The ball and roller EHL reduced matrices do not reconstruct the existing total EHL matrix.');
end
modal.C_dissipative_r = modal.C_rayleigh_r + modal.C_foundation_r + modal.C_ehl_r;
modal.G_r = Phi_t'*G_t*Phi_t;
modal.final_ratio_x = ratio_x(retained); modal.final_ratio_y = ratio_y(retained);
modal.effective_mass_x = ratio_x; modal.effective_mass_y = ratio_y;
modal.mass_orthogonality_error = candidate.mass_error;
modal.stiffness_diagonalization_error = candidate.stiffness_error;
modal.frequency_consistency_error = candidate.frequency_consistency_error;
modal.frequency_tolerance = frequency_tolerance;
modal.maximum_ritz_backward_error = ritz_error;
modal.ritz_mass_orthogonality_error = ritz_mass_error;
modal.maximum_fullspace_backward_error = full_error;
modal.matrix_pass = reduced_matrix_pass(modal);
modal.pass = candidate.pass && modal.matrix_pass;
end

function modal = empty_modal()
modal = struct('temperature_C',NaN,'strategy_used','','full_modal_fallback_used',false,'full_modal_solve_count',0, ...
    'reference_subspace_count',0,'retained_mode_count',0,'retained_mode_indices',zeros(0,1),'Phi_t',zeros(0,0), ...
    'mode_shapes_full',zeros(0,0),'frequency_Hz',zeros(0,1),'frequency_full_Hz',zeros(0,1),'omega_rad_s',zeros(0,1), ...
    'M_r',zeros(0,0),'K_r',zeros(0,0),'C_rayleigh_r',zeros(0,0),'C_foundation_r',zeros(0,0),'C_ehl_r',zeros(0,0), ...
    'C_ehl_ball_r',zeros(0,0),'C_ehl_roller_r',zeros(0,0), ...
    'C_dissipative_r',zeros(0,0),'G_r',zeros(0,0),'effective_mass_x',zeros(0,1),'effective_mass_y',zeros(0,1), ...
    'final_ratio_x',NaN,'final_ratio_y',NaN,'mass_orthogonality_error',NaN,'stiffness_diagonalization_error',NaN, ...
    'frequency_consistency_error',NaN,'frequency_tolerance',NaN,'maximum_ritz_backward_error',NaN, ...
    'ritz_mass_orthogonality_error',NaN,'maximum_fullspace_backward_error',NaN,'matrix_pass',false,'pass',false);
end

function validate_inputs(cache, stiffness, damping, shared, cfg)
required_cache = {'M','K_structure','C_foundation','G_unit','modelInfo','transverse_dofs','B_ball','B_roller'};
if ~isstruct(cache) || ~all(isfield(cache, required_cache)) || ~isequal(size(cache.M),[192 192]) || ...
        ~isequal(size(cache.K_structure),[192 192]) || ~isequal(size(cache.C_foundation),[192 192]) || ...
        ~isequal(size(cache.G_unit),[192 192]) || numel(cache.transverse_dofs) ~= 128 || ...
        ~isequal(size(cache.B_ball),[5 192]) || ~isequal(size(cache.B_roller),[5 192])
    error('Stage9B:ModalCache', 'model_cache must provide the fixed 192-DOF matrices and 5-by-192 bearing maps.');
end
if numel(stiffness) ~= 2 || numel(damping) ~= 2 || any(~[stiffness.pass]) || any(~[damping.pass])
    error('Stage9B:ModalWorkpoint', 'Two passing stiffness and damping workpoint results are required.');
end
if ~isstruct(shared) || ~isfield(shared,'reference20') || ~shared.reference20.pass || ...
        ~isfield(shared,'Phi_reference30') || ~isequal(size(shared.Phi_reference30),[128 30]) || ...
        ~isfield(shared,'C_rayleigh_reference_t') || ~isequal(size(shared.C_rayleigh_reference_t),[128 128]) || ...
        ~isfield(shared,'current_temperature_C') || ~isscalar(shared.current_temperature_C)
    error('Stage9B:ModalReference', 'A passing 20 C reference with a fixed 30-mode subspace is required.');
end
if cfg.modal.minimum_mode_count ~= 12 || cfg.modal.preferred_mode_count ~= 22 || cfg.modal.maximum_mode_count ~= 30 || ...
        cfg.modal.certified_reference_count ~= 30 || cfg.modal.effective_mass_target ~= .95
    error('Stage9B:ModalConfiguration', 'The Stage9A modal configuration must remain fixed.');
end
end

function [M_t, K_t, C_ehl_t, C_foundation_t, G_t, C_ehl_ball_t, C_ehl_roller_t] = current_transverse_matrices(cache, stiffness, damping)
K_bearing = cache.B_ball'*stiffness(1).K_local*cache.B_ball + cache.B_roller'*stiffness(2).K_local*cache.B_roller;
C_ehl = cache.B_ball'*damping(1).C_local*cache.B_ball + cache.B_roller'*damping(2).C_local*cache.B_roller;
C_ehl_ball = cache.B_ball'*damping(1).C_local*cache.B_ball;
C_ehl_roller = cache.B_roller'*damping(2).C_local*cache.B_roller;
dofs = cache.transverse_dofs;
M_t = cache.M(dofs,dofs); K_t = (cache.K_structure + K_bearing); K_t = K_t(dofs,dofs);
C_ehl_t = C_ehl(dofs,dofs); C_foundation_t = cache.C_foundation(dofs,dofs); G_t = cache.G_unit(dofs,dofs);
C_ehl_ball_t = C_ehl_ball(dofs,dofs); C_ehl_roller_t = C_ehl_roller(dofs,dofs);
end

function [Phi, frequency_Hz, lambda, M30, K30, backward_error, mass_error] = ritz_modes(Phi_reference, M, K)
M30 = Phi_reference'*M*Phi_reference; K30 = Phi_reference'*K*Phi_reference;
M30 = 0.5*(M30+M30.'); K30 = 0.5*(K30+K30.');
[L,p] = chol(M30,'lower'); if p ~= 0, error('Stage9B:RitzMass', 'The 30-mode projected mass matrix is not positive definite.'); end
A = L\K30/L.'; A = 0.5*(A+A.'); [V,lambda] = eig(A,'vector'); [lambda,order] = sort(real(lambda),'ascend'); V = real(V(:,order));
if any(~isfinite(lambda)) || any(lambda <= 0), error('Stage9B:RitzEigenvalues', 'Projected Ritz eigenvalues must be finite and positive.'); end
V_r = L.'\V; Phi = Phi_reference*V_r; frequency_Hz = sqrt(lambda)/(2*pi);
small_residual = K30*V_r-M30*V_r.*lambda.';
denominator = (norm(K30,2)+abs(lambda).*norm(M30,2)).*vecnorm(V_r,2,1).';
backward_error = max(vecnorm(small_residual,2,1).'./max(denominator,realmin));
mass_error = norm(V_r.'*M30*V_r-eye(30),'fro')/norm(eye(30),'fro');
end

function [ratio_x, ratio_y] = effective_mass_ratios(Phi, M, dofs)
local = mod(dofs(:)-1,6)+1; rx = double(local==1); ry = double(local==2);
mx = rx.'*M*rx; my = ry.'*M*ry;
ratio_x = cumsum((Phi.'*M*rx).^2)/mx; ratio_y = cumsum((Phi.'*M*ry).^2)/my;
end

function value = fullspace_backward_error(Phi, lambda, M, K)
residual = K*Phi-M*Phi.*lambda.';
denominator = (norm(K,2)+abs(lambda).*norm(M,2)).*vecnorm(Phi,2,1).';
value = max(vecnorm(residual,2,1).'./max(denominator,realmin));
end

function [candidate, retained] = select_candidate(Phi, M, K, Cehl, Cfoundation, Crayleigh, G, ratio_x, ratio_y, full_error, ritz_error, ritz_mass_error, cfg)
candidate = struct('mass_error',NaN,'stiffness_error',NaN,'frequency_consistency_error',NaN,'pass',false); retained = [];
for n = cfg.modal.preferred_mode_count:cfg.modal.maximum_mode_count
    P = Phi(:,1:n); Mr = P.'*M*P; Kr = P.'*K*P;
    candidate_n.mass_error = norm(Mr-eye(n),'fro')/norm(eye(n),'fro');
    candidate_n.stiffness_error = norm(Kr-diag(diag(Kr)),'fro')/max(norm(Kr,'fro'),1);
    candidate_n.frequency_consistency_error = full_error;
    dissipative = P.'*(Crayleigh+Cfoundation+Cehl)*P;
    pass = ratio_x(n) >= cfg.modal.effective_mass_target && ratio_y(n) >= cfg.modal.effective_mass_target && ...
        candidate_n.mass_error <= 1e-10 && candidate_n.stiffness_error <= 1e-8 && full_error <= 1e-9 && ...
        ritz_error <= cfg.modal.ritz_backward_tolerance && ritz_mass_error <= 1e-10 && ...
        positive_definite(Mr) && positive_definite(Kr) && positive_semidefinite(dissipative) && ...
        norm((P.'*G*P)+(P.'*G*P).','fro')/max(norm(P.'*G*P,'fro'),1) <= 1e-10;
    candidate_n.pass = pass;
    if pass, candidate = candidate_n; retained = n; return; end
end
end

function pass = reduced_matrix_pass(modal)
pass = all(isfinite([modal.M_r(:); modal.K_r(:); modal.C_rayleigh_r(:); modal.C_foundation_r(:); modal.C_ehl_r(:); modal.G_r(:)])) && ...
    positive_definite(modal.M_r) && positive_definite(modal.K_r) && positive_semidefinite(modal.C_dissipative_r) && ...
    symmetry_error(modal.M_r) <= 1e-10 && symmetry_error(modal.K_r) <= 1e-10 && ...
    symmetry_error(modal.C_rayleigh_r) <= 1e-10 && symmetry_error(modal.C_foundation_r) <= 1e-10 && ...
    symmetry_error(modal.C_ehl_r) <= 1e-10 && norm(modal.G_r+modal.G_r.','fro')/max(norm(modal.G_r,'fro'),1) <= 1e-10;
end

function pass = positive_definite(A)
[~,p] = chol(0.5*(A+A.'),'lower'); pass = p == 0;
end

function pass = positive_semidefinite(A)
e = eig(0.5*(A+A.')); pass = min(e) >= -1e-10*max(1,max(e));
end

function value = symmetry_error(A)
value = norm(A-A.','fro')/max(norm(A,'fro'),1);
end
