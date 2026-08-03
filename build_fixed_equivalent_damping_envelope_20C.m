function envelope = build_fixed_equivalent_damping_envelope_20C( ...
    M, K_tangent_original, C_foundation, observation_dofs, ...
    frequency_1X_Hz, target_zeta, C_rayleigh_legacy, C_ehl_old, options)
%BUILD_FIXED_EQUIVALENT_DAMPING_ENVELOPE_20C Pure full-order Stage C calibration.

if nargin < 9 || isempty(options), options = struct(); end
options = normalize_options(options);
validate_primary_matrices(M, K_tangent_original, C_foundation, ...
    C_rayleigh_legacy, C_ehl_old);

M_symmetry_error = symmetry_error(M);
K_tangent_symmetry_error = symmetry_error(K_tangent_original);
C_foundation_symmetry_error = symmetry_error(C_foundation);
if M_symmetry_error > 1e-12 || C_foundation_symmetry_error > 1e-12
    error('StageC:FIXED_DAMPING_MATRIX_VALIDATION_FAILED', ...
        'M and C_foundation must be symmetric within 1e-12.');
end
if K_tangent_symmetry_error > 1e-8
    error('StageC:REFERENCE_TANGENT_STIFFNESS_NOT_SYMMETRIC', ...
        ['REFERENCE_TANGENT_STIFFNESS_NOT_SYMMETRIC: the original tangent ', ...
        'stiffness exceeds 1e-8 relative asymmetry.']);
elseif K_tangent_symmetry_error > 1e-10
    error('StageC:ReferenceTangentStiffnessSymmetryCaution', ...
        ['REFERENCE_TANGENT_STIFFNESS_SYMMETRY_CAUTION: the original tangent ', ...
        'stiffness is between 1e-10 and 1e-8 relative asymmetry.']);
end
K_ref = 0.5*(K_tangent_original+K_tangent_original.');

modal = physical_modes(M, K_ref, options);
selection = select_1X_anchor_modes(modal.natural_frequencies_Hz, ...
    modal.mass_normalized_modes, observation_dofs, frequency_1X_Hz, ...
    options.participation_threshold);
modal.anchor_modes = selection.anchor_modes;
modal.anchor_frequencies_Hz = selection.anchor_frequencies_Hz;
modal.anchor_participation = selection.anchor_participation;
modal.anchor_distance_from_1X_Hz = selection.anchor_distance_from_1X_Hz;
modal.bracketing_flag = selection.bracketing_flag;
modal.anchor_modes_not_bracketing_1X = ...
    selection.anchor_modes_not_bracketing_1X;
modal.participation_filter_relaxed = selection.participation_filter_relaxed;
modal.all_mode_participation = selection.all_mode_participation;
modal.first_six_mode_indices = 1:min(6,numel(modal.natural_frequencies_Hz));

target_zeta = reshape(target_zeta,1,[]);
if ~isequal(size(target_zeta),[1 3]) || any(~isfinite(target_zeta)) || ...
        any(target_zeta < 0) || any(diff(target_zeta) < 0)
    error('StageC:TargetZeta', ...
        'LOW, NOMINAL and HIGH must be three ordered nonnegative targets.');
end
scenario_names = {'LOW','NOMINAL','HIGH'};
Phi = modal.mass_normalized_modes;
omega = 2*pi*modal.natural_frequencies_Hz(:);
zeta_base = modal_zeta(Phi, C_foundation, omega);
anchor_base = zeta_base(selection.anchor_indices);
if any(anchor_base > target_zeta(3))
    error('StageC:FOUNDATION_DAMPING_ABOVE_HIGH_TARGET', ...
        'Foundation damping already exceeds the HIGH target at an anchor.');
end

C_base_modal = Phi.'*C_foundation*Phi;
offdiag = C_base_modal-diag(diag(C_base_modal));
base_modal_offdiag_ratio = norm(offdiag,'fro')/max(norm(C_base_modal,'fro'),eps);
A = [1/(2*omega(selection.anchor_indices(1))), ...
    omega(selection.anchor_indices(1))/2; ...
    1/(2*omega(selection.anchor_indices(2))), ...
    omega(selection.anchor_indices(2))/2];

alpha = zeros(1,3); beta = zeros(1,3);
solution_method = cell(1,3);
base_exceeds = false(3,2);
anchor_actual = zeros(3,2); anchor_errors = zeros(3,2);
matrix_validation = struct();
zeta_rayleigh = struct(); zeta_total = struct();
old_ehl_sensitivity_error = 0;
legacy_rayleigh_sensitivity_error = 0;
inputs_before = {C_rayleigh_legacy,C_foundation,C_ehl_old};

for s = 1:3
    compensation = max(target_zeta(s)-anchor_base,0).';
    base_exceeds(s,:) = anchor_base > target_zeta(s);
    coefficients = A\compensation;
    if all(coefficients >= 0)
        solution_method{s} = 'DIRECT';
    else
        coefficients = lsqnonneg(A,compensation);
        solution_method{s} = 'NNLS';
    end
    alpha(s) = coefficients(1); beta(s) = coefficients(2);
    C_rayleigh_ref = alpha(s)*M+beta(s)*K_ref;
    formal = assemble_equivalent_frequency_domain_damping( ...
        'fixed_equivalent_system', C_rayleigh_legacy, C_foundation, ...
        C_ehl_old, C_rayleigh_ref);
    formal_ehl_changed = assemble_equivalent_frequency_domain_damping( ...
        'fixed_equivalent_system', C_rayleigh_legacy, C_foundation, ...
        -C_ehl_old, C_rayleigh_ref);
    formal_legacy_changed = assemble_equivalent_frequency_domain_damping( ...
        'fixed_equivalent_system', -C_rayleigh_legacy, C_foundation, ...
        C_ehl_old, C_rayleigh_ref);
    old_ehl_sensitivity_error = max(old_ehl_sensitivity_error, ...
        relative_error(formal_ehl_changed.C_formal,formal.C_formal));
    legacy_rayleigh_sensitivity_error = max(legacy_rayleigh_sensitivity_error, ...
        relative_error(formal_legacy_changed.C_formal,formal.C_formal));

    zeta_R = modal_zeta(Phi,C_rayleigh_ref,omega);
    zeta_T = modal_zeta(Phi,formal.C_formal,omega);
    zeta_rayleigh.(scenario_names{s}) = reshape(zeta_R(1:min(6,end)),1,[]);
    zeta_total.(scenario_names{s}) = reshape(zeta_T(1:min(6,end)),1,[]);
    anchor_actual(s,:) = zeta_T(selection.anchor_indices);
    anchor_errors(s,:) = anchor_actual(s,:)-target_zeta(s);
    validate_anchor_error(anchor_actual(s,:), anchor_base, target_zeta(s), ...
        base_exceeds(s,:), solution_method{s});

    validation = validate_formal_matrix(formal.C_formal,M);
    matrix_validation.(scenario_names{s}) = validation;
    if ~validation.passed
        error('StageC:FIXED_DAMPING_MATRIX_VALIDATION_FAILED', ...
            'A formal fixed damping matrix failed finite/symmetry/PSD checks.');
    end
    damping.(['C_rayleigh_ref_' scenario_names{s}]) = C_rayleigh_ref;
    damping.(['C_formal_' scenario_names{s}]) = formal.C_formal;
end

if ~isequaln(inputs_before,{C_rayleigh_legacy,C_foundation,C_ehl_old}) || ...
        old_ehl_sensitivity_error > 1e-12 || ...
        legacy_rayleigh_sensitivity_error > 1e-12
    error('StageC:FIXED_DAMPING_MATRIX_VALIDATION_FAILED', ...
        'Formal damping depends on legacy damping or modified its inputs.');
end
if ~ordered_scenarios(zeta_total,scenario_names) || ...
        any(diff(anchor_actual,1,1) < -1e-12,'all')
    error('StageC:DAMPING_SCENARIO_ORDERING_FAILED', ...
        'The actual damping envelope is not ordered LOW to HIGH.');
end

damping.scenarios = scenario_names;
damping.target_zeta = target_zeta;
damping.C_foundation_full = C_foundation;
damping.K_ref_20C = K_ref;
damping.alpha = alpha;
damping.beta = beta;
damping.solution_method = solution_method;
damping.zeta_base_1to6 = reshape(zeta_base(1:min(6,end)),1,[]);
damping.zeta_rayleigh_1to6 = zeta_rayleigh;
damping.zeta_total_1to6 = zeta_total;
damping.zeta_base_anchor = anchor_base;
damping.anchor_actual_zeta = anchor_actual;
damping.anchor_target_errors = anchor_errors;
damping.base_exceeds_target_flags = base_exceeds;
damping.base_damping_exceeds_target = any(base_exceeds,'all');
damping.base_modal_offdiag_ratio = base_modal_offdiag_ratio;
damping.matrix_validation = matrix_validation;
damping.old_ehl_excluded = true;
damping.legacy_rayleigh_excluded = true;
damping.gyroscopic_term_is_separate = true;
damping.frozen_across_temperatures = true;

validation = struct('passed',true, ...
    'M_symmetry_error',M_symmetry_error, ...
    'K_tangent_symmetry_error',K_tangent_symmetry_error, ...
    'C_foundation_symmetry_error',C_foundation_symmetry_error, ...
    'old_ehl_sensitivity_error',old_ehl_sensitivity_error, ...
    'legacy_rayleigh_sensitivity_error',legacy_rayleigh_sensitivity_error, ...
    'mass_normalization_passed',modal.mass_normalization_error <= 1e-8, ...
    'scenario_ordering_passed',true,'formal_matrices_frozen',true, ...
    'newmark_used',false,'frequency_response_used',false);
envelope = struct();
envelope.static = struct('M',M,'K_tangent_original',K_tangent_original, ...
    'K_ref_symmetric',K_ref);
envelope.modal = modal;
envelope.damping = damping;
envelope.validation = validation;
end

function options = normalize_options(options)
defaults = struct('minimum_mode_count',6, ...
    'rigid_body_frequency_threshold_Hz',1e-6, ...
    'participation_threshold',1e-10);
if ~isstruct(options) || ~all(ismember(fieldnames(options),fieldnames(defaults)))
    error('StageC:Options','Unknown Stage C modal option.');
end
names = fieldnames(defaults);
for k = 1:numel(names)
    if ~isfield(options,names{k}), options.(names{k}) = defaults.(names{k}); end
end
if ~isscalar(options.minimum_mode_count) || options.minimum_mode_count < 2 || ...
        options.minimum_mode_count ~= floor(options.minimum_mode_count) || ...
        ~isscalar(options.rigid_body_frequency_threshold_Hz) || ...
        options.rigid_body_frequency_threshold_Hz < 0 || ...
        ~isscalar(options.participation_threshold) || options.participation_threshold < 0
    error('StageC:Options','Invalid Stage C modal option.');
end
end

function validate_primary_matrices(M,K,C,CR,CE)
matrices = {M,K,C,CR,CE};
if any(cellfun(@(A) ~isnumeric(A) || ~isreal(A) || ~ismatrix(A),matrices))
    error('StageC:FIXED_DAMPING_MATRIX_VALIDATION_FAILED','All matrices must be real numeric arrays.');
end
n = size(M,1);
if n < 2 || any(cellfun(@(A) ~isequal(size(A),[n n]),matrices)) || ...
        any(cellfun(@(A) any(~isfinite(A),'all'),matrices))
    error('StageC:FIXED_DAMPING_MATRIX_VALIDATION_FAILED','All matrices must be finite square arrays of equal size.');
end
if symmetry_error(CR) > 1e-12 || symmetry_error(CE) > 1e-12
    error('StageC:FIXED_DAMPING_MATRIX_VALIDATION_FAILED','Legacy audit matrices must be symmetric within 1e-12.');
end
end

function modal = physical_modes(M,K,options)
M_s = 0.5*(M+M.'); K_s = 0.5*(K+K.');
[U,mu] = eig(full(M_s),'vector');
if any(~isfinite(U),'all') || any(~isfinite(mu)) || ...
        max(abs(imag(U)),[],'all') > 1e-10 || max(abs(imag(mu))) > 1e-10
    error('StageC:Modal20CBasisFailed', ...
        '20C_MODAL_BASIS_FAILED: the symmetric mass eigensystem is not finite and real.');
end
[mu,order] = sort(real(mu),'ascend'); U = real(U(:,order));
mass_tolerance = max(size(M_s))*eps(max(abs(mu)));
if any(mu < -mass_tolerance)
    error('StageC:Modal20CBasisFailed', ...
        '20C_MODAL_BASIS_FAILED: the full-order mass matrix has a negative eigenvalue.');
end
U0 = U(:,abs(mu) <= mass_tolerance);
Up = U(:,mu > mass_tolerance);
mass_rank = size(Up,2); mass_nullity = size(U0,2);
if mass_rank < options.minimum_mode_count || mass_rank+mass_nullity ~= size(M,1)
    error('StageC:Modal20CBasisFailed', ...
        '20C_MODAL_BASIS_FAILED: the mass subspaces do not support enough physical modes.');
end

condensation_residual = 0;
if mass_nullity == 0
    X0 = zeros(0,mass_rank);
else
    K00 = 0.5*(U0.'*K_s*U0+(U0.'*K_s*U0).');
    K0p = U0.'*K_s*Up;
    [L0,p0] = chol(full(K00),'lower');
    if p0 ~= 0
        error('StageC:Modal20CBasisFailed', ...
            '20C_MODAL_BASIS_FAILED: the zero-mass stiffness block is not positive definite.');
    end
    X0 = -(L0.'\(L0\K0p));
    condensation_residual = norm(K00*X0+K0p,'fro')/max(norm(K0p,'fro'),1);
    if condensation_residual > 1e-10
        error('StageC:Modal20CBasisFailed', ...
            '20C_MODAL_BASIS_FAILED: the zero-mass condensation residual exceeds 1e-10.');
    end
end
T = Up+U0*X0;
M_c = 0.5*(T.'*M_s*T+(T.'*M_s*T).');
K_c = 0.5*(T.'*K_s*T+(T.'*K_s*T).');
[Lm,pm] = chol(full(M_c),'lower');
if pm ~= 0
    error('StageC:Modal20CBasisFailed', ...
        '20C_MODAL_BASIS_FAILED: the condensed mass matrix is not positive definite.');
end
A = Lm\K_c/Lm.'; A = 0.5*(A+A.');
[V,lambda] = eig(full(A),'vector');
if any(~isfinite(V),'all') || any(~isfinite(lambda)) || ...
        max(abs(imag(V)),[],'all') > 1e-10 || max(abs(imag(lambda))) > 1e-10
    error('StageC:Modal20CBasisFailed', ...
        '20C_MODAL_BASIS_FAILED: the condensed eigensystem is not finite and real.');
end
[lambda,order] = sort(real(lambda),'ascend'); V = real(V(:,order));
Phi = T*(Lm.'\V);
minimum_lambda = max((2*pi*options.rigid_body_frequency_threshold_Hz)^2, ...
    max(size(A))*eps(max(1,max(abs(lambda)))));
retained = lambda > minimum_lambda;
lambda = lambda(retained); Phi = Phi(:,retained);
for j = 1:size(Phi,2)
    [~,pivot] = max(abs(Phi(:,j)));
    if Phi(pivot,j) < 0, Phi(:,j) = -Phi(:,j); end
end
if numel(lambda) < options.minimum_mode_count
    error('StageC:Modal20CBasisFailed', ...
        '20C_MODAL_BASIS_FAILED: too few finite positive physical modes were found.');
end
mass_error = norm(Phi.'*M*Phi-eye(size(Phi,2)),inf);
if ~isfinite(mass_error) || mass_error > 1e-8
    error('StageC:Modal20CMassNormalizationFailed', ...
        ['20C_MODAL_MASS_NORMALIZATION_FAILED: full-order physical modes ', ...
        'did not satisfy mass normalization.']);
end
algebraic_residual = norm(U0.'*K*Phi,'fro')/max(norm(K*Phi,'fro'),1);
modal = struct('natural_frequencies_Hz',reshape(sqrt(lambda)/(2*pi),1,[]), ...
    'physical_mode_indices',1:numel(lambda), ...
    'mass_normalized_modes',Phi,'mass_normalization_error',mass_error, ...
    'rigid_body_frequency_threshold_Hz',options.rigid_body_frequency_threshold_Hz, ...
    'mass_rank',mass_rank,'mass_nullity',mass_nullity, ...
    'mass_tolerance',mass_tolerance, ...
    'zero_mass_condensation_residual',condensation_residual, ...
    'zero_mass_algebraic_residual',algebraic_residual);
end

function zeta = modal_zeta(Phi,C,omega)
zeta = reshape(diag(Phi.'*C*Phi),1,[])./(2*reshape(omega,1,[]));
end

function validate_anchor_error(actual,base,target,saturated,method)
for j = 1:2
    if saturated(j)
        if actual(j) < base(j)-1e-12
            error('StageC:FIXED_EQUIVALENT_DAMPING_CALIBRATION_FAILED', ...
                'A saturated anchor fell below its foundation damping floor.');
        end
    else
        if strcmp(method,'NNLS')
            tolerance = max(1e-3,0.25*target);
        else
            tolerance = max(1e-4,0.05*target);
        end
        if abs(actual(j)-target) > tolerance
            error('StageC:FIXED_EQUIVALENT_DAMPING_CALIBRATION_FAILED', ...
                'An unsaturated anchor missed its target tolerance.');
        end
    end
end
end

function validation = validate_formal_matrix(C,M)
symmetry = symmetry_error(C);
eigenvalues = eig(full(0.5*(C+C.')));
minimum = min(real(eigenvalues));
scale = max(norm(C,2),1);
validation = struct('finite',all(isfinite(C),'all'), ...
    'size_match',isequal(size(C),size(M)), ...
    'symmetry_error',symmetry,'minimum_eigenvalue',minimum, ...
    'psd_tolerance',-1e-10*scale,'passed',false);
validation.passed = validation.finite && validation.size_match && ...
    symmetry <= 1e-12 && minimum >= validation.psd_tolerance;
end

function pass = ordered_scenarios(values,names)
pass = all(values.(names{1}) <= values.(names{2})+1e-12) && ...
    all(values.(names{2}) <= values.(names{3})+1e-12);
end

function value = symmetry_error(A)
value = norm(A-A.','fro')/max(norm(A,'fro'),eps);
end

function value = relative_error(A,B)
value = norm(A-B,'fro')/max(norm(B,'fro'),eps);
end
