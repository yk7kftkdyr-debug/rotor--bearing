function test_stage_c_fixed_damping_envelope
%TEST_STAGE_C_FIXED_DAMPING_ENVELOPE Pure Stage C contract tests.

repository_root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
original_path = path;
path_cleanup = onCleanup(@() path(original_path));
addpath(repository_root);

test_runtime_equivalence_gate();
test_two_dof_modal_normalization();
test_anchor_selection();
test_envelope_calibration_and_isolation();
test_base_damping_limits();
test_canonical_structure_validator();

fprintf('STAGE_C_FIXED_DAMPING_ENVELOPE_PASSED\n');
end

function test_runtime_equivalence_gate
CR = diag([1 2]); CF = diag([3 4]); CE = diag([5 6]);
passing = validate_runtime_legacy_damping_equivalence(CR, CF, CE, CR+CF+CE);
failing = validate_runtime_legacy_damping_equivalence(CR, CF, CE, CR+CF+CE+eye(2));
assert(passing.passed && passing.relative_error <= 1e-12);
assert(~failing.passed && failing.relative_error > 1e-12);
end

function test_two_dof_modal_normalization
omega = [2 3];
M = diag([2 1]); K = diag([8 9]); C = zeros(2);
options = struct('minimum_mode_count',2,'rigid_body_frequency_threshold_Hz',1e-9);
envelope = build_fixed_equivalent_damping_envelope_20C( ...
    M, K, C, 1:2, 0.4, [0.005 0.010 0.020], zeros(2), zeros(2), options);
assert(max(abs(envelope.modal.natural_frequencies_Hz(:).' - omega/(2*pi))) <= 1e-12);
assert(envelope.modal.mass_normalization_error <= 1e-12);
assert(norm(envelope.modal.mass_normalized_modes.'*M* ...
    envelope.modal.mass_normalized_modes-eye(2), inf) <= 1e-12);

M_singular = diag([1 2 0]);
K_singular = [4 0 1; 0 18 2; 1 2 10];
singular = build_fixed_equivalent_damping_envelope_20C( ...
    M_singular, K_singular, zeros(3), 1:3, 0.4, ...
    [0.005 0.010 0.020], zeros(3), zeros(3), options);
assert(singular.modal.mass_rank == 2 && singular.modal.mass_nullity == 1);
assert(singular.modal.mass_normalization_error <= 1e-12);
assert(singular.modal.zero_mass_condensation_residual <= 1e-12);
end

function test_anchor_selection
frequency = [100 160 180 300]; modes = eye(4);
normal = select_1X_anchor_modes(frequency, modes, 1:4, 165, 1e-10);
assert(isequal(normal.anchor_indices, [2 3]));
assert(normal.bracketing_flag && ~normal.participation_filter_relaxed);

not_bracketing = select_1X_anchor_modes([200 250 300], eye(3), 1:3, 165, 1e-10);
assert(isequal(not_bracketing.anchor_indices, [1 2]));
assert(~not_bracketing.bracketing_flag && not_bracketing.anchor_modes_not_bracketing_1X);

relaxed = select_1X_anchor_modes(frequency, modes, 1, 165, 1e-10);
assert(relaxed.participation_filter_relaxed);
assert(numel(relaxed.anchor_indices) == 2);
end

function test_envelope_calibration_and_isolation
frequency = [80 120 160 180 220 300]; omega = 2*pi*frequency;
M = eye(6); K = diag(omega.^2); Cfoundation = diag(2*omega*0.001);
legacy = diag(1:6); ehl = diag(7:12);
envelope = build_fixed_equivalent_damping_envelope_20C( ...
    M, K, Cfoundation, 1:6, 165, [0.005 0.010 0.020], legacy, ehl);

assert(all(envelope.damping.alpha >= 0) && all(envelope.damping.beta >= 0));
assert(all(strcmp(envelope.damping.solution_method, 'DIRECT')));
assert(all(size(envelope.damping.zeta_base_1to6) == [1 6]));
for name = {'LOW','NOMINAL','HIGH'}
    scenario = name{1};
    assert(all(size(envelope.damping.zeta_rayleigh_1to6.(scenario)) == [1 6]));
    assert(all(size(envelope.damping.zeta_total_1to6.(scenario)) == [1 6]));
end
assert(all(envelope.damping.zeta_total_1to6.LOW <= ...
    envelope.damping.zeta_total_1to6.NOMINAL + 1e-12));
assert(all(envelope.damping.zeta_total_1to6.NOMINAL <= ...
    envelope.damping.zeta_total_1to6.HIGH + 1e-12));
assert(envelope.validation.old_ehl_sensitivity_error <= 1e-12);
assert(envelope.validation.legacy_rayleigh_sensitivity_error <= 1e-12);
assert(envelope.validation.passed);
end

function test_base_damping_limits
frequency = [80 120 160 180 220 300]; omega = 2*pi*frequency;
M = eye(6); K = diag(omega.^2);

zeta = [0.001 0.001 0.006 0.006 0.001 0.001];
low_saturated = build_fixed_equivalent_damping_envelope_20C( ...
    M, K, diag(2*omega.*zeta), 1:6, 165, [0.005 0.010 0.020], zeros(6), zeros(6));
assert(any(low_saturated.damping.base_exceeds_target_flags(1,:)));

zeta_nnls = [0.001 0.001 0 0.0006 0.001 0.001];
nnls_case = build_fixed_equivalent_damping_envelope_20C( ...
    M, K, diag(2*omega.*zeta_nnls), 1:6, 165, [0.005 0.010 0.020], zeros(6), zeros(6));
assert(any(strcmp(nnls_case.damping.solution_method, 'NNLS')));

zeta_high = [0.001 0.001 0.021 0.021 0.001 0.001];
assert_error(@() build_fixed_equivalent_damping_envelope_20C( ...
    M, K, diag(2*omega.*zeta_high), 1:6, 165, [0.005 0.010 0.020], zeros(6), zeros(6)), ...
    'StageC:FOUNDATION_DAMPING_ABOVE_HIGH_TARGET');
end

function test_canonical_structure_validator
frequency = [80 120 160 180 220 300]; omega = 2*pi*frequency;
envelope = build_fixed_equivalent_damping_envelope_20C( ...
    eye(6), diag(omega.^2), zeros(6), 1:6, 165, ...
    [0.005 0.010 0.020], zeros(6), zeros(6));
artifact = synthetic_artifact(envelope);
validation = validate_stage_c_fixed_damping_envelope(artifact);
assert(validation.passed);
artifact.extra = true;
assert(~validate_stage_c_fixed_damping_envelope(artifact).passed);
bad_target = synthetic_artifact(envelope);
bad_target.results.damping.target_zeta = [0.004 0.010 0.020];
assert(~validate_stage_c_fixed_damping_envelope(bad_target).passed);
bad_saturation = synthetic_artifact(envelope);
bad_saturation.results.damping.base_exceeds_target_flags(1,1) = ...
    ~bad_saturation.results.damping.base_exceeds_target_flags(1,1);
assert(~validate_stage_c_fixed_damping_envelope(bad_saturation).passed);

source = fileread(fullfile(fileparts(fileparts(fileparts(mfilename('fullpath')))), ...
    'validate_stage_c_fixed_damping_envelope.m'));
assert(isempty(regexpi(source, '\<solve_\w*\s*\(', 'once')));
assert(isempty(regexpi(source, '\<eig(s)?\s*\(', 'once')));
end

function artifact = synthetic_artifact(envelope)
n = size(envelope.static.M,1);
results = struct();
results.meta = struct('branch','test','baseline_branch','test', ...
    'baseline_commit','a','stage_a_commit','b','stage_b_commit','c', ...
    'source_commit','d','creation_time',datetime('now'), ...
    'rotational_speed_rpm',9900,'analysis_type','test', ...
    'damping_model','fixed_equivalent_system','dynamic_ehl_used',false, ...
    'old_ehl_used_in_formal_response',false,'newmark_used',false, ...
    'model_dof_count',n);
results.configuration = struct('damping_model','fixed_equivalent_system', ...
    'mapping_validation_required',true);
results.progress = struct('stage_a_complete',true,'stage_b_complete',true, ...
    'stage_c_static_20C_complete',true,'stage_c_runtime_equivalence_complete',true, ...
    'stage_c_modal_complete',true,'stage_c_damping_envelope_complete',true, ...
    'last_completed_gate','STAGE_C_DAMPING_ENVELOPE', ...
    'stage_d_complete',false,'stage_e_complete',false,'stage_f_complete',false, ...
    'stage_g_complete',false,'stage_h_complete',false);
mapping = struct('validated',true,'dof_indices',1:min(n,6));
results.static.T20 = struct('q_static',zeros(n,1),'thermal_state',struct(), ...
    'bearing_state_front',struct(),'bearing_state_rear',struct(), ...
    'formal_acceptance',true,'acceptance_residuals',struct(), ...
    'dof_mapping',struct(),'bearing_mapping',mapping, ...
    'observation_mapping',mapping,'M',envelope.static.M, ...
    'K_tangent_original',envelope.static.K_tangent_original, ...
    'K_ref_symmetric',envelope.static.K_ref_symmetric, ...
    'G',zeros(n),'C_foundation_full',envelope.damping.C_foundation_full);
results.modal.T20 = envelope.modal;
results.damping = envelope.damping;
Z = zeros(2);
results.validation.runtime_legacy_equivalence = struct('passed',true, ...
    'runtime_equivalence_pending',false,'relative_error',0, ...
    'size_match',true,'all_finite',true,'space','REDUCED_AUDIT_SPACE', ...
    'symmetry_errors',struct(),'source_commit','d', ...
    'C_rayleigh_r',Z,'C_foundation_r',Z,'C_ehl_total_r',Z,'C_total_r',Z, ...
    'gyroscopic_term_excluded',true);
results.validation.stage_c = envelope.validation;
results.decision = struct('status','FIXED_EQUIVALENT_DAMPING_ENVELOPE_ACCEPTED', ...
    'allow_stage_d',true,'stage_d_executed',false);
artifact = struct('results',results);
end

function assert_error(callback, identifier)
try
    callback();
catch exception
    assert(strcmp(exception.identifier, identifier));
    return;
end
error('StageC:ExpectedError', 'Expected %s.', identifier);
end
