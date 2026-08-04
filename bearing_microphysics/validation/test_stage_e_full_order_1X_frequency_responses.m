function test_stage_e_full_order_1X_frequency_responses
%TEST_STAGE_E_FULL_ORDER_1X_FREQUENCY_RESPONSES Pure Stage E TDD contracts.
%
% These tests deliberately exercise only synthetic systems.  They must never
% invoke the formal four-temperature runner or rewrite the canonical MAT.

repository_root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
original_path = path; path_cleanup = onCleanup(@() path(original_path));
addpath(repository_root);

test_complex_force_phase_error_normalization();
test_complex_unbalance_force_four_phase_reconstruction();
test_single_dof_analytic_harmonic_response();
test_two_dof_non_diagonal_damping_response();
test_solver_diagnostic_contract();
test_near_zero_response_is_not_accepted();
test_gyroscopic_sign_contract();
test_severely_ill_conditioned_response_is_not_accepted();
test_metrics_mapping_validation_contract();
test_displacement_velocity_and_acceleration_metrics();
test_scale_aware_orbit_contracts();
test_orbit_phase_transmissibility_and_interface_metrics();
test_near_zero_transmissibility_denominator();
test_temperature_modal_audit_contract();
test_canonical_T20_modal_regression();
test_four_temperature_three_scenario_schedule_and_frozen_inputs();
test_stage_e_pure_contract_surface();
test_stage_e_full_artifact_integration_contracts();
test_stage_e_solver_failure_gate_contract();

fprintf('STAGE_E_FULL_ORDER_1X_FREQUENCY_RESPONSES_PASSED\n');
end

function test_complex_force_phase_error_normalization
omega = 2*pi*165;
context = struct('model_dof_count',4,'Omega_rotor_rad_s',omega, ...
    'omega_exc_rad_s',omega,'unbalance_mass_kg',1e-12, ...
    'eccentricity_m',1e-3,'phase_rad',0,'unbalance_node',16, ...
    'radial_dofs',[1 2],'mapping_source','FORMAL_TEST_MAPPING', ...
    'formal_force_at_time',@(t) small_imperfect_unbalance_force(t,omega));
force = build_stage_e_complex_unbalance_force(context);
reconstructed = real(force.Fhat*exp(1i*force.phase_rad));
column_errors = vecnorm(reconstructed-force.formal_phase_forces_N,2,1);
formal_column_norms = vecnorm(force.formal_phase_forces_N,2,1);
expected_error = max(column_errors)/max(max(formal_column_norms),eps);
assert(expected_error > 0 && expected_error <= 1e-12);
assert(abs(force.four_phase_reconstruction_error-expected_error) <= ...
    100*eps(max(expected_error,eps)));
assert(isequal(force.nonzero_dofs,reshape(find(force.Fhat ~= 0),1,[])));

mismatched = context; mismatched.radial_dofs = [1 3];
assert_error(@() build_stage_e_complex_unbalance_force(mismatched), ...
    'StageE:ForceMapping');
end

function test_complex_unbalance_force_four_phase_reconstruction
omega = 2*pi*165;
context = struct('model_dof_count',4,'Omega_rotor_rad_s',omega, ...
    'omega_exc_rad_s',omega,'unbalance_mass_kg',1e-3, ...
    'eccentricity_m',1e-3,'phase_rad',0,'unbalance_node',16, ...
    'radial_dofs',[1 2],'mapping_source','FORMAL_TEST_MAPPING', ...
    'formal_force_at_time',@(t) formal_test_unbalance_force(t,omega));
force = build_stage_e_complex_unbalance_force(context);

expected_amplitude = 1e-6*omega^2;
assert(isequal(size(force.Fhat),[4 1]));
assert(all(isfinite(force.Fhat)));
assert(abs(force.force_amplitude_N-expected_amplitude) <= 100*eps(expected_amplitude));
assert(force.four_phase_reconstruction_error <= 1e-12);
assert(isequal(force.nonzero_dofs,reshape(find(force.Fhat ~= 0),1,[])));
assert(strcmp(force.mapping_source,'FORMAL_TEST_MAPPING'));
assert(isequal(size(force.formal_phase_forces_N),[4 4]));
end

function test_single_dof_analytic_harmonic_response
M = 2; G = 0; K = 50; C = 3; omega = 4; Omega = 4; Fhat = 7-2i;
response = solve_stage_e_full_order_1X_case(M,G,K,C,Fhat,Omega,omega, ...
    struct('temperature_case_C',20,'scenario','LOW'));
expected_Z = K-omega^2*M+1i*omega*(C+Omega*G);
assert(isequal(size(response.qhat_full),[1 1]));
assert(abs(response.qhat_full-Fhat/expected_Z) <= 100*eps(abs(Fhat/expected_Z)));
assert(response.relative_residual <= 1e-12 && response.finite_solution);
assert(strcmp(response.conditioning_flag,'WELL_CONDITIONED'));
end

function test_two_dof_non_diagonal_damping_response
M = diag([2 3]); G = zeros(2); K = [30 -5;-5 25];
C = [3 1.5;1.5 4]; omega = 5; Omega = 5; Fhat = [2-1i;3+4i];
response = solve_stage_e_full_order_1X_case(M,G,K,C,Fhat,Omega,omega, ...
    struct('temperature_case_C',50,'scenario','NOMINAL'));
Z = K-omega^2*M+1i*omega*(C+Omega*G);
assert(norm(response.qhat_full-Z\Fhat) <= 1e-12*max(norm(Z\Fhat),1));
assert(response.relative_residual <= 1e-12 && response.finite_solution);
assert(ismember(response.conditioning_flag,{'WELL_CONDITIONED', ...
    'ILL_CONDITIONED_DYNAMIC_STIFFNESS', ...
    'SEVERELY_ILL_CONDITIONED_DYNAMIC_STIFFNESS'}));
end

function test_solver_diagnostic_contract
M = eye(3); G = [0 1 0;-1 0 0;0 0 0];
K = [11 2 1;2 13 3;1 3 17]; C = [2 0.4 0;0.4 3 0.2;0 0.2 4];
Omega = 1.7; omega = 2.3; Fhat = 1e-12*[1+2i;-3+0.5i;2-4i];
response = solve_stage_e_full_order_1X_case(M,G,K,C,Fhat,Omega,omega, ...
    struct('temperature_case_C',20,'scenario','LOW'));
Z = K-omega^2*M+1i*omega*(C+Omega*G);
expected_residual = norm(Z*response.qhat_full-Fhat)/max(norm(Fhat),eps);
assert(abs(response.relative_residual-expected_residual) <= ...
    100*eps(max(expected_residual,eps)));
assert(abs(response.norm_qhat_2-norm(response.qhat_full,2)) <= ...
    100*eps(max(response.norm_qhat_2,eps)));
assert(abs(response.norm_qhat_inf-norm(response.qhat_full,inf)) <= ...
    100*eps(max(response.norm_qhat_inf,eps)));
assert(~isfield(response,'dynamic_stiffness_Z'));
end

function test_near_zero_response_is_not_accepted
response = solve_stage_e_full_order_1X_case(0,0,1,0,eps,0,1, ...
    struct('temperature_case_C',20,'scenario','LOW'));
assert(response.finite_solution && response.relative_residual <= 1e-8);
assert(response.norm_qhat_inf <= 100*eps);
assert(~response.accepted);
end

function test_gyroscopic_sign_contract
M = eye(2); K = [20 0;0 30]; C = zeros(2); G = [0 2;-2 0];
omega = 3; Omega = 7; Fhat = [1;2i];
response = solve_stage_e_full_order_1X_case(M,G,K,C,Fhat,Omega,omega, ...
    struct('temperature_case_C',80,'scenario','HIGH', ...
    'gyroscopic_sign_convention','Mddot_plus_C_plus_OmegaG_qdot_plus_Kq'));
expected_Z = K-omega^2*M+1i*omega*(C+Omega*G);
assert(norm(response.qhat_full-expected_Z\Fhat) <= 1e-12);
assert(strcmp(response.gyroscopic_sign_convention, ...
    'Mddot_plus_C_plus_OmegaG_qdot_plus_Kq'));
assert(~response.gyroscopic_matrix_symmetrized);
end

function test_severely_ill_conditioned_response_is_not_accepted
captured_output = evalc("response = severe_solver_call();");
assert(isempty(strtrim(captured_output)));
assert(response.rcond_Z < 1e-14);
assert(strcmp(response.conditioning_flag, ...
    'SEVERELY_ILL_CONDITIONED_DYNAMIC_STIFFNESS'));
assert(response.finite_solution && response.relative_residual <= 1e-8);
assert(~response.accepted);
assert(isfield(response,'solver_warning') && isstruct(response.solver_warning));
assert(all(isfield(response.solver_warning,{'message','identifier'})));
end

function test_stage_e_solver_failure_gate_contract
success = solve_stage_e_full_order_1X_case(1,0,10,1,1,0,2,struct());
assert(isfield(success,'failure_gate') && strcmp(success.failure_gate,''));
zero = solve_stage_e_full_order_1X_case(0,0,1,0,eps,0,1,struct());
assert(strcmp(zero.failure_gate,'FREQUENCY_RESPONSE_NEAR_ZERO_FAILED'));
severe = severe_solver_call();
assert(strcmp(severe.failure_gate, ...
    'SEVERELY_ILL_CONDITIONED_DYNAMIC_STIFFNESS'));
source = fileread(fullfile(fileparts(fileparts(fileparts(mfilename('fullpath')))), ...
    'solve_stage_e_full_order_1X_case.m'));
markers = {'~finite_solution','rcond_Z < 1e-14', ...
    'relative_residual <= 1e-8','norm_qhat_inf > 100*eps'};
positions = cellfun(@(x) strfind(source,x),markers,'UniformOutput',false);
assert(all(cellfun(@(x) ~isempty(x),positions)));
assert(positions{1}(1) < positions{2}(1) && positions{2}(1) < positions{3}(1) && ...
    positions{3}(1) < positions{4}(1));
end

function test_displacement_velocity_and_acceleration_metrics
omega = 2*pi*165;
qhat = complex([3;4;1;2;5;12;8;15], ...
    [4;-3;2;-1;12;-5;15;-8])*1e-6;
metrics = extract_stage_e_frequency_response_metrics(qhat,omega,test_mapping());
R2 = metrics.rotor_metrics.R2;
assert(abs(R2.x_peak_m-abs(qhat(1))) <= eps(abs(qhat(1))));
assert(abs(R2.y_peak_m-abs(qhat(2))) <= eps(abs(qhat(2))));
assert(abs(R2.x_rms_m-abs(qhat(1))/sqrt(2)) <= eps(abs(qhat(1))));
assert(abs(R2.y_rms_m-abs(qhat(2))/sqrt(2)) <= eps(abs(qhat(2))));
assert(abs(R2.x_velocity_peak_m_s-omega*abs(qhat(1))) <= 1e-12);
assert(abs(R2.y_velocity_peak_m_s-omega*abs(qhat(2))) <= 1e-12);
assert(abs(R2.x_acceleration_peak_m_s2-omega^2*abs(qhat(1))) <= 1e-9);
assert(abs(R2.y_acceleration_peak_m_s2-omega^2*abs(qhat(2))) <= 1e-9);
expected_rms = hypot(abs(qhat(1)),abs(qhat(2)))/sqrt(2);
assert(abs(R2.radial_rms_m-expected_rms) <= 100*eps(expected_rms));
assert(norm(R2.velocity_complex_m_s-1i*omega*qhat(1:2)) <= 1e-18);
assert(norm(R2.acceleration_complex_m_s2+omega^2*qhat(1:2)) <= 1e-12);
assert(abs(R2.radial_velocity_rms_m_s-omega*expected_rms) <= 1e-12);
assert(abs(R2.radial_acceleration_rms_m_s2-omega^2*expected_rms) <= 1e-9);

transfer = metrics.transfer_metrics;
assert(abs(transfer.front.displacement_transmissibility-13/5) <= 1e-12);
assert(abs(transfer.front.velocity_transmissibility-13/5) <= 1e-12);
assert(abs(transfer.front.acceleration_transmissibility-13/5) <= 1e-12);
assert(abs(transfer.rear.displacement_transmissibility-17/sqrt(5)) <= 1e-12);
assert(abs(transfer.rear.velocity_transmissibility-17/sqrt(5)) <= 1e-12);
assert(abs(transfer.rear.acceleration_transmissibility-17/sqrt(5)) <= 1e-12);
assert(transfer.front_to_rear_casing.valid);
assert(strcmp(transfer.front_to_rear_casing.status,'TRANSMISSIBILITY_VALID'));
assert(abs(transfer.front_to_rear_casing.value-17/13) <= 1e-12);
assert(transfer.front_to_rear_rotor.valid);
assert(strcmp(transfer.front_to_rear_rotor.status,'TRANSMISSIBILITY_VALID'));
assert(abs(transfer.front_to_rear_rotor.value-sqrt(5)/5) <= 1e-12);

mapping = test_mapping();
expected_relative = {mapping.front_relative_transform*qhat, ...
    mapping.rear_relative_transform*qhat};
interfaces = {metrics.bearing_interface_metrics.front, ...
    metrics.bearing_interface_metrics.rear};
for k = 1:2
    relative = expected_relative{k}; interface = interfaces{k};
    relative_rms = hypot(abs(relative(1)),abs(relative(2)))/sqrt(2);
    assert(norm(interface.relative_displacement_complex_m-relative) <= 1e-18);
    assert(abs(interface.radial_rms_m-relative_rms) <= 1e-18);
    assert(abs(interface.radial_velocity_rms_m_s-omega*relative_rms) <= 1e-12);
    assert(abs(interface.radial_acceleration_rms_m_s2-omega^2*relative_rms) <= 1e-9);
    assert(abs(interface.x_phase_rad-angle(relative(1))) <= 100*eps);
    assert(abs(interface.y_phase_rad-angle(relative(2))) <= 100*eps);
    assert(interface.x_phase_reliable && interface.y_phase_reliable);
    assert(strcmp(interface.x_phase_status,'PHASE_VALID'));
    assert(strcmp(interface.y_phase_status,'PHASE_VALID'));
end
end

function test_metrics_mapping_validation_contract
qhat = ones(8,1); omega = 2*pi*165; mapping = test_mapping();
invalid = mapping; invalid.mapping_validation_required = false;
assert_error(@() extract_stage_e_frequency_response_metrics(qhat,omega,invalid), ...
    'StageE:MetricsMapping');
invalid = mapping; invalid.validated = false;
assert_error(@() extract_stage_e_frequency_response_metrics(qhat,omega,invalid), ...
    'StageE:MetricsMapping');
invalid = mapping; invalid.rotor.R2 = [1 9];
assert_error(@() extract_stage_e_frequency_response_metrics(qhat,omega,invalid), ...
    'StageE:MetricsMapping');
invalid = mapping; invalid.casing.C2 = [5 6.5];
assert_error(@() extract_stage_e_frequency_response_metrics(qhat,omega,invalid), ...
    'StageE:MetricsMapping');
invalid = mapping; invalid.front_relative_transform = zeros(2,7);
assert_error(@() extract_stage_e_frequency_response_metrics(qhat,omega,invalid), ...
    'StageE:MetricsMapping');
invalid = mapping; invalid.rear_relative_transform(1,1) = NaN;
assert_error(@() extract_stage_e_frequency_response_metrics(qhat,omega,invalid), ...
    'StageE:MetricsMapping');
end

function test_scale_aware_orbit_contracts
circular = orbit_for_pair([1e-9;-1i*1e-9]);
assert(abs(circular.axis_ratio-1) <= 1e-12);
assert(~strcmp(circular.rotation_direction,'LINEAR'));
assert(strcmp(circular.rotation_direction,'COUNTERCLOCKWISE'));
assert(strcmp(circular.rotation_status,'ROTATION_DIRECTION_VALID'));
assert(circular.rotation_reliable);
assert(~circular.principal_axis_valid);
assert(strcmp(circular.principal_axis_status, ...
    'PRINCIPAL_AXIS_UNDEFINED_NEAR_CIRCULAR'));

near_zero = orbit_for_pair([0;0]);
assert(~near_zero.rotation_reliable);
assert(strcmp(near_zero.rotation_status, ...
    'ROTATION_DIRECTION_UNRELIABLE_DUE_TO_NEAR_ZERO_ORBIT'));
assert(~near_zero.principal_axis_valid);

ellipse = orbit_for_pair([2e-9;-1i*1e-9]);
assert(abs(ellipse.axis_ratio-0.5) <= 1e-12);
assert(ellipse.rotation_reliable);
assert(strcmp(ellipse.rotation_direction,'COUNTERCLOCKWISE'));
assert(ellipse.principal_axis_valid);
assert(strcmp(ellipse.principal_axis_status,'PRINCIPAL_AXIS_VALID'));
end

function test_orbit_phase_transmissibility_and_interface_metrics
omega = 11;
qhat = [3;4i;2;1i;1;2i;0;0];
metrics = extract_stage_e_frequency_response_metrics(qhat,omega,test_mapping());
orbit = metrics.orbit_metrics.R2;
A = [real(qhat(1)) -imag(qhat(1));real(qhat(2)) -imag(qhat(2))];
singular = svd(A);
assert(abs(orbit.semi_major_axis_m-singular(1)) <= 100*eps(singular(1)));
assert(abs(orbit.semi_minor_axis_m-singular(2)) <= 100*eps(max(singular(2),1)));
assert(abs(orbit.axis_ratio-singular(2)/max(singular(1),eps)) <= 100*eps);
assert(abs(abs(orbit.principal_axis_angle_rad)-pi/2) <= 100*eps(pi));
assert(strcmp(orbit.rotation_direction,'CLOCKWISE'));
assert(strcmp(orbit.rotation_status,'ROTATION_DIRECTION_VALID'));
assert(strcmp(metrics.phase_metrics.C8.y_status, ...
    'PHASE_UNRELIABLE_DUE_TO_NEAR_ZERO_AMPLITUDE'));
assert(norm(metrics.bearing_interface_metrics.front.relative_displacement_complex_m - ...
    test_mapping().front_relative_transform*qhat) <= 1e-12);
assert(metrics.transfer_metrics.velocity_equals_displacement_at_single_frequency);
assert(metrics.transfer_metrics.acceleration_equals_displacement_at_single_frequency);
end

function test_near_zero_transmissibility_denominator
qhat = [0;0;2;1i;0;0;5;6i]*1e-12;
metrics = extract_stage_e_frequency_response_metrics(qhat,2*pi*165,test_mapping());
front = metrics.transfer_metrics.front;
assert(strcmp(front.status,'TRANSMISSIBILITY_DENOMINATOR_NEAR_ZERO'));
assert(~front.valid);
assert(isnan(front.displacement_transmissibility));
assert(isnan(front.velocity_transmissibility));
assert(isnan(front.acceleration_transmissibility));
for name = {'front_to_rear_casing','front_to_rear_rotor'}
    ratio = metrics.transfer_metrics.(name{1});
    assert(isstruct(ratio) && ~ratio.valid && isnan(ratio.value));
    assert(strcmp(ratio.status,'TRANSMISSIBILITY_DENOMINATOR_NEAR_ZERO'));
end
interface = metrics.bearing_interface_metrics.front;
assert(~interface.x_phase_reliable && ~interface.y_phase_reliable);
assert(strcmp(interface.x_phase_status, ...
    'PHASE_UNRELIABLE_DUE_TO_NEAR_ZERO_AMPLITUDE'));
assert(strcmp(interface.y_phase_status, ...
    'PHASE_UNRELIABLE_DUE_TO_NEAR_ZERO_AMPLITUDE'));
end

function test_temperature_modal_audit_contract
M = eye(3); K = diag((2*pi*[100 165 230]).^2);
damping = struct('LOW',0.01*eye(3),'NOMINAL',0.02*eye(3), ...
    'HIGH',0.04*eye(3));
audit = compute_stage_e_temperature_modal_audit(M,K,damping,165,[1 2]);
frequencies_Hz = [100 165 230]; omega = 2*pi*frequencies_Hz;
Phi = audit.mass_normalized_modes;
expected_participation = vecnorm(Phi([1 2],:),2,1)./ ...
    max(vecnorm(Phi,2,1),eps);
assert(isequal(size(audit.first_six_frequency_Hz),[1 3]));
assert(abs(audit.nearest_below_165_Hz.frequency_Hz-100) <= 1e-10);
assert(abs(audit.nearest_above_165_Hz.frequency_Hz-230) <= 1e-10);
assert(abs(audit.nearest_below_165_Hz.distance_Hz-65) <= 1e-10);
assert(abs(audit.nearest_above_165_Hz.distance_Hz-65) <= 1e-10);
assert(norm(expected_participation-[1 1 0]) <= 1e-12);
assert(norm(audit.observation_participation-expected_participation) <= 1e-12);
scenarios = {'LOW','NOMINAL','HIGH'};
for k = 1:numel(scenarios)
    scenario = scenarios{k};
    expected_zeta = diag(damping.(scenario)).'./(2*omega);
    assert(isequal(size(audit.actual_zeta.(scenario)),[1 3]));
    assert(norm(audit.actual_zeta.(scenario)-expected_zeta) <= 1e-12);
    assert(abs(audit.nearest_below_165_Hz.actual_zeta.(scenario)- ...
        expected_zeta(1)) <= 1e-12);
    assert(abs(audit.nearest_above_165_Hz.actual_zeta.(scenario)- ...
        expected_zeta(3)) <= 1e-12);
end
end

function test_canonical_T20_modal_regression
repository_root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
artifact = load(fullfile(repository_root,'results', ...
    'thermal_equivalent_damping_frequency_domain', ...
    'thermal_4T_frequency_domain_results.mat'));
r = artifact.results;
formal_damping = struct('LOW',r.damping.C_formal_LOW, ...
    'NOMINAL',r.damping.C_formal_NOMINAL,'HIGH',r.damping.C_formal_HIGH);
audit = compute_stage_e_temperature_modal_audit(r.static.T20.M, ...
    r.damping.K_ref_20C,formal_damping,165,r.static.T20.observation_mapping);
stored = r.modal.T20;
assert(norm(audit.first_six_frequency_Hz-stored.natural_frequencies_Hz(1:6)) ...
    <= 1e-10*max(norm(stored.natural_frequencies_Hz(1:6)),1));
assert(isequal(stored.anchor_modes,[9 10]));
assert(norm(audit.natural_frequencies_Hz([9 10])-stored.anchor_frequencies_Hz) ...
    <= 1e-10*max(norm(stored.anchor_frequencies_Hz),1));
mass_error = norm(audit.mass_normalized_modes.'*r.static.T20.M* ...
    audit.mass_normalized_modes-eye(size(audit.mass_normalized_modes,2)),inf);
assert(mass_error <= 1e-8);
assert(abs(audit.mass_normalization_error-mass_error) <= 1e-10);
assert(abs(audit.mass_normalization_error-stored.mass_normalization_error) <= 1e-10);
assert(isfield(stored,'all_mode_participation'));
assert(norm(audit.observation_participation-stored.all_mode_participation) <= ...
    1e-10*max(norm(stored.all_mode_participation),1));
end

function test_four_temperature_three_scenario_schedule_and_frozen_inputs
M = eye(2); G = [0 1;-1 0]; K = 100*eye(2); Fhat = [1;1i];
damping = struct('LOW',eye(2),'NOMINAL',2*eye(2),'HIGH',3*eye(2));
before = damping;
temperatures = [20 50 80 100]; scenarios = {'LOW','NOMINAL','HIGH'};
schedule = build_stage_e_case_schedule(temperatures,scenarios);
assert(numel(schedule) == 12);
observed = arrayfun(@(item) sprintf('T%d_%s', ...
    item.temperature_case_C,item.scenario),schedule,'UniformOutput',false);
expected = {'T20_LOW','T20_NOMINAL','T20_HIGH','T50_LOW', ...
    'T50_NOMINAL','T50_HIGH','T80_LOW','T80_NOMINAL','T80_HIGH', ...
    'T100_LOW','T100_NOMINAL','T100_HIGH'};
assert(isequal(observed,expected));
for k = 1:numel(schedule)
    response = solve_stage_e_full_order_1X_case(M,G,K, ...
        damping.(schedule(k).scenario),Fhat,2*pi*165,2*pi*165,schedule(k));
    assert(response.temperature_case_C == schedule(k).temperature_case_C);
    assert(strcmp(response.scenario,schedule(k).scenario));
end
assert(isequaln(damping,before));
end

function test_stage_e_pure_contract_surface
repository_root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
required = {'build_stage_e_complex_unbalance_force.m', ...
    'build_stage_e_case_schedule.m', ...
    'solve_stage_e_full_order_1X_case.m', ...
    'extract_stage_e_frequency_response_metrics.m', ...
    'compute_stage_e_temperature_modal_audit.m'};
for k = 1:numel(required)
    assert(isfile(fullfile(repository_root,required{k})), ...
        'StageE:MissingPlannedAPI','Missing planned Stage E API: %s',required{k});
end
combined = fileread(fullfile(repository_root,required{3}));
assert(isempty(regexpi(combined,'\<inv\s*\(','once')));
assert(isempty(regexpi(combined,'\<pinv\s*\(','once')));
assert(isempty(regexpi(combined,'\<newmark\w*\s*\(','once')));
assert(isempty(regexpi(combined,'dynamic.*contact.*load|rolling.*element.*load','once')));
end

function test_stage_e_full_artifact_integration_contracts
% These are deliberately RED contracts for the Stage E artifact boundary.
% They construct a complete, finite payload without solving any 192-DOF
% frequency-response or modal problem.  The live accepted Stage D artifact is
% read only and is never used as a transaction target.
baseline = synthetic_stage_d_results();

% Keep this first: the intended first RED is the missing frozen snapshot API.
before = stage_e_frozen_input_snapshot(baseline);
assert_stage_e_snapshot(before,baseline);

payload = mock_stage_e_payload(baseline);
[candidate,info] = update_stage_e_canonical_results( ...
    baseline,payload,'stage-e-integration-test');
assert(info.updated && ~info.already_accepted);
assert_stage_e_complete(candidate);
assert_stage_e_response_schema(candidate);
test_stage_e_runner_static_contract(repository_root_from_test());
assert_stage_e_progress_and_validation_schema(candidate);
after = stage_e_frozen_input_snapshot(candidate);
assert(isequaln(before,after), ...
    'StageE:FrozenInputChanged', ...
    'Stage E must preserve every frozen Stage A--D/static/damping input exactly.');
assert(isequaln(candidate.static,baseline.static));
assert(isequaln(candidate.damping.alpha,baseline.damping.alpha));
assert(isequaln(candidate.damping.beta,baseline.damping.beta));
assert(isequaln(candidate.damping.K_ref_20C,baseline.damping.K_ref_20C));
assert(isequaln(candidate.modal.T20,baseline.modal.T20));

validation = validate_stage_e_four_temperature_1X_results(struct('results',candidate));
assert(validation.passed && validation.stage_e_complete);
assert(validation.stage_d_projection.passed, ...
    'StageE:StageDProjection', ...
    'The Stage D projection must remain accepted inside Stage E validation.');

assert_error(@() update_stage_e_canonical_results( ...
    candidate,payload,'stage-e-integration-test'), ...
    'StageE:CanonicalResultArtifactConflict');
forbidden_payload = payload; forbidden_payload.dynamic_contact_used = true;
assert_error(@() update_stage_e_canonical_results( ...
    baseline,forbidden_payload,'stage-e-integration-test'), ...
    'StageE:CanonicalResultArtifactConflict');
for forbidden_name = {'newmark_used','nonlinear_bearing_used'}
    forbidden_payload = payload;
    forbidden_payload.(forbidden_name{1}) = true;
    assert_error(@() update_stage_e_canonical_results( ...
        baseline,forbidden_payload,'stage-e-integration-test'), ...
        'StageE:CanonicalResultArtifactConflict');
end
test_stage_e_validator_rejects_incomplete_or_forbidden_payload(candidate);
test_stage_e_validator_static_recomputation_contract(repository_root_from_test());
test_stage_e_canonical_transaction_contract(baseline,candidate);
end

function test_stage_e_validator_rejects_incomplete_or_forbidden_payload(candidate)
missing = candidate;
missing.frequency_response.T80 = rmfield(missing.frequency_response.T80,'HIGH');
assert(~validate_stage_e_four_temperature_1X_results( ...
    struct('results',missing)).passed);

tampered = candidate;
tampered.frequency_response.T100.NOMINAL.qhat_full(91) = NaN;
assert(~validate_stage_e_four_temperature_1X_results( ...
    struct('results',tampered)).passed);

synchronous_forgery = candidate;
record = synchronous_forgery.frequency_response.T20.LOW;
record.qhat_full(91) = record.qhat_full(91)*(1+0.1i);
record.metrics = extract_stage_e_frequency_response_metrics( ...
    record.qhat_full,record.omega_exc_rad_s,record.mapping_snapshot);
record = copy_metrics_to_response(record);
synchronous_forgery.frequency_response.T20.LOW = record;
assert(~validate_stage_e_four_temperature_1X_results( ...
    struct('results',synchronous_forgery)).passed);

for diagnostic = {'rcond_Z','relative_residual','norm_qhat_2','norm_qhat_inf', ...
        'finite_solution','accepted'}
    forged = candidate; field = diagnostic{1}; value = forged.frequency_response.T20.LOW.(field);
    if islogical(value), forged.frequency_response.T20.LOW.(field) = false;
    else, forged.frequency_response.T20.LOW.(field) = 1.1*value + eps; end
    assert(~validate_stage_e_four_temperature_1X_results(struct('results',forged)).passed);
end
for field = {'unbalance_mass_kg','eccentricity_m','unbalance_U_kg_m', ...
        'phase_rad','actual_node','mapping_source'}
    forged = candidate; value = forged.force_definition.(field{1});
    if isnumeric(value), forged.force_definition.(field{1}) = value+1e-6;
    else, forged.force_definition.(field{1}) = 'TAMPERED'; end
    assert(~validate_stage_e_four_temperature_1X_results(struct('results',forged)).passed);
end
forged = candidate; forged.frequency_response.T50.HIGH.force_definition.phase_rad = 0.1;
assert(~validate_stage_e_four_temperature_1X_results(struct('results',forged)).passed);

stale_metrics = candidate;
stale_metrics.frequency_response.T20.LOW.qhat_full(91) = ...
    stale_metrics.frequency_response.T20.LOW.qhat_full(91)*(1+0.1i);
assert(~validate_stage_e_four_temperature_1X_results( ...
    struct('results',stale_metrics)).passed);
force_support = candidate;
force_support.complex_force.nonzero_dofs = [1 2];
assert(~validate_stage_e_four_temperature_1X_results( ...
    struct('results',force_support)).passed);
force_audit = candidate; force_audit.force_audit.passed = false;
assert(~validate_stage_e_four_temperature_1X_results( ...
    struct('results',force_audit)).passed);
mapping_audit = candidate; mapping_audit.mapping_audit.passed = false;
assert(~validate_stage_e_four_temperature_1X_results( ...
    struct('results',mapping_audit)).passed);
R16_mismatch = candidate; R16_mismatch.mapping_snapshot.rotor.R16 = [1 2];
assert(~validate_stage_e_four_temperature_1X_results( ...
    struct('results',R16_mismatch)).passed);
modal_zeta = candidate;
modal_zeta.modal_audits.T50.actual_zeta.LOW(1) = 0.99;
assert(~validate_stage_e_four_temperature_1X_results( ...
    struct('results',modal_zeta)).passed);
modal_endpoint = candidate;
modal_endpoint.modal_audits.T80.nearest_below_165_Hz.actual_zeta.HIGH = 0.99;
assert(~validate_stage_e_four_temperature_1X_results( ...
    struct('results',modal_endpoint)).passed);
modal_participation = candidate;
idx = modal_participation.modal_audits.T100.observation_dofs(1);
assert(modal_participation.modal_audits.T100.observation_participation(idx) ~= 0);
modal_participation.modal_audits.T100.observation_participation(idx) = ...
    modal_participation.modal_audits.T100.observation_participation(idx)+0.25;
assert(~validate_stage_e_four_temperature_1X_results( ...
    struct('results',modal_participation)).passed);
modal_vs_audit = candidate;
modal_vs_audit.modal.T50.actual_zeta.NOMINAL(1) = 0.99;
assert(~validate_stage_e_four_temperature_1X_results( ...
    struct('results',modal_vs_audit)).passed);
damping_vs_audit = candidate;
damping_vs_audit.damping.actual_zeta_4T.T20.HIGH(1) = 0.99;
assert(~validate_stage_e_four_temperature_1X_results( ...
    struct('results',damping_vs_audit)).passed);

assert(~candidate.meta.dynamic_ehl_used && ...
    ~candidate.meta.old_ehl_used_in_formal_response);
tampering = { ...
    {'frequency_response','T50','LOW','dynamic_contact_used'}, ...
    {'frequency_response','T80','HIGH','newmark_used'}, ...
    {'frequency_response','T100','NOMINAL','nonlinear_bearing_used'}, ...
    {'meta','newmark_used'}, {'meta','dynamic_ehl_used'}, ...
    {'meta','old_ehl_used_in_formal_response'}, {'meta','dynamic_contact_used'}, ...
    {'meta','nonlinear_bearing_used'}, ...
    {'newmark'}, {'time_response'}, {'dynamic'}, {'contact'}, {'nonlinearity'}};
for k = 1:numel(tampering)
    assert_validator_rejects_forbidden_field(candidate,tampering{k});
end

forbidden_nested = {'frequency_response','T50','LOW','metrics','contact_loss_gate'};
assert_validator_rejects_forbidden_field(candidate,forbidden_nested);
forbidden_nested = {'frequency_response','T80','NOMINAL','metrics','Qmax_N'};
assert_validator_rejects_forbidden_field(candidate,forbidden_nested);
forbidden_nested = {'frequency_response','T100','HIGH','metrics','dynamic_contact_force'};
assert_validator_rejects_forbidden_field(candidate,forbidden_nested);
full_Z = {'frequency_response','T20','LOW','dynamic_stiffness_Z'};
assert_validator_rejects_forbidden_field(candidate,full_Z);
test_recursive_forbidden_paths(candidate);
end

function assert_validator_rejects_forbidden_field(candidate,names)
tampered = set_nested_test_field(candidate,names,true);
assert(~validate_stage_e_four_temperature_1X_results( ...
    struct('results',tampered)).passed, ...
    'StageE:ForbiddenDynamicPathAccepted', ...
    'Forbidden dynamic/nonlinear path was accepted: %s.',strjoin(names,'.'));
end

function value = set_nested_test_field(value,names,replacement)
if numel(names) == 1
    if ~isstruct(value), value = struct(); end
    value.(names{1}) = replacement;
else
    if ~isstruct(value), value = struct(); end
    if ~isfield(value,names{1}) || ~isstruct(value.(names{1}))
        value.(names{1}) = struct();
    end
    value.(names{1}) = set_nested_test_field( ...
        value.(names{1}),names(2:end),replacement);
end
end

function test_recursive_forbidden_paths(candidate)
paths = {{'audit_a','Qmax_N'}, {'validation','nested','Qmin_N'}, ...
    {'meta','a','b','contact_loss'}, {'payload','a','b','c','dynamic_contact'}, ...
    {'response_audit','a','b','c','d','newmark'}, ...
    {'metrics_audit','a','b','c','d','e','nonlinear_gate'}, ...
    {'other','a','b','c','d','e','dynamic_stiffness_Z'}};
for k = 1:numel(paths), assert_validator_rejects_forbidden_field(candidate,paths{k}); end
end

function test_stage_e_canonical_transaction_contract(baseline,candidate)
temporary_directory = tempname;
mkdir(temporary_directory);
cleanup = onCleanup(@() rmdir(temporary_directory,'s')); %#ok<NASGU>
canonical = fullfile(temporary_directory, ...
    'thermal_4T_frequency_domain_results.mat');
results = baseline; save(canonical,'results');
bytes_before = read_file_bytes(canonical);
semantic_before = load(canonical);

% The baseline lock must reject a candidate before it can create a transaction.
source = java.nio.file.Paths.get(canonical,javaArray('java.lang.String',0));
options = javaArray('java.nio.file.OpenOption',1);
options(1) = java.nio.file.StandardOpenOption.WRITE;
channel = java.nio.channels.FileChannel.open(source,options);
file_lock = channel.tryLock();
lock_cleanup = onCleanup(@() release_test_lock(file_lock,channel)); %#ok<NASGU>
assert_error(@() stage_e_canonical_transaction('save',canonical,candidate), ...
    'StageE:CanonicalResultArtifactConflict');
assert(isequal(read_file_bytes(canonical),bytes_before));
semantic_after = load(canonical);
assert(isequaln(semantic_after.results,semantic_before.results));
assert(~isfile([canonical '.stage_e_transaction.mat']));
audit = stage_e_canonical_transaction('audit',canonical);
assert(audit.passed && isequal(audit.observed_files, ...
    {'thermal_4T_frequency_domain_results.mat'}));
file_lock.release(); channel.close();

saved = stage_e_canonical_transaction('save',canonical,candidate);
assert(validate_stage_e_four_temperature_1X_results(saved).passed);
assert(~isfile([canonical '.stage_e_transaction.mat']));
audit = stage_e_canonical_transaction('audit',canonical);
assert(audit.passed && isequal(audit.observed_files, ...
    {'thermal_4T_frequency_domain_results.mat'}));

% An invalid candidate uses a fresh Stage D baseline, never an already-complete
% Stage E canonical artifact.
invalid_directory = fullfile(temporary_directory,'invalid_candidate');
mkdir(invalid_directory);
invalid_canonical = fullfile(invalid_directory, ...
    'thermal_4T_frequency_domain_results.mat');
results = baseline; save(invalid_canonical,'results');
bytes_before = read_file_bytes(invalid_canonical);
semantic_before = load(invalid_canonical);
invalid = candidate;
invalid.frequency_response.T20.LOW.qhat_full(91) = Inf;
assert_error(@() stage_e_canonical_transaction('save',invalid_canonical,invalid), ...
    'StageE:CanonicalResultArtifactConflict');
assert(isequal(read_file_bytes(invalid_canonical),bytes_before));
semantic_after = load(invalid_canonical);
assert(isequaln(semantic_after.results,semantic_before.results));
assert(~isfile([invalid_canonical '.stage_e_transaction.mat']));
assert(stage_e_canonical_transaction('audit',invalid_canonical).passed);

% A candidate projected from a different (but still Stage-D-valid) baseline
% must fail compare-and-swap without changing the canonical bytes.
cas_directory = fullfile(temporary_directory,'cas_projection_mismatch');
mkdir(cas_directory);
cas_canonical = fullfile(cas_directory,'thermal_4T_frequency_domain_results.mat');
results = baseline; results.validation.integration_benign = struct('marker',true);
assert(validate_stage_d_four_temperature_results(struct('results',results)).passed);
save(cas_canonical,'results'); cas_bytes = read_file_bytes(cas_canonical);
assert_error(@() stage_e_canonical_transaction('save',cas_canonical,candidate), ...
    'StageE:CanonicalResultArtifactConflict');
assert(isequal(read_file_bytes(cas_canonical),cas_bytes));

extra = fullfile(temporary_directory,'unexpected.txt');
fid = fopen(extra,'w'); fprintf(fid,'unexpected'); fclose(fid);
assert(~stage_e_canonical_transaction('audit',canonical).passed);
delete(extra);
assert(stage_e_canonical_transaction('audit',canonical).passed);
end

function test_stage_e_runner_static_contract(repository_root)
runner_path = fullfile(repository_root, ...
    'run_stage_e_build_four_temperature_1X_responses.m');
assert(isfile(runner_path),'StageE:MissingPlannedAPI', ...
    'Missing planned Stage E runner: %s',runner_path);
runner = fileread(runner_path);
assert(contains(runner,"23a975c2c762504dc4ae62f07379737582fd783f"));
assert(contains(runner,"feat: add full-order 1X thermal frequency responses"));
for required = {"9900","165","Omega_rotor_rad_s","omega_exc_rad_s", ...
        "unbalance_node", "16", "unbalance_mass_kg", "eccentricity_m", ...
        "phase_rad", "1e-3", "1e-6", "requested_label", "actual_node", ...
        "actual_global_dofs", "mapping_source"}
    assert(contains(runner,required{1}),'StageE:RunnerForceGate', ...
        'Runner omits required force/mapping gate: %s.',required{1});
end
omega_1X = 2*pi*165; one_ulp = eps(omega_1X);
assert(abs((omega_1X+one_ulp)-omega_1X) <= 10*eps(omega_1X));
assert(isempty(regexp(runner,'rotational_speed_rad_s\s*~=|omega_1X_rad_s\s*~=','once')));
reference_gate = '2\s*\*\s*pi\s*\*\s*165';
threshold_gate = 'eps\s*\(\s*2\s*\*\s*pi\s*\*\s*165\s*\)';
gate_positions = zeros(1,2); gate_index = 0;
for field = {'rotational_speed_rad_s','omega_1X_rad_s'}
    gate_index = gate_index+1;
    pattern = ['abs\s*\([^\r\n]*' field{1} '\s*-\s*' reference_gate ...
        '[^\r\n]*\)\s*>\s*' threshold_gate];
    gate_positions(gate_index) = regexp(runner,pattern,'start','once');
    assert(~isempty(gate_positions(gate_index)), ...
        'StageE:RunnerOneULPGate','Runner must use the 1ULP gate for %s.',field{1});
end
omega_assignment = regexp(runner, ...
    'omega\s*=\s*2\s*\*\s*pi\s*\*\s*165\s*;','start','once');
force_call_after_gate = regexp(runner, ...
    'build_stage_e_complex_unbalance_force\s*\(','start','once');
solver_call_after_gate = regexp(runner, ...
    'solve_stage_e_full_order_1X_case\s*\(','start','once');
assert(~isempty(omega_assignment) && max(gate_positions) < omega_assignment && ...
    omega_assignment < force_call_after_gate && omega_assignment < solver_call_after_gate);
assert(isempty(regexp(runner,'omega\s*=\s*cfg\.omega_1X_rad_s','once')));
assert(~isempty(regexp(runner,'''omega_exc_rad_s''\s*,\s*omega','once')) && ...
    ~isempty(regexp(runner,'''Omega_rotor_rad_s''\s*,\s*omega','once')));
assert(~isempty(regexp(runner, ...
    'solve_stage_e_full_order_1X_case[\s\S]{0,500}omega\s*,\s*omega','once')));
expected = {'bearing_microphysics/validation/test_stage_d_four_temperature_static_states.m', ...
    'bearing_microphysics/validation/test_stage_e_full_order_1X_frequency_responses.m', ...
    'build_stage_e_complex_unbalance_force.m','build_stage_e_case_schedule.m', ...
    'solve_stage_e_full_order_1X_case.m', ...
    'extract_stage_e_frequency_response_metrics.m', ...
    'compute_stage_e_temperature_modal_audit.m', ...
    'build_stage_e_observation_mapping.m','stage_e_frozen_input_snapshot.m', ...
    'update_stage_e_canonical_results.m', ...
    'validate_stage_e_four_temperature_1X_results.m', ...
    'stage_e_canonical_transaction.m', ...
    'run_stage_e_build_four_temperature_1X_responses.m'};
assert(numel(expected) == 13 && ~any(contains(expected,'thermal_4T_frequency_domain_results.mat')));
assert(contains(runner,'isequal(changed,expected)'));
assert(~isempty(regexp(runner,'system\s*\(.*git diff --name-only 23a975c2c762504dc4ae62f07379737582fd783f\.\.HEAD','once')));
assert(contains(runner,'strsplit') || contains(runner,'regexp('));
assert(contains(runner,'git status --porcelain=v1'));
assert(contains(runner,'git log -1 --format=%P HEAD'));
assert(contains(runner,'git log -1 --format=%s HEAD'));
assert(~isempty(regexp(runner,'\[diff_status\s*,\s*diff_output\]\s*=\s*system\s*\(','once')));
assert(~isempty(regexp(runner,'diff_status\s*~=\s*0','once')));
assert(~isempty(regexp(runner,'changed\s*=.*diff_output','once')));
assert(isempty(regexp(runner,'changed\s*=\s*expected','once')));
assert(~isempty(regexp(runner,'\[status_status\s*,\s*status_output\]\s*=\s*system\s*\(','once')));
assert(~isempty(regexp(runner,'status_status\s*~=\s*0','once')));
assert(~isempty(regexp(runner,'isempty\s*\(\s*strtrim\s*\(\s*status_output\s*\)\s*\)','once')));
assert(~isempty(regexp(runner,'\[parent_status\s*,\s*parent_output\]\s*=\s*system\s*\(','once')));
assert(~isempty(regexp(runner,'parent_status\s*~=\s*0','once')));
assert(contains(runner,"strcmp(strtrim(parent_output),'23a975c2c762504dc4ae62f07379737582fd783f')"));
assert(~isempty(regexp(runner,'\[subject_status\s*,\s*subject_output\]\s*=\s*system\s*\(','once')));
assert(~isempty(regexp(runner,'subject_status\s*~=\s*0','once')));
assert(contains(runner,"strcmp(strtrim(subject_output),'feat: add full-order 1X thermal frequency responses')"));
tokens = regexp(runner,'''([^'']+\.m)''','tokens');
literal_files = unique(cellfun(@(x) x{1},tokens,'UniformOutput',false));
assert(isequal(sort(literal_files),sort(expected)), ...
    'StageE:SourceWhitelist','Runner literal expected list differs from exact tracked whitelist.');
snapshot_starts = regexp(runner,'stage_e_frozen_input_snapshot\s*\(','start');
assert(numel(snapshot_starts) == 2,'StageE:RunnerLayerCount', ...
    'Runner must snapshot frozen inputs before and after Stage E work.');
call_patterns = { ...
    'build_stage_e_complex_unbalance_force\s*\(',1; ...
    'build_stage_e_observation_mapping\s*\(',1; ...
    'compute_stage_e_temperature_modal_audit\s*\(',1; ...
    'build_stage_e_case_schedule\s*\(',1; ...
    'solve_stage_e_full_order_1X_case\s*\(',1; ...
    'extract_stage_e_frequency_response_metrics\s*\(',1; ...
    'update_stage_e_canonical_results\s*\(',1; ...
    'validate_stage_e_four_temperature_1X_results\s*\(',1; ...
    'stage_e_canonical_transaction\s*\(\s*''save''',1};
direct_starts = zeros(1,size(call_patterns,1));
for k = 1:size(call_patterns,1)
    matches = regexp(runner,call_patterns{k,1},'start');
    assert(numel(matches) == call_patterns{k,2},'StageE:RunnerLayerCount', ...
        'Runner must call %s exactly %d time(s).',call_patterns{k,1},call_patterns{k,2});
    direct_starts(k) = matches(end);
end
starts = [snapshot_starts(1) direct_starts(1:6) ...
    snapshot_starts(2) direct_starts(7:end)];
assert(all(diff(starts) > 0),'StageE:RunnerLayerOrder', ...
    'Runner must snapshot, solve, validate, then transactionally save in order.');
assert(~isempty(regexp(runner,'pre_audit\.passed','once')));
assert(contains(runner,'isequaln(snapshot_before,snapshot_after)'));
assert(~isempty(regexp(runner,'candidate_validation\.passed','once')));
assert(contains(runner,'PENDING_INDEPENDENT_RELOAD'));
assert(isempty(regexp(runner,'stage_e_complete\s*=\s*true','once')));
assert(~isempty(regexp(runner,'response\.failure_gate','once')));
assert(contains(runner,'FIRST_FAILED_CASE') && contains(runner,'FIRST_FAILED_GATE'));
schedule_loop = regexp(runner,'for\s+k\s*=\s*1\s*:\s*numel\s*\(\s*schedule\s*\)','start','once');
solver_call = regexp(runner,'solve_stage_e_full_order_1X_case\s*\(','start','once');
failure_branch = regexp(runner,'if\s+~\s*response\.accepted','start','once');
failure_error = regexp(runner,'error\s*\(','start');
response_write = regexp(runner,'frequency_response','start');
assert(~isempty(schedule_loop) && ~isempty(solver_call) && ~isempty(failure_branch) && ...
    ~isempty(failure_error) && ~isempty(response_write));
error_after = failure_error(find(failure_error > failure_branch,1,'first'));
response_write_after = response_write(find(response_write > failure_branch,1,'first'));
assert(~isempty(error_after) && ~isempty(response_write_after));
assert(schedule_loop < solver_call && solver_call < failure_branch && ...
    failure_branch < error_after && error_after < response_write_after, ...
    'StageE:FirstFailureOrder','First rejected response must error before it is persisted.');
assert(isempty(regexp(runner,'first_failed_case\s*=|failure_reason\s*=','once')), ...
    'StageE:FailureCanonicalMutation','Runner must leave canonical failure state unchanged.');
for gate = {'FREQUENCY_RESPONSE_RESIDUAL_FAILED', ...
        'FREQUENCY_RESPONSE_NONFINITE_FAILED', ...
        'FREQUENCY_RESPONSE_NEAR_ZERO_FAILED', ...
        'SEVERELY_ILL_CONDITIONED_DYNAMIC_STIFFNESS'}
    assert(contains(runner,gate{1}),'StageE:RunnerFailureGate', ...
        'Runner must preserve the specific failure gate %s.',gate{1});
end
for pattern = {'\<inv\s*\(','\<pinv\s*\(','\<newmark\w*\s*\(', ...
        '\<nonlinear[^\r\n(]*bearing\w*\s*\(', ...
        '\<dynamic[^\r\n(]*contact\w*\s*\(','writetable\s*\(', ...
        'writecell\s*\(','diary\s*\('}
    assert(isempty(regexpi(runner,pattern{1},'once')), ...
        'StageE:ForbiddenRunnerSource','Forbidden Stage E runner construct: %s',pattern{1});
end
end

function results = accepted_stage_d_results
repository_root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
artifact = load(fullfile(repository_root,'results', ...
    'thermal_equivalent_damping_frequency_domain', ...
    'thermal_4T_frequency_domain_results.mat'));
stage_d = validate_stage_d_four_temperature_results(artifact);
assert(stage_d.passed && stage_d.stage_d_complete, ...
    'StageE:AcceptedStageDRequired', ...
    'The Stage E integration fixture requires accepted live Stage D data.');
results = artifact.results;
end

function results = synthetic_stage_d_results
% The accepted artifact is read only.  This in-memory clone is deliberately a
% simple 192-DOF system so the integration contract never solves the formal
% rotor/bearing model or rewrites its canonical MAT.
results = accepted_stage_d_results(); n = 192;
omega_reference = 2*pi*165;
results.configuration.rotational_speed_rad_s = ...
    omega_reference+eps(omega_reference);
results.configuration.omega_1X_rad_s = omega_reference-eps(omega_reference);
assert(abs(results.configuration.rotational_speed_rad_s-omega_reference) <= ...
    eps(omega_reference) && ...
    abs(results.configuration.omega_1X_rad_s-omega_reference) <= eps(omega_reference));
frequencies = [60:10:130 150 180 200:10:(200+10*(n-11))];
frequencies = frequencies(1:n);
K20 = diag((2*pi*frequencies).^2); M = eye(n); G = zeros(n);
results.static.T20.M = M; results.static.T20.G = G;
results.static.T20.K_tangent_original = K20;
for temperature = [50 80 100]
    field = sprintf('T%d',temperature);
    results.static.(field).K_tangent = K20*(1+temperature*1e-3);
end
results.damping.K_ref_20C = K20;
results.damping.C_foundation_full = zeros(n);
for scenario = {'LOW','NOMINAL','HIGH'}
    name = scenario{1}; scale = find(strcmp(scenario,{'LOW','NOMINAL','HIGH'}));
    C = scale*0.1*eye(n);
    results.damping.(['C_rayleigh_ref_' name]) = C;
    results.damping.(['C_formal_' name]) = C;
end
results.modal.T20 = struct('natural_frequencies_Hz',frequencies, ...
    'physical_mode_indices',1:n,'mass_normalized_modes',eye(n), ...
    'mass_normalization_error',0,'rigid_body_frequency_threshold_Hz',1e-6, ...
    'mass_rank',n,'mass_nullity',0,'mass_tolerance',eps, ...
    'zero_mass_condensation_residual',0,'zero_mass_algebraic_residual',0, ...
    'anchor_modes',[9 10],'anchor_frequencies_Hz',frequencies([9 10]), ...
    'all_mode_participation',ones(1,n));
results.validation.stage_d.frozen_stage_c_reference = ...
    synthetic_stage_d_frozen_summary(results);
assert(validate_stage_d_four_temperature_results(struct('results',results)).passed, ...
    'StageE:SyntheticStageD','Synthetic Stage D fixture must remain accepted.');
end

function summary = synthetic_stage_d_frozen_summary(results)
d = results.damping; weights = reshape(linspace(1,2,numel(d.K_ref_20C)),size(d.K_ref_20C));
matrix = @(A) struct('size',size(A),'class',class(A),'norm_fro',norm(A,'fro'), ...
    'norm_one',norm(A,1),'trace',trace(A),'sum',sum(A,'all'), ...
    'sum_abs',sum(abs(A),'all'),'max_abs',max(abs(A),[],'all'), ...
    'weighted_sum',sum(A.*weights,'all'));
summary = struct('alpha',d.alpha,'beta',d.beta, ...
    'anchor_modes',results.modal.T20.anchor_modes, ...
    'damping_serialized_sha256',test_serialized_sha256(d), ...
    'anchor_modes_serialized_sha256',test_serialized_sha256(results.modal.T20.anchor_modes), ...
    'T20_serialized_sha256',test_serialized_sha256(results.static.T20), ...
    'K_ref_20C',matrix(d.K_ref_20C), ...
    'C_rayleigh_ref_LOW',matrix(d.C_rayleigh_ref_LOW), ...
    'C_rayleigh_ref_NOMINAL',matrix(d.C_rayleigh_ref_NOMINAL), ...
    'C_rayleigh_ref_HIGH',matrix(d.C_rayleigh_ref_HIGH), ...
    'C_formal_LOW',matrix(d.C_formal_LOW), ...
    'C_formal_NOMINAL',matrix(d.C_formal_NOMINAL), ...
    'C_formal_HIGH',matrix(d.C_formal_HIGH), ...
    'old_ehl_excluded',d.old_ehl_excluded, ...
    'legacy_rayleigh_excluded',d.legacy_rayleigh_excluded, ...
    'frozen_across_temperatures',d.frozen_across_temperatures, ...
    'comparison','MATLAB_SERIALIZED_SHA256_AND_DETERMINISTIC_SUMMARY');
end

function hash = test_serialized_sha256(value)
engine = java.security.MessageDigest.getInstance('SHA-256');
engine.update(getByteStreamFromArray(value));
hash = lower(reshape(dec2hex(typecast(engine.digest(),'uint8'),2).',1,[]));
end

function payload = mock_stage_e_payload(baseline)
% The fabricated records use the already-tested pure metric function, but do
% not use the full-order solver or the formal temperature-modal audit.
temperatures = [20 50 80 100]; scenarios = {'LOW','NOMINAL','HIGH'};
schedule = build_stage_e_case_schedule(temperatures,scenarios);
complex_force = mock_complex_force();
mapping = build_stage_e_observation_mapping( ...
    baseline.static.T20,baseline.configuration,complex_force.nonzero_dofs);
assert_stage_e_observation_mapping(mapping,complex_force.nonzero_dofs);
force_definition = struct('rotational_speed_rpm',9900,'frequency_exc_Hz',165, ...
    'Omega_rotor_rad_s',2*pi*165,'omega_exc_rad_s',2*pi*165, ...
    'requested_label','R16','actual_node',16,'unbalance_node',16, ...
    'actual_global_dofs',complex_force.nonzero_dofs, ...
    'unbalance_mass_kg',1e-3,'eccentricity_m',1e-3, ...
    'unbalance_U_kg_m',1e-6,'phase_rad',0, ...
    'mapping_source','FORMAL_UNBALANCE_FORCE_SUPPORT');
responses = struct();
for k = 1:numel(schedule)
    item = schedule(k); field = sprintf('T%d',item.temperature_case_C);
    [qhat,rcond_Z,relative_residual] = synthetic_response( ...
        baseline,item,complex_force.Fhat);
    metrics = extract_stage_e_frequency_response_metrics(qhat,2*pi*165,mapping);
    record = struct('qhat_full',qhat,'rcond_Z',rcond_Z, ...
        'relative_residual',relative_residual,'finite_solution',all(isfinite(qhat)), ...
        'norm_qhat_2',norm(qhat,2),'norm_qhat_inf',norm(qhat,inf), ...
        'conditioning_flag','WELL_CONDITIONED','accepted',true, ...
        'solver','backslash','solver_warning',struct('message','','identifier','', ...
        'output',''),'gyroscopic_sign_convention', ...
        'Mddot_plus_C_plus_OmegaG_qdot_plus_Kq', ...
        'gyroscopic_matrix_symmetrized',false,'omega_exc_rad_s',2*pi*165, ...
        'Omega_rotor_rad_s',2*pi*165, ...
        'temperature_case_C',item.temperature_case_C,'scenario',item.scenario, ...
        'frequency_exc_Hz',165,'force_definition',force_definition, ...
        'Fhat',complex_force.Fhat, ...
        'metrics',metrics,'schema_version','stage-e-response-v1', ...
        'dynamic_contact_used',false,'newmark_used',false, ...
        'nonlinear_bearing_used',false,'mapping_snapshot',mapping, ...
        'source_commit','stage-e-integration-test');
    record = copy_metrics_to_response(record);
    responses.(field).(item.scenario) = record;
end
modal = struct('T20',baseline.modal.T20);
modal_audits = struct();
actual_zeta_4T = struct();
for temperature = temperatures
    field = sprintf('T%d',temperature);
    K = synthetic_K_for_temperature(baseline,temperature);
    damping = struct('LOW',baseline.damping.C_formal_LOW, ...
        'NOMINAL',baseline.damping.C_formal_NOMINAL, ...
        'HIGH',baseline.damping.C_formal_HIGH);
    audit = compute_stage_e_temperature_modal_audit( ...
        baseline.static.T20.M,K,damping,165, ...
        unique([mapping.rotor.R2 mapping.rotor.R10 mapping.casing.C2 mapping.casing.C8]));
    audit.temperature_case_C = temperature;
    audit.schema_version = 'stage-e-modal-audit-v1';
    modal_audits.(field) = audit;
    actual_zeta_4T.(field) = audit.actual_zeta;
    if temperature ~= 20, modal.(field) = audit; end
end

function [qhat,rcond_Z,relative_residual] = synthetic_response(baseline,item,Fhat)
M = baseline.static.T20.M; G = baseline.static.T20.G;
K = synthetic_K_for_temperature(baseline,item.temperature_case_C);
C = baseline.damping.(['C_formal_' item.scenario]);
omega = 2*pi*165; Z = K-omega^2*M+1i*omega*(C+omega*G);
qhat = Z\Fhat; rcond_Z = rcond(Z);
relative_residual = norm(Z*qhat-Fhat)/max(norm(Fhat),eps);
end

function K = synthetic_K_for_temperature(baseline,temperature)
field = sprintf('T%d',temperature);
if temperature == 20, K = baseline.static.T20.K_tangent_original;
else, K = baseline.static.(field).K_tangent; end
end
force_audit = struct('passed',true,'R16_actual_dofs',complex_force.nonzero_dofs, ...
    'force_amplitude_N',complex_force.force_amplitude_N, ...
    'force_support_matches_mapping',true,'source','TEST_COMPLEX_FORCE_AUDIT');
mapping_audit = struct('passed',true,'R16_actual_supported',true, ...
    'force_nonzero_dofs',complex_force.nonzero_dofs, ...
    'source','TEST_OBSERVATION_MAPPING_AUDIT');
payload = struct('frequency_response',responses,'modal',modal, ...
    'modal_audits',modal_audits,'actual_zeta_4T',actual_zeta_4T, ...
    'complex_force',complex_force,'mapping_snapshot',mapping, ...
    'force_definition',force_definition, ...
    'mapping_audit',mapping_audit,'force_audit',force_audit, ...
    'source_commit','stage-e-integration-test', ...
    'dynamic_contact_used',false,'newmark_used',false, ...
    'nonlinear_bearing_used',false,'schema_version','stage-e-payload-v1');
end

function qhat = mock_qhat_192(index)
qhat = complex((1:192).',(192:-1:1).')*index*1e-9;
end

function force = mock_complex_force
omega = 2*pi*165; radial_dofs = [91 92];
context = struct('model_dof_count',192,'Omega_rotor_rad_s',omega, ...
    'omega_exc_rad_s',omega,'unbalance_mass_kg',1e-3, ...
    'eccentricity_m',1e-3,'phase_rad',0,'unbalance_node',16, ...
    'radial_dofs',radial_dofs,'mapping_source','FORMAL_UNBALANCE_FORCE_SUPPORT', ...
    'formal_force_at_time',@(t) synthetic_r16_force(t,omega,radial_dofs));
force = build_stage_e_complex_unbalance_force(context);
assert(isequal(size(force.Fhat),[192 1]) && ...
    isequal(force.nonzero_dofs,radial_dofs) && ...
    isequal(size(force.formal_phase_forces_N),[192 4]));
end

function assert_stage_e_observation_mapping(mapping,actual_dofs)
required = {'rotor','casing','front_relative_transform','rear_relative_transform', ...
    'mapping_validation_required','validated','requested_label','actual_node', ...
    'actual_global_dofs','mapping_source'};
assert(all(isfield(mapping,required)) && mapping.mapping_validation_required && ...
    mapping.validated);
assert(all(isfield(mapping.rotor,{'R2','R10','R16'})) && ...
    all(isfield(mapping.casing,{'C2','C8'})));
assert(isequal(mapping.rotor.R16,actual_dofs));
assert(strcmp(mapping.requested_label,'R16') && mapping.actual_node == 16 && ...
    isequal(mapping.actual_global_dofs,actual_dofs));
assert(isequal(size(mapping.front_relative_transform),[2 192]) && ...
    isequal(size(mapping.rear_relative_transform),[2 192]));
end

function record = copy_metrics_to_response(record)
names = {'bearing_interface_metrics','rotor_metrics','casing_metrics', ...
    'orbit_metrics','transfer_metrics','phase_metrics'};
for k = 1:numel(names)
    record.(names{k}) = record.metrics.(names{k});
end
end

function assert_stage_e_response_schema(results)
required = {'qhat_full','rcond_Z','relative_residual','finite_solution', ...
    'norm_qhat_2','norm_qhat_inf','conditioning_flag','accepted','solver', ...
    'solver_warning','gyroscopic_sign_convention', ...
    'gyroscopic_matrix_symmetrized','omega_exc_rad_s','Omega_rotor_rad_s', ...
    'temperature_case_C','scenario','metrics','schema_version', ...
    'dynamic_contact_used','newmark_used','nonlinear_bearing_used', ...
    'frequency_exc_Hz','force_definition','Fhat', ...
    'bearing_interface_metrics','rotor_metrics','casing_metrics','orbit_metrics', ...
    'transfer_metrics','phase_metrics','mapping_snapshot','source_commit'};
metric_names = {'bearing_interface_metrics','rotor_metrics','casing_metrics', ...
    'orbit_metrics','transfer_metrics','phase_metrics'};
for temperature = [20 50 80 100]
    for scenario = {'LOW','NOMINAL','HIGH'}
        record = results.frequency_response.(sprintf('T%d',temperature)).(scenario{1});
        assert(all(isfield(record,required)) && record.frequency_exc_Hz == 165);
        assert(isequal(record.Fhat,results.complex_force.Fhat));
        assert(isequaln(record.force_definition,results.force_definition));
        assert(record.force_definition.rotational_speed_rpm == 9900 && ...
            record.force_definition.actual_node == 16 && ...
            record.force_definition.unbalance_mass_kg == 1e-3 && ...
            record.force_definition.eccentricity_m == 1e-3 && ...
            record.force_definition.unbalance_U_kg_m == 1e-6 && ...
            record.force_definition.phase_rad == 0);
        assert(record.omega_exc_rad_s == 2*pi*165 && ...
            record.Omega_rotor_rad_s == 2*pi*165 && ...
            record.force_definition.omega_exc_rad_s == 2*pi*165 && ...
            record.force_definition.Omega_rotor_rad_s == 2*pi*165);
        assert(isequal(record.mapping_snapshot.rotor.R16, ...
            results.complex_force.nonzero_dofs));
        for name = metric_names
            assert(isequaln(record.(name{1}),record.metrics.(name{1})));
        end
    end
end
end

function assert_stage_e_progress_and_validation_schema(results)
progress_fields = {'stage_e_complex_force_complete','stage_e_modal_T20_complete', ...
    'stage_e_modal_T50_complete','stage_e_modal_T80_complete', ...
    'stage_e_modal_T100_complete','stage_e_T20_complete','stage_e_T50_complete', ...
    'stage_e_T80_complete','stage_e_T100_complete','stage_e_modal_audit_complete', ...
    'stage_e_complete', ...
    'first_failed_case','failure_reason'};
assert(all(isfield(results.progress,progress_fields)));
assert(all(cellfun(@(name) results.progress.(name),progress_fields(1:end-3))) && ...
    results.progress.stage_e_complete && isempty(results.progress.first_failed_case) && ...
    isempty(results.progress.failure_reason));
validation_fields = {'snapshot_before','snapshot_after','damping_hashes', ...
    'static_hashes','matrices_unchanged','gyroscopic_convention', ...
    'force_reconstruction_error','response_diagnostics','units','mapping', ...
    'additional_results_check','forbidden_flags'};
assert(all(isfield(results.validation.stage_e,validation_fields)));
v = results.validation.stage_e;
assert(isequaln(v.snapshot_before,v.snapshot_after) && v.matrices_unchanged && ...
    isequal(size(v.response_diagnostics),[4 3]) && strcmp(v.units,'SI') && ...
    v.additional_results_check.passed && all(~struct2array(v.forbidden_flags)));
assert(strcmp(results.decision.status,'FOUR_TEMPERATURE_1X_RESPONSES_ACCEPTED'));
end

function test_stage_e_validator_static_recomputation_contract(repository_root)
validator_path = fullfile(repository_root,'validate_stage_e_four_temperature_1X_results.m');
assert(isfile(validator_path),'StageE:MissingPlannedAPI');
source = fileread(validator_path);
for pattern = {'K-omega\^2\*M','1i\*omega\*\(C\+Omega\*G\)', ...
        'rcond\s*\(','relative_residual','norm_qhat_2','norm_qhat_inf', ...
        'compute_stage_e_temperature_modal_audit\s*\('}
    assert(~isempty(regexp(source,pattern{1},'once')), ...
        'StageE:ValidatorRecomputation','Validator must recompute %s.',pattern{1});
end
assert(~isempty(regexp(source,'fieldnames\s*\(','once')) && ...
    ~isempty(regexp(source,'recurs','once')), ...
    'StageE:RecursiveForbiddenScan','Validator must recursively inspect fields.');
end

function repository_root = repository_root_from_test
repository_root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
end

function audit = mock_modal_audit(baseline,mapping,temperature)
% Reuse the accepted Stage C physical modal basis, rather than pretending that
% an identity matrix is normalized under this mass matrix.  This is test data,
% not a fresh modal solve.
Phi = baseline.modal.T20.mass_normalized_modes;
frequencies = baseline.modal.T20.natural_frequencies_Hz;
n = numel(frequencies);
mass_error = norm(Phi.'*baseline.static.T20.M*Phi-eye(size(Phi,2)),inf);
assert(mass_error <= 1e-8,'StageE:MockModalBasis', ...
    'The accepted Stage C modal basis must remain M-normalized.');
actual_zeta = struct();
omega = 2*pi*frequencies;
for scenario = {'LOW','NOMINAL','HIGH'}
    name = scenario{1}; C = baseline.damping.(['C_formal_' name]);
    actual_zeta.(name) = reshape(diag(Phi.'*C*Phi),1,[])./(2*omega);
end
observation_dofs = unique([mapping.rotor.R2 mapping.rotor.R10 ...
    mapping.casing.C2 mapping.casing.C8]);
participation = vecnorm(Phi(observation_dofs,:),2,1)./ ...
    max(vecnorm(Phi,2,1),eps);
below = find(frequencies < 165,1,'last');
above = find(frequencies > 165,1,'first');
assert(~isempty(below) && ~isempty(above),'StageE:MockModalBracket');
audit = struct('natural_frequencies_Hz',frequencies, ...
    'physical_mode_indices',baseline.modal.T20.physical_mode_indices, ...
    'mass_normalized_modes',Phi, ...
    'mass_normalization_error',mass_error, ...
    'rigid_body_frequency_threshold_Hz',baseline.modal.T20.rigid_body_frequency_threshold_Hz, ...
    'mass_rank',baseline.modal.T20.mass_rank, ...
    'mass_nullity',baseline.modal.T20.mass_nullity, ...
    'mass_tolerance',baseline.modal.T20.mass_tolerance, ...
    'zero_mass_condensation_residual',baseline.modal.T20.zero_mass_condensation_residual, ...
    'zero_mass_algebraic_residual',baseline.modal.T20.zero_mass_algebraic_residual, ...
    'first_six_frequency_Hz',frequencies(1:6), ...
    'observation_dofs',observation_dofs, ...
    'observation_participation',participation, ...
    'actual_zeta',actual_zeta, ...
    'nearest_below_165_Hz',mock_modal_endpoint(below,frequencies,actual_zeta), ...
    'nearest_above_165_Hz',mock_modal_endpoint(above,frequencies,actual_zeta), ...
    'frequency_1X_Hz',165,'temperature_case_C',temperature, ...
    'schema_version','stage-e-modal-audit-v1');
end

function endpoint = mock_modal_endpoint(index,frequencies,actual_zeta)
endpoint = struct('mode_index',index,'frequency_Hz',frequencies(index), ...
    'distance_Hz',abs(frequencies(index)-165),'actual_zeta',struct( ...
    'LOW',actual_zeta.LOW(index),'NOMINAL',actual_zeta.NOMINAL(index), ...
    'HIGH',actual_zeta.HIGH(index)));
end

function F = synthetic_r16_force(t,omega,radial_dofs)
F = zeros(192,1); amplitude = 1e-6*omega^2; phase = omega*t;
F(radial_dofs) = amplitude*[cos(phase);sin(phase)];
end

function assert_stage_e_snapshot(snapshot,baseline)
assert(isstruct(snapshot) && isfield(snapshot,'hashes'));
names = {'T20_q_static','T50_q_static','T80_q_static','T100_q_static', ...
    'T20_thermal_state','T50_thermal_state','T80_thermal_state','T100_thermal_state', ...
    'T20_K_t','T50_K_t','T80_K_t','T100_K_t', ...
    'M','G','K_ref_20C','alpha','beta','C_foundation_full', ...
    'C_rayleigh_ref_LOW','C_rayleigh_ref_NOMINAL','C_rayleigh_ref_HIGH', ...
    'C_formal_LOW','C_formal_NOMINAL','C_formal_HIGH','T20_anchor_modes'};
assert(all(isfield(snapshot.hashes,names)),'StageE:FrozenSnapshotCoverage');
assert(isequaln(snapshot.static,baseline.static));
assert(isequaln(snapshot.damping,struct('K_ref_20C',baseline.damping.K_ref_20C, ...
    'alpha',baseline.damping.alpha,'beta',baseline.damping.beta, ...
    'C_foundation_full',baseline.damping.C_foundation_full, ...
    'C_rayleigh_ref_LOW',baseline.damping.C_rayleigh_ref_LOW, ...
    'C_rayleigh_ref_NOMINAL',baseline.damping.C_rayleigh_ref_NOMINAL, ...
    'C_rayleigh_ref_HIGH',baseline.damping.C_rayleigh_ref_HIGH, ...
    'C_formal_LOW',baseline.damping.C_formal_LOW, ...
    'C_formal_NOMINAL',baseline.damping.C_formal_NOMINAL, ...
    'C_formal_HIGH',baseline.damping.C_formal_HIGH)));
assert(isequaln(snapshot.modal_T20_anchor_modes,baseline.modal.T20.anchor_modes));
end

function assert_stage_e_complete(results)
temperatures = [20 50 80 100]; scenarios = {'LOW','NOMINAL','HIGH'};
for temperature = temperatures
    field = sprintf('T%d',temperature);
    assert(isfield(results.frequency_response,field));
    for k = 1:numel(scenarios)
        record = results.frequency_response.(field).(scenarios{k});
        assert(isequal(size(record.qhat_full),[192 1]) && ...
            all(isfinite(record.qhat_full)) && record.accepted && ...
            record.relative_residual <= 1e-8 && record.rcond_Z >= 1e-14);
    end
end
assert(isfield(results.modal,'T20') && isfield(results.modal,'T50') && ...
    isfield(results.modal,'T80') && isfield(results.modal,'T100'));
assert(isfield(results.damping,'actual_zeta_4T'));
assert(isfield(results,'complex_force') && isfield(results,'mapping_snapshot') && ...
    isfield(results,'mapping_audit') && isfield(results,'force_audit') && ...
    isfield(results,'modal_audits'));
for temperature = temperatures
    field = sprintf('T%d',temperature);
    audit = results.modal_audits.(field);
    assert(isequaln(results.damping.actual_zeta_4T.(field),audit.actual_zeta));
end
assert(results.progress.stage_e_complete && ...
    results.validation.stage_e.passed && results.decision.allow_stage_f);
end

function bytes = read_file_bytes(path)
fid = fopen(path,'rb');
assert(fid >= 0,'StageE:FileRead','Unable to read %s.',path);
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
bytes = fread(fid,Inf,'*uint8');
end

function release_test_lock(file_lock,channel)
if ~isempty(file_lock) && file_lock.isValid(), file_lock.release(); end
if channel.isOpen(), channel.close(); end
end

function mapping = test_mapping
mapping = struct();
mapping.rotor = struct('R2',[1 2],'R10',[3 4]);
mapping.casing = struct('C2',[5 6],'C8',[7 8]);
mapping.front_relative_transform = [1 0 0 0 -1 0 0 0;0 1 0 0 0 -1 0 0];
mapping.rear_relative_transform = [0 0 1 0 0 0 -1 0;0 0 0 1 0 0 0 -1];
mapping.mapping_validation_required = true;
mapping.validated = true;
end

function orbit = orbit_for_pair(pair)
qhat = zeros(8,1); qhat(1:2) = pair;
metrics = extract_stage_e_frequency_response_metrics(qhat,2*pi*165,test_mapping());
orbit = metrics.orbit_metrics.R2;
end

function F = formal_test_unbalance_force(t,omega)
amplitude = 1e-6*omega^2;
F = zeros(4,1);
F(1:2) = amplitude*[cos(omega*t);sin(omega*t)];
end

function F = small_imperfect_unbalance_force(t,omega)
amplitude = 1e-9;
phase = omega*t;
F = zeros(4,1);
F(1:2) = amplitude*[cos(phase);sin(phase)];
if abs(phase-pi/2) <= 10*eps(pi)
    F(1) = F(1)+4e-13*amplitude;
end
end

function response = severe_solver_call
M = zeros(2); G = zeros(2); K = diag([1e-16 1]); C = zeros(2);
response = solve_stage_e_full_order_1X_case(M,G,K,C,[1;1],1,1, ...
    struct('temperature_case_C',100,'scenario','LOW'));
end

function assert_error(callback,identifier)
try
    callback();
catch exception
    assert(strcmp(exception.identifier,identifier), ...
        'Unexpected identifier: %s',exception.identifier);
    return;
end
error('StageE:ExpectedError','Expected %s.',identifier);
end
