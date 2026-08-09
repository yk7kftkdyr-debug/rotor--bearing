function test_stage_f_rolling_element_loads_and_nonlinearity_gate(run_stage_a_to_e)
%TEST_STAGE_F_ROLLING_ELEMENT_LOADS_AND_NONLINEARITY_GATE Stage F RED tests.
%
% Synthetic pure-function tests only.  This suite must never execute the
% formal Stage F runner, rewrite the canonical MAT, rerun Stage E, or invoke
% Newmark.  Rear-bearing slices are always named and audited as slices; only
% the complex sums of nine slices are roller-level loads.

if nargin < 1, run_stage_a_to_e = true; end
assert(islogical(run_stage_a_to_e) && isscalar(run_stage_a_to_e));

repository_root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
original_path = path;
path_cleanup = onCleanup(@() path(original_path));
addpath(repository_root);

% RED order is intentional.  Until Stage F production code exists, this is
% the first and only expected failure.
test_planned_stage_f_api_surface(repository_root);

test_formal_five_dof_mapping_contract_rejections();
test_canonical_mapping_and_formal_scan_provenance(repository_root);
test_ball_hertz_derivative_matches_central_difference();
test_roller_slice_derivative_matches_central_difference();
test_zero_qhat_has_zero_recovered_load_and_zero_nonlinearity();
test_statically_unloaded_slice_is_not_contact_loss();
test_loaded_slice_contact_loss_is_detected();
test_r_delta_three_classes();
test_element_force_sum_matches_tangent_force();
test_layered_tangent_gate_failures_are_distinct();
test_rear_30_by_9_slice_audit_and_complex_roller_aggregation();
test_active_contact_set_change_is_detected();
test_pure_1X_dft_has_no_2X_or_3X();
test_known_1X_2X_3X_dft_is_exact();
test_epsilon_nonincrease_for_smaller_perturbation();
test_scan_is_nominal_only_and_exactly_32_phases();
test_formal_preflight_rejects_tampered_frozen_parameters(repository_root);
test_radial_resultant_uses_phase_accurate_peak_and_rms();
test_precommit_candidate_validation_and_simple_runner_contract(repository_root);
test_validator_recomputes_and_detects_tampering();
test_validator_rejects_coordinated_scan_tampering();
test_validator_rejects_complete_recovered_load_tampering();
test_validator_private_harmonic_provenance_audit();
test_required_source_provenance_cannot_be_deleted();
test_required_harmonic_force_and_thermal_contracts();
test_first_trigger_precise_independent_rebuild();
test_mandatory_trigger_and_rear_index_decode(repository_root);
test_contact_force_sign_semantics(repository_root);
test_stage_f_schema_and_forbidden_payloads();
test_scientific_save_and_computation_failure_atomicity();
test_runner_source_and_change_scope_gate(repository_root);
test_all_stage_f_sources_respect_scope(repository_root);
if run_stage_a_to_e
    test_stage_a_to_e_regressions_remain_green();
end

fprintf('STAGE_F_ROLLING_ELEMENT_LOADS_AND_NONLINEARITY_GATE_PASSED\n');
end

function test_canonical_mapping_and_formal_scan_provenance(repository_root)
canonical = fullfile(repository_root,'results', ...
    'thermal_equivalent_damping_frequency_domain', ...
    'thermal_4T_frequency_domain_results.mat');
artifact = load(canonical); results = artifact.results;
before = canonical_stage_a_to_e_categories(results);
temperatures = [20 50 80 100]; bearings = {'front','rear'};
for temperature = temperatures
    tf = sprintf('T%d',temperature);
    for bearing_index = 1:numel(bearings)
        bearing = bearings{bearing_index};
        mapping = build_stage_f_rolling_element_linearization_mapping( ...
            results,temperature,bearing);
        static_state = results.static.(tf);
        contact = static_state.(['bearing_state_' bearing]);
        expected_B = results.static.T20.bearing_mapping.([bearing '_B']);
        if temperature == 20
            expected_K = static_state.(['bearing_stiffness_' bearing]).K_local;
        else
            expected_K = static_state.(['K_b_' bearing]);
        end
        [expected_slice_mask,expected_roller_mask,expected_k,source] = ...
            canonical_contact_linearization_expectations( ...
            static_state,contact,temperature,bearing);
        assert(isequaln(mapping.interface_transform,expected_B));
        assert(isequaln(mapping.Q_static_slice,reshape(contact.Q,size(mapping.Q_static_slice))));
        assert(isequaln(mapping.delta_static_slice, ...
            reshape(contact.delta,size(mapping.delta_static_slice))));
        assert(isequaln(mapping.loaded_mask_static_slice, ...
            reshape(expected_slice_mask,size(mapping.loaded_mask_static_slice))));
        assert(isequaln(mapping.loaded_mask_static_roller, ...
            reshape(expected_roller_mask,size(mapping.loaded_mask_static_roller))));
        if strcmp(bearing,'front')
            assert(mapping.element_count == 22 && mapping.slice_count == 1);
            assert(isequal(size(mapping.Q_static_slice),[22 1]));
            assert(isequal(size(mapping.delta_static_slice),[22 1]));
            assert(isequal(size(mapping.k_static_slice),[22 1]));
            assert(isequal(size(mapping.loaded_mask_static_slice),[22 1]));
            assert(isequal(size(mapping.loaded_mask_static_roller),[22 1]));
        else
            assert(mapping.element_count == 30 && mapping.slice_count == 9);
            assert(isequal(size(mapping.Q_static_slice),[30 9]));
            assert(isequal(size(mapping.delta_static_slice),[30 9]));
            assert(isequal(size(mapping.k_static_slice),[30 9]));
            assert(isequal(size(mapping.loaded_mask_static_slice),[30 9]));
            assert(isequal(size(mapping.loaded_mask_static_roller),[30 1]));
        end
        assert(relative_error(mapping.k_static_slice, ...
            reshape(expected_k,size(mapping.k_static_slice))) <= 1e-10, ...
            'StageF:CanonicalContactStiffnessSource', ...
            'Mapping contact stiffness does not match its formal source.');
        assert(isequaln(mapping.K_b_local,expected_K));
        assert(isequal(size(mapping.contact_jacobian_local,2),5));
        assert(isequal(size(mapping.force_assembly_local,1),5));
        analytic_tangent = mapping.force_assembly_local* ...
            diag(mapping.k_static_slice(:))*mapping.contact_jacobian_local;
        assert(relative_error(mapping.K_b_local_analytic,analytic_tangent) <= 10*eps, ...
            'StageF:AnalyticTangentConstruction', ...
            'Saved analytic tangent must be A*diag(k)*J exactly.');
        assert(mapping.tangent_audit.analytic_vs_saved_relative_error <= 1e-8, ...
            'StageF:AnalyticVsFDTangentAudit', ...
            'Analytic-vs-saved tangent audit exceeds its fixed 1e-8 threshold.');
        assert(mapping.tangent_audit.analytic_vs_saved_threshold == 1e-8 && ...
            strcmp(mapping.tangent_audit.status,'ANALYTIC_VS_FD_TANGENT_AUDIT_PASSED'), ...
            'StageF:AnalyticVsFDTangentAudit', ...
            'The analytic-vs-saved tangent audit must save its fixed threshold and status.');
        assert(strcmp(mapping.mapping_provenance.canonical_sha256, ...
            '3f0e6e3b379cda670ef173682c58d17a19fc6b30f761ed59a00f3c203793abfd'));
        assert(strcmp(mapping.mapping_provenance.contact_kernel_sha256, ...
            'd82ae8cc92a1f1fb55d9d0c14da3efa196e9f085ce22abc5cdd2b345f93af5c8'));
        assert(strcmp(mapping.mapping_provenance.stage_d_source_commit, ...
            results.meta.stage_d_source_commit));
        assert(strcmp(mapping.mapping_provenance.stage_e_source_commit, ...
            results.meta.stage_e_source_commit));
        assert(contains(mapping.mapping_provenance.contact_state_path,tf) && ...
            contains(mapping.mapping_provenance.contact_state_path,bearing));
        assert(strcmp(mapping.mapping_provenance.contact_state_source_path, ...
            source.contact_state_path));
        assert(strcmp(mapping.mapping_provenance.contact_state_source_mode, ...
            source.contact_state_mode));
        assert(strcmp(mapping.mapping_provenance.loaded_mask_source_path, ...
            source.loaded_mask_path));
        assert(strcmp(mapping.mapping_provenance.loaded_mask_source_mode, ...
            source.loaded_mask_mode));
        assert(strcmp(mapping.mapping_provenance.contact_stiffness_source_path, ...
            source.contact_stiffness_path));
        assert(strcmp(mapping.mapping_provenance.contact_stiffness_source_mode, ...
            source.contact_stiffness_mode));
        assert(strcmp(mapping.mapping_provenance.contact_kernel_function, ...
            'nonlinear_bearing_force'));
        assert(isequal(size(mapping.interface_transform* ...
            results.frequency_response.(tf).NOMINAL.qhat_full),[5 1]));
        recovered = recover_stage_f_rolling_element_1X_loads(mapping, ...
            results.frequency_response.(tf).NOMINAL.qhat_full);
        assert(isequal(size(recovered.slice_level.Qhat_slice), ...
            size(mapping.Q_static_slice)));
        assert(isequal(size(recovered.roller_level.Qhat_roller), ...
            [mapping.element_count 1]));
    end
end
test_canonical_contact_linearization_rejections(results);
assert(isequaln(canonical_stage_a_to_e_categories(results),before), ...
    'StageF:CanonicalReadOnlyIntegration', ...
    'Formal mapping construction mutated canonical Stage A--E data.');

mapping = build_stage_f_rolling_element_linearization_mapping(results,20,'front');
request = formal_scan_request_from_canonical(results,20,'front',mapping);
evaluator = formal_nonlinear_evaluator_provenance(repository_root,results,20);

linear_substitute = evaluator;
linear_substitute.function_handle = @linear_scan_evaluator;
linear_substitute.function_name = 'linear_scan_evaluator';
assert_gate_error(@() evaluate_stage_f_nonlinear_bearing_force_scan( ...
    request,linear_substitute), ...
    'StageF:NonlinearEvaluatorProvenance', ...
    'NONLINEAR_FORCE_EVALUATOR_PROVENANCE_INVALID');

forged = evaluator; forged.source_sha256 = repmat('0',1,64);
assert_gate_error(@() evaluate_stage_f_nonlinear_bearing_force_scan( ...
    request,forged), ...
    'StageF:NonlinearEvaluatorProvenance', ...
    'NONLINEAR_FORCE_EVALUATOR_PROVENANCE_INVALID');

% A valid formal evaluator reaches ordinary request validation.  Removing
% qhat proves provenance was accepted while guaranteeing no phase is run.
incomplete = rmfield(request,'qhat_full');
assert_gate_error(@() evaluate_stage_f_nonlinear_bearing_force_scan( ...
    incomplete,evaluator), ...
    'StageF:ScanRequestIncomplete','STAGE_F_COMPUTATION_FAILED');
end

function [expected_slice_mask,expected_roller_mask,expected_k,source] = ...
        canonical_contact_linearization_expectations(static_state,contact,temperature,bearing)
tf = sprintf('T%d',temperature);
contact_path = ['results.static.' tf '.bearing_state_' bearing];
if temperature == 20
    assert(isfield(contact,'film') && isfield(contact.film,'loaded_mask') && ...
        isfield(contact,'raw_contact') && isfield(contact.raw_contact,'loaded'), ...
        'StageF:CanonicalT20ContactState', ...
        'T20 formal bearing state must expose film.loaded_mask and raw_contact.loaded.');
    assert(isequaln(contact.film.loaded_mask,contact.raw_contact.loaded), ...
        'StageF:CanonicalT20ContactMask', ...
        'T20 film and raw-contact active sets must agree.');
    assert(isfield(contact.raw_contact,'Q') && isfield(contact.raw_contact,'delta') && ...
        isequaln(contact.Q,contact.raw_contact.Q) && ...
        isequaln(contact.delta,contact.raw_contact.delta), ...
        'StageF:CanonicalT20ContactState', ...
        'T20 primary and raw contact Q/delta states must agree.');
    expected_slice_mask = logical(contact.film.loaded_mask);
    expected_k = zeros(size(contact.delta));
    if strcmp(bearing,'front'), exponent = 3/2; else, exponent = 10/9; end
    loaded = expected_slice_mask;
    assert(all(contact.Q(loaded) > 0,'all') && ...
        all(contact.delta(loaded) > 0,'all'), ...
        'StageF:CanonicalT20ContactState', ...
        'T20 loaded contacts must have positive formal Q and delta.');
    expected_k(loaded) = exponent*contact.Q(loaded)./contact.delta(loaded);
    source = struct( ...
        'contact_state_path',contact_path, ...
        'contact_state_mode','T20_DIRECT_CANONICAL_BEARING_STATE', ...
        'loaded_mask_path',[contact_path '.film.loaded_mask + ' contact_path '.raw_contact.loaded'], ...
        'loaded_mask_mode','T20_FORMAL_FILM_AND_RAW_CONTACT_AGREEMENT', ...
        'contact_stiffness_path',[contact_path '.Q/.delta + ' contact_path '.raw_contact.Q/.delta'], ...
        'contact_stiffness_mode','FORMAL_DERIVATIVE_IDENTITY_K_EQUALS_P_Q_OVER_DELTA');
else
    mask_field = ['loaded_mask_' bearing];
    stiffness_field = ['contact_stiffness_' bearing];
    assert(isfield(contact,'film') && isfield(contact.film,'loaded_mask') && ...
        isfield(contact,'raw_contact') && isfield(contact.raw_contact,'loaded') && ...
        isfield(static_state,mask_field) && isfield(static_state,stiffness_field), ...
        'StageF:CanonicalStaticContactLinearization', ...
        'Stage D formal film/raw masks and saved contact stiffness are required above T20.');
    assert(isequaln(contact.film.loaded_mask,contact.raw_contact.loaded), ...
        'StageF:CanonicalStaticContactLinearization', ...
        'Stage D film and raw-contact slice active sets must agree above T20.');
    expected_slice_mask = logical(contact.film.loaded_mask);
    expected_k = static_state.(stiffness_field);
    if strcmp(bearing,'rear')
        expected_roller_mask = logical(static_state.loaded_mask_rear);
        assert(isequaln(expected_roller_mask,any(expected_slice_mask,2).'), ...
            'StageF:CanonicalStaticContactLinearization', ...
            'Rear roller mask must be loaded_mask_rear and equal any(slice_mask,2).');
    else
        expected_roller_mask = logical(static_state.loaded_mask_front);
        assert(isequaln(expected_roller_mask,expected_slice_mask), ...
            'StageF:CanonicalStaticContactLinearization', ...
            'Front roller mask must agree with its single-slice formal active set.');
    end
    source = struct( ...
        'contact_state_path',contact_path, ...
        'contact_state_mode','STAGE_D_EXTRACTED_STATIC_CONTACT_LINEARIZATION', ...
        'loaded_mask_path',[contact_path '.film.loaded_mask + ' contact_path '.raw_contact.loaded'], ...
        'loaded_mask_mode','STAGE_D_FORMAL_FILM_AND_RAW_CONTACT_SLICE_AGREEMENT', ...
        'contact_stiffness_path',['results.static.' tf '.' stiffness_field], ...
        'contact_stiffness_mode','STAGE_D_EXTRACTED_CONTACT_STIFFNESS');
end
if temperature == 20
    if strcmp(bearing,'rear')
        expected_roller_mask = any(expected_slice_mask,2);
    else
        expected_roller_mask = expected_slice_mask;
    end
end
end

function test_canonical_contact_linearization_rejections(results)
for temperature = [50 80 100]
    tf = sprintf('T%d',temperature);
    for bearing = {'front','rear'}
        bn = bearing{1};
        missing_mask = results;
        missing_mask.static.(tf) = rmfield(missing_mask.static.(tf), ...
            ['loaded_mask_' bn]);
        assert_static_contact_linearization_rejection( ...
            @() build_stage_f_rolling_element_linearization_mapping( ...
            missing_mask,temperature,bn));

        missing_stiffness = results;
        missing_stiffness.static.(tf) = rmfield(missing_stiffness.static.(tf), ...
            ['contact_stiffness_' bn]);
        assert_static_contact_linearization_rejection( ...
            @() build_stage_f_rolling_element_linearization_mapping( ...
            missing_stiffness,temperature,bn));

        tampered_mask = results;
        field = ['loaded_mask_' bn]; value = tampered_mask.static.(tf).(field);
        value(1) = ~value(1); tampered_mask.static.(tf).(field) = value;
        assert_static_contact_linearization_rejection( ...
            @() build_stage_f_rolling_element_linearization_mapping( ...
            tampered_mask,temperature,bn));

        tampered_stiffness = results;
        field = ['contact_stiffness_' bn];
        tampered_stiffness.static.(tf).(field)(1) = ...
            1.01*tampered_stiffness.static.(tf).(field)(1)+eps;
        assert_static_contact_linearization_rejection( ...
            @() build_stage_f_rolling_element_linearization_mapping( ...
            tampered_stiffness,temperature,bn));

        state_field = ['bearing_state_' bn];
        missing_film_mask = results;
        missing_film_mask.static.(tf).(state_field).film = rmfield( ...
            missing_film_mask.static.(tf).(state_field).film,'loaded_mask');
        assert_static_contact_linearization_rejection( ...
            @() build_stage_f_rolling_element_linearization_mapping( ...
            missing_film_mask,temperature,bn));

        missing_raw_mask = results;
        missing_raw_mask.static.(tf).(state_field).raw_contact = rmfield( ...
            missing_raw_mask.static.(tf).(state_field).raw_contact,'loaded');
        assert_static_contact_linearization_rejection( ...
            @() build_stage_f_rolling_element_linearization_mapping( ...
            missing_raw_mask,temperature,bn));

        tampered_raw_mask = results;
        raw_mask = tampered_raw_mask.static.(tf).(state_field).raw_contact.loaded;
        raw_mask(1) = ~raw_mask(1);
        tampered_raw_mask.static.(tf).(state_field).raw_contact.loaded = raw_mask;
        assert_static_contact_linearization_rejection( ...
            @() build_stage_f_rolling_element_linearization_mapping( ...
            tampered_raw_mask,temperature,bn));
    end
end

for bearing = {'front','rear'}
    bn = bearing{1};
    state_field = ['bearing_state_' bn];
    missing_film_mask = results;
    missing_film_mask.static.T20.(state_field).film = rmfield( ...
        missing_film_mask.static.T20.(state_field).film,'loaded_mask');
    assert_static_contact_linearization_rejection( ...
        @() build_stage_f_rolling_element_linearization_mapping( ...
        missing_film_mask,20,bn));

    missing_raw_mask = results;
    missing_raw_mask.static.T20.(state_field).raw_contact = rmfield( ...
        missing_raw_mask.static.T20.(state_field).raw_contact,'loaded');
    assert_static_contact_linearization_rejection( ...
        @() build_stage_f_rolling_element_linearization_mapping( ...
        missing_raw_mask,20,bn));

    tampered_raw_mask = results;
    raw_mask = tampered_raw_mask.static.T20.(state_field).raw_contact.loaded;
    raw_mask(1) = ~raw_mask(1);
    tampered_raw_mask.static.T20.(state_field).raw_contact.loaded = raw_mask;
    assert_static_contact_linearization_rejection( ...
        @() build_stage_f_rolling_element_linearization_mapping( ...
        tampered_raw_mask,20,bn));

    tampered_Q = results;
    tampered_Q.static.T20.(state_field).raw_contact.Q(1) = ...
        1.01*tampered_Q.static.T20.(state_field).raw_contact.Q(1)+eps;
    assert_static_contact_linearization_rejection( ...
        @() build_stage_f_rolling_element_linearization_mapping( ...
        tampered_Q,20,bn));

    tampered_delta = results;
    tampered_delta.static.T20.(state_field).raw_contact.delta(1) = ...
        1.01*tampered_delta.static.T20.(state_field).raw_contact.delta(1)+eps;
    assert_static_contact_linearization_rejection( ...
        @() build_stage_f_rolling_element_linearization_mapping( ...
        tampered_delta,20,bn));

    tampered_K = results;
    K_field = ['bearing_stiffness_' bn];
    tampered_K.static.T20.(K_field).K_local(1,1) = ...
        1.01*tampered_K.static.T20.(K_field).K_local(1,1)+eps;
    assert_analytic_vs_fd_tangent_audit_rejection( ...
        @() build_stage_f_rolling_element_linearization_mapping( ...
        tampered_K,20,bn));
end
end

function assert_static_contact_linearization_rejection(callback)
assert_gate_error(callback, ...
    'StageF:StaticContactLinearizationStateInconsistent', ...
    'STATIC_CONTACT_LINEARIZATION_STATE_INCONSISTENT');
end

function assert_analytic_vs_fd_tangent_audit_rejection(callback)
assert_gate_error(callback, ...
    'StageF:AnalyticVsFDTangentAuditFailed', ...
    'ANALYTIC_VS_FD_TANGENT_AUDIT_FAILED');
end

function test_formal_five_dof_mapping_contract_rejections
fixture = force_consistent_ball_fixture();
mapping = build_stage_f_rolling_element_linearization_mapping(fixture);
assert(isequal(mapping.local_dof_order, ...
    {'ux','uy','uz','theta_x','theta_y'}));
assert(isequal(size(mapping.interface_transform,1),5));
assert(relative_error(mapping.contact_jacobian_local, ...
    fixture.formal_contact_jacobian_snapshot) <= 10*eps);
assert(relative_error(mapping.force_assembly_local, ...
    fixture.formal_force_assembly_snapshot) <= 10*eps);

wrong_assembly = fixture;
wrong_assembly.formal_force_assembly_snapshot = ...
    wrong_assembly.formal_contact_jacobian_snapshot.';
wrong_analytic_tangent = wrong_assembly.formal_force_assembly_snapshot* ...
    diag(wrong_assembly.k_static_slice)* ...
    wrong_assembly.formal_contact_jacobian_snapshot;
wrong_assembly_error = relative_error(wrong_analytic_tangent,fixture.K_b_local);
assert(wrong_assembly_error > 1e-8 && ...
    abs(wrong_assembly_error-3.06e-2) <= 5e-4, ...
    'StageF:FrontAssemblyDirectionFixture', ...
    'A=J'' must disagree with the front KKT/cos(alpha) saved tangent by about 3.06e-2.');
assert_analytic_vs_fd_tangent_audit_rejection(@() ...
    build_stage_f_rolling_element_linearization_mapping(wrong_assembly));

missing = rmfield(fixture,'bearing_interface_transform_5dof');
assert_gate_error(@() build_stage_f_rolling_element_linearization_mapping(missing), ...
    'StageF:RollingElementLinearizationMappingNotAvailable', ...
    'ROLLING_ELEMENT_LINEARIZATION_MAPPING_NOT_AVAILABLE');

not_five_dof = fixture;
not_five_dof.bearing_interface_transform_5dof = ...
    not_five_dof.bearing_interface_transform_5dof(1:4,:);
assert_gate_error(@() build_stage_f_rolling_element_linearization_mapping( ...
    not_five_dof), ...
    'StageF:RollingElementLinearizationMappingNotAvailable', ...
    'ROLLING_ELEMENT_LINEARIZATION_MAPPING_NOT_AVAILABLE');

forged = fixture;
forged.mapping_provenance.contact_kernel_sha256 = repmat('0',1,64);
assert_gate_error(@() build_stage_f_rolling_element_linearization_mapping(forged), ...
    'StageF:RollingElementLinearizationMappingNotAvailable', ...
    'ROLLING_ELEMENT_LINEARIZATION_MAPPING_NOT_AVAILABLE');

mutations = {@tamper_Q_static,@tamper_delta_static, ...
    @tamper_k_static,@tamper_loaded_mask};
for k = 1:numel(mutations)
    inconsistent = mutations{k}(fixture);
    assert_gate_error(@() build_stage_f_rolling_element_linearization_mapping( ...
        inconsistent), ...
        'StageF:StaticContactLinearizationStateInconsistent', ...
        'STATIC_CONTACT_LINEARIZATION_STATE_INCONSISTENT');
end
end

function test_planned_stage_f_api_surface(repository_root)
required = { ...
    'build_stage_f_rolling_element_linearization_mapping.m', ...
    'recover_stage_f_rolling_element_1X_loads.m', ...
    'evaluate_stage_f_nonlinear_bearing_force_scan.m', ...
    'compute_stage_f_bearing_force_harmonics.m', ...
    'validate_stage_f_results.m', ...
    'stage_f_canonical_transaction.m', ...
    'run_stage_f_recover_loads_and_evaluate_nonlinearity.m'};
for k = 1:numel(required)
    assert(isfile(fullfile(repository_root,required{k})), ...
        'StageF:MissingPlannedAPI','Missing planned Stage F API: %s',required{k});
end
end

function test_ball_hertz_derivative_matches_central_difference
delta = [2.0;3.5;5.0]*1e-6;
KH = 4.2e9;
fixture = ball_mapping_fixture(delta,KH,true(size(delta)));
before = fixture;
mapping = build_stage_f_rolling_element_linearization_mapping(fixture);
assert(isequaln(fixture,before),'StageF:MappingInputMutation', ...
    'Mapping builder must not mutate its input.');
h = 1e-6*min(delta);
central = (KH*(delta+h).^(3/2)-KH*(delta-h).^(3/2))/(2*h);
analytic = 1.5*KH*sqrt(delta);
assert(relative_error(central,analytic) <= 1e-9,'StageF:BallHertzDerivative', ...
    'Ball Hertz analytic derivative must match central difference.');
assert(relative_error(mapping.k_static_slice,analytic) <= 10*eps, ...
    'StageF:BallHertzDerivative','Stored ball tangent is inconsistent.');
assert(strcmp(mapping.contact_stiffness_source, ...
    'FORMAL_BALL_HERTZ_DERIVATIVE'));
end

function test_roller_slice_derivative_matches_central_difference
delta = reshape(linspace(1e-6,4e-6,270),30,9);
Kline = 2.5e8; coefficient = Kline/9;
fixture = rear_mapping_fixture(delta,Kline,true(30,9));
mapping = build_stage_f_rolling_element_linearization_mapping(fixture);
h = 1e-7*min(delta,[],'all');
central = (coefficient*(delta+h).^(10/9)- ...
    coefficient*(delta-h).^(10/9))/(2*h);
analytic = (10/9)*coefficient*delta.^(1/9);
assert(relative_error(central,analytic) <= 1e-8,'StageF:RollerSliceDerivative', ...
    'Roller slice analytic derivative must match central difference.');
assert(relative_error(mapping.k_static_slice,analytic) <= 10*eps, ...
    'StageF:RollerSliceDerivative','Stored roller-slice tangent is inconsistent.');
assert(strcmp(mapping.contact_stiffness_source, ...
    'FORMAL_ROLLER_SLICE_LAW_DERIVATIVE'));
end

function test_zero_qhat_has_zero_recovered_load_and_zero_nonlinearity
mapping = build_stage_f_rolling_element_linearization_mapping( ...
    force_consistent_ball_fixture());
recovered = recover_stage_f_rolling_element_1X_loads(mapping,zeros(5,1));
assert(all(recovered.slice_level.delta_hat_slice == 0,'all'));
assert(all(recovered.slice_level.Qhat_slice == 0,'all'));
assert(all(recovered.roller_level.Qhat_roller == 0,'all'));

request = scan_request(zeros(2,1),zeros(2,1),true(2,1));
scan = evaluate_stage_f_nonlinear_bearing_force_scan( ...
    request,@linear_scan_evaluator);
assert(scan.epsilon_NL == 0,'StageF:ZeroQhatNonlinearity', ...
    'Zero qhat must have zero nonlinear error.');
assert(all(scan.exact_dynamic_force_history == 0,'all'));
assert(all(scan.linear_dynamic_force_history == 0,'all'));
end

function test_statically_unloaded_slice_is_not_contact_loss
fixture = ball_mapping_fixture([2e-6;0;3e-6],4e9,[true;false;true]);
mapping = build_stage_f_rolling_element_linearization_mapping(fixture);
target_delta_hat = [0;1;0];
qhat = pinv(mapping.contact_jacobian_local)*target_delta_hat;
out = recover_stage_f_rolling_element_1X_loads(mapping,qhat);
assert(~out.slice_level.potential_contact_loss_slice(2));
assert(~out.slice_level.contact_loss_evaluated_slice(2));
assert(strcmp(out.slice_level.contact_loss_reason_slice{2}, ...
    'STATICALLY_UNLOADED_ELEMENT'));
assert(isnan(out.slice_level.r_delta_slice(2)));
assert(out.slice_level.Qhat_slice(2) == 0 && ...
    out.slice_level.Qmin_linear_slice(2) == 0);
end

function test_loaded_slice_contact_loss_is_detected
fixture = ball_mapping_fixture(2e-6,4e9,true);
mapping = build_stage_f_rolling_element_linearization_mapping(fixture);
target_delta_hat = 1.01*fixture.Q_static_slice/fixture.k_static_slice;
qhat = pinv(mapping.contact_jacobian_local)*target_delta_hat;
out = recover_stage_f_rolling_element_1X_loads(mapping,qhat);
assert(out.slice_level.Qmin_linear_slice <= 0);
assert(out.slice_level.potential_contact_loss_slice);
assert(out.slice_level.contact_loss_evaluated_slice);
assert(out.hard_gate.loaded_slice_contact_loss);
assert(strcmp(out.hard_gate.status, ...
    'REPRESENTATIVE_NONLINEAR_VALIDATION_REQUIRED'));
end

function test_r_delta_three_classes
ratios = [0.10 0.15 0.21];
expected = {'STRONG_LOCAL_LINEARITY','ACCEPTABLE_WITH_CAUTION', ...
    'LOCAL_LINEARITY_LIMIT_EXCEEDED'};
observed = cell(size(expected));
for k = 1:numel(ratios)
    fixture = ball_mapping_fixture(1,2,true);
    mapping = build_stage_f_rolling_element_linearization_mapping(fixture);
    qhat = pinv(mapping.contact_jacobian_local)*ratios(k);
    out = recover_stage_f_rolling_element_1X_loads(mapping,qhat);
    observed{k} = out.slice_level.linearity_class_slice{1};
    assert(abs(out.slice_level.r_delta_slice-ratios(k)) <= 10*eps);
end
assert(isequal(observed,expected));
end

function test_element_force_sum_matches_tangent_force
mapping = build_stage_f_rolling_element_linearization_mapping( ...
    force_consistent_ball_fixture());
qhat = [0.2+0.3i;-0.1+0.4i;0.05-0.2i;0;0];
out = recover_stage_f_rolling_element_1X_loads(mapping,qhat);
expected_elements = mapping.force_assembly_local* ...
    out.slice_level.Qhat_slice(:);
expected_analytic_tangent = mapping.K_b_local_analytic* ...
    (mapping.interface_transform*qhat);
assert(relative_error(expected_elements,expected_analytic_tangent) <= 1e-12);
assert(out.force_assembly_validation.element_vs_analytic_tangent_relative_error <= 1e-10, ...
    'StageF:RollingElementForceAssembly', ...
    'Contact-load assembly must match analytic A*diag(k)*J*u_rel within 1e-10.');
assert(out.force_assembly_validation.element_vs_analytic_tangent_threshold == 1e-10 && ...
    strcmp(out.force_assembly_validation.element_vs_analytic_tangent_gate_status, ...
    'ROLLING_ELEMENT_FORCE_ASSEMBLY_MATCHED'), ...
    'StageF:RollingElementForceAssembly', ...
    'Core assembly Gate must retain its fixed threshold and status.');
assert(out.force_assembly_validation.analytic_vs_saved_tangent_relative_error <= 1e-8 && ...
    out.force_assembly_validation.analytic_vs_saved_tangent_threshold == 1e-8 && ...
    strcmp(out.force_assembly_validation.analytic_vs_saved_tangent_gate_status, ...
    'ANALYTIC_VS_FD_TANGENT_AUDIT_PASSED'), ...
    'StageF:AnalyticVsFDTangentAudit', ...
    'Secondary tangent audit must retain its own fixed threshold and status.');
assert(isequaln(out.dynamic_resultant.local_complex,expected_elements));
end

function test_layered_tangent_gate_failures_are_distinct
fixture = force_consistent_ball_fixture();
saved_tangent_mismatch = fixture;
saved_tangent_mismatch.K_b_local(1,1) = ...
    saved_tangent_mismatch.K_b_local(1,1)+1e-4;
assert_gate_error(@() build_stage_f_rolling_element_linearization_mapping( ...
    saved_tangent_mismatch), ...
    'StageF:AnalyticVsFDTangentAuditFailed', ...
    'ANALYTIC_VS_FD_TANGENT_AUDIT_FAILED');

mapping = build_stage_f_rolling_element_linearization_mapping(fixture);
mapping.K_b_local_analytic(1,1) = mapping.K_b_local_analytic(1,1)+1;
assert_gate_error(@() recover_stage_f_rolling_element_1X_loads( ...
    mapping,[1;0;0;0;0]), ...
    'StageF:RollingElementForceAssemblyMismatch', ...
    'ROLLING_ELEMENT_FORCE_ASSEMBLY_MISMATCH');
end

function test_rear_30_by_9_slice_audit_and_complex_roller_aggregation
fixture = cancellation_rear_fixture();
mapping = build_stage_f_rolling_element_linearization_mapping(fixture);
tilt_amplitude = 12/max(abs(fixture.formal_geometry.slice_z_m));
out = recover_stage_f_rolling_element_1X_loads( ...
    mapping,[0;0;0;0;tilt_amplitude]);

assert(strcmp(out.slice_level.level_label,'SLICE_LEVEL'));
assert(isequal(size(out.slice_level.delta_hat_slice),[30 9]));
assert(isequal(size(out.slice_level.k_static_slice),[30 9]));
assert(isequal(size(out.slice_level.Qhat_slice),[30 9]));
assert(strcmp(out.roller_level.level_label,'ROLLER_LEVEL'));
assert(isequal(size(out.roller_level.Q_static_roller),[30 1]));
assert(isequal(size(out.roller_level.Qhat_roller),[30 1]));

expected_Qstatic = sum(fixture.Q_static_slice,2);
expected_Qhat = sum(out.slice_level.Qhat_slice,2);
assert(isequaln(out.roller_level.Q_static_roller,expected_Qstatic));
assert(isequaln(out.roller_level.Qhat_roller,expected_Qhat));
assert(isequaln(out.roller_level.Qmax_linear_roller, ...
    expected_Qstatic+abs(expected_Qhat)));
assert(isequaln(out.roller_level.Qmin_linear_roller, ...
    expected_Qstatic-abs(expected_Qhat)));

% Roller 1 has +12 and -12 slice phasors.  The complex roller phasor is
% zero; summing slice magnitudes or slice upper/lower bounds would be wrong.
assert(abs(out.roller_level.Qhat_roller(1)) <= 100*eps);
assert(abs(out.roller_level.Qmax_linear_roller(1)-expected_Qstatic(1)) <= ...
    100*eps(expected_Qstatic(1)));
assert(sum(abs(out.slice_level.Qhat_slice(1,:))) > 0);
assert(out.roller_level.Qmax_linear_roller(1) ~= ...
    sum(out.slice_level.Qmax_linear_slice(1,:)));
assert(out.roller_level.Qmin_linear_roller(1) ~= ...
    sum(out.slice_level.Qmin_linear_slice(1,:)));

loaded_ratios = abs(out.slice_level.delta_hat_slice(1,:))./ ...
    fixture.delta_static_slice(1,:);
assert(out.roller_level.r_delta_roller(1) == max(loaded_ratios));
assert(out.slice_level.potential_contact_loss_slice(1,1));
assert(~out.roller_level.potential_contact_loss_roller(1));
assert(out.hard_gate.loaded_slice_contact_loss, ...
    'StageF:LoadedSliceMustTriggerConservativeGate', ...
    'A loaded rear slice loss must conservatively trigger the hard Gate.');
assert(isfield(out.slice_level,'potential_contact_loss_slice') && ...
    isfield(out.roller_level,'potential_contact_loss_roller'));
assert(~any(contains(fieldnames(out.slice_level),'rolling_element')), ...
    'StageF:SliceMisnamedAsRollingElement', ...
    'Slice-level fields must not call slices rolling elements.');
end

function test_active_contact_set_change_is_detected
request = scan_request(zeros(2,1),[2;0],true(2,1));
scan = evaluate_stage_f_nonlinear_bearing_force_scan( ...
    request,@active_set_scan_evaluator);
assert(scan.active_set_changed);
assert(~isempty(scan.first_active_set_change));
assert(scan.first_active_set_change.phase_index >= 1 && ...
    scan.first_active_set_change.phase_index <= 32);
assert(any(scan.first_active_set_change.slice_indices == 1));
assert(isequal(size(scan.active_set_history),[2 32]));
end

function test_pure_1X_dft_has_no_2X_or_3X
theta = 2*pi*(0:31)/32;
F1 = [3-2i;-1+4i];
history = real(F1*exp(1i*theta));
audit = compute_stage_f_bearing_force_harmonics(history,theta);
assert(relative_error(audit.F1_complex,F1) <= 1e-12);
assert(audit.H2_H1 <= 1e-12 && audit.H3_H1 <= 1e-12);
end

function test_known_1X_2X_3X_dft_is_exact
theta = 2*pi*(0:31)/32;
F1 = [2+3i;-4+1i]; F2 = [0.5-0.25i;1+0.75i];
F3 = [-0.2+0.4i;0.3-0.1i];
history = real(F1*exp(1i*theta)+F2*exp(2i*theta)+ ...
    F3*exp(3i*theta));
audit = compute_stage_f_bearing_force_harmonics(history,theta);
assert(relative_error(audit.F1_complex,F1) <= 1e-12);
assert(relative_error(audit.F2_complex,F2) <= 1e-12);
assert(relative_error(audit.F3_complex,F3) <= 1e-12);
assert(abs(audit.H2_H1-norm(F2)/norm(F1)) <= 1e-12);
assert(abs(audit.H3_H1-norm(F3)/norm(F1)) <= 1e-12);
end

function test_epsilon_nonincrease_for_smaller_perturbation
large = scan_request(zeros(2,1),[0.20+0.1i;-0.15+0.05i],true(2,1));
small = large; small.qhat_full = 0.5*large.qhat_full;
large_scan = evaluate_stage_f_nonlinear_bearing_force_scan( ...
    large,@weakly_nonlinear_scan_evaluator);
small_scan = evaluate_stage_f_nonlinear_bearing_force_scan( ...
    small,@weakly_nonlinear_scan_evaluator);
assert(small_scan.epsilon_NL <= large_scan.epsilon_NL+100*eps, ...
    'StageF:SmallPerturbationMonotonicity', ...
    'epsilon_NL must not increase when the perturbation is reduced.');
end

function test_formal_preflight_rejects_tampered_frozen_parameters(repository_root)
% The formal evaluator must reject untrusted formal parameters before a
% nonlinear kernel call.  A request's flags are not a substitute for
% reconstructing the frozen thermal/bearing configuration.
request = formal_preflight_request_fixture(repository_root);
evaluator = request.evaluator;
request = rmfield(request,'evaluator');
mutations = { ...
    @tamper_formal_microphysics_thermal_state; ...
    @tamper_formal_microphysics_thermal_mode; ...
    @tamper_formal_microphysics_thermal_enable; ...
    @tamper_formal_bearing_configuration};
for k = 1:numel(mutations)
    tampered = mutations{k}(request);
    assert_gate_error(@() evaluate_stage_f_nonlinear_bearing_force_scan( ...
        tampered,evaluator), ...
        'StageF:FormalFrozenParameterMismatch', ...
        'FORMAL_FROZEN_PARAMETER_MISMATCH');
end
end

function test_radial_resultant_uses_phase_accurate_peak_and_rms
% Orthogonal component phasors never peak simultaneously.  Fx=1 and Fy=i
% therefore has a radial peak and RMS of one, not sqrt(2) and one/sqrt(2).
fixture = ball_mapping_fixture(ones(4,1),2/3,true(4,1));
mapping = build_stage_f_rolling_element_linearization_mapping(fixture);
lever = fixture.formal_geometry.axial_lever_m;
target = [1;1i;0;-lever*1i;lever];
qhat = pinv(mapping.K_b_local_analytic)*target;
recovered = recover_stage_f_rolling_element_1X_loads(mapping,qhat);
resultant = recovered.dynamic_resultant;
assert(abs(resultant.Fx_hat-1) <= 1e-10 && abs(resultant.Fy_hat-1i) <= 1e-10);
assert(abs(resultant.radial_force_peak-1) <= 1e-10, ...
    'StageF:RadialPeakPhaseAccuracy', ...
    'Fx=1 and Fy=i must have radial peak one rather than sqrt(2).');
assert(abs(resultant.radial_force_rms-1) <= 1e-10, ...
    'StageF:RadialRmsPhaseAccuracy', ...
    'Fx=1 and Fy=i must have radial RMS one.');
end

function test_precommit_candidate_validation_and_simple_runner_contract(repository_root)
% The 600-second hard stop belongs to the outer formal-run command, not to
% Stage F MATLAB production code.  The transaction validates its temporary
% candidate in a fresh MATLAB before the single atomic replacement; the
% runner performs no fallible post-commit reload or second validation.
transaction = fileread(fullfile(repository_root,'stage_f_canonical_transaction.m'));
runner = fileread(fullfile(repository_root, ...
    'run_stage_f_recover_loads_and_evaluate_nonlinearity.m'));
validate_at = strfind(transaction,'run_independent_readonly_validation(temporary_path)');
cas_at = strfind(transaction,"file_sha256(canonical_path),expected_canonical_sha256");
replace_at = strfind(transaction,'atomic_replace(temporary_path,canonical_path)');
assert(isscalar(validate_at) && ~isempty(cas_at) && isscalar(replace_at) && ...
    validate_at < cas_at(end) && cas_at(end) < replace_at, ...
    'StageF:PrecommitCandidateValidationOrder', ...
    'Candidate validation and final CAS must precede the only atomic replacement.');
after_replace = extractAfter(transaction,replace_at);
assert(~contains(after_replace,'run_independent_readonly_validation'), ...
    'StageF:PostcommitValidationForbidden', ...
    'Transaction must not run a second fallible validator after atomic replacement.');
assert(isempty(regexp(runner,'\<toc\s*\(|ProcessBuilder|destroyForcibly|enforce_runtime','once')), ...
    'StageF:InternalWatchdogForbidden', ...
    'The hard timeout is owned by the outer formal-run command, not the MATLAB runner.');
save_at = strfind(runner,"stage_f_canonical_transaction('save'");
assert(isscalar(save_at) && ...
    isempty(regexp(extractAfter(runner,save_at), ...
    "\<load\s*\(|validate_stage_f_results\s*\(|stage_f_canonical_transaction\('audit'", ...
    'once')), ...
    'StageF:PostcommitRunnerWorkForbidden', ...
    'Runner must return immediately after the single transaction save.');
end

function test_scan_is_nominal_only_and_exactly_32_phases
request = scan_request(zeros(2,1),zeros(2,1),true(2,1));
request.scenario = 'LOW';
assert_error(@() evaluate_stage_f_nonlinear_bearing_force_scan( ...
    request,@must_not_be_called), 'StageF:NonNominalNonlinearScan');
request.scenario = 'NOMINAL'; request.phase_count = 31;
assert_error(@() evaluate_stage_f_nonlinear_bearing_force_scan( ...
    request,@must_not_be_called), 'StageF:PhaseCount');
request.phase_count = 32;
scan = evaluate_stage_f_nonlinear_bearing_force_scan( ...
    request,@linear_scan_evaluator);
assert(scan.phase_count == 32 && numel(scan.phase_values_rad) == 32);
assert(max(abs(scan.phase_values_rad-2*pi*(0:31)/32)) <= 10*eps);
end

function test_validator_recomputes_and_detects_tampering
[baseline,candidate] = mock_stage_f_artifact(false);
validation = validate_stage_f_results(candidate,baseline);
assert(validation.passed,'StageF:ValidSyntheticArtifactRejected', ...
    'Validator rejected the internally consistent synthetic artifact.');
cases = { ...
    @tamper_delta_hat,'DELTA_HAT'; ...
    @tamper_Qhat,'QHAT'; ...
    @tamper_roller_Qhat,'QHAT_ROLLER'; ...
    @tamper_Qstatic_roller,'QSTATIC_ROLLER'; ...
    @tamper_Qmax_slice,'QMAX_SLICE'; ...
    @tamper_Qmax,'QMAX'; ...
    @tamper_Qmin,'QMIN'; ...
    @tamper_Qmin_roller,'QMIN_ROLLER'; ...
    @tamper_r_delta,'R_DELTA'; ...
    @tamper_r_delta_roller,'R_DELTA_ROLLER'; ...
    @tamper_linearity_class,'LINEARITY_CLASS'; ...
    @tamper_contact_loss,'CONTACT_LOSS'; ...
    @tamper_contact_loss_roller,'CONTACT_LOSS_ROLLER'; ...
    @tamper_resultant,'RESULTANT'; ...
    @tamper_assembly_core_error,'ASSEMBLY_CORE'; ...
    @tamper_assembly_core_threshold,'ASSEMBLY_CORE_THRESHOLD'; ...
    @tamper_assembly_core_status,'ASSEMBLY_CORE_STATUS'; ...
    @tamper_analytic_vs_saved_tangent_error,'TANGENT_AUDIT'; ...
    @tamper_analytic_vs_saved_tangent_threshold,'TANGENT_AUDIT_THRESHOLD'; ...
    @tamper_analytic_vs_saved_tangent_status,'TANGENT_AUDIT_STATUS'; ...
    @tamper_epsilon_NL,'EPSILON_NL'; ...
    @tamper_maximum_r_delta_nominal,'MAXIMUM_R_DELTA_NOMINAL'; ...
    @tamper_potential_contact_loss_nominal,'POTENTIAL_CONTACT_LOSS_NOMINAL'; ...
    @tamper_front_bearing_gate_status,'BEARING_GATE_STATUS'; ...
    @tamper_rear_bearing_gate_status,'BEARING_GATE_STATUS'; ...
    @tamper_active_set_history,'ACTIVE_SET'; ...
    @tamper_active_set_changed,'ACTIVE_SET'; ...
    @tamper_first_active_set_change,'FIRST_ACTIVE_SET_CHANGE'; ...
    @tamper_phase_values_rad,'PHASE_VALUES_RAD'; ...
    @tamper_exact_dynamic_force_history,'EXACT_DYNAMIC_FORCE_HISTORY'; ...
    @tamper_linear_dynamic_force_history,'LINEAR_DYNAMIC_FORCE_HISTORY'; ...
    @tamper_F1,'F1'; ...
    @tamper_F2,'F2'; ...
    @tamper_F3,'F3'; ...
    @tamper_H2,'H2'; ...
    @tamper_H3,'H3'; ...
    @tamper_scan_extra_low,'SCAN_SCHEDULE'; ...
    @tamper_scan_missing,'SCAN_SCHEDULE'; ...
    @tamper_scan_duplicate_replacing_required_case,'SCAN_SCHEDULE'; ...
    @tamper_scan_phase_count,'SCAN_SCHEDULE'; ...
    @tamper_scan_evaluator,'EVALUATOR'; ...
    @tamper_final_gate,'FINAL_GATE'; ...
    @tamper_decision_gate,'FINAL_GATE'; ...
    @tamper_saved_validation_gate,'FINAL_GATE'; ...
    @tamper_stage_e_validation,'FROZEN_VALIDATION'; ...
    @delete_stage_d_validation,'FROZEN_VALIDATION'; ...
    @tamper_stage_d_static,'FROZEN_STATIC'; ...
    @delete_stage_d_static,'FROZEN_STATIC'; ...
    @tamper_stage_c_damping,'FROZEN_DAMPING'; ...
    @delete_stage_c_damping,'FROZEN_DAMPING'; ...
    @tamper_stage_e_response,'FROZEN_RESPONSE'; ...
    @delete_stage_e_response,'FROZEN_RESPONSE'; ...
    @tamper_frozen_stage,'FROZEN'};
for k = 1:size(cases,1)
    tampered = cases{k,1}(candidate);
    v = validate_stage_f_results(tampered,baseline);
    assert(~v.passed && diagnostics_contain(v,cases{k,2}), ...
        'StageF:ValidatorDerivedFieldTamper', ...
        'Validator did not independently reject tampered %s.',cases{k,2});
end
end

function test_validator_rejects_coordinated_scan_tampering
% Every value below remains internally consistent if the validator accepts
% the candidate's own scan payload as authority.  Formal validation must
% instead reconstruct from the frozen baseline mapping, qhat and evaluator.
[baseline,candidate] = mock_stage_f_artifact(false);
collusions = { ...
    @collude_linear_force_history_epsilon_and_gate,'INDEPENDENT_SCAN_REBUILD'; ...
    @collude_static_reference_and_exact_history,'INDEPENDENT_SCAN_REBUILD'; ...
    @collude_active_history_change_first_and_gate,'INDEPENDENT_SCAN_REBUILD'};
for k = 1:size(collusions,1)
    tampered = collusions{k,1}(candidate);
    validation = validate_stage_f_results(tampered,baseline);
    assert(~validation.passed && diagnostics_contain(validation,collusions{k,2}), ...
        'StageF:ValidatorCoordinatedScanTamper', ...
        'Validator trusted a coordinated candidate scan mutation.');
end
end

function test_validator_rejects_complete_recovered_load_tampering
% These fields are derived output, not trusted evidence.  In particular the
% validator must cover static/mask inputs, complex representations, loss
% rationale, roller class, and every saved resultant representation.
[baseline,candidate] = mock_stage_f_artifact(false);
cases = { ...
    @tamper_Qstatic_slice,'QSTATIC_SLICE'; ...
    @tamper_delta_static_slice,'DELTA_STATIC_SLICE'; ...
    @tamper_k_static_slice,'K_STATIC_SLICE'; ...
    @tamper_loaded_mask_slice,'LOADED_MASK_SLICE'; ...
    @tamper_loaded_mask_roller,'LOADED_MASK_ROLLER'; ...
    @tamper_delta_hat_real,'DELTA_HAT_COMPONENTS'; ...
    @tamper_delta_hat_imag,'DELTA_HAT_COMPONENTS'; ...
    @tamper_delta_hat_magnitude,'DELTA_HAT_COMPONENTS'; ...
    @tamper_delta_hat_phase,'DELTA_HAT_COMPONENTS'; ...
    @tamper_Qhat_real,'QHAT_COMPONENTS'; ...
    @tamper_Qhat_imag,'QHAT_COMPONENTS'; ...
    @tamper_Qhat_magnitude,'QHAT_COMPONENTS'; ...
    @tamper_Qhat_phase,'QHAT_COMPONENTS'; ...
    @tamper_contact_loss_evaluated,'CONTACT_LOSS_EVALUATED'; ...
    @tamper_contact_loss_reason,'CONTACT_LOSS_REASON'; ...
    @tamper_roller_linearity_class,'LINEARITY_CLASS_ROLLER'; ...
    @tamper_global_resultant,'RESULTANT_GLOBAL'; ...
    @tamper_resultant_fx,'FX_HAT'; ...
    @tamper_resultant_fy,'FY_HAT'; ...
    @tamper_radial_peak,'RADIAL_PEAK'; ...
    @tamper_radial_rms,'RADIAL_RMS'};
for k = 1:size(cases,1)
    validation = validate_stage_f_results(cases{k,1}(candidate),baseline);
    assert(~validation.passed && diagnostics_contain(validation,cases{k,2}), ...
        'StageF:ValidatorRecoveredLoadTamper', ...
        'Validator did not independently reject recovered-load tamper %s.', ...
        cases{k,2});
end
end

function test_validator_private_harmonic_provenance_audit
repository_root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
validator = fileread(fullfile(repository_root,'validate_stage_f_results.m'));
assert(contains(validator,'independently_compute_stage_f_harmonics') && ...
    isempty(regexp(validator, ...
    '\<compute_stage_f_bearing_force_harmonics\s*\(','once')), ...
    'StageF:PrivateHarmonicRecomputation', ...
    'Validator must own its DFT equations rather than call the production harmonic helper.');

[baseline,candidate] = mock_stage_f_artifact(false);
candidate = enrich_stage_f_audit_fixture(candidate,baseline);
assert(validate_stage_f_results(candidate,baseline).passed, ...
    'StageF:EnrichedFixtureRejected','Enriched audit fixture must be valid before tampering.');
tampered = candidate;
tampered.nonlinearity_gate.T20.front.F2_phase_rad(1) = ...
    tampered.nonlinearity_gate.T20.front.F2_phase_rad(1)+0.1;
validation = validate_stage_f_results(tampered,baseline);
assert(~validation.passed && diagnostics_contain(validation,'HARMONIC_AUDIT'), ...
    'StageF:HarmonicAuditTamper', ...
    'Validator must reject F1/F2/F3 norm, phase, and semantic-label tampering.');

tampered = candidate;
tampered.nonlinearity_gate.T20.front.thermal_state_frozen = false;
validation = validate_stage_f_results(tampered,baseline);
assert(~validation.passed && diagnostics_contain(validation,'THERMAL_FROZEN'), ...
    'StageF:ThermalFreezeAudit','Validator must verify each saved scan is frozen.');

tampered = candidate;
tampered.bearing_contact.T20.LOW.rear.force_assembly_validation.analytic_definition_relative_error = 1e-2;
validation = validate_stage_f_results(tampered,baseline);
assert(~validation.passed && diagnostics_contain(validation,'FORCE_AUDIT'), ...
    'StageF:TwoLayerForceAudit','Validator must verify every saved force-assembly audit field.');
end

function test_required_source_provenance_cannot_be_deleted
[baseline,candidate] = mock_stage_f_artifact(false);
candidate = enrich_stage_f_audit_fixture(candidate,baseline);
assert(validate_stage_f_results(candidate,baseline).passed, ...
    'StageF:ProvenanceFixtureRejected', ...
    'Complete minimal source provenance must validate before tampering.');
for field = {'static_state_source_commit','stage_e_response_source_commit', ...
        'stage_f_source_commit'}
    tampered = candidate;
    item = tampered.bearing_contact.T20.LOW.front;
    item = rmfield(item,field{1});
    tampered.bearing_contact.T20.LOW.front = item;
    validation = validate_stage_f_results(tampered,baseline);
    assert(~validation.passed && diagnostics_contain(validation,'SOURCE_COMMIT'), ...
        'StageF:RequiredSourceProvenance', ...
        'Deleting required provenance %s must fail.',field{1});
end
tampered = candidate;
tampered.stage_f_execution_audit = rmfield( ...
    tampered.stage_f_execution_audit,{'source_commit','elapsed_seconds'});
validation = validate_stage_f_results(tampered,baseline);
assert(~validation.passed && diagnostics_contain(validation,'SOURCE_COMMIT'), ...
    'StageF:ProvenanceDowngrade', ...
    'Deleting the formal audit provenance must not downgrade validation.');
end

function test_required_harmonic_force_and_thermal_contracts
[baseline,candidate] = mock_stage_f_artifact(false);
candidate = enrich_stage_f_audit_fixture(candidate,baseline);
assert(validate_stage_f_results(candidate,baseline).passed, ...
    'StageF:ScientificAuditFixtureRejected', ...
    'Complete harmonic, force and thermal audit must validate before tampering.');
tampered = candidate;
scan = tampered.nonlinearity_gate.T20.front;
scan = rmfield(scan,{'F1_norm','F2_norm','F3_norm','F1_phase_rad', ...
    'F2_phase_rad','F3_phase_rad','F1_force_semantics', ...
    'F2_force_semantics','F3_force_semantics','diagnostic_label'});
tampered.nonlinearity_gate.T20.front = scan;
validation = validate_stage_f_results(tampered,baseline);
assert(~validation.passed && diagnostics_contain(validation,'HARMONIC_AUDIT'), ...
    'StageF:RequiredHarmonicAudit', ...
    'F1X--F3X representations and force semantics must be mandatory.');

tampered = candidate;
tampered.nonlinearity_gate.T20.front.thermal_frozen_summary.after.viscosity_Pa_s = ...
    2*tampered.nonlinearity_gate.T20.front.thermal_frozen_summary.after.viscosity_Pa_s;
validation = validate_stage_f_results(tampered,baseline);
assert(~validation.passed && diagnostics_contain(validation,'THERMAL_FROZEN'), ...
    'StageF:ThermalFrozenSummary', ...
    'Changing a frozen thermal summary across the 32-phase scan must fail.');

tampered = candidate;
tampered.bearing_contact.T20.LOW.front.contact_force_resultant.local_complex(1) = ...
    tampered.bearing_contact.T20.LOW.front.contact_force_resultant.local_complex(1)+1;
validation = validate_stage_f_results(tampered,baseline);
assert(~validation.passed && diagnostics_contain(validation,'CONTACT_FORCE_RESULTANT'), ...
    'StageF:ContactForceSemantics', ...
    'The +A*Q contact force and -A*Q rotor reaction must be checked separately.');
end

function test_first_trigger_precise_independent_rebuild
[baseline,candidate] = mock_stage_f_artifact(true);
candidate = enrich_stage_f_audit_fixture(candidate,baseline);
candidate.progress.stage_f_first_trigger = enrich_first_trigger( ...
    candidate.progress.stage_f_first_trigger,candidate);
assert(validate_stage_f_results(candidate,baseline).passed, ...
    'StageF:FirstTriggerFixtureRejected','Complete first-trigger fixture must validate before tampering.');
tampered = candidate;
tampered.progress.stage_f_first_trigger.phase_index = 31;
tampered.progress.stage_f_first_trigger.phase_rad = pi;
tampered.progress.stage_f_first_trigger.slice_indices = [9 8];
tampered.progress.stage_f_first_trigger.trigger_value = 1;
tampered.progress.stage_f_first_trigger.threshold = 0.2;
validation = validate_stage_f_results(tampered,baseline);
assert(~validation.passed && diagnostics_contain(validation,'FIRST_TRIGGER'), ...
    'StageF:FirstTriggerIndependentRebuild', ...
    'First trigger must be fully and independently recomputed, including phase, indices, value and threshold.');
end

function test_mandatory_trigger_and_rear_index_decode(repository_root)
[baseline,candidate] = mock_stage_f_artifact(true);
candidate = enrich_stage_f_audit_fixture(candidate,baseline);
candidate.progress.stage_f_first_trigger = enrich_first_trigger( ...
    candidate.progress.stage_f_first_trigger,candidate);
candidate.progress.stage_f_first_trigger.temperature = 20;
candidate.progress.stage_f_first_trigger.phase = NaN;
candidate.progress.stage_f_first_trigger.roller_id = 1;
candidate.progress.stage_f_first_trigger.slice_id = 1;
assert(validate_stage_f_results(candidate,baseline).passed, ...
    'StageF:MandatoryTriggerFixtureRejected', ...
    'Complete first-trigger fields must validate before deletion.');
tampered = candidate;
tampered.progress.stage_f_first_trigger = rmfield( ...
    tampered.progress.stage_f_first_trigger, ...
    {'phase_index','phase_rad','slice_indices','trigger_value','threshold', ...
    'temperature','phase','roller_id','slice_id'});
validation = validate_stage_f_results(tampered,baseline);
assert(~validation.passed && diagnostics_contain(validation,'FIRST_TRIGGER'), ...
    'StageF:MandatoryFirstTriggerFields', ...
    'Deleting precise first-trigger fields must fail.');

runner = fileread(fullfile(repository_root, ...
    'run_stage_f_recover_loads_and_evaluate_nonlinearity.m'));
validator = fileread(fullfile(repository_root,'validate_stage_f_results.m'));
for source = {runner,validator}
    assert(contains(source{1},'decode_active_contact_indices') && ...
        contains(source{1},'[30 9]') && contains(source{1},'[22 1]'), ...
        'StageF:RearActiveIndexDecode', ...
        'Active-set linear indices must decode to rear 30x9 or front 22x1 roller/slice IDs.');
end
end

function test_contact_force_sign_semantics(repository_root)
recovery = fileread(fullfile(repository_root, ...
    'recover_stage_f_rolling_element_1X_loads.m'));
harmonics = fileread(fullfile(repository_root, ...
    'compute_stage_f_bearing_force_harmonics.m'));
for required = {'contact_load_resultant','bearing_force_on_rotor', ...
        'CONTACT_LOAD_PLUS_AQ','FORCE_ON_ROTOR_MINUS_AQ'}
    assert(contains(recovery,required{1}), ...
        'StageF:ForceSignSemantics','Recovery must declare %s.',required{1});
end
for required = {'F1_force_semantics','F2_force_semantics', ...
        'FORCE_ON_ROTOR_MINUS_AQ'}
    assert(contains(harmonics,required{1}), ...
        'StageF:HarmonicForceSignSemantics', ...
        'Harmonic output must declare %s so F1/F2 phase has unambiguous sign.',required{1});
end
end

function test_stage_f_schema_and_forbidden_payloads
[baseline,candidate] = mock_stage_f_artifact(false);
assert(isequaln(stage_f_projection(candidate),baseline), ...
    'StageF:StageAToEProjectionChanged','Stage A--E projection changed.');
for temperature = [20 50 80 100]
    tf = sprintf('T%d',temperature);
    for scenario = {'LOW','NOMINAL','HIGH'}
        sf = scenario{1};
        assert(isfield(candidate.bearing_contact.(tf).(sf),'front'));
        for bearing = {'front','rear'}
            recovered = candidate.bearing_contact.(tf).(sf).(bearing{1});
            assert(isfield(recovered.mapping_snapshot,'loaded_mask_static_slice') && ...
                isfield(recovered.mapping_snapshot,'loaded_mask_static_roller'));
            audit = recovered.force_assembly_validation;
            assert(isfinite(audit.element_vs_analytic_tangent_relative_error) && ...
                isfinite(audit.analytic_vs_saved_tangent_relative_error) && ...
                audit.element_vs_analytic_tangent_threshold == 1e-10 && ...
                audit.analytic_vs_saved_tangent_threshold == 1e-8, ...
                'StageF:LayeredAssemblyAuditSchema', ...
                'Every 4T x 3-scenario recovered load must save both fixed-threshold audits.');
        end
        rear = candidate.bearing_contact.(tf).(sf).rear;
        assert(isequal(size(rear.slice_level.delta_hat_slice),[30 9]));
        assert(isequal(size(rear.slice_level.k_static_slice),[30 9]));
        assert(isequal(size(rear.slice_level.Qhat_slice),[30 9]));
        assert(isequaln(rear.roller_level.loaded_mask_static_roller, ...
            any(rear.slice_level.loaded_mask_static_slice,2)));
    end
    assert(isequal(sort(fieldnames(candidate.nonlinearity_gate.(tf))), ...
        {'front';'rear'}));
    assert(strcmp(candidate.nonlinearity_gate.(tf).front.scenario,'NOMINAL'));
    assert(candidate.nonlinearity_gate.(tf).front.phase_count == 32);
    assert(isfield(candidate.nonlinearity_gate.(tf).front,'maximum_r_delta_nominal') && ...
        isfield(candidate.nonlinearity_gate.(tf).front,'potential_contact_loss_nominal') && ...
        isfield(candidate.nonlinearity_gate.(tf).front,'gate_status'));
    assert(strcmp(candidate.nonlinearity_gate.(tf).rear.scenario,'NOMINAL'));
    assert(candidate.nonlinearity_gate.(tf).rear.phase_count == 32);
    assert(isfield(candidate.nonlinearity_gate.(tf).rear,'maximum_r_delta_nominal') && ...
        isfield(candidate.nonlinearity_gate.(tf).rear,'potential_contact_loss_nominal') && ...
        isfield(candidate.nonlinearity_gate.(tf).rear,'gate_status'));
end
assert(isequal(sort(fieldnames(candidate.nonlinearity_gate)), ...
    {'T100';'T20';'T50';'T80'}));
manifest = candidate.stage_f_execution_audit.scan_requests;
assert(numel(manifest) == 8 && ...
    candidate.stage_f_execution_audit.scan_request_count == 8);
observed = arrayfun(@(x) sprintf('T%d_%s_%s_%d', ...
    x.temperature_case_C,x.bearing,x.scenario,x.phase_count), ...
    manifest,'UniformOutput',false);
expected = {'T20_front_NOMINAL_32','T20_rear_NOMINAL_32', ...
    'T50_front_NOMINAL_32','T50_rear_NOMINAL_32', ...
    'T80_front_NOMINAL_32','T80_rear_NOMINAL_32', ...
    'T100_front_NOMINAL_32','T100_rear_NOMINAL_32'};
assert(isequal(sort(observed),sort(expected)));
assert(all([manifest.thermal_state_frozen]) && ...
    all(strcmp({manifest.evaluator_function},'nonlinear_bearing_force')) && ...
    candidate.stage_f_execution_audit.LOW_scans == 0 && ...
    candidate.stage_f_execution_audit.HIGH_scans == 0);
required_progress = {'stage_f_load_recovery_complete', ...
    'stage_f_nonlinear_scan_complete','stage_f_complete', ...
    'stage_f_gate_status','stage_f_first_trigger','allow_stage_g', ...
    'last_completed_gate'};
assert(all(isfield(candidate.progress,required_progress)));
assert(~recursive_field_present(candidate,'q_m'));
assert(~recursive_field_present(candidate,'newmark'));

forbidden = candidate; forbidden.nonlinearity_gate.T20.front.q_m = zeros(2,32);
assert(~validate_stage_f_results(forbidden,baseline).passed);
forbidden = candidate; forbidden.newmark_history = zeros(2,32);
assert(~validate_stage_f_results(forbidden,baseline).passed);
end

function test_scientific_save_and_computation_failure_atomicity
[baseline,scientific] = mock_stage_f_artifact(true);
validation = validate_stage_f_results(scientific,baseline);
assert(validation.passed && strcmp(validation.gate_status, ...
    'REPRESENTATIVE_NONLINEAR_VALIDATION_REQUIRED'));

temporary_directory = tempname; mkdir(temporary_directory);
cleanup = onCleanup(@() rmdir(temporary_directory,'s'));
canonical = fullfile(temporary_directory, ...
    'thermal_4T_frequency_domain_results.mat');
results = baseline; save(canonical,'results');
expected_hash = file_sha256(canonical);
assert(isequal(directory_file_names(temporary_directory), ...
    {'thermal_4T_frequency_domain_results.mat'}));

% A transaction MAT can pre-exist as another process's state.  Stage F must
% fail closed and must not delete or overwrite it during its own cleanup.
foreign_transaction = [canonical '.stage_f_transaction.mat'];
foreign_owner = struct('owner','other_process','nonce',uint64(17));
save(foreign_transaction,'foreign_owner');
foreign_bytes = read_file_bytes(foreign_transaction);
assert_gate_error(@() stage_f_canonical_transaction( ...
    'save',canonical,scientific,expected_hash), ...
    'StageF:CanonicalResultArtifactConflict','STAGE_F_COMPUTATION_FAILED');
assert(isfile(foreign_transaction) && ...
    isequal(read_file_bytes(foreign_transaction),foreign_bytes), ...
    'StageF:ForeignTransactionPreserved', ...
    'A pre-existing transaction MAT belongs to another process and must remain byte-identical.');
delete(foreign_transaction); % Test-owned fixture cleanup after the assertion.

saved = stage_f_canonical_transaction('save',canonical,scientific,expected_hash);
assert(strcmp(saved.decision.status, ...
    'REPRESENTATIVE_NONLINEAR_VALIDATION_REQUIRED'));
assert(saved.progress.stage_f_complete && ~saved.progress.allow_stage_g);
assert(~isfile([canonical '.stage_f_transaction.mat']));
assert(isequal(directory_file_names(temporary_directory), ...
    {'thermal_4T_frequency_domain_results.mat'}));
audit = stage_f_canonical_transaction('audit',canonical);
assert(audit.passed && isequal(audit.observed_files, ...
    {'thermal_4T_frequency_domain_results.mat'}));

% A semantically valid candidate must still lose CAS when the canonical
% bytes changed after its expected hash was captured.
results = baseline; save(canonical,'results');
stale_hash = file_sha256(canonical);
results.concurrent_writer_marker = uint64(1); save(canonical,'results');
concurrent_bytes = read_file_bytes(canonical);
assert_gate_error(@() stage_f_canonical_transaction( ...
    'save',canonical,scientific,stale_hash), ...
    'StageF:CanonicalResultArtifactConflict','STAGE_F_COMPUTATION_FAILED');
assert(isequal(read_file_bytes(canonical),concurrent_bytes));
assert(~isfile([canonical '.stage_f_transaction.mat']));
assert(isequal(directory_file_names(temporary_directory), ...
    {'thermal_4T_frequency_domain_results.mat'}));

% An exclusive lock conflict is rejected before any transaction MAT exists.
results = baseline; save(canonical,'results');
locked_hash = file_sha256(canonical); locked_bytes = read_file_bytes(canonical);
[channel,file_lock] = acquire_test_lock(canonical);
lock_cleanup = onCleanup(@() release_test_lock(file_lock,channel));
assert_gate_error(@() stage_f_canonical_transaction( ...
    'save',canonical,scientific,locked_hash), ...
    'StageF:CanonicalResultArtifactConflict','STAGE_F_COMPUTATION_FAILED');
assert(isequal(read_file_bytes(canonical),locked_bytes));
assert(~isfile([canonical '.stage_f_transaction.mat']));
file_lock.release(); channel.close(); clear lock_cleanup;

results = baseline; save(canonical,'results');
bytes_before = read_file_bytes(canonical);
expected_hash = file_sha256(canonical);
invalid = scientific;
invalid.bearing_contact.T20.LOW.front.slice_level.Qhat_slice(1) = NaN;
assert_error(@() stage_f_canonical_transaction( ...
    'save',canonical,invalid,expected_hash), ...
    'StageF:ComputationFailed');
assert(isequal(read_file_bytes(canonical),bytes_before), ...
    'StageF:ComputationFailureCanonicalMutation', ...
    'Computation failure changed canonical bytes.');
assert(~isfile([canonical '.stage_f_transaction.mat']), ...
    'StageF:ComputationFailureTempLeak', ...
    'Computation failure left a transaction file.');
assert(isequal(directory_file_names(temporary_directory), ...
    {'thermal_4T_frequency_domain_results.mat'}));
end

function test_runner_source_and_change_scope_gate(repository_root)
runner_path = fullfile(repository_root, ...
    'run_stage_f_recover_loads_and_evaluate_nonlinearity.m');
runner = fileread(runner_path);
parent_commit = '7be725a500f4ae3960d5af8e076b98e150fb1dce';
subject = 'feat: add rolling-element load recovery and nonlinear validity gate';
assert(contains(runner,parent_commit) && contains(runner,subject), ...
    'StageF:RunnerCommitGate','Runner commit gate is incomplete.');
expected = { ...
    'bearing_microphysics/validation/test_stage_f_rolling_element_loads_and_nonlinearity_gate.m', ...
    'build_stage_f_rolling_element_linearization_mapping.m', ...
    'recover_stage_f_rolling_element_1X_loads.m', ...
    'evaluate_stage_f_nonlinear_bearing_force_scan.m', ...
    'compute_stage_f_bearing_force_harmonics.m', ...
    'validate_stage_f_results.m', ...
    'stage_f_canonical_transaction.m', ...
    'run_stage_f_recover_loads_and_evaluate_nonlinearity.m'};
whitelist_block = regexp(runner, ...
    '(?s)stage_f_change_scope_expected_files\s*=\s*\{(.*?)\};', ...
    'tokens','once');
assert(~isempty(whitelist_block), ...
    'StageF:SourceWhitelist', ...
    'Runner must define an explicit stage_f_change_scope_expected_files constant.');
whitelist_tokens = regexp(whitelist_block{1},'''([^'']+\.m)''','tokens');
whitelist = cellfun(@(x) x{1},whitelist_tokens,'UniformOutput',false);
assert(isequal(sort(whitelist),sort(expected)), ...
    'StageF:SourceWhitelist','Runner source whitelist is not exact.');
assert(~isempty(regexp(runner, ...
    ['git diff --name-only ' parent_commit '\.\.HEAD'],'once')));
assert(contains(runner,'git status --porcelain=v1'));
assert(contains(runner,'git log -1 --format=%P HEAD'));
assert(contains(runner,'git log -1 --format=%s HEAD'));
assert(contains(runner,'isequal(sort(changed),sort(stage_f_change_scope_expected_files))'), ...
    'StageF:SourceWhitelist', ...
    'Runner must compare changed files against its explicit whitelist constant.');
assert(contains(runner,'3f0e6e3b379cda670ef173682c58d17a19fc6b30f761ed59a00f3c203793abfd'));
assert(contains(runner,'0cab3dc49dfffc4192066402cb70302f084cb45d'));
assert(contains(runner,'stage_e_complete'));
assert(contains(runner,'FOUR_TEMPERATURE_1X_RESPONSES_ACCEPTED'));
assert(contains(runner,'NOMINAL') && contains(runner,'32'));
assert(contains(runner,'stage_f_canonical_transaction'));
assert(contains(runner,'build_stage_f_rolling_element_linearization_mapping'));
assert(contains(runner,'evaluate_stage_f_nonlinear_bearing_force_scan'));
assert(contains(runner,'stage_f_execution_audit') && ...
    contains(runner,'scan_requests'));
assert(contains(runner,'nonlinear_bearing_force') && ...
    contains(runner,'thermal_state_frozen'));
assert(isempty(regexp(runner,'@\s*linear_scan_evaluator','once')), ...
    'StageF:RunnerEvaluatorProvenance', ...
    'Formal runner must not substitute a linear scan evaluator.');

for forbidden = {'\<newmark\w*\s*\(','run_stage_e_build_four_temperature_1X_responses\s*\(', ...
        'solve_stage_e_full_order_1X_case\s*\(','run_stage_d_four_temperature_static_states\s*\(', ...
        'writetable\s*\(','writecell\s*\(','diary\s*\('}
    assert(isempty(regexpi(runner,forbidden{1},'once')), ...
        'StageF:ForbiddenRunnerSource','Forbidden runner construct: %s',forbidden{1});
end
end

function test_all_stage_f_sources_respect_scope(repository_root)
production = { ...
    'build_stage_f_rolling_element_linearization_mapping.m', ...
    'recover_stage_f_rolling_element_1X_loads.m', ...
    'evaluate_stage_f_nonlinear_bearing_force_scan.m', ...
    'compute_stage_f_bearing_force_harmonics.m', ...
    'validate_stage_f_results.m', ...
    'stage_f_canonical_transaction.m', ...
    'run_stage_f_recover_loads_and_evaluate_nonlinearity.m'};
for k = 1:numel(production)
    source = fileread(fullfile(repository_root,production{k}));
    for forbidden = {'\<newmark\w*\s*\(', ...
            'run_stage_e_build_four_temperature_1X_responses\s*\(', ...
            'solve_stage_e_full_order_1X_case\s*\(', ...
            'run_stage_d_four_temperature_static_states\s*\(', ...
            'run_all_cases[^\r\n]*\(', ...
            'dynamic[^\r\n]*ehl[^\r\n]*\(', ...
            '\.(txt|csv|png|jpg|fig|xlsx|pdf)'''}
        assert(isempty(regexpi(source,forbidden{1},'once')), ...
            'StageF:ForbiddenProductionSource', ...
            '%s contains forbidden scope: %s',production{k},forbidden{1});
    end
end

read_only_files = setdiff(production,{'stage_f_canonical_transaction.m'});
for k = 1:numel(read_only_files)
    source = fileread(fullfile(repository_root,read_only_files{k}));
    for forbidden_write = {'\<save\s*\(','\<movefile\s*\(', ...
            '\<copyfile\s*\(','\<writetable\s*\(', ...
            '\<writecell\s*\(','\<exportgraphics\s*\(', ...
            '\<diary\s*\(','fopen\s*\([^\r\n]*[''" ]w'}
        assert(isempty(regexpi(source,forbidden_write{1},'once')), ...
            'StageF:OutOfScopeWrite', ...
            '%s must remain read-only.',read_only_files{k});
    end
end

transaction = fileread(fullfile(repository_root,'stage_f_canonical_transaction.m'));
for required = {'StandardCopyOption.ATOMIC_MOVE', ...
        'StandardCopyOption.REPLACE_EXISTING','tryLock','channel.force(true)', ...
        '.stage_f_transaction.mat','expected_canonical_sha256'}
    assert(contains(transaction,required{1}), ...
        'StageF:AtomicTransactionContract', ...
        'Stage F transaction omits %s.',required{1});
end
assert(isempty(regexpi(transaction,'\<movefile\s*\(','once')), ...
    'StageF:AtomicTransactionContract', ...
    'Atomic replacement must use Java NIO ATOMIC_MOVE, not movefile.');
for forbidden_write = {'\<writetable\s*\(','\<writecell\s*\(', ...
        '\<exportgraphics\s*\(','\<diary\s*\(', ...
        'fopen\s*\([^\r\n]*[''" ]w'}
    assert(isempty(regexpi(transaction,forbidden_write{1},'once')), ...
        'StageF:OutOfScopeWrite', ...
        'Transaction may write only its temporary MAT and canonical MAT.');
end

validator = fileread(fullfile(repository_root,'validate_stage_f_results.m'));
assert(isempty(regexp(validator, ...
    '\<recover_stage_f_rolling_element_1X_loads\s*\(','once')), ...
    'StageF:IndependentValidatorFormula', ...
    'Validator must use independent recovery formulas, never call recover_stage_f_rolling_element_1X_loads.');
assert(~contains(validator,'mapping = actual.mapping_snapshot') && ...
    contains(validator,'build_stage_f_rolling_element_linearization_mapping') && ...
    contains(validator,'baseline.frequency_response'), ...
    'StageF:IndependentValidatorBaseline', ...
    'Formal validator must rebuild mapping and qhat from the frozen baseline, not candidate payloads.');

recovery = fileread(fullfile(repository_root, ...
    'recover_stage_f_rolling_element_1X_loads.m'));
assert(contains(recovery,'stage_e_response_source_commit') && ...
    contains(recovery,'stage_f_source_commit') && ...
    isempty(regexp(recovery,'\<source_commit\>','once')), ...
    'StageF:UnambiguousProvenance', ...
    'Recovered results must split Stage E response and Stage F source commits; source_commit is ambiguous.');

assert(contains(transaction,'stage_f_independent_readonly_validation') && ...
    contains(transaction,'-batch'), ...
    'StageF:IndependentMatlabValidation', ...
    'The temporary candidate must be validated in an independent MATLAB -batch process.');
end

function test_stage_a_to_e_regressions_remain_green
test_stage_a_equivalent_frequency_mainline_contract;
test_stage_b_formal_damping_isolation;
test_stage_c_fixed_damping_envelope;
test_stage_d_four_temperature_static_states;
test_stage_e_full_order_1X_frequency_responses;
end

function fixture = ball_mapping_fixture(delta,KH,loaded)
delta = delta(:); loaded = logical(loaded(:)); n = numel(delta);
Q = zeros(n,1); k = zeros(n,1);
Q(loaded) = KH*delta(loaded).^(3/2);
k(loaded) = 1.5*KH*sqrt(delta(loaded));
theta = 2*pi*(0:n-1).'/n; alpha = 14*pi/180; lever = 0.019;
% The front bearing has a KKT-handled axial coordinate.  Its recovery
% Jacobian therefore projects uz out, while force assembly retains the
% formal radial cos(alpha) direction and corresponding moment arms.
J = [cos(theta) sin(theta) zeros(n,1) ...
    -lever*sin(theta) lever*cos(theta)];
A = cos(alpha)*J.';
fixture = struct('bearing_label','front','contact_kind','ball', ...
    'element_count',n,'slice_count',1,'delta_static_slice',delta, ...
    'Q_static_slice',Q,'k_static_slice',k, ...
    'loaded_mask_static_slice',loaded, ...
    'formal_contact_jacobian_snapshot',J, ...
    'formal_force_assembly_snapshot',A, ...
    'bearing_interface_transform_5dof',eye(5), ...
    'K_b_local',A*diag(k)*J, ...
    'contact_stiffness_source','FORMAL_BALL_HERTZ_DERIVATIVE', ...
    'formal_contact_law',struct('coefficient',KH,'exponent',3/2), ...
    'formal_geometry',struct('theta_rad',theta,'slice_z_m',0, ...
        'contact_angle_rad',alpha,'axial_lever_m',lever, ...
        'local_dof_order',{{'ux','uy','uz','theta_x','theta_y'}}), ...
    'mapping_provenance',formal_mapping_provenance('front'));
end

function fixture = rear_mapping_fixture(delta,Kline,loaded)
assert(isequal(size(delta),[30 9]));
loaded = logical(loaded); coefficient = Kline/9;
Q = zeros(30,9); k = zeros(30,9);
Q(loaded) = coefficient*delta(loaded).^(10/9);
k(loaded) = (10/9)*coefficient*delta(loaded).^(1/9);
theta = 2*pi*(0:29).'/30; slice_z = linspace(-0.0075,0.0075,9);
J = zeros(270,5);
for slice_index = 1:9
    rows = (1:30)+(slice_index-1)*30;
    z = slice_z(slice_index);
    J(rows,:) = [cos(theta) sin(theta) zeros(30,1) ...
        -z*sin(theta) z*cos(theta)];
end
A = J.';
fixture = struct('bearing_label','rear','contact_kind','roller', ...
    'element_count',30,'slice_count',9,'delta_static_slice',delta, ...
    'Q_static_slice',Q,'k_static_slice',k, ...
    'loaded_mask_static_slice',loaded, ...
    'formal_contact_jacobian_snapshot',J, ...
    'formal_force_assembly_snapshot',A, ...
    'bearing_interface_transform_5dof',eye(5), ...
    'K_b_local',A*diag(k(:))*J, ...
    'contact_stiffness_source','FORMAL_ROLLER_SLICE_LAW_DERIVATIVE', ...
    'formal_contact_law',struct('coefficient',coefficient,'exponent',10/9), ...
    'formal_geometry',struct('theta_rad',theta,'slice_z_m',slice_z, ...
        'contact_angle_rad',0,'axial_lever_m',0, ...
        'local_dof_order',{{'ux','uy','uz','theta_x','theta_y'}}), ...
    'mapping_provenance',formal_mapping_provenance('rear'));
end

function fixture = force_consistent_ball_fixture
delta = [1;1;1]; KH = 2/3;
fixture = ball_mapping_fixture(delta,KH,true(3,1));
end

function fixture = cancellation_rear_fixture
delta = 10*ones(30,9); Kline = 9*(9/10)*10^(-1/9);
fixture = rear_mapping_fixture(delta,Kline,true(30,9));
end

function provenance = formal_mapping_provenance(bearing_label)
if strcmp(bearing_label,'front')
    state_path = 'results.static.T*.bearing_state_front';
    transform_path = 'results.static.T20.bearing_mapping.front_B';
else
    state_path = 'results.static.T*.bearing_state_rear';
    transform_path = 'results.static.T20.bearing_mapping.rear_B';
end
provenance = struct( ...
    'schema','STAGE_F_FORMAL_5DOF_MAPPING_PROVENANCE_V1', ...
    'accepted_stage_d_commit','23a975c2c762504dc4ae62f07379737582fd783f', ...
    'accepted_stage_e_commit','0cab3dc49dfffc4192066402cb70302f084cb45d', ...
    'contact_state_path',state_path,'interface_transform_path',transform_path, ...
    'contact_kernel_file','nonlinear_bearing_force.m', ...
    'contact_kernel_sha256', ...
        'd82ae8cc92a1f1fb55d9d0c14da3efa196e9f085ce22abc5cdd2b345f93af5c8', ...
    'static_extractor_file','extract_stage_d_static_state.m', ...
    'static_extractor_sha256', ...
        '94ca056ad4b7f867851f7698f1241e9f85d55a011d904ba2620fa30395387661', ...
    'interface_mapping_file','build_stage_e_observation_mapping.m', ...
    'interface_mapping_sha256', ...
        'c74a92064886aa66e46a9a7b97ece787e4e4603ea198ab2814ad1450c7461b5b', ...
    'verified',true);
end

function fixture = tamper_Q_static(fixture)
fixture.Q_static_slice(1) = fixture.Q_static_slice(1)*1.01;
end

function fixture = tamper_delta_static(fixture)
fixture.delta_static_slice(1) = fixture.delta_static_slice(1)*1.01;
end

function fixture = tamper_k_static(fixture)
fixture.k_static_slice(1) = fixture.k_static_slice(1)*1.01;
end

function fixture = tamper_loaded_mask(fixture)
fixture.loaded_mask_static_slice(1) = false;
end

function request = scan_request(q_static,qhat,loaded)
request = struct('scenario','NOMINAL','phase_count',32, ...
    'q_static',q_static(:),'qhat_full',qhat(:), ...
    'linear_force_hat',qhat(:), ...
    'loaded_mask_static_slice',logical(loaded(:)));
end

function request = formal_scan_request_from_canonical(results,temperature,bearing,mapping)
tf = sprintf('T%d',temperature);
static_state = results.static.(tf);
request = struct( ...
    'evaluation_mode','FORMAL_FROZEN_NONLINEAR_BEARING_FORCE', ...
    'scenario','NOMINAL','phase_count',32, ...
    'temperature_case_C',temperature,'bearing',bearing, ...
    'q_static',static_state.q_static, ...
    'qhat_full',results.frequency_response.(tf).NOMINAL.qhat_full, ...
    'mapping_snapshot',mapping, ...
    'frozen_thermal_state',static_state.thermal_state, ...
    'contact_state_snapshot',static_state.(['bearing_state_' bearing]), ...
    'static_state_source',['results.static.' tf], ...
    'response_source',['results.frequency_response.' tf '.NOMINAL.qhat_full'], ...
    'thermal_state_source',['results.static.' tf '.thermal_state'], ...
    'canonical_sha256', ...
        '3f0e6e3b379cda670ef173682c58d17a19fc6b30f761ed59a00f3c203793abfd', ...
    'thermal_state_frozen',true,'thermal_update_allowed',false, ...
    'static_resolve_allowed',false,'frequency_resolve_allowed',false);
end

function evaluator = formal_nonlinear_evaluator_provenance(root,results,temperature)
tf = sprintf('T%d',temperature);
evaluator = struct( ...
    'function_handle',@nonlinear_bearing_force, ...
    'function_name','nonlinear_bearing_force', ...
    'source_file',fullfile(root,'nonlinear_bearing_force.m'), ...
    'source_sha256', ...
        'd82ae8cc92a1f1fb55d9d0c14da3efa196e9f085ce22abc5cdd2b345f93af5c8', ...
    'formal_static_geometry_source', ...
        'nonlinear_bearing_force/stage4a_bearing_force', ...
    'frozen_thermal_state',results.static.(tf).thermal_state, ...
    'thermal_state_source',['results.static.' tf '.thermal_state'], ...
    'stage_d_source_commit',results.meta.stage_d_source_commit, ...
    'stage_e_source_commit',results.meta.stage_e_source_commit, ...
    'thermal_state_frozen',true,'thermal_update_allowed',false, ...
    'canonical_write_allowed',false);
end

function request = formal_preflight_request_fixture(repository_root)
mapping = build_stage_f_rolling_element_linearization_mapping( ...
    force_consistent_ball_fixture());
frozen_thermal_state = struct('temperature_case_C',20,'frozen',true);
parameters = struct('bearing',struct('configuration','FORMAL_FROZEN'), ...
    'microphysics',struct('thermal',struct('enabled',true, ...
    'mode','frozen_external'),'thermal_state',frozen_thermal_state));
evaluator = struct('function_handle',@nonlinear_bearing_force, ...
    'function_name','nonlinear_bearing_force', ...
    'source_file',fullfile(repository_root,'nonlinear_bearing_force.m'), ...
    'source_sha256', ...
        'd82ae8cc92a1f1fb55d9d0c14da3efa196e9f085ce22abc5cdd2b345f93af5c8', ...
    'formal_static_geometry_source', ...
        'nonlinear_bearing_force/stage4a_bearing_force', ...
    'frozen_thermal_state',frozen_thermal_state, ...
    'thermal_state_source','results.static.T20.thermal_state', ...
    'stage_d_source_commit','23a975c2c762504dc4ae62f07379737582fd783f', ...
    'stage_e_source_commit','0cab3dc49dfffc4192066402cb70302f084cb45d', ...
    'thermal_state_frozen',true,'thermal_update_allowed',false, ...
    'canonical_write_allowed',false);
request = struct( ...
    'evaluation_mode','FORMAL_FROZEN_NONLINEAR_BEARING_FORCE', ...
    'scenario','NOMINAL','phase_count',32, ...
    'temperature_case_C',20,'bearing','front','bearing_index',1, ...
    'q_static',zeros(5,1),'qhat_full',zeros(5,1), ...
    'mapping_snapshot',mapping,'formal_parameters',parameters, ...
    'frozen_thermal_state',frozen_thermal_state, ...
    'contact_state_snapshot',struct('raw_contact',struct('f5',zeros(5,1))), ...
    'static_state_source','results.static.T20', ...
    'response_source','results.frequency_response.T20.NOMINAL.qhat_full', ...
    'thermal_state_source','results.static.T20.thermal_state', ...
    'canonical_sha256', ...
        '3f0e6e3b379cda670ef173682c58d17a19fc6b30f761ed59a00f3c203793abfd', ...
    'thermal_state_frozen',true,'thermal_update_allowed',false, ...
    'static_resolve_allowed',false,'frequency_resolve_allowed',false, ...
    'evaluator',evaluator);
end

function request = tamper_formal_microphysics_thermal_state(request)
request.formal_parameters.microphysics.thermal_state.temperature_case_C = 999;
end

function request = tamper_formal_microphysics_thermal_mode(request)
request.formal_parameters.microphysics.thermal.mode = 'recompute';
end

function request = tamper_formal_microphysics_thermal_enable(request)
request.formal_parameters.microphysics.thermal.enabled = false;
end

function request = tamper_formal_bearing_configuration(request)
request.formal_parameters.bearing.configuration = 'UNTRUSTED_REPLACEMENT';
end

function [force,loaded] = linear_scan_evaluator(q)
force = q(:); loaded = true(size(q(:)));
end

function [force,loaded] = weakly_nonlinear_scan_evaluator(q)
q = q(:); force = q+0.4*q.^2; loaded = true(size(q));
end

function [force,loaded] = active_set_scan_evaluator(q)
q = q(:); force = q; loaded = [q(1) >= 0;true];
end

function [force,loaded] = must_not_be_called(~)
force = []; loaded = []; %#ok<NASGU>
error('StageF:EvaluatorCalled','Evaluator must not be called.');
end

function [baseline,candidate] = mock_stage_f_artifact(scientific_failure)
baseline = struct();
baseline.stage_a_to_e_sentinel = reshape(1:9,3,3);
baseline.meta = struct('stage_d_source_commit', ...
    '23a975c2c762504dc4ae62f07379737582fd783f', ...
    'stage_e_source_commit','0cab3dc49dfffc4192066402cb70302f084cb45d');
baseline.configuration = struct('temperature_cases_C',[20 50 80 100], ...
    'damping_scenarios',{{'LOW','NOMINAL','HIGH'}}, ...
    'analysis_type','FULL_ORDER_1X_FREQUENCY_DOMAIN_WITH_CONTACT_NONLINEARITY_GATE');
baseline.progress = struct('stage_e_complete',true, ...
    'last_completed_gate','FOUR_TEMPERATURE_1X_RESPONSES_ACCEPTED');
baseline.decision = struct('status','FOUR_TEMPERATURE_1X_RESPONSES_ACCEPTED', ...
    'allow_stage_f',true);
baseline.validation = struct( ...
    'runtime_legacy_equivalence',struct('passed',true,'sentinel',11), ...
    'stage_c',struct('passed',true,'sentinel',12), ...
    'stage_d',struct('passed',true,'sentinel',13), ...
    'stage_e',struct('passed',true,'sentinel',14));
baseline.damping = struct('alpha',1.25,'beta',2.5, ...
    'C_formal_LOW',magic(3),'C_formal_NOMINAL',2*magic(3), ...
    'C_formal_HIGH',3*magic(3),'frozen_across_temperatures',true);
temperatures = [20 50 80 100]; scenarios = {'LOW','NOMINAL','HIGH'};
front_mapping = build_stage_f_rolling_element_linearization_mapping( ...
    force_consistent_ball_fixture());
rear_mapping = build_stage_f_rolling_element_linearization_mapping( ...
    cancellation_rear_fixture());
candidate = baseline;
for temperature = temperatures
    tf = sprintf('T%d',temperature);
    baseline.static.(tf) = struct('q_static',(1:5).'+temperature, ...
        'thermal_state',struct('temperature_case_C',temperature, ...
            'frozen',true),'stage_a_to_e_static_sentinel',magic(3)+temperature);
    candidate.static.(tf) = baseline.static.(tf);
    for k = 1:numel(scenarios)
        sf = scenarios{k}; qhat = zeros(5,1);
        if scientific_failure && temperature == 20 && strcmp(sf,'LOW')
            qhat(1) = 1;
        end
        baseline.frequency_response.(tf).(sf).qhat_full = qhat;
        candidate.frequency_response.(tf).(sf).qhat_full = qhat;
        candidate.bearing_contact.(tf).(sf).front = ...
            recover_stage_f_rolling_element_1X_loads(front_mapping,qhat);
        candidate.bearing_contact.(tf).(sf).rear = ...
            recover_stage_f_rolling_element_1X_loads(rear_mapping,qhat);
    end
    scan = evaluate_stage_f_nonlinear_bearing_force_scan( ...
        scan_request(zeros(5,1),zeros(5,1),true(3,1)), ...
        @five_dof_linear_scan_evaluator);
    harmonics = compute_stage_f_bearing_force_harmonics( ...
        scan.exact_dynamic_force_history,scan.phase_values_rad);
    candidate.nonlinearity_gate.(tf).front = merge_scan(scan,harmonics);
    candidate.nonlinearity_gate.(tf).rear = merge_scan(scan,harmonics);
end
% Candidate must contain the same Stage E responses that became part of the
% baseline fixture.
candidate.frequency_response = baseline.frequency_response;
candidate.stage_f_execution_audit = struct( ...
    'scan_requests',mock_scan_request_manifest(), ...
    'scan_request_count',8, ...
    'evaluator_function','nonlinear_bearing_force', ...
    'evaluator_source_sha256', ...
        'd82ae8cc92a1f1fb55d9d0c14da3efa196e9f085ce22abc5cdd2b345f93af5c8', ...
    'thermal_state_frozen',true,'LOW_scans',0,'HIGH_scans',0);
if scientific_failure
    gate = 'REPRESENTATIVE_NONLINEAR_VALIDATION_REQUIRED'; allow = false;
    trigger = struct('temperature_case_C',20,'scenario','LOW', ...
        'bearing','front','roller_index',1,'slice_index',1, ...
        'metric','POTENTIAL_CONTACT_LOSS');
else
    gate = 'FREQUENCY_DOMAIN_LINEARIZATION_ACCEPTED'; allow = true;
    trigger = struct([]);
end
candidate.progress.stage_f_load_recovery_complete = true;
candidate.progress.stage_f_nonlinear_scan_complete = true;
candidate.progress.stage_f_complete = true;
candidate.progress.stage_f_gate_status = gate;
candidate.progress.stage_f_first_trigger = trigger;
candidate.progress.allow_stage_g = allow;
candidate.progress.last_completed_gate = gate;
candidate.decision = struct('status',gate,'allow_stage_g',allow);
candidate.validation.stage_f = struct('passed',true,'gate_status',gate);
candidate = enrich_stage_f_audit_fixture(candidate,baseline);
if ~isempty(candidate.progress.stage_f_first_trigger)
    candidate.progress.stage_f_first_trigger = enrich_first_trigger( ...
        candidate.progress.stage_f_first_trigger,candidate);
    candidate.progress.stage_f_first_trigger.temperature = ...
        candidate.progress.stage_f_first_trigger.temperature_case_C;
    candidate.progress.stage_f_first_trigger.phase = ...
        candidate.progress.stage_f_first_trigger.phase_rad;
    candidate.progress.stage_f_first_trigger.roller_id = ...
        candidate.progress.stage_f_first_trigger.roller_index;
    candidate.progress.stage_f_first_trigger.slice_id = ...
        candidate.progress.stage_f_first_trigger.slice_index;
end
end

function [force,loaded] = five_dof_linear_scan_evaluator(q)
force = q(:); loaded = true(3,1);
end

function manifest = mock_scan_request_manifest
temperatures = [20 50 80 100]; bearings = {'front','rear'};
manifest = repmat(struct('temperature_case_C',0,'bearing','', ...
    'scenario','NOMINAL','phase_count',32, ...
    'thermal_state_frozen',true, ...
    'evaluator_function','nonlinear_bearing_force'),1,8);
index = 0;
for temperature = temperatures
    for bearing_index = 1:numel(bearings)
        index = index+1;
        manifest(index).temperature_case_C = temperature;
        manifest(index).bearing = bearings{bearing_index};
    end
end
end

function out = merge_scan(scan,harmonics)
out = scan;
names = fieldnames(harmonics);
for k = 1:numel(names), out.(names{k}) = harmonics.(names{k}); end
out.maximum_r_delta_nominal = 0;
out.potential_contact_loss_nominal = false;
out.gate_status = 'FREQUENCY_DOMAIN_LINEARIZATION_ACCEPTED';
end

function candidate = enrich_stage_f_audit_fixture(candidate,baseline)
stage_f_commit = '1111111111111111111111111111111111111111';
for temperature = [20 50 80 100]
    tf = sprintf('T%d',temperature);
    for scenario = {'LOW','NOMINAL','HIGH'}
        sf = scenario{1};
        for bearing = {'front','rear'}
            bn = bearing{1};
            recovered = candidate.bearing_contact.(tf).(sf).(bn);
            recovered.stage_e_response_source_commit = ...
                baseline.meta.stage_e_source_commit;
            recovered.stage_f_source_commit = stage_f_commit;
            recovered.static_state_source_commit = ...
                baseline.meta.stage_d_source_commit;
            recovered.contact_force_resultant = ...
                recovered.contact_load_resultant;
            candidate.bearing_contact.(tf).(sf).(bn) = recovered;
        end
    end
    for bearing = {'front','rear'}
        bn = bearing{1};
        scan = candidate.nonlinearity_gate.(tf).(bn);
        scan.F1_norm = norm(scan.F1_complex,2);
        scan.F2_norm = norm(scan.F2_complex,2);
        scan.F3_norm = norm(scan.F3_complex,2);
        scan.F1_phase_rad = angle(scan.F1_complex);
        scan.F2_phase_rad = angle(scan.F2_complex);
        scan.F3_phase_rad = angle(scan.F3_complex);
        scan.F1_force_semantics = 'FORCE_ON_ROTOR_MINUS_AQ';
        scan.F2_force_semantics = 'FORCE_ON_ROTOR_MINUS_AQ';
        scan.F3_force_semantics = 'FORCE_ON_ROTOR_MINUS_AQ';
        scan.thermal_state_frozen = true;
        frozen = struct('temperature_C',temperature, ...
            'viscosity_Pa_s',1,'clearance_m',1, ...
            'contact_state_sha256',repmat('a',1,64));
        scan.thermal_frozen_summary = struct('before',frozen,'after',frozen);
        candidate.nonlinearity_gate.(tf).(bn) = scan;
    end
end
candidate.stage_f_execution_audit.source_commit = stage_f_commit;
candidate.stage_f_execution_audit.elapsed_seconds = 1;
end

function trigger = enrich_first_trigger(trigger,candidate)
% The synthetic scientific-failure fixture has a static contact-loss trigger:
% NaN phase is precise because no scan phase caused this load-only failure.
recovered = candidate.bearing_contact.T20.LOW.front;
index = sub2ind(size(recovered.slice_level.Qmin_linear_slice), ...
    trigger.roller_index,trigger.slice_index);
trigger.phase_index = NaN;
trigger.phase_rad = NaN;
trigger.slice_indices = trigger.slice_index;
trigger.trigger_value = recovered.slice_level.Qmin_linear_slice(index);
trigger.threshold = 0;
end

function value = tamper_delta_hat(value)
value.bearing_contact.T20.LOW.rear.slice_level.delta_hat_slice(1,1) = ...
    value.bearing_contact.T20.LOW.rear.slice_level.delta_hat_slice(1,1)+1;
end

function value = tamper_Qhat(value)
value.bearing_contact.T20.LOW.rear.slice_level.Qhat_slice(1,1) = ...
    value.bearing_contact.T20.LOW.rear.slice_level.Qhat_slice(1,1)+1;
end

function value = tamper_roller_Qhat(value)
value.bearing_contact.T20.LOW.rear.roller_level.Qhat_roller(1) = ...
    value.bearing_contact.T20.LOW.rear.roller_level.Qhat_roller(1)+1;
end

function value = tamper_Qstatic_roller(value)
value.bearing_contact.T20.LOW.rear.roller_level.Q_static_roller(1) = ...
    value.bearing_contact.T20.LOW.rear.roller_level.Q_static_roller(1)+1;
end

function value = tamper_Qmax_slice(value)
value.bearing_contact.T20.LOW.rear.slice_level.Qmax_linear_slice(1,1) = ...
    value.bearing_contact.T20.LOW.rear.slice_level.Qmax_linear_slice(1,1)+1;
end

function value = tamper_Qmax(value)
value.bearing_contact.T20.LOW.rear.roller_level.Qmax_linear_roller(1) = ...
    value.bearing_contact.T20.LOW.rear.roller_level.Qmax_linear_roller(1)+1;
end

function value = tamper_Qmin(value)
value.bearing_contact.T20.LOW.rear.slice_level.Qmin_linear_slice(1,1) = ...
    value.bearing_contact.T20.LOW.rear.slice_level.Qmin_linear_slice(1,1)-1;
end

function value = tamper_Qmin_roller(value)
value.bearing_contact.T20.LOW.rear.roller_level.Qmin_linear_roller(1) = ...
    value.bearing_contact.T20.LOW.rear.roller_level.Qmin_linear_roller(1)-1;
end

function value = tamper_r_delta(value)
value.bearing_contact.T20.LOW.rear.slice_level.r_delta_slice(1,1) = ...
    value.bearing_contact.T20.LOW.rear.slice_level.r_delta_slice(1,1)+0.1;
end

function value = tamper_r_delta_roller(value)
value.bearing_contact.T20.LOW.rear.roller_level.r_delta_roller(1) = ...
    value.bearing_contact.T20.LOW.rear.roller_level.r_delta_roller(1)+0.1;
end

function value = tamper_linearity_class(value)
value.bearing_contact.T20.LOW.rear.slice_level.linearity_class_slice{1,1} = ...
    'LOCAL_LINEARITY_LIMIT_EXCEEDED';
end

function value = tamper_contact_loss(value)
field = 'potential_contact_loss_slice';
old = value.bearing_contact.T20.LOW.rear.slice_level.(field)(1,1);
value.bearing_contact.T20.LOW.rear.slice_level.(field)(1,1) = ~old;
end

function value = tamper_contact_loss_roller(value)
field = 'potential_contact_loss_roller';
old = value.bearing_contact.T20.LOW.rear.roller_level.(field)(1);
value.bearing_contact.T20.LOW.rear.roller_level.(field)(1) = ~old;
end

function value = tamper_resultant(value)
value.bearing_contact.T20.LOW.rear.dynamic_resultant.local_complex(1) = ...
    value.bearing_contact.T20.LOW.rear.dynamic_resultant.local_complex(1)+1;
end

function value = tamper_assembly_core_error(value)
field = 'element_vs_analytic_tangent_relative_error';
value.bearing_contact.T20.LOW.rear.force_assembly_validation.(field) = 1e-2;
end

function value = tamper_assembly_core_threshold(value)
field = 'element_vs_analytic_tangent_threshold';
value.bearing_contact.T20.LOW.rear.force_assembly_validation.(field) = 1e-9;
end

function value = tamper_assembly_core_status(value)
field = 'element_vs_analytic_tangent_gate_status';
value.bearing_contact.T20.LOW.rear.force_assembly_validation.(field) = ...
    'ROLLING_ELEMENT_FORCE_ASSEMBLY_MISMATCH';
end

function value = tamper_analytic_vs_saved_tangent_error(value)
field = 'analytic_vs_saved_tangent_relative_error';
value.bearing_contact.T20.LOW.rear.force_assembly_validation.(field) = 1e-4;
end

function value = tamper_analytic_vs_saved_tangent_threshold(value)
field = 'analytic_vs_saved_tangent_threshold';
value.bearing_contact.T20.LOW.rear.force_assembly_validation.(field) = 1e-7;
end

function value = tamper_analytic_vs_saved_tangent_status(value)
field = 'analytic_vs_saved_tangent_gate_status';
value.bearing_contact.T20.LOW.rear.force_assembly_validation.(field) = ...
    'ANALYTIC_VS_FD_TANGENT_AUDIT_FAILED';
end

function value = tamper_epsilon_NL(value)
value.nonlinearity_gate.T20.front.epsilon_NL = 0.2;
end

function value = tamper_maximum_r_delta_nominal(value)
value.nonlinearity_gate.T20.front.maximum_r_delta_nominal = 0.2;
end

function value = tamper_potential_contact_loss_nominal(value)
field = 'potential_contact_loss_nominal';
old = value.nonlinearity_gate.T20.front.(field);
value.nonlinearity_gate.T20.front.(field) = ~old;
end

function value = tamper_front_bearing_gate_status(value)
value.nonlinearity_gate.T20.front.gate_status = ...
    'REPRESENTATIVE_NONLINEAR_VALIDATION_REQUIRED';
end

function value = tamper_rear_bearing_gate_status(value)
value.nonlinearity_gate.T20.rear.gate_status = ...
    'REPRESENTATIVE_NONLINEAR_VALIDATION_REQUIRED';
end

function value = tamper_active_set_history(value)
old = value.nonlinearity_gate.T20.front.active_set_history(1,1);
value.nonlinearity_gate.T20.front.active_set_history(1,1) = ~old;
end

function value = tamper_active_set_changed(value)
value.nonlinearity_gate.T20.front.active_set_changed = ...
    ~value.nonlinearity_gate.T20.front.active_set_changed;
end

function value = collude_linear_force_history_epsilon_and_gate(value)
scan = value.nonlinearity_gate.T20.front;
theta = 2*pi*(0:31)/32;
scan.linear_force_hat = zeros(size(scan.linear_force_hat));
scan.linear_force_hat(1) = 1;
scan.linear_dynamic_force_history = real(scan.linear_force_hat*exp(1i*theta));
exact_dynamic = scan.exact_force_history-scan.static_force_reference(:);
scan.epsilon_NL = history_rms_norm(exact_dynamic-scan.linear_dynamic_force_history)/ ...
    max(history_rms_norm(exact_dynamic),eps);
scan.gate_status = 'REPRESENTATIVE_NONLINEAR_VALIDATION_REQUIRED';
value.nonlinearity_gate.T20.front = scan;
value = synchronize_candidate_final_gate(value);
end

function value = collude_static_reference_and_exact_history(value)
scan = value.nonlinearity_gate.T20.front;
shift = (1:numel(scan.static_force_reference)).';
scan.static_force_reference = scan.static_force_reference+shift;
scan.exact_force_history = scan.exact_force_history+shift;
value.nonlinearity_gate.T20.front = scan;
end

function value = collude_active_history_change_first_and_gate(value)
scan = value.nonlinearity_gate.T20.front;
scan.active_set_history(1,1) = ~scan.loaded_mask_static_slice(1);
scan.active_set_changed = true;
scan.first_active_set_change = struct('phase_index',1,'phase_rad',0, ...
    'slice_indices',1,'static_mask',logical(scan.loaded_mask_static_slice(:)), ...
    'phase_mask',logical(scan.active_set_history(:,1)));
scan.gate_status = 'REPRESENTATIVE_NONLINEAR_VALIDATION_REQUIRED';
value.nonlinearity_gate.T20.front = scan;
value = synchronize_candidate_final_gate(value);
end

function value = synchronize_candidate_final_gate(value)
gate = 'REPRESENTATIVE_NONLINEAR_VALIDATION_REQUIRED';
value.progress.stage_f_gate_status = gate;
value.progress.allow_stage_g = false;
value.progress.last_completed_gate = gate;
value.progress.stage_f_first_trigger = struct( ...
    'temperature_case_C',20,'scenario','NOMINAL','bearing','front', ...
    'roller_index',NaN,'slice_index',NaN,'metric','EPSILON_NL');
value.decision.status = gate;
value.decision.allow_stage_g = false;
value.validation.stage_f.gate_status = gate;
end

function value = tamper_first_active_set_change(value)
value.nonlinearity_gate.T20.front.first_active_set_change = struct( ...
    'phase_index',1,'slice_indices',1,'static_mask',true,'phase_mask',false);
end

function value = tamper_Qstatic_slice(value)
value.bearing_contact.T20.LOW.rear.slice_level.Q_static_slice(1,1) = ...
    value.bearing_contact.T20.LOW.rear.slice_level.Q_static_slice(1,1)+1;
end

function value = tamper_delta_static_slice(value)
value.bearing_contact.T20.LOW.rear.slice_level.delta_static_slice(1,1) = ...
    value.bearing_contact.T20.LOW.rear.slice_level.delta_static_slice(1,1)+1;
end

function value = tamper_k_static_slice(value)
value.bearing_contact.T20.LOW.rear.slice_level.k_static_slice(1,1) = ...
    value.bearing_contact.T20.LOW.rear.slice_level.k_static_slice(1,1)+1;
end

function value = tamper_loaded_mask_slice(value)
field = 'loaded_mask_static_slice';
value.bearing_contact.T20.LOW.rear.slice_level.(field)(1,1) = false;
end

function value = tamper_loaded_mask_roller(value)
field = 'loaded_mask_static_roller';
value.bearing_contact.T20.LOW.rear.roller_level.(field)(1) = false;
end

function value = tamper_delta_hat_real(value)
field = 'delta_hat_real_slice';
value.bearing_contact.T20.LOW.rear.slice_level.(field)(1,1) = 1;
end

function value = tamper_delta_hat_imag(value)
field = 'delta_hat_imag_slice';
value.bearing_contact.T20.LOW.rear.slice_level.(field)(1,1) = 1;
end

function value = tamper_delta_hat_magnitude(value)
field = 'delta_hat_magnitude_slice';
value.bearing_contact.T20.LOW.rear.slice_level.(field)(1,1) = 1;
end

function value = tamper_delta_hat_phase(value)
field = 'delta_hat_phase_rad_slice';
value.bearing_contact.T20.LOW.rear.slice_level.(field)(1,1) = 1;
end

function value = tamper_Qhat_real(value)
field = 'Qhat_real_slice';
value.bearing_contact.T20.LOW.rear.slice_level.(field)(1,1) = 1;
end

function value = tamper_Qhat_imag(value)
field = 'Qhat_imag_slice';
value.bearing_contact.T20.LOW.rear.slice_level.(field)(1,1) = 1;
end

function value = tamper_Qhat_magnitude(value)
field = 'Qhat_magnitude_slice';
value.bearing_contact.T20.LOW.rear.slice_level.(field)(1,1) = 1;
end

function value = tamper_Qhat_phase(value)
field = 'Qhat_phase_rad_slice';
value.bearing_contact.T20.LOW.rear.slice_level.(field)(1,1) = 1;
end

function value = tamper_contact_loss_evaluated(value)
field = 'contact_loss_evaluated_slice';
value.bearing_contact.T20.LOW.rear.slice_level.(field)(1,1) = false;
end

function value = tamper_contact_loss_reason(value)
field = 'contact_loss_reason_slice';
value.bearing_contact.T20.LOW.rear.slice_level.(field){1,1} = 'FORGED_REASON';
end

function value = tamper_roller_linearity_class(value)
field = 'linearity_class_roller';
value.bearing_contact.T20.LOW.rear.roller_level.(field){1,1} = ...
    'LOCAL_LINEARITY_LIMIT_EXCEEDED';
end

function value = tamper_global_resultant(value)
field = 'global_complex';
value.bearing_contact.T20.LOW.rear.dynamic_resultant.(field)(1) = 1;
end

function value = tamper_resultant_fx(value)
field = 'Fx_hat';
value.bearing_contact.T20.LOW.rear.dynamic_resultant.(field) = 1;
end

function value = tamper_resultant_fy(value)
field = 'Fy_hat';
value.bearing_contact.T20.LOW.rear.dynamic_resultant.(field) = 1i;
end

function value = tamper_radial_peak(value)
field = 'radial_force_peak';
value.bearing_contact.T20.LOW.rear.dynamic_resultant.(field) = 1;
end

function value = tamper_radial_rms(value)
field = 'radial_force_rms';
value.bearing_contact.T20.LOW.rear.dynamic_resultant.(field) = 1;
end

function value = tamper_phase_values_rad(value)
value.nonlinearity_gate.T20.front.phase_values_rad(2) = ...
    value.nonlinearity_gate.T20.front.phase_values_rad(2)+0.01;
end

function value = tamper_exact_dynamic_force_history(value)
value.nonlinearity_gate.T20.front.exact_dynamic_force_history(1,1) = 1;
end

function value = tamper_linear_dynamic_force_history(value)
value.nonlinearity_gate.T20.front.linear_dynamic_force_history(1,1) = 1;
end

function value = tamper_F1(value)
value.nonlinearity_gate.T20.front.F1_complex(1) = ...
    value.nonlinearity_gate.T20.front.F1_complex(1)+1;
end

function value = tamper_F2(value)
value.nonlinearity_gate.T20.front.F2_complex(1) = ...
    value.nonlinearity_gate.T20.front.F2_complex(1)+1;
end

function value = tamper_F3(value)
value.nonlinearity_gate.T20.front.F3_complex(1) = ...
    value.nonlinearity_gate.T20.front.F3_complex(1)+1;
end

function value = tamper_H2(value)
value.nonlinearity_gate.T20.front.H2_H1 = 0.99;
end

function value = tamper_H3(value)
value.nonlinearity_gate.T20.front.H3_H1 = 0.99;
end

function value = tamper_scan_extra_low(value)
extra = value.stage_f_execution_audit.scan_requests(1);
extra.scenario = 'LOW';
value.stage_f_execution_audit.scan_requests(end+1) = extra;
value.stage_f_execution_audit.scan_request_count = 9;
end

function value = tamper_scan_missing(value)
value.stage_f_execution_audit.scan_requests(end) = [];
value.stage_f_execution_audit.scan_request_count = 7;
end

function value = tamper_scan_duplicate_replacing_required_case(value)
requests = value.stage_f_execution_audit.scan_requests;
requests(end) = requests(1);
value.stage_f_execution_audit.scan_requests = requests;
value.stage_f_execution_audit.scan_request_count = 8;
end

function value = tamper_scan_phase_count(value)
value.stage_f_execution_audit.scan_requests(1).phase_count = 31;
end

function value = tamper_scan_evaluator(value)
value.stage_f_execution_audit.scan_requests(1).evaluator_function = ...
    'linear_scan_evaluator';
end

function value = tamper_final_gate(value)
value.progress.stage_f_gate_status = ...
    'REPRESENTATIVE_NONLINEAR_VALIDATION_REQUIRED';
value.progress.allow_stage_g = false;
end

function value = tamper_decision_gate(value)
value.decision.status = 'REPRESENTATIVE_NONLINEAR_VALIDATION_REQUIRED';
value.decision.allow_stage_g = false;
end

function value = tamper_saved_validation_gate(value)
value.validation.stage_f.gate_status = ...
    'REPRESENTATIVE_NONLINEAR_VALIDATION_REQUIRED';
end

function value = tamper_stage_e_validation(value)
value.validation.stage_e.sentinel = -14;
end

function value = delete_stage_d_validation(value)
value.validation = rmfield(value.validation,'stage_d');
end

function value = tamper_stage_d_static(value)
value.static.T50.stage_a_to_e_static_sentinel(1) = -1;
end

function value = delete_stage_d_static(value)
value.static = rmfield(value.static,'T80');
end

function value = tamper_stage_c_damping(value)
value.damping.C_formal_NOMINAL(1) = -1;
end

function value = delete_stage_c_damping(value)
value = rmfield(value,'damping');
end

function value = tamper_stage_e_response(value)
value.frequency_response.T100.HIGH.qhat_full(1) = 1;
end

function value = delete_stage_e_response(value)
value.frequency_response.T50 = rmfield(value.frequency_response.T50,'LOW');
end

function value = tamper_frozen_stage(value)
value.stage_a_to_e_sentinel(1) = -1;
end

function projection = stage_f_projection(candidate)
projection = candidate;
for name = {'bearing_contact','nonlinearity_gate','stage_f_execution_audit'}
    if isfield(projection,name{1}), projection = rmfield(projection,name{1}); end
end
if isfield(projection,'validation') && isfield(projection.validation,'stage_f')
    projection.validation = rmfield(projection.validation,'stage_f');
end
for name = {'stage_f_load_recovery_complete','stage_f_nonlinear_scan_complete', ...
        'stage_f_complete','stage_f_gate_status','stage_f_first_trigger', ...
        'allow_stage_g'}
    if isfield(projection.progress,name{1})
        projection.progress = rmfield(projection.progress,name{1});
    end
end
projection.progress.last_completed_gate = ...
    'FOUR_TEMPERATURE_1X_RESPONSES_ACCEPTED';
projection.decision = struct('status','FOUR_TEMPERATURE_1X_RESPONSES_ACCEPTED', ...
    'allow_stage_f',true);
end

function snapshot = canonical_stage_a_to_e_categories(results)
names = {'meta','configuration','progress','static','modal','damping', ...
    'validation','decision','frequency_response','modal_audits', ...
    'complex_force','force_definition','mapping_snapshot','mapping_audit', ...
    'force_audit'};
snapshot = struct();
for k = 1:numel(names)
    name = names{k};
    assert(isfield(results,name),'StageF:CanonicalStageAECategory', ...
        'Canonical lacks Stage A--E category %s.',name);
    snapshot.(name) = results.(name);
end
end

function present = recursive_field_present(value,needle)
present = false;
if ~isstruct(value), return; end
names = fieldnames(value);
for k = 1:numel(names)
    name = names{k};
    if contains(lower(name),lower(needle))
        present = true; return;
    end
    for j = 1:numel(value)
        if recursive_field_present(value(j).(name),needle)
            present = true; return;
        end
    end
end
end

function pass = diagnostics_contain(validation,token)
pass = false;
for field = {'diagnostic_identifiers','diagnostics','messages'}
    name = field{1};
    if isfield(validation,name)
        value = validation.(name);
        if ischar(value) || isstring(value), value = cellstr(value); end
        if iscell(value) && any(contains(upper(string(value)),upper(token)))
            pass = true; return;
        end
    end
end
end

function value = relative_error(actual,expected)
value = norm(actual(:)-expected(:))/max(norm(expected(:)),eps);
end

function value = history_rms_norm(history)
value = sqrt(mean(sum(abs(history).^2,1)));
end

function bytes = read_file_bytes(path)
fid = fopen(path,'rb');
assert(fid >= 0,'StageF:FileRead','Unable to read %s.',path);
cleanup = onCleanup(@() fclose(fid));
bytes = fread(fid,Inf,'*uint8');
end

function hash = file_sha256(path)
bytes = read_file_bytes(path);
digest = java.security.MessageDigest.getInstance('SHA-256');
digest.update(typecast(bytes,'int8'));
raw = typecast(digest.digest(),'uint8');
hash = lower(reshape(dec2hex(raw,2).',1,[]));
end

function names = directory_file_names(directory)
entries = dir(directory); entries = entries(~[entries.isdir]);
names = sort({entries.name});
end

function [channel,file_lock] = acquire_test_lock(path)
source = java.nio.file.Paths.get(path,javaArray('java.lang.String',0));
options = javaArray('java.nio.file.OpenOption',1);
options(1) = java.nio.file.StandardOpenOption.WRITE;
channel = java.nio.channels.FileChannel.open(source,options);
file_lock = channel.tryLock();
assert(~isempty(file_lock),'StageF:TestLock','Unable to acquire test lock.');
end

function release_test_lock(file_lock,channel)
if ~isempty(file_lock) && file_lock.isValid(), file_lock.release(); end
if ~isempty(channel) && channel.isOpen(), channel.close(); end
end

function assert_gate_error(callback,identifier,gate)
try
    callback();
catch exception
    assert(strcmp(exception.identifier,identifier), ...
        'Unexpected identifier: %s (expected %s).',exception.identifier,identifier);
    assert(contains(exception.message,gate), ...
        'StageF:MissingFailureGate','Expected failure Gate %s.',gate);
    return;
end
error('StageF:ExpectedError','Expected %s / %s.',identifier,gate);
end

function assert_error(callback,identifier)
try
    callback();
catch exception
    assert(strcmp(exception.identifier,identifier), ...
        'Unexpected identifier: %s (expected %s).',exception.identifier,identifier);
    return;
end
error('StageF:ExpectedError','Expected %s.',identifier);
end
