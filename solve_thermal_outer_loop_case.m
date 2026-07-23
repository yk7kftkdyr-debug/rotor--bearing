function [record, continuation_out, shared_out] = solve_thermal_outer_loop_case(case_index, T_oil_C, params_base, model_cache, continuation_in, shared_in, cfg)
%SOLVE_THERMAL_OUTER_LOOP_CASE Execute one temperature case entirely in memory.

record = empty_thermal_case_record(cfg); continuation_out = record.resume_state; shared_out = shared_in;
record.meta.case_index = case_index; record.meta.T_oil_C = T_oil_C; record.meta.status = "running";
record.meta.started_at = datetime('now'); record.meta.source_commit = string(cfg.minimum_required_commit); record.meta.config_signature = string(cfg.config_signature);
total_timer = tic;
try
    validate_case_inputs(case_index, T_oil_C, params_base, model_cache, cfg);
    [q_initial, initialization] = initial_state_for_case(case_index, T_oil_C, continuation_in);
    record.initialization = initialization;

    thermal_timer = tic;
    thermal_result = solve_thermal_mechanical_outer_loop(params_base, params_base.bearing, T_oil_C, q_initial);
    record.runtime.thermal_outer_loop_s = toc(thermal_timer);
    require_thermal_gate(thermal_result, cfg);
    params_frozen = frozen_parameters(params_base, model_cache.modelInfo, thermal_result.thermal_state);

    stiffness_timer = tic;
    stiffness_results(1) = linearize_bearing_workpoint(thermal_result.q_static, params_frozen, 1, struct());
    stiffness_results(2) = linearize_bearing_workpoint(thermal_result.q_static, params_frozen, 2, struct());
    record.runtime.bearing_stiffness_s = toc(stiffness_timer);
    if ~all([stiffness_results.pass]), error('Stage9B:StiffnessGate', 'Both bearing workpoint stiffness matrices must pass.'); end

    damping_timer = tic;
    contacts = thermal_result.contact_state.bearings;
    damping_results(1) = build_ehl_damping_matrix(contacts(1), contacts(1), contacts(1).microphysics_state.temperature.contact_geometry, params_frozen.bearing(1));
    damping_results(2) = build_ehl_damping_matrix(contacts(2), contacts(2), contacts(2).microphysics_state.temperature.contact_geometry, params_frozen.bearing(2));
    record.runtime.ehl_damping_s = toc(damping_timer);
    if ~all([damping_results.pass]), error('Stage9B:DampingGate', 'Both bearing EHL damping matrices must pass.'); end

    linearization_timer = tic;
    validation = validate_workpoint_linearization(thermal_result.q_static, params_frozen, stiffness_results, struct());
    record.runtime.linearization_s = toc(linearization_timer);
    if ~validation.pass, error('Stage9B:LinearizationGate', 'The Stage7 workpoint linearization check must pass.'); end

    if case_index == 1 && abs(T_oil_C-20) <= 100*eps(max(1,abs(T_oil_C)))
        [reference20, basis20] = build_20C_reference(params_frozen, stiffness_results, damping_results);
        shared_out.reference20 = reference20; shared_out.basis20 = basis20;
        shared_out.C_rayleigh_reference_t = reference20.C_rayleigh_reference_t;
        shared_out.Phi_reference30 = reference20.mode_shapes_full(:,1:cfg.modal.certified_reference_count);
    else
        require_shared_reference(shared_in);
        reference20 = shared_in.reference20; basis20 = shared_in.basis20;
    end
    shared_for_modal = shared_out;
    shared_for_modal.current_temperature_C = T_oil_C;
    modal_timer = tic;
    modal = build_temperature_modal_basis(model_cache, stiffness_results, damping_results, shared_for_modal, cfg);
    record.runtime.modal_reduction_s = toc(modal_timer);
    if ~modal.pass, error('Stage9B:ModalGate', 'The projected temperature modal basis did not pass its required gates.'); end
    record.audit.damping_contribution = damping_contribution_snapshot(modal, cfg.modal.tracked_mode_count);

    if case_index == 1
        tracking = track_thermal_modes(reference20.mode_shapes_full(:,1:cfg.modal.tracked_mode_count), ...
            reference20.mode_shapes_full(:,1:cfg.modal.tracked_mode_count), reference20.M_t, ...
            reference20.frequency_Hz(1:cfg.modal.tracked_mode_count), reference20.frequency_Hz(1:cfg.modal.tracked_mode_count), cfg);
        shared_out.previous_tracked_modes = reference20.mode_shapes_full(:,1:cfg.modal.tracked_mode_count);
        shared_out.previous_tracked_frequency_Hz = reference20.frequency_Hz(1:cfg.modal.tracked_mode_count);
        dynamics_reference = reference20; dynamics_basis = basis20;
    else
        tracking = track_thermal_modes(shared_in.previous_tracked_modes, modal.mode_shapes_full(:,1:cfg.modal.tracked_mode_count), ...
            reference20.M_t, shared_in.previous_tracked_frequency_Hz, modal.frequency_full_Hz(1:cfg.modal.tracked_mode_count), cfg);
        if ~tracking.tracking_pass, error('Stage9B:TrackingGate', 'Temperature modal tracking did not pass MAC and cluster-MAC gates.'); end
        shared_out.previous_tracked_modes = tracking.tracked_modes;
        shared_out.previous_tracked_frequency_Hz = tracking.tracked_frequency_Hz(:);
        dynamics_reference = reference20; dynamics_reference.temperature_C = T_oil_C; dynamics_basis = modal;
    end
    if ~tracking.tracking_pass, error('Stage9B:TrackingGate', 'The 20 C identity MAC tracking check did not pass.'); end

    dynamics_timer = tic;
    dynamics = simulate_reduced_free_decay(dynamics_reference, dynamics_basis, validation, struct());
    record.runtime.free_decay_s = toc(dynamics_timer);
    if ~dynamics.pass, error('Stage9B:DynamicsGate', 'Reduced free decay must pass all Stage8B gates.'); end

    record = map_record(record, thermal_result, contacts, stiffness_results, damping_results, validation, modal, tracking, dynamics, params_base, cfg);
    record.runtime.total_case_s = toc(total_timer); record.meta.completed_at = datetime('now'); record.meta.elapsed_s = record.runtime.total_case_s;
    record.meta.status = "completed";
    continuation_out = continuation_from_record(record, thermal_result);
catch ME
    record.meta.status = "failed"; record.meta.completed_at = datetime('now'); record.runtime.total_case_s = toc(total_timer); record.meta.elapsed_s = record.runtime.total_case_s;
    record.error = struct('identifier',string(ME.identifier),'message',string(ME.message),'function_name',string(ME.stack(1).name), ...
        'temperature_C',T_oil_C,'outer_iteration',NaN,'mechanical_load_step',NaN,'newton_iteration',NaN);
end
end

function validate_case_inputs(case_index, T_oil_C, params, cache, cfg)
if ~isscalar(case_index) || case_index ~= floor(case_index) || case_index < 1 || case_index > numel(cfg.temperature_list_C) || ...
        cfg.temperature_list_C(case_index) ~= T_oil_C || ~isfinite(T_oil_C)
    error('Stage9B:CaseIdentity', 'case_index and T_oil_C must identify one configured temperature.');
end
required = {'M','K_structure','C_foundation','G_unit','modelInfo','transverse_dofs','B_ball','B_roller'};
if ~isstruct(cache) || ~all(isfield(cache,required)) || ~isequal(size(cache.M),[192 192]) || numel(cache.transverse_dofs) ~= 128
    error('Stage9B:ModelCache', 'A prebuilt 192-DOF model_cache is required; this function does not build the model.');
end
if ~isstruct(params) || ~isfield(params,'bearing') || numel(params.bearing) ~= 2
    error('Stage9B:Params', 'params_base must provide the two bearing records.');
end
end

function [q_initial, initialization] = initial_state_for_case(case_index, T_oil_C, continuation)
initialization = struct('continuation_used',false,'continued_from_temperature_C',NaN,'cold_start_fallback_used',false, ...
    'q0_source',"cold_start",'thermal_state_source',"T_oil_seed",'modal_full_fallback_used',false);
q_initial = [];
if case_index > 1 && isstruct(continuation) && isfield(continuation,'valid') && continuation.valid
    q_initial = continuation.q_static;
    initialization.continuation_used = true;
    initialization.continued_from_temperature_C = NaN;
    initialization.q0_source = "previous_case_q_static";
    initialization.thermal_state_source = "previous_temperature_rise_seed";
end
if case_index == 1 && abs(T_oil_C-20) > eps
    error('Stage9B:ReferenceCase', 'Stage9B-1 only authorizes the 20 C reference case.');
end
end

function require_thermal_gate(result, cfg)
if ~result.converged || result.outer_iterations > cfg.max_outer_iterations || result.temperature_residual_C > cfg.temperature_tolerance_C || ...
        result.power_residual_W > cfg.power_tolerance_W || result.mechanical_residual > cfg.mechanical_tolerance || ...
        result.axial_constraint_error_m > cfg.axial_constraint_tolerance_m || result.relative_Q_change > cfg.relative_load_tolerance || ...
        result.relative_film_change > cfg.relative_film_tolerance || result.final_consistency_recompute_count ~= 1
    error('Stage9B:ThermalGate', 'The frozen thermal-mechanical workpoint did not satisfy all Stage5 gates.');
end
end

function params = frozen_parameters(params, modelInfo, thermal_state)
params.modelInfo = modelInfo; params.num_rotor_dof = modelInfo.num_rotor_dof; params.num_case_dof = modelInfo.num_case_dof;
params.microphysics = microphysics_config(struct('thermal',struct('enabled',true,'mode','frozen_external'),'thermal_state',thermal_state));
end

function [reference, basis] = build_20C_reference(params, stiffness, damping)
evalc('reference = compute_reference_rayleigh_damping(params, stiffness, damping, struct());');
if ~reference.pass, error('Stage9B:ReferenceGate', 'The 20 C reference model must pass.'); end
evalc('basis = build_modal_reduction_basis(reference, struct());');
if ~basis.pass || basis.retained_mode_count ~= 22, error('Stage9B:ReferenceBasis', 'The 20 C reference must retain the verified 22-mode basis.'); end
end

function require_shared_reference(shared)
if ~isstruct(shared) || ~all(isfield(shared,{'reference20','basis20','C_rayleigh_reference_t','Phi_reference30','previous_tracked_modes','previous_tracked_frequency_Hz'}))
    error('Stage9B:ContinuationReference', 'Later temperatures require the previous 20 C shared reference state.');
end
end

function record = map_record(record, thermal, contacts, stiffness, damping, validation, modal, tracking, dynamics, params, cfg)
record.convergence = struct('outer_iterations',thermal.outer_iterations,'temperature_residual_C',thermal.temperature_residual_C, ...
    'power_residual_W',thermal.power_residual_W,'mechanical_residual',thermal.mechanical_residual, ...
    'axial_constraint_error_m',thermal.axial_constraint_error_m,'relative_Q_change',thermal.relative_Q_change, ...
    'relative_film_change',thermal.relative_film_change,'final_consistency_recompute_count',thermal.final_consistency_recompute_count, ...
    'pass',thermal.converged,'iteration_index',(1:thermal.outer_iterations).','ball_temperature_C',zeros(0,1), ...
    'roller_temperature_C',zeros(0,1),'temperature_residual_history_C',zeros(0,1),'power_residual_history_W',zeros(0,1),'mechanical_residual_history',zeros(0,1));
record.thermal.ball = thermal_record(thermal.thermal_state.ball, thermal.film_state.ball, thermal.Qfric_ball_W, params.bearing(1));
record.thermal.roller = thermal_record(thermal.thermal_state.roller, thermal.film_state.roller, thermal.Qfric_roller_W, params.bearing(2));
record.contact.ball = contact_record(contacts(1), 'ball'); record.contact.roller = contact_record(contacts(2), 'roller');
record.bearing.ball = bearing_record(stiffness(1), damping(1)); record.bearing.roller = bearing_record(stiffness(2), damping(2));
record.linearization = struct('requested_amplitude_m',cfg.linearization_amplitudes_m(1),'used_amplitude_m',validation.used_amplitude_m, ...
    'fallback_used',validation.fallback_used,'ball_error_plus',validation.bearing(1).force_error_plus, ...
    'ball_error_minus',validation.bearing(1).force_error_minus,'roller_error_plus',validation.bearing(2).force_error_plus, ...
    'roller_error_minus',validation.bearing(2).force_error_minus,'ball_loaded_set_stable',validation.bearing(1).major_loaded_set_stable, ...
    'roller_loaded_set_stable',validation.bearing(2).major_loaded_set_stable,'pass',validation.pass);
record.modal = struct('strategy_used',string(modal.strategy_used),'full_modal_fallback_used',modal.full_modal_fallback_used, ...
    'reference_subspace_count',modal.reference_subspace_count,'retained_mode_count',modal.retained_mode_count, ...
    'tracked_mode_count',cfg.modal.tracked_mode_count,'reference_frequency_Hz',tracking.tracked_frequency_Hz, ...
    'tracked_frequency_Hz',tracking.tracked_frequency_Hz,'frequency_shift_percent',zeros(1,cfg.modal.tracked_mode_count), ...
    'MAC_diagonal',tracking.MAC_diagonal,'minimum_MAC',tracking.minimum_MAC,'cluster_tracking_used',tracking.cluster_tracking_used, ...
    'effective_mass_x',modal.final_ratio_x,'effective_mass_y',modal.final_ratio_y,'mass_orthogonality_error',modal.mass_orthogonality_error, ...
    'stiffness_diagonalization_error',modal.stiffness_diagonalization_error,'frequency_consistency_error',modal.frequency_consistency_error, ...
    'frequency_consistency_tolerance',modal.frequency_tolerance,'maximum_ritz_backward_error',modal.maximum_ritz_backward_error, ...
    'maximum_state_real_part',dynamics.maximum_state_real_part,'tracking_pass',tracking.tracking_pass,'pass',modal.pass && tracking.tracking_pass);
record.dynamics = struct('used_amplitude_m',dynamics.used_amplitude_m,'dt_s',dynamics.dt,'internal_step_count',dynamics.n_internal_steps, ...
    'output_stride',dynamics.output_stride,'saved_step_count',dynamics.n_saved_steps,'end_time_s',dynamics.time(end), ...
    'maximum_state_real_part',dynamics.maximum_state_real_part,'maximum_energy_growth_ratio',dynamics.maximum_energy_growth_ratio, ...
    'initial_energy_J',dynamics.initial_energy_J,'final_energy_J',dynamics.final_energy_J,'factorization_count',dynamics.factorization_count,'pass',dynamics.pass);
record.response.time_s = dynamics.time; record.response.saved_internal_step_index = round(dynamics.time/dynamics.dt);
record.response.uniform_sample_dt_s = dynamics.output_stride*dynamics.dt; record.response.displacement_m = dynamics.displacement;
record.response.velocity_m_s = dynamics.velocity; record.response.acceleration_m_s2 = dynamics.acceleration;
record.response.fft_valid_sample_count = nnz(mod(record.response.saved_internal_step_index, dynamics.output_stride) == 0);
record.audit.bearing_force_call_count = sum([stiffness.force_evaluation_count]) + 3;
record.audit.thermal_update_count = thermal.outer_iterations + thermal.final_consistency_recompute_count;
record.audit.stiffness_difference_call_count = sum([stiffness.force_evaluation_count]);
record.audit.newmark_force_call_count = dynamics.newmark_force_call_count; record.audit.newmark_thermal_update_count = dynamics.thermal_update_count;
record.audit.newmark_factorization_count = dynamics.factorization_count; record.audit.full_modal_solve_count = modal.full_modal_solve_count; record.audit.checkpoint_write_count = 0;
record.warnings = strings(0,1);
end

function output = thermal_record(state, film, power, bearing)
output = struct('T_final_C',state.T_final,'T_film_C',state.T_film,'T_inner_C',state.T_inner,'T_outer_C',state.T_outer, ...
    'T_element_C',state.T_element,'viscosity_Pa_s',state.eta,'pressure_viscosity_Pa_inv',state.alpha_p, ...
    'working_clearance_m',state.working_clearance,'clearance_change_m',state.working_clearance-bearing.clearance0, ...
    'thermal_preload',state.thermal_preload,'friction_power_W',power,'minimum_loaded_film_m',film.hmin_m);
end

function output = contact_record(state, type)
if strcmp(type,'ball'), loads = reshape(state.Q,1,[]); else, loads = sum(state.Q,2).'; end
mask = loads > 0; maximum = max(loads); if any(mask), nonuniformity = maximum/mean(loads(mask)); index = find(loads == maximum,1); else, nonuniformity = NaN; index = NaN; end
output = struct('body_load_N',loads,'loaded_mask',mask,'loaded_contact_count',nnz(mask),'total_contact_load_N',sum(loads), ...
    'maximum_contact_load_N',maximum,'load_nonuniformity',nonuniformity,'maximum_load_body_index',index);
end

function output = bearing_record(stiffness, damping)
Kxx = stiffness.K_local(1,1); Kyy = stiffness.K_local(2,2); Cxx = damping.C_local(1,1); Cyy = damping.C_local(2,2);
output = struct('K_local',stiffness.K_local,'C_ehl_local',damping.C_local,'active_dof_indices',stiffness.active_dof_indices, ...
    'stiffness_eigenvalues',reshape(stiffness.eigenvalues_symmetric_part,1,[]),'damping_eigenvalues',reshape(damping.eigenvalues_active,1,[]), ...
    'radial_Kxx_N_m',Kxx,'radial_Kyy_N_m',Kyy,'radial_Cxx_Ns_m',Cxx,'radial_Cyy_Ns_m',Cyy, ...
    'stiffness_anisotropy',max(abs([Kxx Kyy]))/max(min(abs([Kxx Kyy])),eps),'damping_anisotropy',max(abs([Cxx Cyy]))/max(min(abs([Cxx Cyy])),eps), ...
    'stiffness_pass',stiffness.pass,'damping_pass',damping.pass);
end

function continuation = continuation_from_record(record, thermal)
continuation = record.resume_state; continuation.valid = record.convergence.pass;
continuation.q_static = thermal.q_static; continuation.ball_T_final_C = thermal.thermal_state.ball.T_final; continuation.roller_T_final_C = thermal.thermal_state.roller.T_final;
continuation.ball_Q = thermal.contact_state.bearings(1).Q; continuation.roller_Q = thermal.contact_state.bearings(2).Q;
continuation.ball_film = thermal.film_state.ball.h_m; continuation.roller_film = thermal.film_state.roller.h_m;
continuation.ball_loaded_mask = thermal.film_state.ball.loaded_mask; continuation.roller_loaded_mask = thermal.film_state.roller.loaded_mask;
end

function snapshot = damping_contribution_snapshot(modal, tracked_mode_count)
required = {'M_r','omega_rad_s','C_rayleigh_r','C_foundation_r','C_ehl_r','retained_mode_count'};
if ~isstruct(modal) || ~all(isfield(modal, required)) || ~isscalar(tracked_mode_count) || ...
        tracked_mode_count ~= 6 || ~isfinite(tracked_mode_count)
    damping_runtime_state_missing('The retained modal damping state is incomplete.');
end

Mr = modal.M_r;
omega = modal.omega_rad_s(:);
CRr = modal.C_rayleigh_r;
CFr = modal.C_foundation_r;
CEr = modal.C_ehl_r;
n = modal.retained_mode_count;
if ~isscalar(n) || n ~= floor(n) || n < tracked_mode_count || ...
        ~isequal(size(Mr), [n n]) || ~isequal(size(CRr), [n n]) || ...
        ~isequal(size(CFr), [n n]) || ~isequal(size(CEr), [n n]) || numel(omega) ~= n || ...
        ~isreal(Mr) || ~isreal(omega) || ~isreal(CRr) || ~isreal(CFr) || ~isreal(CEr) || ...
        any(~isfinite([Mr(:); omega; CRr(:); CFr(:); CEr(:)]))
    damping_runtime_state_missing('The retained modal damping arrays are invalid.');
end

if symmetry_error(Mr) > 1e-10 || symmetry_error(CRr) > 1e-10 || ...
        symmetry_error(CFr) > 1e-10 || symmetry_error(CEr) > 1e-10 || ...
        ~positive_definite(Mr) || ~positive_semidefinite(CRr) || ...
        ~positive_semidefinite(CFr) || ~positive_semidefinite(CEr) || any(omega(1:tracked_mode_count) <= 0)
    damping_runtime_state_missing('The retained modal damping matrices failed the audit gate.');
end

CR = 0.5*(CRr + CRr.');
CF = 0.5*(CFr + CFr.');
CE = 0.5*(CEr + CEr.');
CT = CR + CF + CE;
n_track = 6;
mode_index = 1:n_track;
omega6 = reshape(omega(mode_index), 1, n_track);
modal_mass6 = reshape(diag(Mr(mode_index, mode_index)), 1, n_track);
c_rayleigh_diag6 = reshape(diag(CR(mode_index, mode_index)), 1, n_track);
c_foundation_diag6 = reshape(diag(CF(mode_index, mode_index)), 1, n_track);
c_ehl_diag6 = reshape(diag(CE(mode_index, mode_index)), 1, n_track);
denominator6 = 2 .* omega6 .* modal_mass6;
if ~isequal(size(omega6), [1 n_track]) || ~isequal(size(modal_mass6), [1 n_track]) || ...
        ~isequal(size(c_rayleigh_diag6), [1 n_track]) || ~isequal(size(c_foundation_diag6), [1 n_track]) || ...
        ~isequal(size(c_ehl_diag6), [1 n_track]) || ~isequal(size(denominator6), [1 n_track])
    damping_audit_vector_shape_error('The six-mode damping inputs must be row vectors.');
end
if ~isreal(omega6) || ~isreal(modal_mass6) || ~isreal(denominator6) || ...
        any(~isfinite([omega6 modal_mass6 c_rayleigh_diag6 c_foundation_diag6 c_ehl_diag6 denominator6])) || ...
        any(omega6 <= 0) || any(modal_mass6 <= 0) || any(denominator6 <= 0)
    damping_runtime_state_missing('The six-mode damping inputs are invalid.');
end

zeta_rayleigh = c_rayleigh_diag6 ./ denominator6;
zeta_foundation = c_foundation_diag6 ./ denominator6;
zeta_ehl = c_ehl_diag6 ./ denominator6;
zeta_total = zeta_rayleigh + zeta_foundation + zeta_ehl;
if ~isequal(size(zeta_rayleigh), [1 n_track]) || ~isequal(size(zeta_foundation), [1 n_track]) || ...
        ~isequal(size(zeta_ehl), [1 n_track]) || ~isequal(size(zeta_total), [1 n_track])
    damping_audit_vector_shape_error('The six-mode damping contributions must be row vectors.');
end
component_zeta = [zeta_rayleigh; zeta_foundation; zeta_ehl];
if any(~isfinite([zeta_rayleigh zeta_foundation zeta_ehl zeta_total])) || any(zeta_total <= 0) || ...
        any(component_zeta < -1e-12*max(1, abs(zeta_total)), 'all')
    damping_runtime_state_missing('The modal damping contributions are not physically admissible.');
end

ehl_fraction = zeta_ehl./zeta_total;
zeta_identity_residual = max(abs(zeta_total-zeta_rayleigh-zeta_foundation-zeta_ehl));
if ~isequal(size(ehl_fraction), [1 n_track])
    damping_audit_vector_shape_error('The six-mode EHL fractions must be a row vector.');
end
if zeta_identity_residual > 1e-10*max(1, max(abs(zeta_total))) || ...
        max(abs(ehl_fraction-zeta_ehl./zeta_total)) > 1e-12
    damping_runtime_state_missing('The six-mode damping identities are inconsistent.');
end
trace_rayleigh = trace(CR);
trace_foundation = trace(CF);
trace_ehl = trace(CE);
trace_total = trace(CT);
if ~all(isfinite([trace_rayleigh trace_foundation trace_ehl trace_total])) || trace_total <= 0
    damping_runtime_state_missing('The retained modal damping traces are invalid.');
end

ehl_trace_fraction = trace_ehl/trace_total;
frobenius_ehl_fraction = norm(CE, 'fro')/norm(CT, 'fro');
maximum_ehl_modal_fraction = max(ehl_fraction);
if ~all(isfinite([ehl_fraction ehl_trace_fraction frobenius_ehl_fraction maximum_ehl_modal_fraction]))
    damping_runtime_state_missing('The modal damping contribution fractions are invalid.');
end

if maximum_ehl_modal_fraction < 0.10 && ehl_trace_fraction < 0.10
    classification = "LOW_EHL_SYSTEM_INFLUENCE";
elseif maximum_ehl_modal_fraction > 0.50 || ehl_trace_fraction > 0.50
    classification = "HIGH_EHL_SYSTEM_INFLUENCE_REQUIRES_MODEL_REVIEW";
else
    classification = "MODERATE_EHL_SYSTEM_INFLUENCE";
end

snapshot = struct('available', true, 'coordinate_space', "MASS_NORMALIZED_REDUCED_MODAL", ...
    'retained_mode_count', n, 'tracked_mode_count', tracked_mode_count, ...
    'zeta_rayleigh_1to6', zeta_rayleigh, 'zeta_foundation_1to6', zeta_foundation, ...
    'zeta_ehl_1to6', zeta_ehl, 'zeta_total_1to6', zeta_total, ...
    'ehl_fraction_1to6', ehl_fraction, 'trace_rayleigh', trace_rayleigh, ...
    'trace_foundation', trace_foundation, 'trace_ehl', trace_ehl, 'trace_total', trace_total, ...
    'ehl_trace_fraction', ehl_trace_fraction, 'frobenius_ehl_fraction', frobenius_ehl_fraction, ...
    'maximum_ehl_modal_fraction', maximum_ehl_modal_fraction, 'classification', classification, ...
    'gyroscopic_term_excluded', true, 'source_note', ...
    "Audit snapshot of dissipative damping contributions in the retained reduced modal space. Rayleigh, foundation and EHL matrices are taken directly from the already assembled runtime state. The gyroscopic term is excluded because it is non-dissipative. No damping coefficient is recomputed or modified.");
end

function value = symmetry_error(A)
value = norm(A-A.', 'fro')/max(norm(A, 'fro'), 1);
end

function pass = positive_definite(A)
[~, p] = chol(0.5*(A+A.'), 'lower');
pass = p == 0;
end

function pass = positive_semidefinite(A)
eigenvalues = eig(0.5*(A+A.'));
pass = min(eigenvalues) >= -1e-12*max(1, max(abs(eigenvalues)));
end

function damping_runtime_state_missing(message)
error('DAMPING_RUNTIME_STATE_MISSING', '%s', message);
end

function damping_audit_vector_shape_error(message)
error('DAMPING_AUDIT_VECTOR_SHAPE_ERROR', '%s', message);
end
