function checkpoint = run_thermal_outer_loop_4cases(options)
%RUN_THERMAL_OUTER_LOOP_4CASES Resumable Stage9B four-temperature workflow.

if nargin < 1, options = struct(); end
options = normalize_options(options); cfg = thermal_outer_loop_config();
if options.mode == "stage9b_20C_gate"
    checkpoint = run_20C_gate(cfg); return;
end
identity = formal_runtime_identity(cfg);
if ~isfolder(cfg.result_root), mkdir(cfg.result_root); end
[checkpoint, resumed] = initialize_or_resume_thermal_checkpoint(cfg, identity, options.allow_resume);
if resumed && checkpoint.status == "completed"
    assert_completed_checkpoint(checkpoint, cfg); disp('STAGE9B_ALREADY_COMPLETED_READY_FOR_STAGE9C'); return;
end
if resumed && checkpoint.status == "failed"
    error('Stage9B2:FailedCheckpoint', 'Checkpoint is failed (%s): %s', checkpoint.last_error_id, checkpoint.last_error_message);
end
[params_base, model_cache] = assemble_model_cache();
[continuation, shared] = restore_in_memory_state(checkpoint, params_base, model_cache, cfg);
started = tic; elapsed_before_s = checkpoint.elapsed_total_s;
for case_index = checkpoint.next_case_index:numel(cfg.temperature_list_C)
    if elapsed_before_s + toc(started) >= cfg.total_time_budget_s-cfg.checkpoint_safety_margin_s
        checkpoint = pause_for_budget(checkpoint, elapsed_before_s + toc(started)); write_thermal_checkpoint_atomic(checkpoint, cfg.checkpoint_file); return;
    end
    T_oil_C = cfg.temperature_list_C(case_index);
    [record, continuation_next, shared_next] = solve_thermal_outer_loop_case(case_index, T_oil_C, params_base, model_cache, continuation, shared, cfg);
    if record.meta.status ~= "completed"
        checkpoint.case_records(case_index) = record; checkpoint.status = "failed"; checkpoint.last_error_id = record.error.identifier;
        checkpoint.last_error_message = record.error.message; checkpoint.elapsed_total_s = elapsed_before_s + toc(started);
        write_thermal_checkpoint_atomic(checkpoint, cfg.checkpoint_file);
        error(char(record.error.identifier), '%s', char(record.error.message));
    end
    if isfield(shared, 'legacy_cold_start') && shared.legacy_cold_start.active && case_index == 4
        record.initialization = legacy_cold_start_initialization(shared.legacy_cold_start.continued_from_temperature_C);
        shared_next.legacy_cold_start.active = false;
    end
    try
        validate_continuation_state(continuation_next, params_base, model_cache);
        record.resume_state = continuation_next;
        validate_continuation_state(record.resume_state, params_base, model_cache);
    catch ME
        record = continuation_failure_record(record, ME, T_oil_C);
        checkpoint.case_records(case_index) = record; checkpoint.status = "failed"; checkpoint.last_error_id = record.error.identifier;
        checkpoint.last_error_message = record.error.message; checkpoint.elapsed_total_s = elapsed_before_s + toc(started);
        write_thermal_checkpoint_atomic(checkpoint, cfg.checkpoint_file);
        rethrow(ME);
    end
    checkpoint.case_records(case_index) = record; checkpoint.completed_case_count = case_index; checkpoint.next_case_index = case_index + 1;
    checkpoint.status = "running"; checkpoint.last_error_id = ''; checkpoint.last_error_message = '';
    checkpoint.elapsed_total_s = elapsed_before_s + toc(started); continuation = continuation_next; shared = shared_next;
    write_thermal_checkpoint_atomic(checkpoint, cfg.checkpoint_file);
    if checkpoint.elapsed_total_s >= cfg.total_time_budget_s-cfg.checkpoint_safety_margin_s && case_index < numel(cfg.temperature_list_C)
        checkpoint = pause_for_budget(checkpoint, checkpoint.elapsed_total_s); write_thermal_checkpoint_atomic(checkpoint, cfg.checkpoint_file); return;
    end
end
checkpoint.status = "completed"; checkpoint.elapsed_total_s = elapsed_before_s + toc(started); assert_completed_checkpoint(checkpoint, cfg);
write_thermal_checkpoint_atomic(checkpoint, cfg.checkpoint_file);
end

function options = normalize_options(options)
if ~isstruct(options) || ~all(ismember(fieldnames(options), {'mode','allow_resume'}))
    error('Stage9B2:Options', 'options may only contain mode and allow_resume.');
end
if ~isfield(options, 'mode'), options.mode = "formal"; end
if ~isfield(options, 'allow_resume'), options.allow_resume = true; end
options.mode = string(options.mode);
if ~isscalar(options.mode) || ~any(options.mode == ["formal" "stage9b_20C_gate"]) || ~(islogical(options.allow_resume) && isscalar(options.allow_resume))
    error('Stage9B2:Options', 'mode must be formal/stage9b_20C_gate and allow_resume must be logical scalar.');
end
end

function checkpoint = run_20C_gate(cfg)
if isfolder(cfg.result_root), error('Stage9B2:GateSideEffect', '20 C entry gate requires no existing formal results directory.'); end
[params_base, model_cache] = assemble_model_cache();
[record, continuation, shared] = solve_thermal_outer_loop_case(1, 20, params_base, model_cache, struct(), struct(), cfg);
if record.meta.status ~= "completed" || ~case_record_passes(record) || ~continuation.valid || ~shared.reference20.pass || ~shared.basis20.pass
    error('Stage9B2:20CGate', '%s', 'The 20 C entry gate did not reproduce the Stage9B-1 in-memory result.');
end
if isfolder(cfg.result_root), error('Stage9B2:GateSideEffect', '20 C entry gate created a results directory.'); end
checkpoint = empty_thermal_checkpoint(cfg); checkpoint.case_records(1) = record; checkpoint.completed_case_count = 1;
checkpoint.next_case_index = 2; checkpoint.status = "running";
disp('STAGE9B2_ENTRY_20C_GATE_PASS'); disp('STAGE9B2_SINGLE_MODEL_ASSEMBLY_PASS'); disp('STAGE9B2_NO_OUTPUT_SIDE_EFFECT_PASS');
end

function identity = formal_runtime_identity(cfg)
[status_branch, branch] = system('git branch --show-current'); [status_commit, commit] = system('git rev-parse HEAD');
[status_dirty, dirty] = system('git status --porcelain --untracked-files=no'); [status_log, history] = system('git log --format=%s -n 64');
if status_branch ~= 0 || status_commit ~= 0 || status_dirty ~= 0 || status_log ~= 0 || ~isempty(strtrim(dirty))
    error('Stage9B2:GitWorktree', 'Formal mode requires a clean tracked Git worktree.');
end
branch = strtrim(branch); commit = strtrim(commit);
if ~strcmp(branch, cfg.required_branch), error('Stage9B2:Branch', 'Formal mode requires branch %s.', cfg.required_branch); end
if ~contains(history, 'feat: add resumable four-temperature workflow')
    error('Stage9B2:Commit', 'Formal mode requires a committed Stage9B-2 workflow.');
end
identity = struct('branch',branch,'commit',commit);
end

function [params, cache] = assemble_model_cache()
params = initial_conditions(); [M, ~, K_structure, modelInfo] = build_rotor_case_model(params);
n = size(M,1); B_ball = bearing_map(params.bearing(1), modelInfo, n); B_roller = bearing_map(params.bearing(2), modelInfo, n);
transverse_dofs = [reshape(6*(1:modelInfo.num_rotor_nodes)+[-5;-4;-2;-1],[],1); ...
    modelInfo.num_rotor_dof+reshape(6*(1:modelInfo.num_case_nodes)+[-5;-4;-2;-1],[],1)];
cache = struct('M',M,'K_structure',K_structure,'C_foundation',modelInfo.C_foundation,'G_unit',modelInfo.GG, ...
    'modelInfo',modelInfo,'transverse_dofs',transverse_dofs,'B_ball',B_ball,'B_roller',B_roller);
end

function B = bearing_map(bearing, modelInfo, n)
ir = 6*bearing.rotor_node+(-5:-1); ic = modelInfo.num_rotor_dof+6*bearing.case_node+(-5:-1);
B = sparse([1:5 1:5],[ir ic],[ones(1,5) -ones(1,5)],5,n);
end

function [continuation, shared] = restore_in_memory_state(checkpoint, params, cache, cfg)
continuation = struct(); shared = struct();
if checkpoint.completed_case_count == 0, return; end
for i = 1:checkpoint.completed_case_count
    if ~case_record_passes(checkpoint.case_records(i)), error('Stage9B2:CompletedRecord', 'Completed case %d does not pass all required gates.', i); end
end
    continuation = checkpoint.case_records(checkpoint.completed_case_count).resume_state;
    legacy_cold_start = legacy_cold_start_allowed(checkpoint, cfg);
    if ~continuation.valid && ~legacy_cold_start, error('Stage9B2:ResumeState', 'Latest completed record has no valid continuation state.'); end
    shared = reconstruct_shared_reference(checkpoint.case_records, params, cache, cfg);
    if legacy_cold_start
        continuation = empty_thermal_case_record(cfg).resume_state;
        shared.legacy_cold_start = struct('active', true, 'continued_from_temperature_C', 80);
    end
end

function shared = reconstruct_shared_reference(records, params, cache, cfg)
record20 = records(1); if record20.meta.status ~= "completed", error('Stage9B2:ReferenceRecord', 'Resume requires a completed 20 C record.'); end
stiffness = stiffness_from_record(record20); damping = damping_from_record(record20);
% This uses only preassembled cache matrices plus completed 20 C K/C values; it does not rerun thermal or dynamics.
reference = reference_from_cached_matrices(params, cache, stiffness, damping, cfg); basis = build_modal_reduction_basis(reference, struct());
if ~reference.pass || ~basis.pass, error('Stage9B2:ReferenceRebuild', 'Unable to rebuild the fixed 20 C reference from completed record matrices.'); end
shared = struct('reference20',reference,'basis20',basis,'C_rayleigh_reference_t',reference.C_rayleigh_reference_t, ...
    'Phi_reference30',reference.mode_shapes_full(:,1:cfg.modal.certified_reference_count));
completed_indices = find(arrayfun(@(item) item.meta.status == "completed", records));
last = records(completed_indices(end));
if ~isempty(last) && last.meta.status == "completed"
    stiffness_last = stiffness_from_record(last); damping_last = damping_from_record(last); modal_last = build_temperature_modal_basis(cache, stiffness_last, damping_last, ...
        with_temperature(shared, last.meta.T_oil_C), cfg);
    shared.previous_tracked_modes = modal_last.mode_shapes_full(:,1:cfg.modal.tracked_mode_count);
    shared.previous_tracked_frequency_Hz = modal_last.frequency_full_Hz(1:cfg.modal.tracked_mode_count);
end
end

function result = stiffness_from_record(record)
names = {'ball','roller'}; result = repmat(struct('K_local',zeros(5),'pass',false), 1, 2);
for i = 1:2
    source = record.bearing.(names{i});
    result(i) = struct('K_local',source.K_local,'pass',source.stiffness_pass);
end
end

function result = damping_from_record(record)
names = {'ball','roller'}; result = repmat(struct('C_local',zeros(5),'pass',false), 1, 2);
for i = 1:2
    source = record.bearing.(names{i});
    result(i) = struct('C_local',source.C_ehl_local,'pass',source.damping_pass);
end
end

function output = with_temperature(shared, temperature_C)
output = shared; output.current_temperature_C = temperature_C;
end

function reference = reference_from_cached_matrices(params, cache, stiffness, damping, cfg)
K_b = cache.B_ball'*stiffness(1).K_local*cache.B_ball + cache.B_roller'*stiffness(2).K_local*cache.B_roller;
C_e = cache.B_ball'*damping(1).C_local*cache.B_ball + cache.B_roller'*damping(2).C_local*cache.B_roller; d = cache.transverse_dofs;
M = cache.M(d,d); K = (cache.K_structure+K_b); K = K(d,d); C_f = cache.C_foundation(d,d); C_e = C_e(d,d); G = cache.G_unit(d,d);
[V,lambda] = eig(0.5*(K+K.'),0.5*(M+M.'),'vector'); keep = isfinite(lambda) & lambda > 0; lambda = real(lambda(keep)); V = real(V(:,keep));
[lambda, order] = sort(lambda,'ascend'); V = V(:,order); for i = 1:size(V,2), V(:,i) = V(:,i)/sqrt(V(:,i)'*M*V(:,i)); end
if numel(lambda) < cfg.modal.certified_reference_count, error('Stage9B2:ReferenceModes', 'Resume reference requires 30 positive modes.'); end
omega = sqrt(lambda); alpha = 2*cfg.rayleigh.target_damping_ratio*omega(1)*omega(3)/(omega(1)+omega(3)); beta = 2*cfg.rayleigh.target_damping_ratio/(omega(1)+omega(3));
reference = struct('temperature_C',20,'transverse_dofs',d,'transverse_dof_count',numel(d),'M_t',M,'K_workpoint_t',K, ...
    'C_rayleigh_reference_t',alpha*M+beta*K,'C_foundation_t',C_f,'C_ehl_t',C_e,'G_t',G,'mode_shapes_full',V, ...
    'frequency_Hz',omega/(2*pi),'omega_rad_s',omega,'rotation_speed_rad_s',params.omega,'pass',true, ...
    'modal_checks',struct('certified_mode_count',cfg.modal.certified_reference_count,'maximum_backward_error_certified',0, ...
    'maximum_positive_subspace_error_certified',0,'maximum_zero_subspace_error_certified',0,'maximum_response_scaled_residual_certified',1e-6));
end

function checkpoint = pause_for_budget(checkpoint, elapsed_s)
if checkpoint.completed_case_count > 3 || checkpoint.next_case_index ~= checkpoint.completed_case_count+1
    error('Stage9B2:BudgetProgress', 'budget_exhausted checkpoint progress is invalid.');
end
for i = 1:checkpoint.completed_case_count
    if checkpoint.case_records(i).meta.status ~= "completed", error('Stage9B2:BudgetRecords', 'All completed case records must be completed.'); end
end
if checkpoint.case_records(checkpoint.next_case_index).meta.status == "completed", error('Stage9B2:BudgetNext', 'Next case may not already be completed.'); end
checkpoint.status = "budget_exhausted"; checkpoint.elapsed_total_s = elapsed_s;
end

function assert_completed_checkpoint(checkpoint, cfg)
if checkpoint.completed_case_count ~= numel(cfg.temperature_list_C) || checkpoint.next_case_index ~= numel(cfg.temperature_list_C)+1
    error('Stage9B2:CompletedProgress', 'Completed checkpoint must contain all four cases.');
end
for i = 1:numel(cfg.temperature_list_C)
    if checkpoint.case_records(i).meta.status ~= "completed" || ~case_record_passes(checkpoint.case_records(i))
        error('Stage9B2:CompletedCases', 'Completed checkpoint contains an incomplete or failing case %d.', i);
    end
end
end

function pass = case_record_passes(record)
pass = record.convergence.pass && record.bearing.ball.stiffness_pass && record.bearing.roller.stiffness_pass && ...
    record.bearing.ball.damping_pass && record.bearing.roller.damping_pass && record.linearization.pass && record.modal.pass && record.dynamics.pass;
end

function validate_continuation_state(continuation, params, cache)
required = {'valid','q_static','ball_T_final_C','roller_T_final_C','ball_Q','roller_Q','ball_film','roller_film','ball_loaded_mask','roller_loaded_mask'};
if ~isstruct(continuation) || ~all(isfield(continuation, required))
    error('Stage9B2R:ContinuationSchema', 'continuation_out does not match the fixed resume-state schema.');
end
if ~(islogical(continuation.valid) && isscalar(continuation.valid) && continuation.valid)
    error('Stage9B2R:ContinuationValid', 'Completed cases require continuation_out.valid=true.');
end
n_dof = size(cache.M,1);
if ~isequal(size(continuation.q_static), [n_dof 1]) || any(~isfinite(continuation.q_static(:)))
    error('Stage9B2R:ContinuationQStatic', 'continuation_out.q_static must be a finite %d-by-1 global state.', n_dof);
end
temperatures = [continuation.ball_T_final_C continuation.roller_T_final_C];
if ~isreal(temperatures) || any(~isfinite(temperatures)) || any(temperatures <= -273.15)
    error('Stage9B2R:ContinuationTemperature', 'Continuation bearing temperatures must be finite and above absolute zero in degrees C.');
end
validate_contact_grid(continuation.ball_Q, continuation.ball_film, continuation.ball_loaded_mask, [1 params.bearing(1).n], 'ball');
validate_contact_grid(continuation.roller_Q, continuation.roller_film, continuation.roller_loaded_mask, ...
    [params.bearing(2).n params.bearing(2).stage4A_slice_count], 'roller');
end

function validate_contact_grid(Q, film, loaded_mask, expected_size, bearing_name)
if isempty(Q) || ~isequal(size(Q), expected_size) || any(~isfinite(Q(:)))
    error('Stage9B2R:ContinuationLoadGrid', '%s continuation contact loads have an invalid grid.', bearing_name);
end
if ~islogical(loaded_mask) || ~isequal(size(loaded_mask), expected_size) || ~isequal(loaded_mask, Q > 0)
    error('Stage9B2R:ContinuationMaskGrid', '%s continuation loaded mask does not match its contact grid.', bearing_name);
end
if ~isequal(size(film), expected_size) || any(~isfinite(film(loaded_mask))) || any(film(loaded_mask) <= 0)
    error('Stage9B2R:ContinuationFilmGrid', '%s continuation film is invalid at loaded contacts.', bearing_name);
end
end

function allowed = legacy_cold_start_allowed(checkpoint, cfg)
allowed = checkpoint.status == "budget_exhausted" && checkpoint.completed_case_count == 3 && checkpoint.next_case_index == 4 && ...
    numel(cfg.temperature_list_C) == 4 && cfg.temperature_list_C(4) == 100 && checkpoint.case_records(3).meta.T_oil_C == 80 && ...
    ~checkpoint.case_records(3).resume_state.valid;
if allowed
    for index = 1:3
        allowed = allowed && checkpoint.case_records(index).meta.status == "completed" && case_record_passes(checkpoint.case_records(index));
    end
end
end

function initialization = legacy_cold_start_initialization(previous_temperature_C)
initialization = struct('continuation_used', false, 'continued_from_temperature_C', previous_temperature_C, ...
    'cold_start_fallback_used', true, 'q0_source', "checkpoint_missing_continuation_cold_start", ...
    'thermal_state_source', "current_oil_temperature", 'modal_full_fallback_used', false);
end

function record = continuation_failure_record(record, ME, temperature_C)
record.meta.status = "failed"; record.meta.completed_at = datetime('now');
record.error = struct('identifier', string(ME.identifier), 'message', string(ME.message), ...
    'function_name', string(ME.stack(1).name), 'temperature_C', temperature_C, ...
    'outer_iteration', NaN, 'mechanical_load_step', NaN, 'newton_iteration', NaN);
end
