function checkpoint = run_thermal_outer_loop_4cases(options)
%RUN_THERMAL_OUTER_LOOP_4CASES Resumable Stage9B four-temperature workflow.

if nargin < 1, options = struct(); end
options = normalize_options(options); cfg = thermal_outer_loop_config();
if options.mode == "stage9b_20C_gate"
    checkpoint = run_20C_gate(cfg); return;
end
identity = formal_runtime_identity(cfg);
if ~isfolder(cfg.result_root), mkdir(cfg.result_root); end
migrate_final_case_checkpoint_if_allowed(cfg, identity);
[checkpoint, resumed] = initialize_or_resume_thermal_checkpoint(cfg, identity, options.allow_resume);
if resumed && checkpoint.status == "completed"
    assert_completed_checkpoint(checkpoint, cfg); disp('STAGE9B_ALREADY_COMPLETED_READY_FOR_STAGE9C'); return;
end
if resumed && checkpoint.status == "failed"
    error('Stage9B2:FailedCheckpoint', 'Checkpoint is failed (%s): %s', checkpoint.last_error_id, checkpoint.last_error_message);
end
[params_base, model_cache] = assemble_model_cache();
protected_records = checkpoint.case_records(1:checkpoint.completed_case_count); protected_elapsed_s = checkpoint.elapsed_total_s;
[continuation, shared, restore_info] = restore_in_memory_state(checkpoint, params_base, model_cache, cfg);
assert_completed_records_unchanged(checkpoint, protected_records, protected_elapsed_s);
if restore_info.performed
    disp('STAGE9B2R4_REFERENCE20_PASS'); disp('STAGE9B2R4_MODAL80_ANCHOR_PASS');
    if ~restore_info.used_50C_fallback, disp('STAGE9B2R4_MINIMUM_RECONSTRUCTION_PASS'); end
    if restore_info.final_case_cold_start, disp('STAGE9B2R4_FINAL_CASE_COLD_START_PASS'); end
end
started = tic; elapsed_before_s = checkpoint.elapsed_total_s;
for case_index = checkpoint.next_case_index:numel(cfg.temperature_list_C)
    if elapsed_before_s + toc(started) >= cfg.total_time_budget_s-cfg.checkpoint_safety_margin_s && ...
            ~final_case_budget_override_allowed(options, checkpoint, cfg)
        if checkpoint.status == "budget_exhausted" && final_case_checkpoint_preconditions(checkpoint, cfg)
            return;
        end
        checkpoint = pause_for_budget(checkpoint, elapsed_before_s + toc(started)); write_thermal_checkpoint_atomic(checkpoint, cfg.checkpoint_file); return;
    end
    if final_case_budget_override_allowed(options, checkpoint, cfg), disp('STAGE9B2R4_BUDGET_OVERRIDE_GATE_PASS'); end
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
    assert_completed_records_unchanged(checkpoint, protected_records, protected_elapsed_s);
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
fprintf('STAGE9B_RUNTIME_ACCEPTANCE_PASS=%d\n', checkpoint.elapsed_total_s <= cfg.total_time_budget_s);
end

function options = normalize_options(options)
if ~isstruct(options) || ~all(ismember(fieldnames(options), {'mode','allow_resume','allow_final_case_after_budget'}))
    error('Stage9B2:Options', 'options may only contain mode, allow_resume and allow_final_case_after_budget.');
end
if ~isfield(options, 'mode'), options.mode = "formal"; end
if ~isfield(options, 'allow_resume'), options.allow_resume = true; end
if ~isfield(options, 'allow_final_case_after_budget'), options.allow_final_case_after_budget = false; end
options.mode = string(options.mode);
if ~isscalar(options.mode) || ~any(options.mode == ["formal" "stage9b_20C_gate"]) || ~(islogical(options.allow_resume) && isscalar(options.allow_resume)) || ...
        ~(islogical(options.allow_final_case_after_budget) && isscalar(options.allow_final_case_after_budget))
    error('Stage9B2:Options', 'mode must be formal/stage9b_20C_gate; allow_resume and allow_final_case_after_budget must be logical scalars.');
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

function [continuation, shared, restore_info] = restore_in_memory_state(checkpoint, params, cache, cfg)
continuation = struct(); shared = struct(); restore_info = struct('performed',false,'used_50C_fallback',false,'final_case_cold_start',false);
if checkpoint.completed_case_count == 0, return; end
for i = 1:checkpoint.completed_case_count
    if ~case_record_passes(checkpoint.case_records(i)), error('Stage9B2:CompletedRecord', 'Completed case %d does not pass all required gates.', i); end
end
    continuation = checkpoint.case_records(checkpoint.completed_case_count).resume_state;
    legacy_cold_start = legacy_cold_start_allowed(checkpoint, cfg);
    if ~continuation.valid && ~legacy_cold_start, error('Stage9B2:ResumeState', 'Latest completed record has no valid continuation state.'); end
    [shared, restore_info] = reconstruct_shared_reference(checkpoint.case_records, params, cache, cfg);
    if legacy_cold_start
        continuation = empty_thermal_case_record(cfg).resume_state;
        shared.legacy_cold_start = struct('active', true, 'continued_from_temperature_C', 80);
        restore_info.final_case_cold_start = true;
    end
end

function [shared, restore_info] = reconstruct_shared_reference(records, params, cache, cfg)
restore_info = struct('performed',true,'used_50C_fallback',false,'final_case_cold_start',false);
record20 = records(1); record80 = records(3);
if record20.meta.status ~= "completed" || record80.meta.status ~= "completed", error('Stage9B2R4:ReferenceRecord', 'Resume requires completed 20 C and 80 C records.'); end
reference20 = compute_reference_rayleigh_damping(frozen_reference_parameters(params, cache.modelInfo, 20), ...
    stiffness_from_record(record20), damping_from_record(record20), struct());
if ~reference20.pass || reference20.modal_checks.certified_mode_count ~= cfg.modal.certified_reference_count
    error('Stage9B2R4:Reference20', 'The rebuilt 20 C reference did not pass its 30-mode certification.');
end
shared = struct('reference20',reference20,'basis20',struct(),'C_rayleigh_reference_t',reference20.C_rayleigh_reference_t, ...
    'Phi_reference30',reference20.mode_shapes_full(:,1:cfg.modal.certified_reference_count));
modal80 = build_temperature_modal_basis(cache, stiffness_from_record(record80), damping_from_record(record80), with_temperature(shared, 80), cfg);
tracking80 = anchored_tracking(reference20.mode_shapes_full(:,1:cfg.modal.tracked_mode_count), reference20.frequency_Hz(1:cfg.modal.tracked_mode_count), ...
    modal80, reference20.M_t, record80, cfg);
if ~tracking80.pass
    record50 = records(2); modal50 = build_temperature_modal_basis(cache, stiffness_from_record(record50), damping_from_record(record50), with_temperature(shared, 50), cfg);
    tracking50 = track_thermal_modes(reference20.mode_shapes_full(:,1:cfg.modal.tracked_mode_count), modal50.Phi_t, reference20.M_t, ...
        reference20.frequency_Hz(1:cfg.modal.tracked_mode_count), modal50.frequency_Hz, cfg);
    if ~modal50.pass || ~tracking50.tracking_pass, error('Stage9B2R4:Modal50', 'The required 50 C modal fallback did not pass.'); end
    tracking80 = anchored_tracking(tracking50.tracked_modes, tracking50.tracked_frequency_Hz, modal80, reference20.M_t, record80, cfg);
    if ~tracking80.pass, error('Stage9B2R4:Modal80Anchor', 'The rebuilt 80 C modal state did not match its stored frequency anchor.'); end
    restore_info.used_50C_fallback = true;
end
shared.previous_tracked_modes = tracking80.tracking.tracked_modes;
shared.previous_tracked_frequency_Hz = tracking80.tracking.tracked_frequency_Hz(:);
end

function result = stiffness_from_record(record)
names = {'ball','roller'}; result = repmat(struct('K_local',zeros(5),'active_dof_indices',zeros(1,5),'pass',false), 1, 2);
for i = 1:2
    source = record.bearing.(names{i});
    result(i) = struct('K_local',source.K_local,'active_dof_indices',source.active_dof_indices,'pass',source.stiffness_pass);
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

function params_frozen = frozen_reference_parameters(params, modelInfo, T_oil_C)
params_frozen = params; params_frozen.modelInfo = modelInfo; params_frozen.num_rotor_dof = modelInfo.num_rotor_dof; params_frozen.num_case_dof = modelInfo.num_case_dof;
thermal_state = struct('ball',struct('T_oil',T_oil_C),'roller',struct('T_oil',T_oil_C));
params_frozen.microphysics = struct('thermal',struct('enabled',true,'mode','frozen_external'),'thermal_state',thermal_state);
end

function anchored = anchored_tracking(previous_modes, previous_frequency_Hz, modal, M_t, record80, cfg)
anchored = struct('tracking',struct(),'frequency_anchor_error',Inf,'frequency_anchor_tolerance',NaN,'pass',false);
stored_frequency = record80.modal.tracked_frequency_Hz; stored_MAC = record80.modal.MAC_diagonal;
if ~modal.pass || numel(stored_frequency) ~= cfg.modal.tracked_mode_count || numel(stored_MAC) ~= cfg.modal.tracked_mode_count || ...
        any(~isfinite(stored_frequency)) || any(stored_frequency <= 0) || any(~isfinite(stored_MAC)) || any(stored_MAC < 0 | stored_MAC > 1+100*eps)
    return;
end
tracking = track_thermal_modes(previous_modes, modal.Phi_t, M_t, previous_frequency_Hz, modal.frequency_Hz, cfg);
tol80 = max(1e-6, record80.modal.frequency_consistency_tolerance);
if ~isfinite(tol80) || tol80 > 1e-4, return; end
anchored.tracking = tracking; anchored.frequency_anchor_tolerance = tol80;
anchored.frequency_anchor_error = max(abs(tracking.tracked_frequency_Hz-stored_frequency)./max(abs(stored_frequency),1));
anchored.pass = tracking.tracking_pass && isfinite(anchored.frequency_anchor_error) && anchored.frequency_anchor_error <= tol80;
end

function assert_completed_records_unchanged(checkpoint, records_before, elapsed_before_s)
count = numel(records_before);
if ~isequaln(checkpoint.case_records(1:count), records_before) || checkpoint.elapsed_total_s ~= elapsed_before_s
    error('Stage9B2R4:CompletedRecordsChanged', 'Recovery and final-case execution may not modify completed records or prior elapsed time.');
end
end

function allowed = final_case_budget_override_allowed(options, checkpoint, cfg)
allowed = options.allow_final_case_after_budget && checkpoint.status == "budget_exhausted" && checkpoint.completed_case_count == 3 && ...
    checkpoint.next_case_index == 4 && numel(cfg.temperature_list_C) == 4 && cfg.temperature_list_C(4) == 100 && ...
    checkpoint.case_records(3).meta.T_oil_C == 80 && ~checkpoint.case_records(3).resume_state.valid && ...
    result_directory_has_only_checkpoint(cfg);
if allowed
    for index = 1:3
        allowed = allowed && checkpoint.case_records(index).meta.status == "completed" && case_record_passes(checkpoint.case_records(index));
    end
end
end

function migrate_final_case_checkpoint_if_allowed(cfg, identity)
if ~isfile(cfg.checkpoint_file), return; end
loaded = load(cfg.checkpoint_file);
if ~isequal(fieldnames(loaded), {'checkpoint'}), error('Stage9B2R4:CheckpointVariable', 'The sole checkpoint must contain only checkpoint.'); end
checkpoint = loaded.checkpoint;
legacy_commit = '6e2d8295f8a52e2fa5c243fe0d40b6bc6cced147';
if ~strcmp(checkpoint.source_commit, legacy_commit), return; end
if ~final_case_checkpoint_preconditions(checkpoint, cfg) || ~result_directory_has_only_checkpoint(cfg)
    error('Stage9B2R4:MigrationPrecondition', 'The current checkpoint is not the authorized final-case migration state.');
end
[ancestor_status, ~] = system(sprintf('git merge-base --is-ancestor %s %s', legacy_commit, identity.commit));
[diff_status, changed_files] = system(sprintf('git diff --name-only %s..%s', legacy_commit, identity.commit));
changed_files = string(splitlines(strtrim(changed_files))); changed_files = changed_files(changed_files ~= "");
if ancestor_status ~= 0 || diff_status ~= 0 || ~isequal(changed_files, "run_thermal_outer_loop_4cases.m")
    error('Stage9B2R4:MigrationCommit', 'Final-case migration requires the legacy commit to be an ancestor with only run_thermal_outer_loop_4cases.m changed.');
end
records_before = checkpoint.case_records(1:3); elapsed_before_s = checkpoint.elapsed_total_s;
checkpoint.source_commit = identity.commit;
assert_completed_records_unchanged(checkpoint, records_before, elapsed_before_s);
write_thermal_checkpoint_atomic(checkpoint, cfg.checkpoint_file);
end

function allowed = final_case_checkpoint_preconditions(checkpoint, cfg)
allowed = checkpoint.status == "budget_exhausted" && checkpoint.completed_case_count == 3 && checkpoint.next_case_index == 4 && ...
    numel(checkpoint.case_records) == 4 && numel(cfg.temperature_list_C) == 4 && cfg.temperature_list_C(4) == 100 && ...
    checkpoint.case_records(3).meta.T_oil_C == 80 && ~checkpoint.case_records(3).resume_state.valid;
if allowed
    for index = 1:3
        allowed = allowed && checkpoint.case_records(index).meta.status == "completed" && case_record_passes(checkpoint.case_records(index));
    end
end
end

function pass = result_directory_has_only_checkpoint(cfg)
entries = dir(cfg.result_root); names = string({entries.name}); names = names(names ~= "." & names ~= "..");
pass = isscalar(names) && names == "thermal_4cases_checkpoint.mat";
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
