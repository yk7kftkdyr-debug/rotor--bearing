function [results,info] = update_stage_d_canonical_results( ...
    results,payload,stage_d_source_commit,operation)
%UPDATE_STAGE_D_CANONICAL_RESULTS Pure canonical Stage D state transitions.

if nargin < 4 || isempty(operation), operation = 'append'; end
before = results;
baseline = validate_stage_d_four_temperature_results(struct('results',results));
if ~baseline.passed
    if ~baseline.frozen_damping_unchanged
        error('StageD:FrozenDampingArtifactChanged', ...
            'FROZEN_DAMPING_ARTIFACT_CHANGED: the frozen Stage C reference changed.');
    end
    error('StageD:CanonicalResultArtifactConflict', ...
        'CANONICAL_RESULT_ARTIFACT_CONFLICT: the canonical artifact is invalid.');
end
if isfield(results.meta,'stage_d_source_commit') && ...
        ~strcmp(char(results.meta.stage_d_source_commit),stage_d_source_commit)
    error('StageD:CanonicalResultArtifactConflict', ...
        'CANONICAL_RESULT_ARTIFACT_CONFLICT: Stage D source commit differs.');
end

switch operation
    case 'mark_attempt'
        temperature = payload.temperature_C;
        results = initialize_stage_d(results,baseline,stage_d_source_commit, ...
            payload.additional_results_check);
        results.progress = initialize_progress(results.progress);
        assert_no_failure(results.progress);
        if temperature ~= baseline.next_temperature_C
            error('StageD:ContinuationOrder', ...
                'Stage D temperatures must be attempted in the order 50, 80, 100 C.');
        end
        attempted_field = sprintf('stage_d_T%d_attempted',temperature);
        if results.progress.(attempted_field)
            error('StageD:FormalAttemptAlreadyConsumed', ...
                'FOUR_TEMPERATURE_STATIC_STATE_FAILED: formal attempt already consumed.');
        end
        results.progress.(attempted_field) = true;
        results.progress.current_attempt_temperature_C = temperature;
        results.progress.current_attempt_started_at = datetime('now');
        results.progress.current_attempt_process_id = payload.process_id;
        results.progress.current_attempt_session_id = payload.session_id;
        results.decision.status = 'STAGE_D_STATIC_SOLVE_IN_PROGRESS';
        results.decision.stage_d_status = 'STAGE_D_STATIC_SOLVE_IN_PROGRESS';
        info = struct('updated',true,'operation',operation,'temperature_C',temperature);

    case 'append'
        state = payload; temperature = state.requested_case_temperature_C;
        field = sprintf('T%d',temperature);
        if isfield(results.static,field)
            if isequaln(results.static.(field),state)
                info = struct('updated',false,'operation',operation, ...
                    'already_accepted',true,'temperature_C',temperature); return;
            end
            error('StageD:CanonicalResultArtifactConflict', ...
                'CANONICAL_RESULT_ARTIFACT_CONFLICT: accepted state cannot be overwritten.');
        end
        assert_no_failure(results.progress);
        if temperature ~= baseline.next_temperature_C || ...
                ~attempt_consumed(results.progress,temperature)
            error('StageD:ContinuationOrder', ...
                'A persisted attempt marker for the next temperature is required.');
        end
        case_validation = validate_stage_d_static_temperature_case( ...
            state,results.meta.model_dof_count);
        if ~case_validation.passed
            error('StageD:UnacceptedState', ...
                'FOUR_TEMPERATURE_STATIC_STATE_FAILED: only accepted states may be appended.');
        end
        results.static.(field) = state;
        results.validation.stage_d.(field) = case_validation;
        results.progress.(['stage_d_' field '_complete']) = true;
        results.progress.last_completed_gate = ['STAGE_D_' field '_ACCEPTED'];
        results.progress.last_solver_process_id = ...
            results.progress.current_attempt_process_id;
        results.progress.last_solver_session_id = ...
            results.progress.current_attempt_session_id;
        results.progress.current_attempt_temperature_C = NaN;
        results.progress.current_attempt_started_at = NaT;
        results.progress.current_attempt_process_id = NaN;
        results.progress.current_attempt_session_id = '';
        results.progress.stage_d_static_states_complete = temperature == 100;
        results.progress.stage_d_complete = false;
        results.decision.status = 'STAGE_D_STATIC_STATES_IN_PROGRESS';
        results.decision.stage_d_status = 'STAGE_D_STATIC_STATES_IN_PROGRESS';
        if temperature == 100
            results.decision.status = 'PENDING_INDEPENDENT_RELOAD';
            results.decision.stage_d_status = 'PENDING_INDEPENDENT_RELOAD';
        end
        results.decision.allow_stage_e = false;
        info = struct('updated',true,'operation',operation, ...
            'already_accepted',false,'temperature_C',temperature);

    case 'record_failure'
        temperature = payload.temperature_C;
        if ~attempt_consumed(results.progress,temperature) || ...
                isfield(results.static,sprintf('T%d',temperature))
            error('StageD:CanonicalResultArtifactConflict', ...
                'Failure may only be recorded for the consumed unaccepted attempt.');
        end
        if isfinite(results.progress.first_failed_temperature_C)
            error('StageD:FormalAttemptAlreadyConsumed', ...
                'FOUR_TEMPERATURE_STATIC_STATE_FAILED: first failure is already recorded.');
        end
        results.progress.first_failed_temperature_C = temperature;
        results.progress.failure_reason = payload.main_status;
        results.progress.failure_gate = payload.failure_gate;
        results.progress.last_solver_process_id = ...
            results.progress.current_attempt_process_id;
        results.progress.last_solver_session_id = ...
            results.progress.current_attempt_session_id;
        results.progress.current_attempt_temperature_C = NaN;
        results.progress.current_attempt_started_at = NaT;
        results.progress.current_attempt_process_id = NaN;
        results.progress.current_attempt_session_id = '';
        results.progress.stage_d_complete = false;
        results.validation.stage_d.failure = struct( ...
            'temperature_C',temperature,'main_status',payload.main_status, ...
            'failure_gate',payload.failure_gate, ...
            'diagnostic_identifier',payload.diagnostic_identifier, ...
            'diagnostic_message',payload.diagnostic_message, ...
            'source_commit',stage_d_source_commit,'unaccepted_state_saved',false);
        results.decision.status = payload.main_status;
        results.decision.stage_d_status = payload.main_status;
        results.decision.allow_stage_e = false;
        info = struct('updated',true,'operation',operation,'temperature_C',temperature);

    case 'finalize'
        if ~baseline.all_static_states_accepted || ~baseline.finalization_pending || ...
                isfinite(baseline.failed_temperature_C)
            error('StageD:ResultReloadFailed', ...
                'STAGE_D_RESULT_RELOAD_FAILED: accepted T50/T80/T100 are required.');
        end
        if ~payload.additional_results_check.passed
            error('StageD:ResultReloadFailed', ...
                'STAGE_D_RESULT_RELOAD_FAILED: additional result files exist.');
        end
        if payload.process_id == results.progress.last_solver_process_id || ...
                strcmp(payload.session_id,results.progress.last_solver_session_id)
            error('StageD:ResultReloadFailed', ...
                'STAGE_D_RESULT_RELOAD_FAILED: finalization requires a new MATLAB process.');
        end
        results.validation.stage_d.additional_results_check = ...
            payload.additional_results_check;
        results.validation.stage_d.independent_reload = struct( ...
            'passed',true,'checked_at',datetime('now'), ...
            'source_commit',stage_d_source_commit, ...
            'finalize_process_id',payload.process_id, ...
            'finalize_session_id',payload.session_id, ...
            'last_solver_process_id',results.progress.last_solver_process_id, ...
            'last_solver_session_id',results.progress.last_solver_session_id, ...
            'solver_rerun',false,'frequency_response_run',false,'newmark_run',false);
        results.progress.stage_d_complete = true;
        results.progress.last_completed_gate = 'STAGE_D_INDEPENDENT_RELOAD_ACCEPTED';
        results.decision.status = 'FOUR_TEMPERATURE_STATIC_STATES_ACCEPTED';
        results.decision.stage_d_status = 'FOUR_TEMPERATURE_STATIC_STATES_ACCEPTED';
        results.decision.stage_d_executed = true;
        results.decision.allow_stage_d = false;
        results.decision.allow_stage_e = true;
        info = struct('updated',true,'operation',operation,'temperature_C',NaN);

    otherwise
        error('StageD:CanonicalOperation','Unknown Stage D canonical operation.');
end

previous_sequence = 0;
if isfield(before.progress,'stage_d_transaction_sequence')
    previous_sequence = before.progress.stage_d_transaction_sequence;
end
results.progress.stage_d_transaction_sequence = previous_sequence+1;

if ~isequaln(before.damping,results.damping) || ...
        ~isequaln(before.static.T20,results.static.T20) || ...
        ~isequaln(before.modal.T20.anchor_modes,results.modal.T20.anchor_modes)
    error('StageD:FrozenDampingArtifactChanged', ...
        'FROZEN_DAMPING_ARTIFACT_CHANGED: Stage D modified a frozen Stage C value.');
end
post = validate_stage_d_four_temperature_results(struct('results',results));
if ~post.passed
    error('StageD:CanonicalResultArtifactConflict', ...
        'CANONICAL_RESULT_ARTIFACT_CONFLICT: updated artifact failed validation.');
end
end

function results = initialize_stage_d(results,baseline,source_commit,audit)
if ~isfield(results.validation,'stage_d'), results.validation.stage_d = struct(); end
if ~isfield(results.validation.stage_d,'frozen_stage_c_reference')
    if ~audit.passed
        error('StageD:CanonicalResultArtifactConflict', ...
            'CANONICAL_RESULT_ARTIFACT_CONFLICT: additional result files exist.');
    end
    results.validation.stage_d.frozen_stage_c_reference = ...
        baseline.frozen_stage_c_summary;
    results.validation.stage_d.T20_reference_review = struct( ...
        'passed',baseline.checks.T20 && baseline.checks.stage_c_baseline, ...
        'source_commit',results.static.T20.source_commit, ...
        'q_static_size',size(results.static.T20.q_static), ...
        'formal_acceptance',results.static.T20.formal_acceptance, ...
        'stage_c_validator_passed',baseline.checks.stage_c_baseline, ...
        'reviewed_without_resolve',true);
end
results.validation.stage_d.additional_results_check = audit;
results.meta.stage_d_source_commit = source_commit;
results.progress = initialize_progress(results.progress);
results.decision.status = 'STAGE_D_INITIALIZED';
results.decision.stage_d_status = 'STAGE_D_INITIALIZED';
results.decision.stage_d_executed = true;
results.decision.allow_stage_d = false;
results.decision.allow_stage_e = false;
end

function progress = initialize_progress(progress)
defaults = struct('stage_d_T50_attempted',false,'stage_d_T80_attempted',false, ...
    'stage_d_T100_attempted',false,'stage_d_T50_complete',false, ...
    'stage_d_T80_complete',false,'stage_d_T100_complete',false, ...
    'stage_d_static_states_complete',false,'stage_d_complete',false, ...
    'first_failed_temperature_C',NaN,'failure_reason','', ...
    'failure_gate','','current_attempt_temperature_C',NaN, ...
    'current_attempt_started_at',NaT,'current_attempt_process_id',NaN, ...
    'current_attempt_session_id','','last_solver_process_id',NaN, ...
    'last_solver_session_id','','stage_d_transaction_sequence',0);
names = fieldnames(defaults);
for k = 1:numel(names)
    if ~isfield(progress,names{k}), progress.(names{k}) = defaults.(names{k}); end
end
end

function pass = attempt_consumed(progress,temperature)
field = sprintf('stage_d_T%d_attempted',temperature);
pass = isfield(progress,field) && logical(progress.(field));
end

function assert_no_failure(progress)
if isfield(progress,'first_failed_temperature_C') && ...
        isfinite(progress.first_failed_temperature_C)
    error('StageD:FormalAttemptAlreadyConsumed', ...
        'FOUR_TEMPERATURE_STATIC_STATE_FAILED: Stage D stopped at its first failure.');
end
end
