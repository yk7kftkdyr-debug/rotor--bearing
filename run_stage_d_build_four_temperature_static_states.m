function results = run_stage_d_build_four_temperature_static_states( ...
    requested_temperature_C,action)
%RUN_STAGE_D_BUILD_FOUR_TEMPERATURE_STATIC_STATES One supervised transition.

if nargin < 2 || isempty(action), action = 'solve'; end
repository_root = fileparts(mfilename('fullpath'));
identity = require_source_state(repository_root);
canonical_path = fullfile(repository_root,'results', ...
    'thermal_equivalent_damping_frequency_domain', ...
    'thermal_4T_frequency_domain_results.mat');
stage_d_canonical_transaction('recover',canonical_path);
if ~isfile(canonical_path)
    error('StageD:BaselineMismatch', ...
        'STAGE_D_BASELINE_MISMATCH: canonical Stage C MAT is missing.');
end
audit = stage_d_canonical_transaction('audit',canonical_path);
if ~audit.passed
    error('StageD:CanonicalResultArtifactConflict', ...
        'CANONICAL_RESULT_ARTIFACT_CONFLICT: unexpected result files exist.');
end
try
    artifact = load(canonical_path);
catch
    error('StageD:CanonicalResultArtifactConflict', ...
        'CANONICAL_RESULT_ARTIFACT_CONFLICT: canonical MAT cannot be loaded.');
end
validation = validate_stage_d_four_temperature_results(artifact);
if ~validation.passed
    if ~validation.frozen_damping_unchanged
        error('StageD:FrozenDampingArtifactChanged', ...
            'FROZEN_DAMPING_ARTIFACT_CHANGED: frozen Stage C data changed.');
    end
    if isfield(validation.checks,'T20') && ~validation.checks.T20
        error('StageD:T20ReferenceStateInvalid', ...
            'T20_REFERENCE_STATE_INVALID: accepted 20 C reference is incomplete.');
    end
    error('StageD:CanonicalResultArtifactConflict', ...
        'CANONICAL_RESULT_ARTIFACT_CONFLICT: canonical Stage C/Stage D is invalid.');
end
results = artifact.results;
process_id = feature('getpid');
session_id = char(java.util.UUID.randomUUID);

switch action
    case 'finalize'
        final_payload = struct('additional_results_check',audit, ...
            'process_id',process_id,'session_id',session_id);
        [results,~] = update_stage_d_canonical_results(results, ...
            final_payload,identity.source_commit,'finalize');
        stage_d_canonical_transaction('save',canonical_path,results);
        final_artifact = load(canonical_path);
        final_validation = validate_stage_d_four_temperature_results(final_artifact);
        final_audit = stage_d_canonical_transaction('audit',canonical_path);
        if ~final_validation.passed || ~final_validation.stage_d_complete || ...
                ~final_audit.passed
            error('StageD:ResultReloadFailed', ...
                'STAGE_D_RESULT_RELOAD_FAILED: independent canonical reload failed.');
        end
        results = final_artifact.results;
        disp('FOUR_TEMPERATURE_STATIC_STATES_ACCEPTED');
        return;

    case 'record_timeout'
        accepted_field = sprintf('T%d',requested_temperature_C);
        if isfield(results.static,accepted_field)
            fprintf('STAGE_D_%s_ALREADY_ACCEPTED_AFTER_TRANSACTION_RECOVERY\n', ...
                accepted_field);
            return;
        end
        payload = failure_payload(requested_temperature_C, ...
            'STATIC_TEMPERATURE_CASE_TIMEOUT','EXTERNAL_25_MINUTE_HARD_STOP', ...
            'StageD:StaticTemperatureCaseTimeout', ...
            'The external supervisor terminated the single formal attempt at 25 minutes.');
        [results,~] = update_stage_d_canonical_results( ...
            results,payload,identity.source_commit,'record_failure');
        stage_d_canonical_transaction('save',canonical_path,results);
        error('StageD:StaticTemperatureCaseTimeout', ...
            'STATIC_TEMPERATURE_CASE_TIMEOUT: temperature %g C.',requested_temperature_C);

    case 'solve'
        % Continue below.

    otherwise
        error('StageD:CanonicalOperation','Unknown Stage D runner action.');
end

if validation.stage_d_complete
    disp('STAGE_D_ALREADY_COMPLETE'); return;
end
if isfinite(validation.failed_temperature_C)
    error('StageD:FormalAttemptAlreadyConsumed', ...
        'FOUR_TEMPERATURE_STATIC_STATE_FAILED: Stage D already stopped at %g C.', ...
        validation.failed_temperature_C);
end
if requested_temperature_C ~= validation.next_temperature_C
    existing_field = sprintf('T%d',requested_temperature_C);
    if isfield(results.static,existing_field)
        fprintf('STAGE_D_%s_ALREADY_ACCEPTED\n',existing_field); return;
    end
    error('StageD:ContinuationOrder', ...
        'Stage D must solve exactly the next temperature in 50, 80, 100 C order.');
end

previous_temperature_C = previous_temperature(requested_temperature_C);
previous = results.static.(sprintf('T%d',previous_temperature_C));
initial_state = struct('requested_case_temperature_C',previous_temperature_C, ...
    'q_static',previous.q_static,'M_reference',results.static.T20.M, ...
    'G_reference',results.static.T20.G);
params = initial_conditions();
attempt = struct('temperature_C',requested_temperature_C, ...
    'additional_results_check',audit,'process_id',process_id,'session_id',session_id);
[results,~] = update_stage_d_canonical_results( ...
    results,attempt,identity.source_commit,'mark_attempt');
try
    stage_d_canonical_transaction('save',canonical_path,results);
catch mark_exception
    stage_d_canonical_transaction('recover',canonical_path);
    persisted = load(canonical_path);
    if ~attempt_is_persisted(persisted.results,requested_temperature_C,session_id)
        rethrow(mark_exception);
    end
    results = persisted.results;
end
try
    state = solve_stage_d_static_temperature_case(requested_temperature_C, ...
        initial_state,params,identity.source_commit);
    [results,~] = update_stage_d_canonical_results( ...
        results,state,identity.source_commit,'append');
    stage_d_canonical_transaction('save',canonical_path,results);
catch exception
    stage_d_canonical_transaction('recover',canonical_path);
    canonical_attempt = load(canonical_path);
    accepted_field = sprintf('T%d',requested_temperature_C);
    if isfield(canonical_attempt.results.static,accepted_field)
        results = canonical_attempt.results;
        fprintf('STAGE_D_T%d_ACCEPTED_AND_RECOVERED_FROM_ATOMIC_TRANSACTION\n', ...
            requested_temperature_C);
        return;
    end
    failure = mapped_failure(requested_temperature_C,exception);
    [failed_results,~] = update_stage_d_canonical_results( ...
        canonical_attempt.results,failure,identity.source_commit,'record_failure');
    stage_d_canonical_transaction('save',canonical_path,failed_results);
    error('StageD:FourTemperatureStaticStateFailed', ...
        '%s: first failed temperature %g C; gate %s.', ...
        failure.main_status,requested_temperature_C,failure.failure_gate);
end
fprintf('STAGE_D_T%d_ACCEPTED_AND_CANONICAL_MAT_UPDATED\n', ...
    requested_temperature_C);
if requested_temperature_C == 100
    disp('STAGE_D_PENDING_INDEPENDENT_RELOAD');
end
end

function identity = require_source_state(repository_root)
original = pwd; cleanup = onCleanup(@() cd(original)); cd(repository_root);
[s1,branch] = system('git branch --show-current');
[s2,commit] = system('git rev-parse HEAD');
[s3,dirty] = system('git status --porcelain=v1');
[s4,parent] = system('git rev-parse HEAD^');
[s5,subject] = system('git log -1 --format=%s');
[s6,names] = system( ...
    'git diff --name-only 51a1da02819f3d7af5b7dcbe070d6ee669442af3..HEAD');
expected = sort({'bearing_microphysics/validation/test_stage_d_four_temperature_static_states.m'; ...
    'compute_stage_d_bearing_statistics.m';'extract_stage_d_static_state.m'; ...
    'run_stage_d_build_four_temperature_static_states.m'; ...
    'solve_stage_d_static_temperature_case.m';'update_stage_d_canonical_results.m'; ...
    'stage_d_canonical_transaction.m'; ...
    'validate_stage_d_four_temperature_results.m'; ...
    'validate_stage_d_static_temperature_case.m'});
changed = splitlines(strtrim(names));
if isscalar(changed) && strlength(changed) == 0, changed = strings(0,1); end
changed = sort(cellstr(changed));
valid = all([s1 s2 s3 s4 s5 s6] == 0) && isempty(strtrim(dirty)) && ...
    strcmp(strtrim(branch),'thermal-equivalent-damping-frequency-domain-v1') && ...
    strcmp(strtrim(parent),'51a1da02819f3d7af5b7dcbe070d6ee669442af3') && ...
    strcmp(strtrim(subject),'feat: build accepted four-temperature static states') && ...
    isequal(changed,expected);
if ~valid
    error('StageD:BaselineMismatch', ...
        'STAGE_D_BASELINE_MISMATCH: exact clean reviewed Stage D source is required.');
end
identity = struct('branch',strtrim(branch),'source_commit',strtrim(commit));
end

function pass = attempt_is_persisted(results,temperature,session_id)
field = sprintf('stage_d_T%d_attempted',temperature);
pass = isfield(results.progress,field) && results.progress.(field) && ...
    results.progress.current_attempt_temperature_C == temperature && ...
    strcmp(results.progress.current_attempt_session_id,session_id);
end

function value = previous_temperature(requested)
switch requested
    case 50, value = 20;
    case 80, value = 50;
    case 100, value = 80;
    otherwise
        error('StageD:ContinuationOrder','Stage D requested temperature is invalid.');
end
end

function payload = mapped_failure(temperature,exception)
if strcmp(exception.identifier,'StageD:TemperatureTangentStiffnessNotSymmetric')
    main_status = 'TEMPERATURE_TANGENT_STIFFNESS_NOT_SYMMETRIC';
elseif strcmp(exception.identifier,'StageD:FrozenDampingArtifactChanged')
    main_status = 'FROZEN_DAMPING_ARTIFACT_CHANGED';
else
    main_status = 'FOUR_TEMPERATURE_STATIC_STATE_FAILED';
end
payload = failure_payload(temperature,main_status,exception.identifier, ...
    exception.identifier,exception.message);
end

function payload = failure_payload(temperature,main_status,failure_gate,identifier,message)
payload = struct('temperature_C',temperature,'main_status',main_status, ...
    'failure_gate',failure_gate,'diagnostic_identifier',identifier, ...
    'diagnostic_message',message);
end
