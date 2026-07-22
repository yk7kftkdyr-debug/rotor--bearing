function [checkpoint, resumed] = initialize_or_resume_thermal_checkpoint(cfg, runtime_identity, allow_resume)
%INITIALIZE_OR_RESUME_THERMAL_CHECKPOINT Validate or initialize the sole checkpoint.

if nargin ~= 3 || ~(islogical(allow_resume) && isscalar(allow_resume))
    error('Stage9A:ResumeArguments', 'allow_resume must be a logical scalar.');
end
identity = validate_runtime_identity(cfg, runtime_identity);
if ~isfile(cfg.checkpoint_file)
    checkpoint = empty_thermal_checkpoint(cfg);
    checkpoint.source_branch = identity.branch;
    checkpoint.source_commit = identity.commit;
    resumed = false;
    return;
end
if ~allow_resume
    error('Stage9A:CheckpointExists', 'Checkpoint exists and allow_resume is false; it will not be overwritten.');
end
loaded = load(cfg.checkpoint_file);
if ~isequal(fieldnames(loaded), {'checkpoint'})
    error('Stage9A:CheckpointVariable', 'Checkpoint file must contain exactly one variable named checkpoint.');
end
checkpoint = loaded.checkpoint;
if controlled_legacy_migration_allowed(checkpoint, cfg, identity)
    completed_records = checkpoint.case_records;
    checkpoint.source_commit = identity.commit;
    if ~isequaln(checkpoint.case_records, completed_records)
        error('Stage9B2R:MigrationRecords', 'Controlled commit migration may not modify completed case records.');
    end
    write_thermal_checkpoint_atomic(checkpoint, cfg.checkpoint_file);
end
validate_checkpoint(checkpoint, cfg, identity);
resumed = true;
end

function identity = validate_runtime_identity(cfg, runtime_identity)
if ~isstruct(runtime_identity) || ~isfield(runtime_identity, 'branch') || ~isfield(runtime_identity, 'commit')
    error('Stage9A:RuntimeIdentity', 'runtime_identity must contain branch and commit.');
end
identity = struct('branch', char(string(runtime_identity.branch)), 'commit', char(string(runtime_identity.commit)));
if ~strcmp(identity.branch, cfg.required_branch)
    error('Stage9A:BranchMismatch', 'Runtime branch does not match cfg.required_branch.');
end
if isempty(identity.commit)
    error('Stage9A:CommitMissing', 'Runtime commit must be nonempty.');
end
end

function validate_checkpoint(checkpoint, cfg, identity)
template = empty_thermal_checkpoint(cfg);
if ~same_schema(checkpoint, template)
    error('Stage9A:CheckpointSchema', 'Checkpoint fields do not match the fixed Stage9A schema.');
end
if ~strcmp(checkpoint.version, cfg.checkpoint_version) || ~strcmp(checkpoint.source_branch, identity.branch) || ...
        ~strcmp(checkpoint.source_commit, identity.commit) || ~strcmp(checkpoint.config_signature, cfg.config_signature) || ...
        ~isequal(checkpoint.temperature_list_C, cfg.temperature_list_C)
    error('Stage9A:CheckpointIdentity', 'Checkpoint identity does not match the current runtime and configuration.');
end
case_count = numel(cfg.temperature_list_C);
if numel(checkpoint.case_records) ~= case_count
    error('Stage9A:CheckpointCases', 'Checkpoint must contain exactly %d case records.', case_count);
end
for case_index = 1:case_count
    record = checkpoint.case_records(case_index);
    if record.meta.case_index ~= case_index || record.meta.T_oil_C ~= cfg.temperature_list_C(case_index) || ...
            ~strcmp(char(record.meta.config_signature), cfg.config_signature)
        error('Stage9A:CheckpointCaseIdentity', 'Checkpoint case %d does not match the fixed temperature schema.', case_index);
    end
end
if ~isscalar(checkpoint.completed_case_count) || checkpoint.completed_case_count < 0 || ...
        checkpoint.completed_case_count > case_count || checkpoint.completed_case_count ~= floor(checkpoint.completed_case_count) || ...
        ~isscalar(checkpoint.next_case_index) || checkpoint.next_case_index ~= checkpoint.completed_case_count + 1
    error('Stage9A:CheckpointProgress', 'Checkpoint completed_case_count and next_case_index are inconsistent.');
end
status = string(checkpoint.status);
allowed_status = ["initialized" "running" "completed" "failed" "budget_exhausted"];
progress_valid = (status == "initialized" && checkpoint.completed_case_count == 0) || ...
    ((status == "running" || status == "failed" || status == "budget_exhausted") && checkpoint.completed_case_count <= case_count-1) || ...
    (status == "completed" && checkpoint.completed_case_count == case_count);
if ~isscalar(status) || ~any(status == allowed_status) || ~progress_valid
    error('Stage9A:CheckpointStatus', 'Checkpoint status is not consistent with its progress.');
end
end

function pass = same_schema(value, template)
if isstruct(template)
    pass = isstruct(value) && numel(value) == numel(template) && isequal(sort(fieldnames(value)), sort(fieldnames(template)));
    if ~pass, return; end
    names = fieldnames(template);
    for value_index = 1:numel(value)
        for name_index = 1:numel(names)
            name = names{name_index};
            if ~same_schema(value(value_index).(name), template(1).(name)), pass = false; return; end
        end
    end
elseif iscell(template)
    pass = iscell(value);
else
    pass = true;
end
end

function allowed = controlled_legacy_migration_allowed(checkpoint, cfg, identity)
legacy_commit = 'b947098687f2d98edf155d8a423e5d35161af49e';
allowed = isstruct(checkpoint) && isfield(checkpoint, 'source_commit') && strcmp(checkpoint.source_commit, legacy_commit);
if ~allowed, return; end
template = empty_thermal_checkpoint(cfg);
if ~same_schema(checkpoint, template) || ~strcmp(checkpoint.version, cfg.checkpoint_version) || ...
        ~strcmp(checkpoint.source_branch, identity.branch) || ~strcmp(checkpoint.config_signature, cfg.config_signature) || ...
        ~isequal(checkpoint.temperature_list_C, cfg.temperature_list_C) || checkpoint.status ~= "budget_exhausted" || ...
        checkpoint.completed_case_count ~= 3 || checkpoint.next_case_index ~= 4 || numel(checkpoint.case_records) ~= 4
    allowed = false; return;
end
for case_index = 1:3
    record = checkpoint.case_records(case_index);
    if record.meta.status ~= "completed" || record.meta.T_oil_C ~= cfg.temperature_list_C(case_index) || ~record_passes_all_gates(record)
        allowed = false; return;
    end
end
if checkpoint.case_records(3).meta.T_oil_C ~= 80 || checkpoint.case_records(3).resume_state.valid
    allowed = false; return;
end
entries = dir(cfg.result_root); names = string({entries.name}); names = names(names ~= "." & names ~= "..");
if numel(names) ~= 1 || names ~= "thermal_4cases_checkpoint.mat"
    allowed = false; return;
end
[ancestor_status, ~] = system(sprintf('git merge-base --is-ancestor %s %s', legacy_commit, identity.commit));
if ancestor_status ~= 0, allowed = false; return; end
[diff_status, changed_files] = system(sprintf('git diff --name-only %s..%s', legacy_commit, identity.commit));
changed_files = string(splitlines(strtrim(changed_files))); changed_files = changed_files(changed_files ~= "");
allowed_files = ["run_thermal_outer_loop_4cases.m"; "initialize_or_resume_thermal_checkpoint.m"];
if diff_status ~= 0 || isempty(changed_files) || ~all(ismember(changed_files, allowed_files))
    allowed = false;
end
end

function pass = record_passes_all_gates(record)
pass = record.convergence.pass && record.bearing.ball.stiffness_pass && record.bearing.roller.stiffness_pass && ...
    record.bearing.ball.damping_pass && record.bearing.roller.damping_pass && record.linearization.pass && record.modal.pass && record.dynamics.pass;
end
