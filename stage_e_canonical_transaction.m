function output = stage_e_canonical_transaction(action,canonical_path,results)
%STAGE_E_CANONICAL_TRANSACTION Audit or atomically replace the canonical MAT.

if nargin < 3, results = struct(); end
switch action
    case 'audit'
        output = directory_audit(canonical_path);
    case 'save'
        output = atomic_save(canonical_path,results);
    otherwise
        error('StageE:CanonicalOperation','Unknown Stage E canonical transaction action.');
end
end

function audit = directory_audit(canonical_path)
directory = fileparts(canonical_path);
[~,name,extension] = fileparts(canonical_path);
expected = {[name extension]};
if ~isfolder(directory)
    observed = {};
else
    entries = dir(directory); entries = entries(~[entries.isdir]);
    observed = sort({entries.name});
end
audit = struct('passed',isequal(observed,expected), ...
    'allowed_files',{expected},'observed_files',{observed}, ...
    'checked_at',datetime('now'),'source','CANONICAL_RESULT_DIRECTORY_SCAN');
end

function artifact = atomic_save(canonical_path,results)
temporary_path = [canonical_path '.stage_e_transaction.mat'];
temporary_cleanup = onCleanup(@() delete_if_present(temporary_path));
if ~isfile(canonical_path) || isfile(temporary_path)
    error('StageE:CanonicalResultArtifactConflict', ...
        'CANONICAL_RESULT_ARTIFACT_CONFLICT: canonical or transaction state is invalid.');
end
[channel,file_lock] = acquire_exclusive_lock(canonical_path);
lock_cleanup = onCleanup(@() release_exclusive_lock(file_lock,channel));
try
    current = load(canonical_path);
    baseline = validate_stage_d_four_temperature_results(current);
    candidate = validate_stage_e_four_temperature_1X_results(struct('results',results));
    projected_stage_d = stage_d_projection_for_cas(results);
    if ~baseline.passed || ~baseline.stage_d_complete || ...
            isfield(current.results,'frequency_response') || ~candidate.passed || ...
            ~isequaln(current.results,projected_stage_d) || ...
            ~isequaln(stage_e_frozen_input_snapshot(current.results), ...
            stage_e_frozen_input_snapshot(results))
        error('StageE:CanonicalResultArtifactConflict', ...
            'CANONICAL_RESULT_ARTIFACT_CONFLICT: transaction compare-and-swap failed.');
    end
    save(temporary_path,'results');
    temporary = load(temporary_path);
    if ~validate_stage_e_four_temperature_1X_results(temporary).passed
        error('StageE:CanonicalResultArtifactConflict', ...
            'CANONICAL_RESULT_ARTIFACT_CONFLICT: transaction MAT failed reload.');
    end
    force_file_to_storage(temporary_path);
    atomic_replace(temporary_path,canonical_path);
    artifact = temporary;
catch exception
    if strcmp(exception.identifier,'StageE:CanonicalResultArtifactConflict')
        rethrow(exception);
    end
    error('StageE:CanonicalResultArtifactConflict', ...
        'CANONICAL_RESULT_ARTIFACT_CONFLICT: %s',exception.message);
end

function projection = stage_d_projection_for_cas(results)
projection = results;
for name = {'frequency_response','complex_force','force_definition', ...
        'mapping_snapshot','mapping_audit','force_audit','modal_audits'}
    if isfield(projection,name{1}), projection = rmfield(projection,name{1}); end
end
for name = {'T50','T80','T100'}
    if isfield(projection.modal,name{1}), projection.modal = rmfield(projection.modal,name{1}); end
end
if isfield(projection.damping,'actual_zeta_4T')
    projection.damping = rmfield(projection.damping,'actual_zeta_4T');
end
for name = {'stage_e_source_commit','dynamic_contact_used','nonlinear_bearing_used'}
    if isfield(projection.meta,name{1}), projection.meta = rmfield(projection.meta,name{1}); end
end
if isfield(projection.validation,'stage_e')
    projection.validation = rmfield(projection.validation,'stage_e');
end
projection.progress.stage_e_complete = false;
projection.progress.last_completed_gate = 'STAGE_D_INDEPENDENT_RELOAD_ACCEPTED';
for name = {'stage_e_complex_force_complete','stage_e_modal_T20_complete', ...
        'stage_e_modal_T50_complete','stage_e_modal_T80_complete', ...
        'stage_e_modal_T100_complete','stage_e_T20_complete','stage_e_T50_complete', ...
        'stage_e_T80_complete','stage_e_T100_complete','stage_e_modal_audit_complete', ...
        'first_failed_case'}
    if isfield(projection.progress,name{1})
        projection.progress = rmfield(projection.progress,name{1});
    end
end
projection.decision.status = 'FOUR_TEMPERATURE_STATIC_STATES_ACCEPTED';
projection.decision.stage_d_status = 'FOUR_TEMPERATURE_STATIC_STATES_ACCEPTED';
projection.decision.allow_stage_e = true;
for name = {'stage_e_status','allow_stage_f'}
    if isfield(projection.decision,name{1}), projection.decision = rmfield(projection.decision,name{1}); end
end
end
end

function [channel,file_lock] = acquire_exclusive_lock(path)
source = java.nio.file.Paths.get(path,javaArray('java.lang.String',0));
options = javaArray('java.nio.file.OpenOption',1);
options(1) = java.nio.file.StandardOpenOption.WRITE;
channel = java.nio.channels.FileChannel.open(source,options);
try
    file_lock = channel.tryLock();
catch
    channel.close();
    error('StageE:CanonicalResultArtifactConflict', ...
        'CANONICAL_RESULT_ARTIFACT_CONFLICT: canonical transaction is concurrent.');
end
if isempty(file_lock)
    channel.close();
    error('StageE:CanonicalResultArtifactConflict', ...
        'CANONICAL_RESULT_ARTIFACT_CONFLICT: canonical transaction is concurrent.');
end
end

function release_exclusive_lock(file_lock,channel)
if ~isempty(file_lock) && file_lock.isValid(), file_lock.release(); end
if ~isempty(channel) && channel.isOpen(), channel.close(); end
end

function force_file_to_storage(path)
source = java.nio.file.Paths.get(path,javaArray('java.lang.String',0));
options = javaArray('java.nio.file.OpenOption',1);
options(1) = java.nio.file.StandardOpenOption.WRITE;
channel = java.nio.channels.FileChannel.open(source,options);
cleanup = onCleanup(@() channel.close());
channel.force(true);
end

function atomic_replace(source_path,target_path)
source = java.nio.file.Paths.get(source_path,javaArray('java.lang.String',0));
target = java.nio.file.Paths.get(target_path,javaArray('java.lang.String',0));
options = javaArray('java.nio.file.CopyOption',2);
options(1) = java.nio.file.StandardCopyOption.ATOMIC_MOVE;
options(2) = java.nio.file.StandardCopyOption.REPLACE_EXISTING;
java.nio.file.Files.move(source,target,options);
end

function delete_if_present(path)
if isfile(path), delete(path); end
end
