function output = stage_d_canonical_transaction(action,canonical_path,results)
%STAGE_D_CANONICAL_TRANSACTION Audit, atomically save, or recover canonical MAT.

if nargin < 3, results = struct(); end
switch action
    case 'audit'
        output = directory_audit(canonical_path);
    case 'save'
        output = atomic_save(canonical_path,results);
    case 'recover'
        recover_transaction(canonical_path);
        output = directory_audit(canonical_path);
    otherwise
        error('StageD:CanonicalOperation','Unknown canonical transaction action.');
end
end

function audit = directory_audit(canonical_path)
directory = fileparts(canonical_path);
if ~isfolder(directory)
    audit = struct('passed',false,'allowed_files',{{}},'observed_files',{{}}, ...
        'checked_at',datetime('now'),'TXT_count',0,'MAT_count',0, ...
        'source','CANONICAL_RESULT_DIRECTORY_SCAN');
    return;
end
entries = dir(directory); entries = entries(~[entries.isdir]);
[~,canonical_name,canonical_extension] = fileparts(canonical_path);
names = sort({entries.name}); expected = {[canonical_name canonical_extension]};
audit = struct('passed',isequal(names,expected), ...
    'allowed_files',{expected},'observed_files',{names}, ...
    'checked_at',datetime('now'),'TXT_count',nnz(endsWith(names,'.txt','IgnoreCase',true)), ...
    'MAT_count',nnz(endsWith(names,'.mat','IgnoreCase',true)), ...
    'source','CANONICAL_RESULT_DIRECTORY_SCAN');
end

function artifact = atomic_save(canonical_path,results)
temporary_path = transaction_path(canonical_path);
if ~isfile(canonical_path)
    error('StageD:CanonicalResultArtifactConflict', ...
        'CANONICAL_RESULT_ARTIFACT_CONFLICT: canonical target is missing.');
end
[channel,file_lock] = acquire_exclusive_lock(canonical_path);
cleanup = onCleanup(@() release_exclusive_lock(file_lock,channel));
current = load(canonical_path);
if ~validate_stage_d_four_temperature_results(current).passed || ...
        transaction_sequence(results) ~= transaction_sequence(current.results)+1
    error('StageD:CanonicalResultArtifactConflict', ...
        'CANONICAL_RESULT_ARTIFACT_CONFLICT: transaction compare-and-swap failed.');
end
if isfile(temporary_path)
    error('StageD:CanonicalResultArtifactConflict', ...
        'CANONICAL_RESULT_ARTIFACT_CONFLICT: unresolved transaction MAT exists.');
end
save(temporary_path,'results');
temporary = load(temporary_path);
if ~validate_stage_d_four_temperature_results(temporary).passed
    error('StageD:CanonicalResultArtifactConflict', ...
        'CANONICAL_RESULT_ARTIFACT_CONFLICT: transaction MAT failed reload.');
end
force_file_to_storage(temporary_path);
atomic_replace(temporary_path,canonical_path);
artifact = load(canonical_path);
if ~validate_stage_d_four_temperature_results(artifact).passed
    error('StageD:ResultReloadFailed', ...
        'STAGE_D_RESULT_RELOAD_FAILED: canonical MAT failed immediate reload.');
end
end

function recover_transaction(canonical_path)
temporary_path = transaction_path(canonical_path);
if ~isfile(temporary_path), return; end
lock_path = canonical_path;
if ~isfile(lock_path), lock_path = temporary_path; end
[channel,file_lock] = acquire_exclusive_lock(lock_path);
cleanup = onCleanup(@() release_exclusive_lock(file_lock,channel));
if ~isfile(temporary_path), return; end
[temporary_valid,temporary] = load_if_valid(temporary_path);
[canonical_valid,canonical] = load_if_valid(canonical_path);
if temporary_valid && (~canonical_valid || ...
        transaction_sequence(temporary.results) > transaction_sequence(canonical.results))
    if canonical_valid && transaction_sequence(temporary.results) ~= ...
            transaction_sequence(canonical.results)+1
        error('StageD:CanonicalResultArtifactConflict', ...
            'CANONICAL_RESULT_ARTIFACT_CONFLICT: transaction sequence is not contiguous.');
    end
    force_file_to_storage(temporary_path);
    atomic_replace(temporary_path,canonical_path);
elseif canonical_valid
    delete(temporary_path);
else
    error('StageD:CanonicalResultArtifactConflict', ...
        'CANONICAL_RESULT_ARTIFACT_CONFLICT: neither canonical nor transaction MAT is valid.');
end
end

function [valid,artifact] = load_if_valid(path)
valid = false; artifact = struct();
if ~isfile(path), return; end
try
    artifact = load(path);
    valid = validate_stage_d_four_temperature_results(artifact).passed;
catch
    valid = false;
end
end

function sequence = transaction_sequence(results)
sequence = 0;
if isfield(results,'progress') && ...
        isfield(results.progress,'stage_d_transaction_sequence')
    sequence = results.progress.stage_d_transaction_sequence;
end
end

function path = transaction_path(canonical_path)
path = [canonical_path '.stage_d_transaction.mat'];
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

function [channel,file_lock] = acquire_exclusive_lock(path)
source = java.nio.file.Paths.get(path,javaArray('java.lang.String',0));
options = javaArray('java.nio.file.OpenOption',1);
options(1) = java.nio.file.StandardOpenOption.WRITE;
channel = java.nio.channels.FileChannel.open(source,options);
try
    file_lock = channel.tryLock();
catch exception
    channel.close();
    error('StageD:CanonicalResultArtifactConflict', ...
        'CANONICAL_RESULT_ARTIFACT_CONFLICT: canonical transaction is concurrent (%s).', ...
        exception.identifier);
end
if isempty(file_lock)
    channel.close();
    error('StageD:CanonicalResultArtifactConflict', ...
        'CANONICAL_RESULT_ARTIFACT_CONFLICT: canonical transaction is concurrent.');
end
end

function release_exclusive_lock(file_lock,channel)
if file_lock.isValid(), file_lock.release(); end
if channel.isOpen(), channel.close(); end
end
