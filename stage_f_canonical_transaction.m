function output = stage_f_canonical_transaction(action,canonical_path,results,expected_canonical_sha256)
%STAGE_F_CANONICAL_TRANSACTION Audit or atomically replace the canonical MAT.

if nargin < 3, results = struct(); end
if nargin < 4, expected_canonical_sha256 = ''; end
switch action
    case 'audit'
        output = directory_audit(canonical_path);
    case 'stage_f_independent_readonly_validation'
        output = independent_readonly_validation(canonical_path);
    case 'save'
        output = atomic_save(canonical_path,results,expected_canonical_sha256);
    otherwise
        error('StageF:CanonicalOperation','Unknown Stage F canonical transaction action.');
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

function run_independent_readonly_validation(candidate_path)
repository_root = fileparts(mfilename('fullpath'));
matlab_executable = fullfile(matlabroot,'bin','matlab');
expression = sprintf(['addpath(''%s''); ' ...
    'stage_f_canonical_transaction(''stage_f_independent_readonly_validation'',''%s'')'], ...
    quote_matlab(repository_root),quote_matlab(candidate_path));
command = sprintf('"%s" -batch "%s"',matlab_executable,expression);
[status,output] = system(command);
if status ~= 0
    error('StageF:IndependentReadonlyValidation', ...
        'STAGE_F_COMPUTATION_FAILED: independent MATLAB validation failed: %s', ...
        strtrim(output));
end
end

function value = quote_matlab(value)
value = strrep(value,'''','''''');
end

function saved = atomic_save(canonical_path,results,expected_canonical_sha256)
temporary_path = [canonical_path '.stage_f_transaction.mat'];
if ~isfile(canonical_path) || isfile(temporary_path) || ...
        ~ischar(expected_canonical_sha256) || numel(expected_canonical_sha256) ~= 64 || ...
        ~directory_audit(canonical_path).passed
    conflict('Canonical path, temporary state, expected SHA, or directory audit is invalid.');
end
[channel,file_lock] = acquire_exclusive_lock(canonical_path);
lock_cleanup = onCleanup(@() release_exclusive_lock(file_lock,channel));
try
    observed_hash = file_sha256(canonical_path);
    if ~strcmp(observed_hash,expected_canonical_sha256)
        conflict('Compare-and-swap rejected a stale canonical SHA-256.');
    end
    current = load(canonical_path);
    if ~isstruct(current) || ~isequal(fieldnames(current),{'results'})
        conflict('Canonical MAT schema is invalid.');
    end
    frozen_projection = build_stage_f_frozen_input_projection(current.results);
    validation = validate_stage_f_results( ...
        results,current.results,frozen_projection);
    if ~validation.passed
        error('StageF:ComputationFailed', ...
            'STAGE_F_COMPUTATION_FAILED: in-memory candidate failed independent validation.');
    end
    claim_temporary_path(temporary_path);
    temporary_cleanup = onCleanup(@() delete_if_present(temporary_path));
    save(temporary_path,'results');
    temporary = load(temporary_path);
    if ~isstruct(temporary) || ~isfield(temporary,'results') || ...
            ~validate_stage_f_results(temporary.results,current.results, ...
            frozen_projection).passed
        error('StageF:ComputationFailed', ...
            'STAGE_F_COMPUTATION_FAILED: transaction MAT failed independent reload.');
    end
    force_file_to_storage(temporary_path);
    run_independent_readonly_validation(temporary_path);
    if ~strcmp(file_sha256(canonical_path),expected_canonical_sha256)
        conflict('Canonical bytes changed during the locked transaction.');
    end
    atomic_replace(temporary_path,canonical_path);
    saved = temporary.results;
catch exception
    if any(strcmp(exception.identifier,{ ...
            'StageF:CanonicalResultArtifactConflict','StageF:ComputationFailed'}))
        rethrow(exception);
    end
    error('StageF:ComputationFailed', ...
        'STAGE_F_COMPUTATION_FAILED: %s',exception.message);
end
end

function validation = independent_readonly_validation(candidate_path)
artifact = load(candidate_path);
if ~isstruct(artifact) || ~isequal(fieldnames(artifact),{'results'})
    error('StageF:IndependentReadonlyValidation', ...
        'STAGE_F_COMPUTATION_FAILED: independent candidate schema is invalid.');
end
suffix = '.stage_f_transaction.mat';
if ~endsWith(candidate_path,suffix)
    error('StageF:IndependentReadonlyValidation', ...
        'STAGE_F_COMPUTATION_FAILED: candidate path is not transactional.');
end
canonical_path = candidate_path(1:end-numel(suffix));
current = load(canonical_path);
if ~isstruct(current) || ~isequal(fieldnames(current),{'results'})
    error('StageF:IndependentReadonlyValidation', ...
        'STAGE_F_COMPUTATION_FAILED: canonical baseline schema is invalid.');
end
frozen_projection = build_stage_f_frozen_input_projection(current.results);
validation = validate_stage_f_results( ...
    artifact.results,current.results,frozen_projection);
if ~validation.passed
    error('StageF:IndependentReadonlyValidation', ...
        'STAGE_F_COMPUTATION_FAILED: independent candidate validation failed: %s', ...
        strjoin(validation.diagnostics,','));
end
end

function claim_temporary_path(path)
target = java.nio.file.Paths.get(path,javaArray('java.lang.String',0));
try
    java.nio.file.Files.createFile(target,javaArray('java.nio.file.attribute.FileAttribute',0));
catch
    conflict('Transaction MAT already belongs to another process.');
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
    conflict('Canonical transaction is concurrent.');
end
if isempty(file_lock)
    channel.close();
    conflict('Canonical transaction is concurrent.');
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

function hash = file_sha256(path)
bytes = read_file_bytes(path);
if isempty(bytes) && ~isfile(path), hash = ''; return; end
digest = java.security.MessageDigest.getInstance('SHA-256');
digest.update(typecast(bytes,'int8'));
raw = typecast(digest.digest(),'uint8');
hash = lower(reshape(dec2hex(raw,2).',1,[]));
end

function bytes = read_file_bytes(path)
fid = fopen(path,'rb');
if fid < 0, bytes = uint8.empty(0,1); return; end
cleanup = onCleanup(@() fclose(fid));
bytes = fread(fid,Inf,'*uint8');
end

function delete_if_present(path)
if isfile(path), delete(path); end
end

function conflict(detail)
error('StageF:CanonicalResultArtifactConflict', ...
    'STAGE_F_COMPUTATION_FAILED: canonical transaction conflict: %s',detail);
end
