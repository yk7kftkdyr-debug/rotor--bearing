function write_thermal_checkpoint_atomic(checkpoint, target_file)
%WRITE_THERMAL_CHECKPOINT_ATOMIC Save, verify, and atomically replace one checkpoint.

if nargin ~= 2 || ~isstruct(checkpoint) || ~(ischar(target_file) || (isstring(target_file) && isscalar(target_file)))
    error('Stage9A:AtomicWriteArguments', 'write_thermal_checkpoint_atomic requires a checkpoint struct and scalar target file.');
end
target_file = char(target_file);
target_directory = fileparts(target_file);
if isempty(target_directory) || ~isfolder(target_directory)
    error('Stage9A:CheckpointDirectory', 'Checkpoint target directory must already exist.');
end
temporary_file = [tempname(target_directory) '.mat'];
cleanup = onCleanup(@() delete_if_present(temporary_file));
save(temporary_file, 'checkpoint', '-v7');
loaded = load(temporary_file);
if ~isequal(fieldnames(loaded), {'checkpoint'}) || ~isequaln(loaded.checkpoint, checkpoint)
    error('Stage9A:CheckpointVerification', 'Temporary checkpoint did not round-trip with the required schema and values.');
end
[moved, message] = movefile(temporary_file, target_file, 'f');
if ~moved
    error('Stage9A:CheckpointMove', 'Atomic checkpoint replacement failed: %s', message);
end
clear cleanup;
end

function delete_if_present(file_name)
if isfile(file_name), delete(file_name); end
end
