function results = run_stage_f_representative_nonlinear_validation
%RUN_STAGE_F_REPRESENTATIVE_NONLINEAR_VALIDATION Two frozen bearing cases only.

root = fileparts(mfilename('fullpath'));
identity = require_reviewed_source(root);
canonical = fullfile(root,'results', ...
    'thermal_equivalent_damping_frequency_domain', ...
    'thermal_4T_frequency_domain_results.mat');
expected_sha256 = ...
    'ad9bc80fb6c1b0ae53375b0b1b73ee0dff7c6a8b387af4afa50a85f1b3658521';
if ~isfile(canonical) || ~strcmp(file_sha256(canonical),expected_sha256)
    error('StageFRepresentative:BaselineMismatch', ...
        'REPRESENTATIVE_NONLINEAR_VALIDATION_FAILED: exact Stage F canonical is required.');
end
artifact = load(canonical); baseline = artifact.results;
if isfield(baseline,'nonlinear_validation') && ...
        isfield(baseline.nonlinear_validation,'representative')
    error('StageFRepresentative:AlreadyComplete', ...
        'REPRESENTATIVE_NONLINEAR_VALIDATION_FAILED: representative result already exists.');
end

projection_before = build_stage_f_frozen_input_projection(baseline);
selection = select_case_B(baseline);
expected_case_B_label = 'T100_NOMINAL_front';
if ~strcmp(selection.label,expected_case_B_label)
    error('StageFRepresentative:CaseSelection', ...
        'REPRESENTATIVE_NONLINEAR_VALIDATION_FAILED: saved epsilon_NL selection changed.');
end
case_A = run_case(baseline,projection_before,20,'front','A');
case_B = run_case(baseline,projection_before, ...
    selection.temperature,selection.bearing,'B');
projection_after = build_stage_f_frozen_input_projection(baseline);
if ~isequaln(projection_before,projection_after)
    error('StageFRepresentative:FrozenInputChanged', ...
        'REPRESENTATIVE_NONLINEAR_VALIDATION_FAILED: frozen input changed.');
end

significant = case_A.active_set_changed || case_A.contact_loss_detected || ...
    case_B.active_set_changed || case_B.contact_loss_detected;
if significant
    gate = 'REPRESENTATIVE_NONLINEAR_EFFECT_SIGNIFICANT';
    allow_stage_g = false;
else
    gate = 'WORKPOINT_LINEARIZATION_LOCAL_DEVIATION_EXPLAINED';
    allow_stage_g = true;
end
representative = struct('schema_version', ...
    'stage-f-representative-nonlinear-validation-v1', ...
    'selection',struct('case_A','T20_NOMINAL_front', ...
    'case_B',selection.label, ...
    'selection_metric','MAXIMUM_SAVED_EPSILON_NL'), ...
    'cases',struct('case_A',case_A,'case_B',case_B), ...
    'gate_status',gate,'allow_stage_g',allow_stage_g, ...
    'source_commit',identity.source_commit);
candidate = baseline;
candidate.nonlinear_validation.representative = representative;
validation = validate_stage_f_representative_results(candidate,baseline);
if ~validation.passed
    error('StageFRepresentative:ValidationFailed', ...
        'REPRESENTATIVE_NONLINEAR_VALIDATION_FAILED: %s', ...
        strjoin(validation.diagnostics,','));
end
candidate.nonlinear_validation.representative.validation = validation;
if ~validate_stage_f_representative_results(candidate,baseline).passed
    error('StageFRepresentative:ValidationFailed', ...
        'REPRESENTATIVE_NONLINEAR_VALIDATION_FAILED: saved validation is inconsistent.');
end
atomic_append(canonical,candidate,baseline,expected_sha256);
results = candidate;
end

function value = run_case(results,projection,temperature,bearing,role)
tf = sprintf('T%d',temperature); static_state = results.static.(tf);
mapping = build_stage_f_rolling_element_linearization_mapping( ...
    results,temperature,bearing);
parameters = formal_parameters(results,static_state);
if strcmp(bearing,'front'), bearing_index = 1; else, bearing_index = 2; end
evaluator = @(q) formal_bearing_state(q,parameters,bearing_index);
qhat = results.frequency_response.(tf).NOMINAL.qhat_full;
linear_force_hat = -(mapping.K_b_local_analytic* ...
    (mapping.interface_transform*qhat));
request = struct('scenario','NOMINAL','phase_count',32, ...
    'q_static',static_state.q_static,'qhat_full',qhat, ...
    'linear_force_hat',linear_force_hat, ...
    'loaded_mask_static',mapping.loaded_mask_static_slice, ...
    'force_semantics','FORCE_ON_ROTOR_MINUS_AQ');
value = evaluate_stage_f_representative_nonlinear_case(request,evaluator);
value.temperature_case_C = temperature;
value.scenario = 'NOMINAL'; value.bearing = bearing;
value.selection_role = role;
item = projection.temperature_cases.(tf);
if strcmp(bearing,'front')
    contact_hash = item.contact_state_front_sha256;
    viscosity = static_state.thermal_state.ball.eta;
else
    contact_hash = item.contact_state_rear_sha256;
    viscosity = static_state.thermal_state.roller.eta;
end
value.frozen_input_audit = struct( ...
    'static_state_summary_sha256',item.static_state_summary_sha256, ...
    'thermal_state_sha256',item.thermal_state_sha256, ...
    'viscosity_sha256',serialized_sha256(viscosity), ...
    'clearance_sha256',item.clearance_sha256, ...
    'contact_state_sha256',contact_hash, ...
    'q_static_sha256',serialized_sha256(static_state.q_static), ...
    'passed',true);
end

function [force,loaded,Q] = formal_bearing_state(q,parameters,bearing_index)
[~,bearing_state] = nonlinear_bearing_force(q,zeros(size(q)), ...
    parameters,parameters.bearing,numel(q));
state = bearing_state.bearings(bearing_index);
force = [state.Fx;state.Fy;state.Fz;state.Mx;state.My];
Q = state.Q(:); loaded = Q > 0;
end

function parameters = formal_parameters(results,static_state)
parameters = initial_conditions();
[~,~,~,model_info] = build_rotor_case_model(parameters);
parameters.modelInfo = model_info;
parameters.num_rotor_dof = model_info.num_rotor_dof;
parameters.num_case_dof = model_info.num_case_dof;
parameters.omega = results.configuration.rotational_speed_rad_s;
parameters.microphysics = microphysics_config(struct('thermal', ...
    struct('enabled',true,'mode','frozen_external'), ...
    'thermal_state',static_state.thermal_state));
end

function selection = select_case_B(results)
best = -Inf; selection = struct('temperature',NaN,'bearing','','label','');
for temperature = [50 80 100]
    tf = sprintf('T%d',temperature);
    for bearing = {'front','rear'}
        name = bearing{1}; epsilon = results.nonlinearity_gate.(tf).(name).epsilon_NL;
        if epsilon > best
            best = epsilon; selection.temperature = temperature;
            selection.bearing = name;
        end
    end
end
selection.label = sprintf('T%d_NOMINAL_%s', ...
    selection.temperature,selection.bearing);
end

function atomic_append(canonical,results,baseline,expected_sha256)
temporary = [canonical '.representative_transaction.mat'];
if isfile(temporary)
    error('StageFRepresentative:TransactionConflict', ...
        'REPRESENTATIVE_NONLINEAR_VALIDATION_FAILED: transaction file exists.');
end
[channel,file_lock] = acquire_lock(canonical);
lock_cleanup = onCleanup(@() release_lock(file_lock,channel));
temporary_cleanup = onCleanup(@() delete_if_present(temporary));
if ~strcmp(file_sha256(canonical),expected_sha256)
    error('StageFRepresentative:TransactionConflict', ...
        'REPRESENTATIVE_NONLINEAR_VALIDATION_FAILED: canonical CAS changed.');
end
current = load(canonical);
if ~isequaln(current.results,baseline)
    error('StageFRepresentative:TransactionConflict', ...
        'REPRESENTATIVE_NONLINEAR_VALIDATION_FAILED: canonical content changed.');
end
save(temporary,'results');
reloaded = load(temporary);
if ~validate_stage_f_representative_results(reloaded.results,baseline).passed
    error('StageFRepresentative:ValidationFailed', ...
        'REPRESENTATIVE_NONLINEAR_VALIDATION_FAILED: transaction reload failed.');
end
force_file(temporary);
if ~strcmp(file_sha256(canonical),expected_sha256)
    error('StageFRepresentative:TransactionConflict', ...
        'REPRESENTATIVE_NONLINEAR_VALIDATION_FAILED: canonical CAS changed.');
end
atomic_replace(temporary,canonical);
end

function [channel,file_lock] = acquire_lock(path)
source = java.nio.file.Paths.get(path,javaArray('java.lang.String',0));
options = javaArray('java.nio.file.OpenOption',1);
options(1) = java.nio.file.StandardOpenOption.WRITE;
channel = java.nio.channels.FileChannel.open(source,options);
try
    file_lock = channel.tryLock();
catch
    channel.close();
    error('StageFRepresentative:TransactionConflict', ...
        'REPRESENTATIVE_NONLINEAR_VALIDATION_FAILED: canonical lock failed.');
end
if isempty(file_lock)
    channel.close();
    error('StageFRepresentative:TransactionConflict', ...
        'REPRESENTATIVE_NONLINEAR_VALIDATION_FAILED: canonical is locked.');
end
end

function release_lock(file_lock,channel)
if ~isempty(file_lock) && file_lock.isValid(), file_lock.release(); end
if ~isempty(channel) && channel.isOpen(), channel.close(); end
end

function force_file(path)
source = java.nio.file.Paths.get(path,javaArray('java.lang.String',0));
options = javaArray('java.nio.file.OpenOption',1);
options(1) = java.nio.file.StandardOpenOption.WRITE;
channel = java.nio.channels.FileChannel.open(source,options);
cleanup = onCleanup(@() channel.close()); channel.force(true);
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

function identity = require_reviewed_source(root)
old = pwd; cleanup = onCleanup(@() cd(old)); cd(root);
expected_files = { ...
    'bearing_microphysics/validation/test_stage_f_representative_nonlinear_validation.m', ...
    'evaluate_stage_f_representative_nonlinear_case.m', ...
    'validate_stage_f_representative_results.m', ...
    'run_stage_f_representative_nonlinear_validation.m'};
parent = 'ceff73601a56774a898c2f7bb51168587cf5bf5d';
subject = 'feat: add representative nonlinear validation';
[s1,status] = system('git status --porcelain=v1');
[s2,parent_observed] = system('git log -1 --format=%P HEAD');
[s3,subject_observed] = system('git log -1 --format=%s HEAD');
[s4,changed_output] = system( ...
    'git diff --name-only ceff73601a56774a898c2f7bb51168587cf5bf5d..HEAD');
[s5,commit] = system('git rev-parse HEAD');
changed = strsplit(strtrim(changed_output),newline);
valid = all([s1 s2 s3 s4 s5] == 0) && isempty(strtrim(status)) && ...
    strcmp(strtrim(parent_observed),parent) && ...
    strcmp(strtrim(subject_observed),subject) && ...
    isequal(sort(changed),sort(expected_files));
if ~valid
    error('StageFRepresentative:SourceMismatch', ...
        'REPRESENTATIVE_NONLINEAR_VALIDATION_FAILED: reviewed source Gate failed.');
end
identity = struct('source_commit',strtrim(commit));
end

function hash = serialized_sha256(value)
bytes = getByteStreamFromArray(value);
engine = java.security.MessageDigest.getInstance('SHA-256');
engine.update(typecast(uint8(bytes),'int8'));
digest = typecast(engine.digest(),'uint8');
hash = lower(reshape(dec2hex(digest,2).',1,[]));
end

function hash = file_sha256(path)
fid = fopen(path,'rb'); cleanup = onCleanup(@() fclose(fid));
bytes = fread(fid,Inf,'*uint8'); engine = java.security.MessageDigest.getInstance('SHA-256');
engine.update(typecast(bytes,'int8')); digest = typecast(engine.digest(),'uint8');
hash = lower(reshape(dec2hex(digest,2).',1,[]));
end
