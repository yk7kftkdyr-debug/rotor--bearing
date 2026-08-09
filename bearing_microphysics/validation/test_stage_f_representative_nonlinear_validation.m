function test_stage_f_representative_nonlinear_validation
%TEST_STAGE_F_REPRESENTATIVE_NONLINEAR_VALIDATION Two-case supplemental Gate.

root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
old_path = path; cleanup = onCleanup(@() path(old_path)); addpath(root);
required = {'evaluate_stage_f_representative_nonlinear_case.m', ...
    'validate_stage_f_representative_results.m', ...
    'run_stage_f_representative_nonlinear_validation.m'};
for k = 1:numel(required)
    assert(isfile(fullfile(root,required{k})), ...
        'StageFRepresentative:MissingAPI', ...
        'Missing representative validation API: %s',required{k});
end

theta = 2*pi*(0:31)/32;
request = struct('scenario','NOMINAL','phase_count',32, ...
    'q_static',[0;0],'qhat_full',[0.2;0.1i], ...
    'linear_force_hat',[-0.2;-0.2i], ...
    'loaded_mask_static',[true;true], ...
    'force_semantics','FORCE_ON_ROTOR_MINUS_AQ');
result = evaluate_stage_f_representative_nonlinear_case( ...
    request,@stable_nonlinear_evaluator);
assert(isequal(result.phase_values_rad,theta));
assert(isequal(size(result.F_exact),[2 32]) && ...
    isequal(size(result.F_linear),[2 32]));
assert(isequal(size(result.loaded_mask_history),[2 32]));
assert(isequal(size(result.Q_min),[2 1]) && all(result.Q_min > 0));
assert(~result.active_set_changed && ~result.contact_loss_detected);
assert(result.epsilon_NL > 0 && isfinite(result.epsilon_NL));
assert(all(isfinite(result.F1_complex)) && ...
    all(isfinite(result.F2_complex)) && all(isfinite(result.F3_complex)));
assert(isfinite(result.H2_H1) && isfinite(result.H3_H1));
assert(max(abs(result.F_linear-result.static_force_reference- ...
    real(request.linear_force_hat*exp(1i*theta))),[],'all') <= 100*eps);

changed = request;
changed.q_static = [0;0]; changed.qhat_full = [0.2;0];
changed.linear_force_hat = [-0.2;0];
changed.loaded_mask_static = [true;true];
significant = evaluate_stage_f_representative_nonlinear_case( ...
    changed,@contact_change_evaluator);
assert(significant.active_set_changed && significant.contact_loss_detected);
assert(any(significant.Q_min <= 0));

test_validator_contract(result);
test_runner_scope(root);
fprintf('STAGE_F_REPRESENTATIVE_NONLINEAR_VALIDATION_TEST_PASSED\n');
end

function test_validator_contract(case_result)
baseline = synthetic_baseline();
projection = build_stage_f_frozen_input_projection(baseline);
case_A = enrich_case(case_result,baseline,projection,'A',20);
case_B = enrich_case(case_result,baseline,projection,'B',100);
representative = struct('schema_version', ...
    'stage-f-representative-nonlinear-validation-v1', ...
    'selection',struct('case_A','T20_NOMINAL_front', ...
    'case_B','T100_NOMINAL_front','selection_metric','MAXIMUM_SAVED_EPSILON_NL'), ...
    'cases',struct('case_A',case_A,'case_B',case_B), ...
    'gate_status','WORKPOINT_LINEARIZATION_LOCAL_DEVIATION_EXPLAINED', ...
    'allow_stage_g',true,'source_commit',repmat('1',1,40));
candidate = baseline;
candidate.nonlinear_validation.representative = representative;
validation = validate_stage_f_representative_results(candidate,baseline);
assert(validation.passed && validation.allow_stage_g);

tampered = candidate;
tampered.nonlinear_validation.representative.cases.case_A.F_exact(1,1) = ...
    tampered.nonlinear_validation.representative.cases.case_A.F_exact(1,1)+1;
validation = validate_stage_f_representative_results(tampered,baseline);
assert(~validation.passed && any(strcmp(validation.diagnostics,'FORCE_HISTORY')));

tampered = candidate;
tampered.static.T20.q_static(1) = 1;
validation = validate_stage_f_representative_results(tampered,baseline);
assert(~validation.passed && any(strcmp(validation.diagnostics,'FROZEN_BASELINE')));

baseline.nonlinear_validation.existing_stage_f_audit = struct('passed',true);
candidate = baseline;
candidate.nonlinear_validation.representative = representative;
tampered = candidate;
tampered.nonlinear_validation.existing_stage_f_audit.passed = false;
validation = validate_stage_f_representative_results(tampered,baseline);
assert(~validation.passed && any(strcmp(validation.diagnostics,'FROZEN_BASELINE')));

tampered = candidate;
tampered.nonlinear_validation.unexpected_sibling = 1;
validation = validate_stage_f_representative_results(tampered,baseline);
assert(~validation.passed && any(strcmp(validation.diagnostics,'FROZEN_BASELINE')));
end

function baseline = synthetic_baseline
baseline = struct('meta',struct('stage_f_source_commit',repmat('0',1,40)), ...
    'progress',struct('stage_f_complete',true), ...
    'decision',struct('allow_stage_g',false), ...
    'validation',struct('stage_f',struct('passed',true)), ...
    'damping',struct('C_formal_LOW',eye(2),'C_formal_NOMINAL',2*eye(2), ...
    'C_formal_HIGH',3*eye(2)));
for temperature = [20 50 80 100]
    tf = sprintf('T%d',temperature);
    baseline.static.(tf) = struct('q_static',[0;0], ...
        'thermal_state',struct('temperature_C',temperature));
    for scenario = {'LOW','NOMINAL','HIGH'}
        baseline.frequency_response.(tf).(scenario{1}) = ...
            struct('qhat_full',[0.2;0.1i],'Fhat',[-0.2;-0.2i]);
    end
    baseline.nonlinearity_gate.(tf).front.epsilon_NL = temperature;
    baseline.nonlinearity_gate.(tf).rear.epsilon_NL = 0;
end
end

function value = enrich_case(value,baseline,projection,role,temperature)
tf = sprintf('T%d',temperature);
value.temperature_case_C = temperature;
value.scenario = 'NOMINAL'; value.bearing = 'front';
value.selection_role = role;
value.frozen_input_audit = struct( ...
    'static_state_summary_sha256', ...
    projection.temperature_cases.(tf).static_state_summary_sha256, ...
    'thermal_state_sha256',projection.temperature_cases.(tf).thermal_state_sha256, ...
    'viscosity_sha256',serialized_sha256([]), ...
    'clearance_sha256',projection.temperature_cases.(tf).clearance_sha256, ...
    'contact_state_sha256', ...
    projection.temperature_cases.(tf).contact_state_front_sha256, ...
    'q_static_sha256',serialized_sha256(baseline.static.(tf).q_static), ...
    'passed',true);
end

function [force,loaded,Q] = stable_nonlinear_evaluator(q)
q = q(:); Q = [1+0.1*q(1);2+0.1*q(2)]; loaded = Q > 0;
force = [-(q(1)+0.5*q(1)^2);-(2*q(2)+0.25*q(2)^2)];
end

function [force,loaded,Q] = contact_change_evaluator(q)
q = q(:); Q = [0.05+q(1);1]; loaded = Q > 0;
force = [-max(Q(1),0);-q(2)];
end

function test_runner_scope(root)
runner = fileread(fullfile(root, ...
    'run_stage_f_representative_nonlinear_validation.m'));
validator = fileread(fullfile(root, ...
    'validate_stage_f_representative_results.m'));
for required = {'T20','T100','NOMINAL','front', ...
        'evaluate_stage_f_representative_nonlinear_case', ...
        'validate_stage_f_representative_results', ...
        'nonlinear_validation','representative','ATOMIC_MOVE'}
    assert(contains(runner,required{1}));
end
for forbidden = {'newmark','fft\s*\(','dynamic[^\r\n]*ehl', ...
        'run_stage_e','run_stage_d','T50.*T80.*T100.*for'}
    assert(isempty(regexpi(runner,forbidden{1},'once')));
end
for required = {'build_stage_f_rolling_element_linearization_mapping', ...
        'baseline.frequency_response','initial_conditions', ...
        'microphysics_config','nonlinear_bearing_force'}
    assert(contains(validator,required{1}), ...
        'StageFRepresentative:ValidatorNotIndependent', ...
        'Formal validator must independently rebuild from baseline using %s.', ...
        required{1});
end
end

function hash = serialized_sha256(value)
bytes = getByteStreamFromArray(value);
engine = java.security.MessageDigest.getInstance('SHA-256');
engine.update(typecast(uint8(bytes),'int8'));
digest = typecast(engine.digest(),'uint8');
hash = lower(reshape(dec2hex(digest,2).',1,[]));
end
