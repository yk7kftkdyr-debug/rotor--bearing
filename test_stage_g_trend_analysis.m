function test_stage_g_trend_analysis
%TEST_STAGE_G_TREND_ANALYSIS Pure Stage G extraction and robustness tests.

root = fileparts(mfilename('fullpath'));
old_path = path; cleanup = onCleanup(@() path(old_path)); addpath(root);
required = {'stage_g_trend_analysis.m','validate_stage_g_results.m'};
for index = 1:numel(required)
    assert(isfile(fullfile(root,required{index})), ...
        'StageG:MissingAPI','Missing Stage G API: %s',required{index});
end

baseline = synthetic_canonical();
trend = stage_g_trend_analysis(baseline);
assert(strcmp(trend.schema_version,'stage-g-trend-robustness-v1'));
assert(strcmp(trend.stage_g_input_hash_before,trend.stage_g_input_hash_after));
assert(trend.input_hash_match);

record = find_temperature_record(trend, ...
    'bearing.front_relative.displacement_rms_m',50,'NOMINAL');
assert(abs(record.delta_normalized-0.5) <= 1e-12);
assert(strcmp(record.trend_direction,'INCREASE'));
reference = find_temperature_record(trend, ...
    'bearing.front_relative.displacement_rms_m',20,'NOMINAL');
assert(reference.delta_normalized == 0 && strcmp(reference.trend_direction,'UNCHANGED'));

robust = find_envelope_record(trend, ...
    'bearing.front_relative.displacement_rms_m',50);
assert(strcmp(robust.classification,'TREND_ROBUST'));
assert(all(strcmp({robust.trend_direction_LOW,robust.trend_direction_NOMINAL, ...
    robust.trend_direction_HIGH},'INCREASE')));

sensitive = find_envelope_record(trend,'rotor.R10.displacement_rms_m',50);
assert(strcmp(sensitive.classification,'DAMPING_SENSITIVE_TREND'));
assert(strcmp(sensitive.trend_direction_LOW,'INCREASE'));
assert(strcmp(sensitive.trend_direction_NOMINAL,'DECREASE'));
assert(any(strcmp(trend.damping_envelope.sensitive_metrics, ...
    'rotor.R10.displacement_rms_m')));

assert(all(isfield(trend,{'temperature_normalization','damping_envelope', ...
    'metric_classification','mechanism_chain','decision','report_data', ...
    'stage_f_interpretation'})));
assert(strcmp(trend.decision.status,'THERMAL_TREND_ROBUSTNESS_ASSESSED'));
assert(trend.decision.four_temperature_complete && ...
    trend.decision.three_damping_scenarios_complete);
assert(strcmp(trend.stage_f_interpretation.conclusion, ...
    'LOCAL_HERTZ_CONTACT_CURVATURE_LINEAR_PREDICTION_DEVIATION'));
assert(~trend.stage_f_interpretation.frequency_domain_results_modified);
assert(any(strcmp(trend.metric_classification.class_C_not_studied, ...
    'dynamic_EHL_damping')));

trend.source_commit = repmat('1',1,40);
candidate = baseline; candidate.trend_robustness = trend;
validation = validate_stage_g_results(candidate,baseline);
assert(validation.passed);

tampered = candidate;
tampered.frequency_response.T50.NOMINAL.qhat_full(1) = ...
    tampered.frequency_response.T50.NOMINAL.qhat_full(1)+1;
validation = validate_stage_g_results(tampered,baseline);
assert(~validation.passed && any(strcmp(validation.diagnostics,'FROZEN_BASELINE')));

tampered = candidate;
tampered.trend_robustness.stage_g_input_hash_after = repmat('f',1,64);
validation = validate_stage_g_results(tampered,baseline);
assert(~validation.passed && any(strcmp(validation.diagnostics,'INPUT_HASH')));

not_ready = baseline; not_ready.progress.allow_stage_g = false;
assert_error(@() stage_g_trend_analysis(not_ready), ...
    'StageG:InputNotReady','STAGE_G_INPUT_NOT_READY');

test_source_scope(root);
fprintf('STAGE_G_TREND_ANALYSIS_TEST_PASSED\n');
end

function record = find_temperature_record(trend,name,temperature,scenario)
records = trend.temperature_normalization.records;
mask = strcmp({records.metric_name},name) & ...
    [records.temperature_C] == temperature & strcmp({records.scenario},scenario);
assert(nnz(mask) == 1); record = records(mask);
end

function record = find_envelope_record(trend,name,temperature)
records = trend.damping_envelope.records;
mask = strcmp({records.metric_name},name) & [records.temperature_C] == temperature;
assert(nnz(mask) == 1); record = records(mask);
end

function results = synthetic_canonical
results = struct;
results.meta = struct('stage_f_source_commit',repmat('0',1,40));
results.configuration = struct('rotational_speed_rad_s',100);
results.progress = struct('stage_e_complete',true,'stage_f_complete',true, ...
    'stage_g_complete',false,'allow_stage_g',true, ...
    'stage_f_gate_status','WORKPOINT_LINEARIZATION_LOCAL_DEVIATION_EXPLAINED');
results.decision = struct('allow_stage_g',true, ...
    'status','WORKPOINT_LINEARIZATION_LOCAL_DEVIATION_EXPLAINED');
results.damping = struct('C_formal_LOW',eye(4), ...
    'C_formal_NOMINAL',2*eye(4),'C_formal_HIGH',3*eye(4));
temperatures = [20 50 80 100]; scenarios = {'LOW','NOMINAL','HIGH'};
response_ratios = [1 1.5 2 2.5];
for temperature_index = 1:numel(temperatures)
    temperature = temperatures(temperature_index);
    tf = sprintf('T%d',temperature); ratio = temperature/20;
    thermal_ball = thermal_state(temperature,1);
    thermal_roller = thermal_state(temperature,2);
    state = struct('q_static',temperature*(1:4).', ...
        'thermal_state',struct('ball',thermal_ball,'roller',thermal_roller), ...
        'bearing_state_front',struct('Q',[1;2;0]), ...
        'bearing_state_rear',struct('Q',[2 1;3 1;0 0]), ...
        'M',eye(4),'G',diag([1 2 3 4]));
    if temperature == 20
        state.K_tangent_original = 10*eye(4);
        state.bearing_stiffness_front = struct('K_local',4*eye(5));
        state.bearing_stiffness_rear = struct('K_local',5*eye(5));
    else
        state.K_tangent = 10*ratio*eye(4);
        state.K_b_front = 4*ratio*eye(5);
        state.K_b_rear = 5*ratio*eye(5);
    end
    results.static.(tf) = state;
    for scenario_index = 1:3
        scenario = scenarios{scenario_index}; damping_scale = [1.1 1 0.9];
        base = 10*response_ratios(temperature_index)*damping_scale(scenario_index);
        response = response_record(base);
        if temperature > 20
            special = [12 8 11]*(temperature/50);
            response.metrics.rotor_metrics.R10.radial_rms_m = ...
                special(scenario_index);
        else
            response.metrics.rotor_metrics.R10.radial_rms_m = 10;
        end
        response.qhat_full = complex(base*(1:4).',scenario_index);
        response.Fhat = complex((1:4).',base);
        results.frequency_response.(tf).(scenario) = response;
        results.bearing_contact.(tf).(scenario).front = ...
            contact_record([1;2;0],scenario_index);
        results.bearing_contact.(tf).(scenario).rear = ...
            contact_record([3;4;0],scenario_index);
    end
    for bearing = {'front','rear'}
        results.nonlinearity_gate.(tf).(bearing{1}) = struct( ...
            'epsilon_NL',0.25,'H2_H1',5e-5,'H3_H1',1e-9, ...
            'potential_contact_loss_nominal',false, ...
            'active_set_changed',false);
    end
end
case_value = struct('epsilon_NL',0.25,'H2_H1',5e-5,'H3_H1',1e-9, ...
    'contact_loss_detected',false,'active_set_changed',false);
results.nonlinear_validation.representative = struct( ...
    'gate_status','WORKPOINT_LINEARIZATION_LOCAL_DEVIATION_EXPLAINED', ...
    'allow_stage_g',true,'validation',struct('passed',true), ...
    'cases',struct('case_A',case_value,'case_B',case_value));
end

function value = thermal_state(temperature,index)
value = struct('T_final',temperature+index,'eta',0.05/temperature, ...
    'film_thickness',1e-6/temperature, ...
    'working_clearance',1e-4+temperature*1e-7);
end

function response = response_record(value)
metric = response_metric(value); metric_r10 = response_metric(value);
metrics = struct('bearing_interface_metrics',struct('front',metric,'rear',metric), ...
    'rotor_metrics',struct('R2',metric,'R10',metric_r10), ...
    'casing_metrics',struct('C2',metric,'C8',metric), ...
    'orbit_metrics',struct('R2',orbit_metric(value),'R10',orbit_metric(value)), ...
    'transfer_metrics',struct('front',transfer_metric(value/10), ...
    'rear',transfer_metric(value/10)));
response = struct('metrics',metrics);
end

function value = response_metric(x)
value = struct('radial_rms_m',x,'radial_velocity_rms_m_s',2*x, ...
    'radial_acceleration_rms_m_s2',3*x);
end

function value = orbit_metric(x)
value = struct('semi_major_axis_m',2*x,'semi_minor_axis_m',x, ...
    'axis_ratio',0.5);
end

function value = transfer_metric(x)
value = struct('displacement_transmissibility',x, ...
    'velocity_transmissibility',x,'acceleration_transmissibility',x, ...
    'valid',true,'status','TRANSMISSIBILITY_VALID');
end

function value = contact_record(Q_static,scenario_index)
loaded = Q_static > 0; Qhat = scenario_index*(1:numel(Q_static)).';
roller = struct('Q_static_roller',Q_static, ...
    'loaded_mask_static_roller',loaded,'Qhat_roller',Qhat, ...
    'Qhat_magnitude_roller',abs(Qhat), ...
    'Qmax_linear_roller',Q_static+abs(Qhat), ...
    'Qmin_linear_roller',Q_static-abs(Qhat));
value = struct('roller_level',roller);
end

function test_source_scope(root)
files = {'stage_g_trend_analysis.m','validate_stage_g_results.m'};
for index = 1:numel(files)
    source = fileread(fullfile(root,files{index}));
    for forbidden = {'solve_stage_d_static_temperature_case\s*\(', ...
            'solve_stage_e_full_order_1X_case\s*\(', ...
            'run_stage_[def][^\r\n]*\(', ...
            'nonlinear_bearing_force\s*\(','newmark','fft\s*\(', ...
            'dynamic_ehl\s*\('}
        assert(isempty(regexpi(source,forbidden{1},'once')), ...
            'StageG:ForbiddenSolverCall','Forbidden Stage G source: %s',forbidden{1});
    end
end
runner = fileread(fullfile(root,'stage_g_trend_analysis.m'));
assert(contains(runner,'tryLock'), ...
    'StageG:MissingCanonicalLock','Stage G append must hold the canonical lock.');
assert(numel(regexp(runner,'file_sha256\(canonical\)','match')) >= 2, ...
    'StageG:MissingFinalCAS','Stage G append must repeat CAS immediately before replace.');
end

function assert_error(action,identifier,message_fragment)
try
    action(); error('StageG:ExpectedError','Expected error was not raised.');
catch error_value
    assert(strcmp(error_value.identifier,identifier));
    assert(contains(error_value.message,message_fragment));
end
end
