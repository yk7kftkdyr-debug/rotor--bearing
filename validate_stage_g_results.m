function validation = validate_stage_g_results(candidate,baseline)
%VALIDATE_STAGE_G_RESULTS Validate an append-only deterministic analysis.

diagnostics = {};
if ~isstruct(candidate) || ~isstruct(baseline) || ...
        ~same_without_trend(candidate,baseline)
    diagnostics{end+1} = 'FROZEN_BASELINE';
end
try
    trend = candidate.trend_robustness;
catch
    validation = finish({'SCHEMA'}); return;
end
required = {'schema_version','source_commit','stage_g_input_hash_before', ...
    'stage_g_input_hash_after','input_hash_match','temperature_normalization', ...
    'damping_envelope','metric_classification','mechanism_chain', ...
    'stage_f_interpretation','report_data','decision'};
if ~isstruct(trend) || ~all(isfield(trend,required)) || ...
        ~strcmp(trend.schema_version,'stage-g-trend-robustness-v1') || ...
        isempty(regexp(trend.source_commit,'^[0-9a-f]{40}$','once'))
    diagnostics{end+1} = 'SCHEMA';
end
try
    expected = stage_g_trend_analysis(baseline);
    if ~strcmp(trend.stage_g_input_hash_before,expected.stage_g_input_hash_before) || ...
            ~strcmp(trend.stage_g_input_hash_after,expected.stage_g_input_hash_after) || ...
            ~trend.input_hash_match
        diagnostics{end+1} = 'INPUT_HASH';
    end
    observed_projection = result_projection(trend);
    expected_projection = result_projection(expected);
    if ~isequaln(observed_projection,expected_projection)
        diagnostics{end+1} = 'ANALYSIS_MISMATCH';
    end
catch
    diagnostics{end+1} = 'ANALYSIS_REBUILD';
end
if ~normalization_contract(trend)
    diagnostics{end+1} = 'TEMPERATURE_NORMALIZATION';
end
if ~envelope_contract(trend)
    diagnostics{end+1} = 'DAMPING_ENVELOPE';
end
if ~mechanism_contract(trend)
    diagnostics{end+1} = 'MECHANISM_CHAIN';
end
if ~stage_f_contract(trend)
    diagnostics{end+1} = 'STAGE_F_INTERPRETATION';
end
if ~decision_contract(trend)
    diagnostics{end+1} = 'DECISION';
end
validation = finish(diagnostics);
end

function pass = same_without_trend(candidate,baseline)
try
    projection = candidate;
    if isfield(projection,'trend_robustness')
        projection = rmfield(projection,'trend_robustness');
    end
    pass = isequaln(projection,baseline);
catch
    pass = false;
end
end

function projection = result_projection(value)
projection = value;
if isfield(projection,'source_commit')
    projection = rmfield(projection,'source_commit');
end
if isfield(projection,'validation')
    projection = rmfield(projection,'validation');
end
end

function pass = normalization_contract(trend)
try
    normalization = trend.temperature_normalization;
    records = normalization.records;
    pass = isequal(normalization.temperatures_C,[20 50 80 100]) && ...
        isequal(normalization.scenarios,{'LOW','NOMINAL','HIGH'}) && ...
        ~isempty(records);
    for index = 1:numel(records)
        record = records(index);
        required = {'metric_name','unit','temperature_C','scenario','value', ...
            'reference_T20','delta_normalized','trend_direction','valid'};
        pass = pass && all(isfield(record,required));
        if record.valid
            expected = (record.value-record.reference_T20)/ ...
                max(abs(record.reference_T20),eps);
            pass = pass && isfinite(record.value) && ...
                isfinite(record.reference_T20) && ...
                same_scalar(record.delta_normalized,expected) && ...
                strcmp(record.trend_direction, ...
                direction_name(record.value-record.reference_T20));
        else
            pass = pass && isnan(record.delta_normalized) && ...
                strcmp(record.trend_direction,'INVALID');
        end
    end
catch
    pass = false;
end
end

function pass = envelope_contract(trend)
try
    records = trend.damping_envelope.records;
    sensitive = cell(1,numel(records)); sensitive_count = 0;
    pass = ~isempty(records);
    for index = 1:numel(records)
        record = records(index);
        required = {'metric_name','temperature_C','LOW','NOMINAL','HIGH', ...
            'minimum','maximum','trend_direction_LOW', ...
            'trend_direction_NOMINAL','trend_direction_HIGH', ...
            'classification','valid'};
        pass = pass && all(isfield(record,required));
        directions = {record.trend_direction_LOW, ...
            record.trend_direction_NOMINAL,record.trend_direction_HIGH};
        robust = record.valid && strcmp(directions{1},directions{2}) && ...
            strcmp(directions{2},directions{3}) && ...
            ~any(strcmp(directions,'INVALID'));
        if robust
            pass = pass && strcmp(record.classification,'TREND_ROBUST');
        else
            pass = pass && strcmp(record.classification,'DAMPING_SENSITIVE_TREND');
            if record.temperature_C > 20
                sensitive_count = sensitive_count+1;
                sensitive{sensitive_count} = record.metric_name;
            end
        end
        if record.valid
            values = [record.LOW record.NOMINAL record.HIGH];
            pass = pass && same_scalar(record.minimum,min(values)) && ...
                same_scalar(record.maximum,max(values));
        end
    end
    sensitive = sensitive(1:sensitive_count);
    pass = pass && isequal(unique(sensitive,'stable'), ...
        trend.damping_envelope.sensitive_metrics);
catch
    pass = false;
end
end

function pass = mechanism_contract(trend)
try
    mechanisms = trend.mechanism_chain;
    required_steps = {'temperature','viscosity','film_thickness','clearance', ...
        'loaded_zone','contact_load','contact_stiffness','vibration_response'};
    pass = ~isempty(mechanisms);
    for index = 1:numel(mechanisms)
        value = mechanisms(index);
        pass = pass && all(isfield(value, ...
            {'metric_name','mechanism_chain','evidence_metric_names', ...
            'interpretation_rule'})) && ...
            isequal(value.mechanism_chain,required_steps) && ...
            numel(value.evidence_metric_names) == numel(required_steps);
    end
catch
    pass = false;
end
end

function pass = stage_f_contract(trend)
try
    value = trend.stage_f_interpretation;
    required = {'nominal_scans','representative_cases', ...
        'representative_gate_status','conclusion','basis', ...
        'frequency_domain_results_modified'};
    pass = all(isfield(value,required)) && numel(value.nominal_scans) == 8 && ...
        strcmp(value.representative_gate_status, ...
        'WORKPOINT_LINEARIZATION_LOCAL_DEVIATION_EXPLAINED') && ...
        strcmp(value.conclusion, ...
        'LOCAL_HERTZ_CONTACT_CURVATURE_LINEAR_PREDICTION_DEVIATION') && ...
        ~value.frequency_domain_results_modified;
    for index = 1:numel(value.nominal_scans)
        scan = value.nominal_scans(index);
        pass = pass && all(isfield(scan,{'epsilon_NL','H2_H1','H3_H1', ...
            'contact_loss','active_set_change'}));
    end
catch
    pass = false;
end
end

function pass = decision_contract(trend)
try
    value = trend.decision;
    pass = strcmp(value.status,'THERMAL_TREND_ROBUSTNESS_ASSESSED') && ...
        value.four_temperature_complete && ...
        value.three_damping_scenarios_complete && ...
        value.mechanism_chain_complete && value.stage_f_limitation_complete && ...
        ~value.damping_sensitive_trend_is_failure;
catch
    pass = false;
end
end

function name = direction_name(value)
if ~isfinite(value)
    name = 'INVALID';
elseif value > 0
    name = 'INCREASE';
elseif value < 0
    name = 'DECREASE';
else
    name = 'UNCHANGED';
end
end

function pass = same_scalar(a,b)
pass = isnumeric(a) && isscalar(a) && isfinite(a) && isfinite(b) && ...
    abs(a-b) <= 1e-12*max(abs(b),1);
end

function validation = finish(diagnostics)
diagnostics = unique(diagnostics,'stable');
validation = struct('passed',isempty(diagnostics), ...
    'diagnostics',{diagnostics}, ...
    'status','THERMAL_TREND_ROBUSTNESS_ASSESSED');
end
