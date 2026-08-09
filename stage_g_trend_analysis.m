function output = stage_g_trend_analysis(varargin)
%STAGE_G_TREND_ANALYSIS Read-only trend analysis or reviewed canonical append.

if nargin == 0
    output = run_reviewed_canonical_append();
elseif nargin == 1 && isstruct(varargin{1})
    output = analyze_existing_results(varargin{1});
else
    error('StageG:InvalidInput','Stage G expects zero arguments or canonical results.');
end
end

function trend = analyze_existing_results(results)
require_stage_g_ready(results);
input_hash_before = frozen_input_hash(results);
[metrics,metric_count] = extract_metrics(results);
metrics = metrics(1:metric_count);
temperature_records = build_temperature_records(metrics);
envelope_records = build_envelope_records(metrics);
sensitive_metrics = sensitive_metric_names(envelope_records);
classification = classify_metrics(metrics);
mechanisms = build_mechanism_chains(metrics);
stage_f = stage_f_interpretation(results);
input_hash_after = frozen_input_hash(results);
trend = struct('schema_version','stage-g-trend-robustness-v1', ...
    'source_commit','', ...
    'stage_g_input_hash_before',input_hash_before, ...
    'stage_g_input_hash_after',input_hash_after, ...
    'input_hash_match',strcmp(input_hash_before,input_hash_after), ...
    'temperature_normalization',struct( ...
    'temperatures_C',[20 50 80 100], ...
    'scenarios',{{'LOW','NOMINAL','HIGH'}}, ...
    'definition','(Y(T,s)-Y(20,s))/max(abs(Y(20,s)),eps)', ...
    'records',temperature_records), ...
    'damping_envelope',struct('definition', ...
    'TREND_ROBUST iff LOW/NOMINAL/HIGH directions agree', ...
    'records',envelope_records,'sensitive_metrics',{sensitive_metrics}), ...
    'metric_classification',classification, ...
    'mechanism_chain',mechanisms, ...
    'stage_f_interpretation',stage_f, ...
    'report_data',build_report_data(temperature_records,envelope_records, ...
    sensitive_metrics,classification,stage_f), ...
    'decision',struct('status','THERMAL_TREND_ROBUSTNESS_ASSESSED', ...
    'four_temperature_complete',true, ...
    'three_damping_scenarios_complete',true, ...
    'mechanism_chain_complete',~isempty(mechanisms), ...
    'stage_f_limitation_complete',true, ...
    'damping_sensitive_trend_present',~isempty(sensitive_metrics), ...
    'damping_sensitive_trend_is_failure',false));
if ~trend.input_hash_match
    error('StageG:FrozenInputChanged', ...
        'STAGE_G_FROZEN_INPUT_CHANGED: Stage G input hash changed.');
end
end

function require_stage_g_ready(results)
ready = isstruct(results) && isfield(results,'progress') && ...
    isfield(results,'decision') && isfield(results,'static') && ...
    isfield(results,'frequency_response') && isfield(results,'bearing_contact') && ...
    isfield(results,'nonlinearity_gate') && ...
    all(isfield(results.progress,{'stage_e_complete','stage_f_complete', ...
    'allow_stage_g'})) && results.progress.stage_e_complete && ...
    results.progress.stage_f_complete && results.progress.allow_stage_g && ...
    isfield(results.decision,'allow_stage_g') && results.decision.allow_stage_g && ...
    isfield(results,'nonlinear_validation') && ...
    isfield(results.nonlinear_validation,'representative');
if ready
    representative = results.nonlinear_validation.representative;
    ready = all(isfield(representative,{'gate_status','allow_stage_g','validation'})) && ...
        strcmp(representative.gate_status, ...
        'WORKPOINT_LINEARIZATION_LOCAL_DEVIATION_EXPLAINED') && ...
        representative.allow_stage_g && representative.validation.passed;
end
if ~ready
    error('StageG:InputNotReady','STAGE_G_INPUT_NOT_READY');
end
for temperature = [20 50 80 100]
    tf = sprintf('T%d',temperature);
    if ~isfield(results.static,tf) || ~isfield(results.frequency_response,tf) || ...
            ~isfield(results.bearing_contact,tf) || ...
            ~isfield(results.nonlinearity_gate,tf)
        error('StageG:InputNotReady','STAGE_G_INPUT_NOT_READY');
    end
    for scenario = {'LOW','NOMINAL','HIGH'}
        name = scenario{1};
        if ~isfield(results.frequency_response.(tf),name) || ...
                ~isfield(results.bearing_contact.(tf),name)
            error('StageG:InputNotReady','STAGE_G_INPUT_NOT_READY');
        end
    end
end
end

function hash = frozen_input_hash(results)
projection = struct('M',results.static.T20.M,'G',results.static.T20.G, ...
    'C_formal_LOW',results.damping.C_formal_LOW, ...
    'C_formal_NOMINAL',results.damping.C_formal_NOMINAL, ...
    'C_formal_HIGH',results.damping.C_formal_HIGH);
for temperature = [20 50 80 100]
    tf = sprintf('T%d',temperature); state = results.static.(tf);
    projection.temperature_cases.(tf).K_t = tangent_matrix(state,temperature);
    projection.temperature_cases.(tf).q_static = state.q_static;
    for scenario = {'LOW','NOMINAL','HIGH'}
        name = scenario{1};
        projection.temperature_cases.(tf).qhat.(name) = ...
            results.frequency_response.(tf).(name).qhat_full;
    end
end
hash = serialized_sha256(projection);
end

function [metrics,count] = extract_metrics(results)
temperatures = [20 50 80 100];
empty_metric = struct('name','','unit','','class','','values',nan(4,3), ...
    'valid',false(4,3));
metrics = repmat(empty_metric,1,80); count = 0;
for bearing = {'front','rear'}
    name = bearing{1};
    [metrics,count] = add_dynamic_metric(metrics,count,results, ...
        ['bearing.' name '_relative.displacement_rms_m'],'m', ...
        @(r) r.metrics.bearing_interface_metrics.(name).radial_rms_m);
    [metrics,count] = add_dynamic_metric(metrics,count,results, ...
        ['bearing.' name '_relative.velocity_rms_m_s'],'m/s', ...
        @(r) r.metrics.bearing_interface_metrics.(name).radial_velocity_rms_m_s);
    [metrics,count] = add_dynamic_metric(metrics,count,results, ...
        ['bearing.' name '_relative.acceleration_rms_m_s2'],'m/s^2', ...
        @(r) r.metrics.bearing_interface_metrics.(name).radial_acceleration_rms_m_s2);
end
for position = {'R2','R10'}
    name = position{1};
    [metrics,count] = add_dynamic_metric(metrics,count,results, ...
        ['rotor.' name '.displacement_rms_m'],'m', ...
        @(r) r.metrics.rotor_metrics.(name).radial_rms_m);
    [metrics,count] = add_dynamic_metric(metrics,count,results, ...
        ['rotor.' name '.velocity_rms_m_s'],'m/s', ...
        @(r) r.metrics.rotor_metrics.(name).radial_velocity_rms_m_s);
    [metrics,count] = add_dynamic_metric(metrics,count,results, ...
        ['rotor.' name '.acceleration_rms_m_s2'],'m/s^2', ...
        @(r) r.metrics.rotor_metrics.(name).radial_acceleration_rms_m_s2);
    [metrics,count] = add_dynamic_metric(metrics,count,results, ...
        ['rotor.' name '.orbit_semi_major_m'],'m', ...
        @(r) r.metrics.orbit_metrics.(name).semi_major_axis_m);
    [metrics,count] = add_dynamic_metric(metrics,count,results, ...
        ['rotor.' name '.orbit_semi_minor_m'],'m', ...
        @(r) r.metrics.orbit_metrics.(name).semi_minor_axis_m);
    [metrics,count] = add_dynamic_metric(metrics,count,results, ...
        ['rotor.' name '.orbit_axis_ratio'],'1', ...
        @(r) r.metrics.orbit_metrics.(name).axis_ratio);
end
for position = {'C2','C8'}
    name = position{1};
    [metrics,count] = add_dynamic_metric(metrics,count,results, ...
        ['casing.' name '.displacement_rms_m'],'m', ...
        @(r) r.metrics.casing_metrics.(name).radial_rms_m);
    [metrics,count] = add_dynamic_metric(metrics,count,results, ...
        ['casing.' name '.velocity_rms_m_s'],'m/s', ...
        @(r) r.metrics.casing_metrics.(name).radial_velocity_rms_m_s);
    [metrics,count] = add_dynamic_metric(metrics,count,results, ...
        ['casing.' name '.acceleration_rms_m_s2'],'m/s^2', ...
        @(r) r.metrics.casing_metrics.(name).radial_acceleration_rms_m_s2);
end
for bearing = {'front','rear'}
    name = bearing{1};
    fields = {'displacement_transmissibility','velocity_transmissibility', ...
        'acceleration_transmissibility'};
    suffixes = {'displacement','velocity','acceleration'};
    for field_index = 1:3
        field = fields{field_index}; suffix = suffixes{field_index};
        [metrics,count] = add_dynamic_metric(metrics,count,results, ...
            ['transmissibility.' name '.' suffix],'1', ...
            @(r) valid_transmissibility(r.metrics.transfer_metrics.(name),field));
    end
end
for bearing = {'front','rear'}
    name = bearing{1};
    [metrics,count] = add_contact_dynamic(metrics,count,results,name, ...
        'Qhat_max_magnitude_N','N',@(x,m) max(abs(x.Qhat_roller(m))));
    [metrics,count] = add_contact_dynamic(metrics,count,results,name, ...
        'Qmax_linear_max_N','N',@(x,m) max(x.Qmax_linear_roller(m)));
    [metrics,count] = add_contact_dynamic(metrics,count,results,name, ...
        'Qmin_linear_min_N','N',@(x,m) min(x.Qmin_linear_roller(m)));
end
for bearing = {'front','rear'}
    name = bearing{1}; values = nan(4,3); valid = false(4,3);
    static_names = {'Qmax_static_N','Qmean_static_N','CV_Q_static', ...
        'loaded_count'}; units = {'N','N','1','count'};
    static_values = nan(4,4);
    for temperature_index = 1:4
        tf = sprintf('T%d',temperatures(temperature_index));
        roller = results.bearing_contact.(tf).NOMINAL.(name).roller_level;
        loaded = logical(roller.loaded_mask_static_roller(:));
        Q = roller.Q_static_roller(:); carried = Q(loaded);
        if isempty(carried)
            static_values(temperature_index,:) = [NaN NaN NaN 0];
        else
            mean_Q = mean(carried);
            static_values(temperature_index,:) = [max(carried) mean_Q ...
                std(carried,0)/max(abs(mean_Q),eps) nnz(loaded)];
        end
    end
    for field_index = 1:4
        values(:,:) = repmat(static_values(:,field_index),1,3);
        valid(:,:) = isfinite(values);
        [metrics,count] = add_metric(metrics,count, ...
            ['bearing.contact.' name '.' static_names{field_index}], ...
            units{field_index},'A_HIGH_CONFIDENCE',values,valid);
    end
end
for bearing = {'front','rear'}
    name = bearing{1};
    if strcmp(name,'front'), thermal_name = 'ball'; else, thermal_name = 'roller'; end
    thermal_fields = {'T_final','eta','film_thickness','working_clearance'};
    suffixes = {'temperature_C','viscosity_Pa_s','film_thickness_m','clearance_m'};
    units = {'degC','Pa*s','m','m'};
    for field_index = 1:4
        values = nan(4,3);
        for temperature_index = 1:4
            tf = sprintf('T%d',temperatures(temperature_index));
            scalar = results.static.(tf).thermal_state.(thermal_name).( ...
                thermal_fields{field_index});
            values(temperature_index,:) = scalar;
        end
        [metrics,count] = add_metric(metrics,count, ...
            ['thermal.' name '.' suffixes{field_index}],units{field_index}, ...
            'A_HIGH_CONFIDENCE',values,isfinite(values));
    end
end
stiffness_names = {'K_b_front_fro_norm_N_m','K_b_rear_fro_norm_N_m', ...
    'K_t_fro_norm_N_m'};
for stiffness_index = 1:3
    values = nan(4,3);
    for temperature_index = 1:4
        temperature = temperatures(temperature_index);
        state = results.static.(sprintf('T%d',temperature));
        if stiffness_index == 1
            matrix = bearing_stiffness(state,temperature,'front');
        elseif stiffness_index == 2
            matrix = bearing_stiffness(state,temperature,'rear');
        else
            matrix = tangent_matrix(state,temperature);
        end
        values(temperature_index,:) = norm(matrix,'fro');
    end
    [metrics,count] = add_metric(metrics,count, ...
        ['stiffness.' stiffness_names{stiffness_index}],'N/m', ...
        'A_HIGH_CONFIDENCE',values,isfinite(values));
end
end

function [metrics,count] = add_dynamic_metric(metrics,count,results,name,unit,extractor)
temperatures = [20 50 80 100]; scenarios = {'LOW','NOMINAL','HIGH'};
values = nan(4,3); valid = false(4,3);
for temperature_index = 1:4
    tf = sprintf('T%d',temperatures(temperature_index));
    for scenario_index = 1:3
        record = results.frequency_response.(tf).(scenarios{scenario_index});
        value = extractor(record); values(temperature_index,scenario_index) = value;
        valid(temperature_index,scenario_index) = isfinite(value);
    end
end
[metrics,count] = add_metric(metrics,count,name,unit, ...
    'B_DAMPING_ENVELOPE',values,valid);
end

function [metrics,count] = add_contact_dynamic( ...
        metrics,count,results,bearing,suffix,unit,extractor)
temperatures = [20 50 80 100]; scenarios = {'LOW','NOMINAL','HIGH'};
values = nan(4,3); valid = false(4,3);
for temperature_index = 1:4
    tf = sprintf('T%d',temperatures(temperature_index));
    for scenario_index = 1:3
        roller = results.bearing_contact.(tf).(scenarios{scenario_index}).( ...
            bearing).roller_level;
        loaded = logical(roller.loaded_mask_static_roller(:));
        if any(loaded)
            values(temperature_index,scenario_index) = extractor(roller,loaded);
            valid(temperature_index,scenario_index) = ...
                isfinite(values(temperature_index,scenario_index));
        end
    end
end
[metrics,count] = add_metric(metrics,count, ...
    ['bearing.contact.' bearing '.' suffix],unit, ...
    'B_DAMPING_ENVELOPE',values,valid);
end

function [metrics,count] = add_metric(metrics,count,name,unit,class_name,values,valid)
count = count+1; metrics(count) = struct('name',name,'unit',unit, ...
    'class',class_name,'values',values,'valid',logical(valid));
end

function records = build_temperature_records(metrics)
temperatures = [20 50 80 100]; scenarios = {'LOW','NOMINAL','HIGH'};
empty = struct('metric_name','','unit','','temperature_C',NaN, ...
    'scenario','','value',NaN,'reference_T20',NaN, ...
    'delta_normalized',NaN,'trend_direction','','valid',false);
records = repmat(empty,1,numel(metrics)*12); record_index = 0;
for metric_index = 1:numel(metrics)
    metric = metrics(metric_index);
    for temperature_index = 1:4
        for scenario_index = 1:3
            record_index = record_index+1;
            value = metric.values(temperature_index,scenario_index);
            reference = metric.values(1,scenario_index);
            valid = metric.valid(temperature_index,scenario_index) && ...
                metric.valid(1,scenario_index) && isfinite(value) && isfinite(reference);
            if valid
                delta = (value-reference)/max(abs(reference),eps);
                direction = direction_name(value-reference);
            else
                delta = NaN; direction = 'INVALID';
            end
            records(record_index) = struct('metric_name',metric.name, ...
                'unit',metric.unit,'temperature_C',temperatures(temperature_index), ...
                'scenario',scenarios{scenario_index},'value',value, ...
                'reference_T20',reference,'delta_normalized',delta, ...
                'trend_direction',direction,'valid',valid);
        end
    end
end
end

function records = build_envelope_records(metrics)
temperatures = [20 50 80 100];
empty = struct('metric_name','','unit','','temperature_C',NaN, ...
    'LOW',NaN,'NOMINAL',NaN,'HIGH',NaN,'minimum',NaN,'maximum',NaN, ...
    'trend_direction_LOW','','trend_direction_NOMINAL','', ...
    'trend_direction_HIGH','','classification','','valid',false);
records = repmat(empty,1,numel(metrics)*4); record_index = 0;
for metric_index = 1:numel(metrics)
    metric = metrics(metric_index);
    for temperature_index = 1:4
        record_index = record_index+1; values = metric.values(temperature_index,:);
        directions = cell(1,3);
        for scenario_index = 1:3
            if metric.valid(temperature_index,scenario_index) && ...
                    metric.valid(1,scenario_index)
                directions{scenario_index} = direction_name( ...
                    values(scenario_index)-metric.values(1,scenario_index));
            else
                directions{scenario_index} = 'INVALID';
            end
        end
        valid = all(metric.valid(temperature_index,:)) && all(isfinite(values));
        robust = valid && strcmp(directions{1},directions{2}) && ...
            strcmp(directions{2},directions{3});
        if robust, classification = 'TREND_ROBUST';
        else, classification = 'DAMPING_SENSITIVE_TREND'; end
        if valid, lower = min(values); upper = max(values);
        else, lower = NaN; upper = NaN; end
        records(record_index) = struct('metric_name',metric.name, ...
            'unit',metric.unit,'temperature_C',temperatures(temperature_index), ...
            'LOW',values(1),'NOMINAL',values(2),'HIGH',values(3), ...
            'minimum',lower,'maximum',upper, ...
            'trend_direction_LOW',directions{1}, ...
            'trend_direction_NOMINAL',directions{2}, ...
            'trend_direction_HIGH',directions{3}, ...
            'classification',classification,'valid',valid);
    end
end
end

function names = sensitive_metric_names(records)
mask = [records.temperature_C] > 20 & ...
    strcmp({records.classification},'DAMPING_SENSITIVE_TREND');
names = unique({records(mask).metric_name},'stable');
end

function output = classify_metrics(metrics)
classes = {metrics.class}; names = {metrics.name};
output = struct('class_A_high_confidence',{names(strcmp(classes, ...
    'A_HIGH_CONFIDENCE'))}, ...
    'class_A_additional',{{'modal_changes'}}, ...
    'class_B_damping_envelope',{names(strcmp(classes,'B_DAMPING_ENVELOPE'))}, ...
    'class_B_reporting_rule','NOMINAL with LOW/HIGH envelope', ...
    'class_C_not_studied',{{'dynamic_EHL_damping', ...
    'exact_nonlinear_periodic_response','system_2X_response','system_3X_response'}});
end

function mechanisms = build_mechanism_chains(metrics)
dynamic = metrics(strcmp({metrics.class},'B_DAMPING_ENVELOPE'));
empty = struct('metric_name','','mechanism_chain',{{}}, ...
    'evidence_metric_names',{{}},'interpretation_rule','');
mechanisms = repmat(empty,1,numel(dynamic));
steps = {'temperature','viscosity','film_thickness','clearance', ...
    'loaded_zone','contact_load','contact_stiffness','vibration_response'};
for index = 1:numel(dynamic)
    if contains(dynamic(index).name,{'rear','R10','C8'})
        bearing = 'rear'; stiffness = 'stiffness.K_b_rear_fro_norm_N_m';
    else
        bearing = 'front'; stiffness = 'stiffness.K_b_front_fro_norm_N_m';
    end
    evidence = {['thermal.' bearing '.temperature_C'], ...
        ['thermal.' bearing '.viscosity_Pa_s'], ...
        ['thermal.' bearing '.film_thickness_m'], ...
        ['thermal.' bearing '.clearance_m'], ...
        ['bearing.contact.' bearing '.loaded_count'], ...
        ['bearing.contact.' bearing '.Qmax_static_N'],stiffness,dynamic(index).name};
    mechanisms(index) = struct('metric_name',dynamic(index).name, ...
        'mechanism_chain',{steps},'evidence_metric_names',{evidence}, ...
        'interpretation_rule', ...
        'Associate observed links; do not infer a recalculated causal response.');
end
end

function output = stage_f_interpretation(results)
temperatures = [20 50 80 100]; bearings = {'front','rear'};
empty = struct('temperature_C',NaN,'bearing','','epsilon_NL',NaN, ...
    'H2_H1',NaN,'H3_H1',NaN,'contact_loss',false, ...
    'active_set_change',false);
scans = repmat(empty,1,8); scan_index = 0;
for temperature = temperatures
    tf = sprintf('T%d',temperature);
    for bearing_index = 1:2
        scan_index = scan_index+1; name = bearings{bearing_index};
        gate = results.nonlinearity_gate.(tf).(name);
        scans(scan_index) = struct('temperature_C',temperature, ...
            'bearing',name,'epsilon_NL',gate.epsilon_NL, ...
            'H2_H1',gate.H2_H1,'H3_H1',gate.H3_H1, ...
            'contact_loss',logical(gate.potential_contact_loss_nominal), ...
            'active_set_change',logical(gate.active_set_changed));
    end
end
representative = results.nonlinear_validation.representative;
output = struct('nominal_scans',scans, ...
    'representative_cases',representative.cases, ...
    'representative_gate_status',representative.gate_status, ...
    'conclusion','LOCAL_HERTZ_CONTACT_CURVATURE_LINEAR_PREDICTION_DEVIATION', ...
    'basis', ...
    'epsilon_NL is about 0.25 while H2/H1 is about 5e-5, H3/H1 is about 1e-9, with no contact loss or active-set change.', ...
    'frequency_domain_results_modified',false);
end

function output = build_report_data(temperature_records,envelope_records, ...
        sensitive_metrics,classification,stage_f)
directions = rmfield(temperature_records, ...
    {'unit','value','reference_T20','delta_normalized','valid'});
output = struct('temperature_change_table',temperature_records, ...
    'damping_envelope_table',envelope_records, ...
    'trend_direction_table',directions, ...
    'damping_sensitive_metrics',{sensitive_metrics}, ...
    'credibility_levels',classification, ...
    'stage_f_explanation',stage_f);
end

function value = valid_transmissibility(record,field)
if record.valid && isfinite(record.(field)), value = record.(field);
else, value = NaN; end
end

function matrix = tangent_matrix(state,temperature)
if temperature == 20, matrix = state.K_tangent_original;
else, matrix = state.K_tangent; end
end

function matrix = bearing_stiffness(state,temperature,bearing)
if temperature == 20
    matrix = state.(['bearing_stiffness_' bearing]).K_local;
else
    matrix = state.(['K_b_' bearing]);
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

function results = run_reviewed_canonical_append
root = fileparts(mfilename('fullpath'));
identity = require_reviewed_source(root);
canonical = fullfile(root,'results', ...
    'thermal_equivalent_damping_frequency_domain', ...
    'thermal_4T_frequency_domain_results.mat');
expected_sha256 = ...
    '3aa00bfe8fcc3a0801b824a7ed70a802335be1475aebd6591d46e75c2e58eca5';
if ~isfile(canonical) || ~strcmp(file_sha256(canonical),expected_sha256)
    error('StageG:BaselineMismatch','STAGE_G_INPUT_NOT_READY');
end
artifact = load(canonical); baseline = artifact.results;
if isfield(baseline,'trend_robustness')
    error('StageG:AlreadyComplete','THERMAL_TREND_ROBUSTNESS_ALREADY_ASSESSED');
end
trend = analyze_existing_results(baseline); trend.source_commit = identity.source_commit;
candidate = baseline; candidate.trend_robustness = trend;
validation = validate_stage_g_results(candidate,baseline);
if ~validation.passed
    error('StageG:ValidationFailed','STAGE_G_VALIDATION_FAILED: %s', ...
        strjoin(validation.diagnostics,','));
end
candidate.trend_robustness.validation = validation;
if ~validate_stage_g_results(candidate,baseline).passed
    error('StageG:ValidationFailed','STAGE_G_VALIDATION_FAILED: saved validation mismatch.');
end
atomic_append(canonical,candidate,baseline,expected_sha256);
results = candidate;
fprintf('THERMAL_TREND_ROBUSTNESS_ASSESSED\n');
end

function atomic_append(canonical,results,baseline,expected_sha256)
temporary = [canonical '.stage_g_transaction.mat'];
if isfile(temporary)
    error('StageG:TransactionConflict','STAGE_G_TRANSACTION_CONFLICT');
end
temporary_cleanup = onCleanup(@() delete_if_present(temporary));
[channel,file_lock] = acquire_lock(canonical);
lock_cleanup = onCleanup(@() release_lock(file_lock,channel));
if ~strcmp(file_sha256(canonical),expected_sha256)
    error('StageG:TransactionConflict','STAGE_G_TRANSACTION_CONFLICT');
end
current = load(canonical);
if ~isequaln(current.results,baseline)
    error('StageG:TransactionConflict','STAGE_G_TRANSACTION_CONFLICT');
end
save(temporary,'results'); reloaded = load(temporary);
if ~validate_stage_g_results(reloaded.results,baseline).passed
    error('StageG:ValidationFailed','STAGE_G_VALIDATION_FAILED: transaction reload failed.');
end
force_file(temporary);
if ~strcmp(file_sha256(canonical),expected_sha256)
    error('StageG:TransactionConflict','STAGE_G_TRANSACTION_CONFLICT');
end
current = load(canonical);
if ~isequaln(current.results,baseline)
    error('StageG:TransactionConflict','STAGE_G_TRANSACTION_CONFLICT');
end
source = java.nio.file.Paths.get(temporary,javaArray('java.lang.String',0));
target = java.nio.file.Paths.get(canonical,javaArray('java.lang.String',0));
options = javaArray('java.nio.file.CopyOption',2);
options(1) = java.nio.file.StandardCopyOption.ATOMIC_MOVE;
options(2) = java.nio.file.StandardCopyOption.REPLACE_EXISTING;
java.nio.file.Files.move(source,target,options);
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
    error('StageG:TransactionConflict','STAGE_G_TRANSACTION_CONFLICT');
end
if isempty(file_lock)
    channel.close();
    error('StageG:TransactionConflict','STAGE_G_TRANSACTION_CONFLICT');
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

function identity = require_reviewed_source(root)
old = pwd; cleanup = onCleanup(@() cd(old)); cd(root);
expected_files = {'stage_g_trend_analysis.m', ...
    'validate_stage_g_results.m','test_stage_g_trend_analysis.m'};
parent = '81cd4fef14f91b2ea7dcd72cac5e2698d1f25b45';
subject = 'feat: assess thermal trend robustness across damping envelope';
[s1,status] = system('git status --porcelain=v1');
[s2,parent_observed] = system('git log -1 --format=%P HEAD');
[s3,subject_observed] = system('git log -1 --format=%s HEAD');
[s4,changed_output] = system(['git diff --name-only ' parent '..HEAD']);
[s5,commit] = system('git rev-parse HEAD');
changed = strsplit(strtrim(changed_output),newline);
valid = all([s1 s2 s3 s4 s5] == 0) && isempty(strtrim(status)) && ...
    strcmp(strtrim(parent_observed),parent) && ...
    strcmp(strtrim(subject_observed),subject) && ...
    isequal(sort(changed),sort(expected_files));
if ~valid, error('StageG:SourceMismatch','STAGE_G_SOURCE_GATE_FAILED'); end
identity = struct('source_commit',strtrim(commit));
end

function delete_if_present(path)
if isfile(path), delete(path); end
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
bytes = fread(fid,Inf,'*uint8');
engine = java.security.MessageDigest.getInstance('SHA-256');
engine.update(typecast(bytes,'int8')); digest = typecast(engine.digest(),'uint8');
hash = lower(reshape(dec2hex(digest,2).',1,[]));
end
