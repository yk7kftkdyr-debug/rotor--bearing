function validation = validate_stage_h_final_results( ...
        candidate,baseline,summary_text,enforce_result_directory)
%VALIDATE_STAGE_H_FINAL_RESULTS Independently validate the final append.

if nargin < 4, enforce_result_directory = false; end
diagnostics = {};
if ~isstruct(candidate) || ~isstruct(baseline) || ...
        ~same_without_final_report(candidate,baseline)
    diagnostics{end+1} = 'FROZEN_BASELINE';
end
try
    report = candidate.final_report;
catch
    validation = finish({'SCHEMA'}); return;
end
required = {'schema_version','source_commit','stage_h_input_hash_before', ...
    'stage_h_input_hash_after','input_hash_match','research_model_boundary', ...
    'old_ehl_excluded','old_ehl_used_in_formal_response','newmark_used', ...
    'dynamic_ehl_used','stage_completion','static_results', ...
    'system_stiffness_results','dynamic_results','bearing_dynamic_loads','nonlinear_gate', ...
    'damping_robustness','credibility','decision','summary_file', ...
    'summary_text_sha256'};
if ~isstruct(report) || ~all(isfield(report,required)) || ...
        ~strcmp(report.schema_version,'stage-h-final-report-v1') || ...
        isempty(regexp(report.source_commit,'^[0-9a-f]{40}$','once'))
    diagnostics{end+1} = 'SCHEMA';
end
try
    [expected,expected_text] = export_stage_h_summary(baseline);
    if ~isequaln(report_projection(report),report_projection(expected))
        diagnostics{end+1} = 'REPORT_MISMATCH';
    end
    if ~strcmp(summary_text,expected_text) || ...
            ~strcmp(report.summary_text_sha256,utf8_sha256(summary_text))
        diagnostics{end+1} = 'SUMMARY_TEXT';
    end
catch
    diagnostics{end+1} = 'REPORT_REBUILD';
end
if ~strcmp(report.stage_h_input_hash_before,report.stage_h_input_hash_after) || ...
        ~report.input_hash_match
    diagnostics{end+1} = 'INPUT_HASH';
end
if ~flags_contract(candidate,report)
    diagnostics{end+1} = 'FORBIDDEN_MODEL';
end
if ~completion_contract(report)
    diagnostics{end+1} = 'STAGE_COMPLETION';
end
if ~content_contract(report,summary_text)
    diagnostics{end+1} = 'CONTENT';
end
if enforce_result_directory && ~result_directory_contract(report,summary_text)
    diagnostics{end+1} = 'RESULT_DIRECTORY';
end
validation = finish(diagnostics);
end

function pass = same_without_final_report(candidate,baseline)
try
    projection = candidate;
    if isfield(projection,'final_report')
        projection = rmfield(projection,'final_report');
    end
    pass = isequaln(projection,baseline);
catch
    pass = false;
end
end

function projection = report_projection(report)
projection = report;
for field = {'source_commit','validation'}
    if isfield(projection,field{1}), projection = rmfield(projection,field{1}); end
end
end

function pass = flags_contract(candidate,report)
try
    pass = candidate.damping.old_ehl_excluded && ...
        ~candidate.meta.old_ehl_used_in_formal_response && ...
        ~candidate.meta.newmark_used && ~candidate.meta.dynamic_ehl_used && ...
        report.old_ehl_excluded && ~report.old_ehl_used_in_formal_response && ...
        ~report.newmark_used && ~report.dynamic_ehl_used;
    for temperature = [20 50 80 100]
        tf = sprintf('T%d',temperature);
        for scenario = {'LOW','NOMINAL','HIGH'}
            response = candidate.frequency_response.(tf).(scenario{1});
            pass = pass && isfield(response,'newmark_used') && ...
                ~response.newmark_used && isfield(response,'dynamic_contact_used') && ...
                ~response.dynamic_contact_used && ...
                isfield(response,'nonlinear_bearing_used') && ...
                ~response.nonlinear_bearing_used;
        end
    end
catch
    pass = false;
end
end

function pass = completion_contract(report)
try
    pass = all(struct2array(report.stage_completion)) && ...
        strcmp(report.decision.status, ...
        'THERMAL_FREQUENCY_DOMAIN_MAINLINE_COMPLETED') && ...
        ~report.decision.physical_results_modified && ...
        ~report.decision.new_computation_performed;
catch
    pass = false;
end
end

function pass = content_contract(report,text)
try
    required_text = {'闭环热效应','准静态EHL接触','固定等效阻尼', ...
        '全阶1X频域响应','不是动态EHL阻尼模型', ...
        'old_ehl_excluded=true', ...
        'WORKPOINT_LINEARIZATION_LOCAL_DEVIATION_EXPLAINED', ...
        '55项指标','51项稳健','4项阻尼敏感', ...
        'NOMINAL + LOW/HIGH范围','global_K_t_fro_norm', ...
        '振动幅值、速度、加速度、传递率、轨迹'};
    pass = numel(report.static_results) == 8 && ...
        numel(report.system_stiffness_results) == 4 && ...
        numel(report.dynamic_results) == 120 && ...
        numel(report.bearing_dynamic_loads) == 24 && ...
        report.damping_robustness.metric_count == 55 && ...
        report.damping_robustness.robust_metric_count == 51 && ...
        report.damping_robustness.sensitive_metric_count == 4 && ...
        strcmp(report.nonlinear_gate.representative_gate_status, ...
        'WORKPOINT_LINEARIZATION_LOCAL_DEVIATION_EXPLAINED');
    for index = 1:numel(required_text)
        pass = pass && contains(text,required_text{index});
    end
catch
    pass = false;
end
end

function pass = result_directory_contract(report,summary_text)
try
    root = fileparts(mfilename('fullpath'));
    directory = fullfile(root,'results', ...
        'thermal_equivalent_damping_frequency_domain');
    entries = dir(directory); entries = entries(~[entries.isdir]);
    names = sort({entries.name});
    expected = sort({'thermal_4T_frequency_domain_results.mat', ...
        'thermal_4T_frequency_domain_summary.txt'});
    summary_path = fullfile(directory,report.summary_file);
    pass = isequal(names,expected) && isfile(summary_path) && ...
        strcmp(fileread(summary_path),summary_text);
catch
    pass = false;
end
end

function hash = utf8_sha256(text)
bytes = unicode2native(text,'UTF-8');
engine = java.security.MessageDigest.getInstance('SHA-256');
engine.update(typecast(uint8(bytes),'int8'));
digest = typecast(engine.digest(),'uint8');
hash = lower(reshape(dec2hex(digest,2).',1,[]));
end

function validation = finish(diagnostics)
diagnostics = unique(diagnostics,'stable');
validation = struct('passed',isempty(diagnostics), ...
    'diagnostics',{diagnostics}, ...
    'status','THERMAL_FREQUENCY_DOMAIN_MAINLINE_COMPLETED');
end
