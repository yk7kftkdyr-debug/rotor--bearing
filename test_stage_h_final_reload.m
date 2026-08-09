function test_stage_h_final_reload
%TEST_STAGE_H_FINAL_RELOAD Validate the frozen Stage G canonical in memory.

root = fileparts(mfilename('fullpath'));
old_path = path; cleanup = onCleanup(@() path(old_path)); addpath(root);
required = {'export_stage_h_summary.m','validate_stage_h_final_results.m'};
for index = 1:numel(required)
    assert(isfile(fullfile(root,required{index})), ...
        'StageH:MissingAPI','Missing Stage H API: %s',required{index});
end
canonical = fullfile(root,'results', ...
    'thermal_equivalent_damping_frequency_domain', ...
    'thermal_4T_frequency_domain_results.mat');
assert(strcmp(file_sha256(canonical), ...
    'f09c66bd83ad5d416970b70e0b5dfad09b0ccb62b7432180b0f8b40aca0e1852'));
artifact = load(canonical); baseline = artifact.results;
assert(~isfield(baseline,'final_report'));

[report,text] = export_stage_h_summary(baseline);
assert(strcmp(report.schema_version,'stage-h-final-report-v1'));
assert(strcmp(report.stage_h_input_hash_before,report.stage_h_input_hash_after));
assert(report.input_hash_match);
assert(strcmp(report.summary_text_sha256,utf8_sha256(text)));
assert(report.old_ehl_excluded && ~report.newmark_used && ...
    ~report.dynamic_ehl_used);
assert(report.stage_completion.stage_a && report.stage_completion.stage_b && ...
    report.stage_completion.stage_c && report.stage_completion.stage_d && ...
    report.stage_completion.stage_e && report.stage_completion.stage_f && ...
    report.stage_completion.stage_g);
assert(numel(report.static_results) == 8);
assert(numel(report.system_stiffness_results) == 4);
assert(isequal([report.system_stiffness_results.temperature_case_C],[20 50 80 100]));
assert(numel(report.bearing_dynamic_loads) == 24);
assert(report.damping_robustness.metric_count == 55);
assert(report.damping_robustness.robust_metric_count == 51);
assert(report.damping_robustness.sensitive_metric_count == 4);
assert(isequal(report.damping_robustness.sensitive_metrics,{ ...
    'casing.C8.displacement_rms_m', ...
    'casing.C8.velocity_rms_m_s', ...
    'casing.C8.acceleration_rms_m_s2', ...
    'bearing.contact.front.Qhat_max_magnitude_N'}));
for phrase = {'闭环热效应','准静态EHL接触','固定等效阻尼', ...
        '全阶1X频域响应','不是动态EHL阻尼模型', ...
        'old_ehl_excluded=true','WORKPOINT_LINEARIZATION_LOCAL_DEVIATION_EXPLAINED', ...
        '55项指标','51项稳健','4项阻尼敏感', ...
        'NOMINAL + LOW/HIGH范围','global_K_t_fro_norm', ...
        '振动幅值、速度、加速度、传递率、轨迹'}
    assert(contains(text,phrase{1}),'StageH:MissingReportText', ...
        'Missing report phrase: %s',phrase{1});
end

report.source_commit = repmat('1',1,40);
candidate = baseline; candidate.final_report = report;
validation = validate_stage_h_final_results(candidate,baseline,text,false);
assert(validation.passed);

tampered = candidate;
tampered.static.T20.q_static(1) = tampered.static.T20.q_static(1)+1;
validation = validate_stage_h_final_results(tampered,baseline,text,false);
assert(~validation.passed && any(strcmp(validation.diagnostics,'FROZEN_BASELINE')));

tampered = candidate;
tampered.final_report.stage_h_input_hash_after = repmat('f',1,64);
validation = validate_stage_h_final_results(tampered,baseline,text,false);
assert(~validation.passed && any(strcmp(validation.diagnostics,'INPUT_HASH')));

test_source_scope(root);
fprintf('STAGE_H_FINAL_RELOAD_TEST_PASSED\n');
end

function test_source_scope(root)
files = {'export_stage_h_summary.m','validate_stage_h_final_results.m'};
for index = 1:numel(files)
    source = fileread(fullfile(root,files{index}));
    for forbidden = {'solve_stage_[defg][^\r\n]*\(', ...
            'run_stage_[defg][^\r\n]*\(', ...
            'nonlinear_bearing_force\s*\(','newmark\s*\(', ...
            'fft\s*\(','dynamic_ehl\s*\(', ...
            '\.(csv|png|jpg|jpeg|svg|pdf|log)'''}
        assert(isempty(regexpi(source,forbidden{1},'once')), ...
            'StageH:ForbiddenOperation','Forbidden Stage H source: %s',forbidden{1});
    end
end
exporter = fileread(fullfile(root,'export_stage_h_summary.m'));
assert(contains(exporter,'tryLock'));
assert(numel(regexp(exporter,'file_sha256\(canonical\)','match')) >= 2);
assert(contains(exporter,'acquire_lock(summary_path)'));
assert(numel(regexp(exporter,'file_sha256\(summary_path\)','match')) >= 2);
end

function hash = file_sha256(path)
fid = fopen(path,'rb'); cleanup = onCleanup(@() fclose(fid));
bytes = fread(fid,Inf,'*uint8');
engine = java.security.MessageDigest.getInstance('SHA-256');
engine.update(typecast(bytes,'int8')); digest = typecast(engine.digest(),'uint8');
hash = lower(reshape(dec2hex(digest,2).',1,[]));
end

function hash = utf8_sha256(text)
bytes = unicode2native(text,'UTF-8');
engine = java.security.MessageDigest.getInstance('SHA-256');
engine.update(typecast(uint8(bytes),'int8'));
digest = typecast(engine.digest(),'uint8');
hash = lower(reshape(dec2hex(digest,2).',1,[]));
end
