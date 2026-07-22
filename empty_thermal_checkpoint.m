function checkpoint = empty_thermal_checkpoint(cfg)
%EMPTY_THERMAL_CHECKPOINT Fixed four-case Stage9 checkpoint schema.

if nargin ~= 1 || ~isstruct(cfg), error('Stage9A:CheckpointConfig', 'empty_thermal_checkpoint requires the Stage9A configuration.'); end
case_records = repmat(empty_thermal_case_record(cfg), 1, numel(cfg.temperature_list_C));
for case_index = 1:numel(case_records)
    case_records(case_index).meta.case_index = case_index;
    case_records(case_index).meta.T_oil_C = cfg.temperature_list_C(case_index);
    case_records(case_index).meta.config_signature = string(cfg.config_signature);
end
checkpoint = struct('version', cfg.checkpoint_version, 'source_branch', cfg.required_branch, ...
    'source_commit', cfg.minimum_required_commit, 'config_signature', cfg.config_signature, ...
    'temperature_list_C', cfg.temperature_list_C, 'completed_case_count', 0, 'next_case_index', 1, ...
    'case_records', case_records, 'elapsed_total_s', 0, 'status', 'initialized', ...
    'last_error_id', '', 'last_error_message', '');
end
