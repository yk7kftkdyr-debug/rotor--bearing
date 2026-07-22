function manifest = export_thermal_4cases_results(checkpoint_file, output_root, options)
%EXPORT_THERMAL_4CASES_RESULTS Export controlled journal-ready four-case results.

if nargin < 3, options = struct(); end
options = normalize_options(options);
if ~isfile(checkpoint_file), error('Stage9C1:CheckpointFile', 'The completed four-case checkpoint does not exist.'); end
checkpoint_info_before = dir(checkpoint_file); checkpoint_data = load(checkpoint_file, 'checkpoint'); checkpoint = checkpoint_data.checkpoint;
validate_checkpoint(checkpoint);
if ~options.write_outputs, error('Stage9C1:WriteOutputs', 'Stage9C-1 requires write_outputs=true for output validation.'); end
if isfolder(output_root)
    entries = dir(output_root); entries = entries(~ismember({entries.name}, {'.', '..'}));
    if ~isempty(entries) && ~options.overwrite_existing, error('Stage9C1:ExistingOutput', 'output_root is not empty and overwrite_existing=false.'); end
else
    mkdir(output_root);
end

records = checkpoint.case_records; derived = repmat(empty_derived_placeholder(), 4, 1);
for index = 1:4
    [records(index), derived(index)] = postprocess_thermal_case(records(index), thermal_outer_loop_config());
end
summary_table = build_summary_table(records);
figure_files = make_thermal_4cases_figures(records, derived, summary_table, output_root);
results = build_results(checkpoint, records, summary_table, derived);
mat_file = fullfile(output_root, 'thermal_4cases_results.mat'); csv_file = fullfile(output_root, 'thermal_4cases_summary.csv'); report_file = fullfile(output_root, 'thermal_4cases_report.txt');
save(mat_file, 'results', '-v7'); writetable(summary_table, csv_file); write_report(report_file, results, summary_table);
file_validation_pass = validate_output_set(output_root, mat_file, csv_file, report_file, figure_files);
checkpoint_info_after = dir(checkpoint_file);
if checkpoint_info_before.bytes ~= checkpoint_info_after.bytes || checkpoint_info_before.datenum ~= checkpoint_info_after.datenum
    error('Stage9C1:CheckpointChanged', 'Stage9C-1 must not modify the formal checkpoint.');
end
checkpoint_deleted = false;
if options.delete_checkpoint_after_success
    delete(checkpoint_file); checkpoint_deleted = true;
end
manifest = struct('output_root', string(output_root), 'mat_file', string(mat_file), 'csv_file', string(csv_file), ...
    'report_file', string(report_file), 'figure_files', figure_files, 'physical_acceptance_pass', results.physical_acceptance_pass, ...
    'runtime_acceptance_pass', results.runtime_acceptance_pass, 'file_validation_pass', file_validation_pass, ...
    'checkpoint_deleted', checkpoint_deleted, 'pass', file_validation_pass && results.physical_acceptance_pass);
end

function options = normalize_options(options)
allowed = {'write_outputs', 'delete_checkpoint_after_success', 'overwrite_existing'};
if ~isstruct(options) || ~all(ismember(fieldnames(options), allowed)), error('Stage9C1:Options', 'options may only contain write_outputs, delete_checkpoint_after_success, and overwrite_existing.'); end
defaults = struct('write_outputs', true, 'delete_checkpoint_after_success', false, 'overwrite_existing', false);
for name = string(allowed)
    if ~isfield(options, name), options.(name) = defaults.(name); end
    if ~(islogical(options.(name)) && isscalar(options.(name))), error('Stage9C1:Options', '%s must be a logical scalar.', name); end
end
end

function validate_checkpoint(checkpoint)
if ~isstruct(checkpoint) || checkpoint.status ~= "completed" || checkpoint.completed_case_count ~= 4 || checkpoint.next_case_index ~= 5 || ...
        ~isequal(checkpoint.temperature_list_C, [20 50 80 100]) || numel(checkpoint.case_records) ~= 4
    error('Stage9C1:CheckpointGate', 'The input must be the completed 20/50/80/100 C checkpoint.');
end
cfg = thermal_outer_loop_config();
for index = 1:4
    record = checkpoint.case_records(index);
    if record.meta.status ~= "completed" || record.meta.T_oil_C ~= checkpoint.temperature_list_C(index) || ~record.convergence.pass || ...
            ~record.bearing.ball.stiffness_pass || ~record.bearing.roller.stiffness_pass || ~record.bearing.ball.damping_pass || ...
            ~record.bearing.roller.damping_pass || ~record.linearization.pass || ~record.modal.pass || ~record.dynamics.pass
        error('Stage9C1:CaseGate', 'Case %d has not passed all physical and numerical gates.', index);
    end
    validate_response_gate(record.response, cfg);
end
end

function validate_response_gate(response, cfg)
if ~isequal(string(response.node_labels(:)), string(cfg.output.node_labels(:))) || ~isequal(string(response.component_labels(:)), string(cfg.output.component_labels(:))) || ...
        size(response.displacement_m, 2) ~= 10 || size(response.velocity_m_s, 2) ~= 10 || size(response.acceleration_m_s2, 2) ~= 10 || ...
        any(diff(response.time_s) <= 0) || any(~isfinite(response.time_s)) || any(~isfinite(response.displacement_m), 'all') || ...
        any(~isfinite(response.velocity_m_s), 'all') || any(~isfinite(response.acceleration_m_s2), 'all')
    error('Stage9C1:ResponseGate', 'The formal checkpoint response does not satisfy the required ten-channel finite-output schema.');
end
end

function summary = build_summary_table(records)
meta = [records.meta]; thermal = [records.thermal]; contact = [records.contact]; bearing = [records.bearing];
ball = [thermal.ball]; roller = [thermal.roller]; ball_contact = [contact.ball]; roller_contact = [contact.roller];
ball_bearing = [bearing.ball]; roller_bearing = [bearing.roller]; T = reshape([meta.T_oil_C], [], 1);
summary = table(T, reshape([ball.T_final_C],[],1), reshape([roller.T_final_C],[],1), reshape([ball.T_film_C],[],1), reshape([roller.T_film_C],[],1), ...
    reshape([ball.viscosity_Pa_s],[],1), reshape([roller.viscosity_Pa_s],[],1), reshape([ball.pressure_viscosity_Pa_inv],[],1), reshape([roller.pressure_viscosity_Pa_inv],[],1), ...
    reshape([ball.working_clearance_m],[],1), reshape([roller.working_clearance_m],[],1), reshape([ball.minimum_loaded_film_m],[],1), reshape([roller.minimum_loaded_film_m],[],1), ...
    reshape([ball.friction_power_W],[],1), reshape([roller.friction_power_W],[],1), reshape([ball_contact.loaded_contact_count],[],1), reshape([roller_contact.loaded_contact_count],[],1), ...
    reshape([ball_contact.maximum_contact_load_N],[],1), reshape([roller_contact.maximum_contact_load_N],[],1), reshape([ball_bearing.radial_Kxx_N_m],[],1), reshape([ball_bearing.radial_Kyy_N_m],[],1), ...
    reshape([roller_bearing.radial_Kxx_N_m],[],1), reshape([roller_bearing.radial_Kyy_N_m],[],1), reshape([ball_bearing.radial_Cxx_Ns_m],[],1), reshape([ball_bearing.radial_Cyy_Ns_m],[],1), ...
    reshape([roller_bearing.radial_Cxx_Ns_m],[],1), reshape([roller_bearing.radial_Cyy_Ns_m],[],1), 'VariableNames', ...
    {'T_oil_C','Ball_T_final_C','Roller_T_final_C','Ball_T_film_C','Roller_T_film_C','Ball_viscosity_Pa_s','Roller_viscosity_Pa_s','Ball_pressure_viscosity_Pa_inv','Roller_pressure_viscosity_Pa_inv', ...
    'Ball_working_clearance_m','Roller_working_clearance_m','Ball_minimum_film_m','Roller_minimum_film_m','Ball_friction_power_W','Roller_friction_power_W','Ball_loaded_count','Roller_loaded_count', ...
    'Ball_maximum_contact_load_N','Roller_maximum_contact_load_N','Ball_Kxx_N_m','Ball_Kyy_N_m','Roller_Kxx_N_m','Roller_Kyy_N_m','Ball_Cxx_Ns_m','Ball_Cyy_Ns_m','Roller_Cxx_Ns_m','Roller_Cyy_Ns_m'});
for mode = 1:6, summary.(sprintf('Tracked_f%d_Hz', mode)) = reshape(arrayfun(@(record) record.modal.tracked_frequency_Hz(mode), records), [], 1); end
modal = [records.modal]; linearization = [records.linearization]; initialization = [records.initialization]; audit = [records.audit]; runtime = [records.runtime]; convergence = [records.convergence];
summary.Minimum_MAC = reshape([modal.minimum_MAC], [], 1); summary.Retained_mode_count = reshape([modal.retained_mode_count], [], 1);
summary.R16_x_peak_m = reshape(arrayfun(@(record) record.paper_metrics.peak_displacement_m(3,1), records), [], 1); summary.R16_x_RMS_m = reshape(arrayfun(@(record) record.paper_metrics.rms_displacement_m(3,1), records), [], 1);
summary.R16_x_dominant_frequency_Hz = reshape(arrayfun(@(record) record.paper_metrics.dominant_frequency_Hz(3,1), records), [], 1); summary.R16_x_identified_damping_ratio = reshape(arrayfun(@(record) record.paper_metrics.identified_damping_ratio(3,1), records), [], 1);
summary.C2_radial_peak_m = reshape(arrayfun(@(record) radial_peak(record, 4), records), [], 1); summary.C8_radial_peak_m = reshape(arrayfun(@(record) radial_peak(record, 5), records), [], 1);
summary.Front_peak_ratio = reshape(arrayfun(@(record) record.paper_metrics.front_casing_rotor_peak_ratio, records), [], 1); summary.Rear_peak_ratio = reshape(arrayfun(@(record) record.paper_metrics.rear_casing_rotor_peak_ratio, records), [], 1);
summary.Front_RMS_ratio = reshape(arrayfun(@(record) record.paper_metrics.front_casing_rotor_rms_ratio, records), [], 1); summary.Rear_RMS_ratio = reshape(arrayfun(@(record) record.paper_metrics.rear_casing_rotor_rms_ratio, records), [], 1);
summary.Outer_iterations = reshape([convergence.outer_iterations], [], 1); summary.Linearization_amplitude_m = reshape([linearization.used_amplitude_m], [], 1);
summary.Cold_start_fallback_used = reshape([initialization.cold_start_fallback_used], [], 1); summary.Full_modal_solve_count = reshape([audit.full_modal_solve_count], [], 1); summary.Case_elapsed_s = reshape([runtime.total_case_s], [], 1);
end

function value = radial_peak(record, node_index)
response = record.response.displacement_m; value = max(hypot(response(:,2*node_index-1), response(:,2*node_index)));
end

function results = build_results(checkpoint, records, summary_table, derived)
results = struct(); results.version = 'thermal-journal-results-v1'; results.source_branch = thermal_outer_loop_config().required_branch; results.source_commit = records(1).meta.source_commit; results.config_signature = checkpoint.config_signature;
results.temperature_list_C = checkpoint.temperature_list_C; results.physical_acceptance_pass = true; results.runtime_acceptance_pass = checkpoint.elapsed_total_s <= 1800; results.total_elapsed_s = checkpoint.elapsed_total_s;
results.case_records = sanitize_case_records(records); results.summary_table = summary_table; results.derived = derived;
results.research_scope = struct('rotation_speed_rpm', 9900, 'unbalance_forcing', false, 'external_harmonic_force', false, ...
    'analysis_type', "热一致工作点附近的小扰动降阶自由衰减", 'contact_limits', "无碰摩与强接触脱离", 'thermal_limits', "降阶 EHL 热模型，不是完整 TEHL/THD");
results.audit = struct('cold_start_fallback_at_100C', true, 'total_elapsed_s', checkpoint.elapsed_total_s, 'all_physical_gates_passed', true, ...
    'runtime_target_s', 1800, 'runtime_acceptance_pass', results.runtime_acceptance_pass, 'rayleigh_reference', "20 ℃固定瑞利阻尼被四温度复用", ...
    'newmark_audit', "Newmark 内部无热更新、轴承力调用和重复分解");
end

function records = sanitize_case_records(records)
records = rmfield(records, {'contact','resume_state'});
for index = 1:numel(records)
    records(index).bearing.ball.K_local = zeros(0,0); records(index).bearing.ball.C_ehl_local = zeros(0,0);
    records(index).bearing.roller.K_local = zeros(0,0); records(index).bearing.roller.C_ehl_local = zeros(0,0);
end
end

function write_report(report_file, results, summary)
file_id = fopen(report_file, 'w', 'n', 'UTF-8'); if file_id < 0, error('Stage9C1:ReportOpen', 'Cannot open the UTF-8 report file.'); end
cleanup = onCleanup(@() fclose(file_id));
fprintf(file_id, '四温度转子—轴承—机匣热耦合结果报告\n\n');
fprintf(file_id, '1. 模型和计算范围\n固定转速 9900 r/min；无转子不平衡强迫、无外部简谐力。结果适用于热一致工作点附近的小扰动降阶自由衰减。\n\n');
fprintf(file_id, '2. 四入口温度收敛结果\n20/50/80/100 ℃均通过物理 Gate；外循环次数为 %s。\n\n', mat2str(summary.Outer_iterations.'));
fprintf(file_id, '3. 轴承温度和润滑物性\n前球/后滚子最终温度与黏度见 thermal_4cases_summary.csv。\n\n');
fprintf(file_id, '4. 工作游隙、油膜和摩擦功率\n承载油膜均为正且有限；数值见汇总表。\n\n');
fprintf(file_id, '5. 接触载荷重分布\n球与滚子滚动体载荷分布见 Fig03 与 Fig04。\n\n');
fprintf(file_id, '6. 轴承刚度与 EHL 阻尼\n局部工作点 Kxx/Kyy 与 Cxx/Cyy 见 Fig05。\n\n');
fprintf(file_id, '7. 前六阶模态频率与 MAC\n20 ℃固定瑞利阻尼被四温度复用；前六阶追踪结果见 Fig06。\n\n');
fprintf(file_id, '8. 自由衰减响应及频谱\nFig07 与 Fig08 显示代表性响应。阻尼识别仅在主峰、峰值数和拟合条件同时满足时给出。\n\n');
fprintf(file_id, '9. 转子—机匣响应比\n前后端指标为自由衰减初始扰动下的响应比，不是稳态传递率。\n\n');
fprintf(file_id, '10. 计算审计与适用边界\n100 ℃因旧 checkpoint 缺少 continuation state，使用一次冷启动。Newmark 内部无热更新、轴承力调用和重复分解。累计运行时间 %.3f s。\n', results.total_elapsed_s);
fprintf(file_id, 'physical_acceptance_pass=true\n'); fprintf(file_id, 'runtime_acceptance_pass=%s\n', lower(string(results.runtime_acceptance_pass)));
fprintf(file_id, '1800 s 性能目标未通过；时间超限不等于物理结果失败。\n');
fprintf(file_id, '适用边界：固定 9900 r/min；无转子不平衡强迫；无外部简谐力；热一致工作点附近小扰动；无碰摩与强接触脱离；降阶 EHL 热模型，不是完整 TEHL/THD。\n');
end

function pass = validate_output_set(output_root, mat_file, csv_file, report_file, figure_files)
loaded = load(mat_file, 'results'); if ~isfield(loaded, 'results'), error('Stage9C1:MatReload', 'The results MAT file cannot be reloaded.'); end
summary = readtable(csv_file); if height(summary) ~= 4, error('Stage9C1:CsvRows', 'The summary CSV must contain exactly four rows.'); end
report = string(fileread(report_file)); if strlength(report) == 0 || ~contains(report, 'runtime_acceptance_pass=false'), error('Stage9C1:ReportAudit', 'The report must explicitly record runtime_acceptance_pass=false.'); end
expected = ["thermal_4cases_results.mat"; "thermal_4cases_summary.csv"; "thermal_4cases_report.txt"; erase(string(figure_files(:)), string(output_root) + string(filesep))];
entries = dir(output_root); actual = string({entries(~[entries.isdir]).name}).';
if numel(figure_files) ~= 16 || ~isequal(sort(actual), sort(expected)) || any(endsWith(actual, [".jpg" ".svg" ".fig"]))
    error('Stage9C1:OutputSet', 'The output directory must contain only the MAT, CSV, TXT, and eight PNG/PDF figure pairs.');
end
pass = true;
end

function derived = empty_derived_placeholder()
derived = struct('frequency_Hz', zeros(0,1), 'displacement_spectrum', zeros(0,10), 'velocity_spectrum', zeros(0,10), ...
    'acceleration_spectrum', zeros(0,10), 'uniform_time_s', zeros(0,1), 'uniform_sample_indices', zeros(0,1), ...
    'uniform_sample_dt_s', NaN, 'damping_fit_R2', nan(5,2), 'damping_fit_valid', false(5,2), ...
    'dominant_to_second_peak_ratio', nan(5,2), 'analysis_frequency_ceiling_Hz', NaN, 'warnings', strings(0,1));
end
