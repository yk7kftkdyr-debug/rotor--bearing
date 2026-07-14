function params = apply_official_case_config(params, varargin)
%APPLY_OFFICIAL_CASE_CONFIG Apply the official baseline case consistently.
% Assembly geometry and external steady loads are separate. Reference
% reactions are retained only for comparison with the reconstructed state.

preserve_output_paths = false;
if nargin >= 2 && isstruct(varargin{1})
    options = varargin{1};
    preserve_output_paths = get_field_default(options, 'preserve_output_paths', false);
end

cfg = official_simulation_config();
params = enforce_bearing_layout(params);

params.case_definition.name = 'stage1_static_equilibrium_ball_roller';
params.case_definition.assembly_application = cfg.assembly.description;
params.case_definition.static_load_application = cfg.static_load.description;
params.reference_ball_reaction = cfg.reference_ball_reaction;
params.reference_roller_reaction = cfg.reference_roller_reaction;

for ib = 1:numel(params.bearing)
    params.bearing(ib).preload_y = 0;
    params.bearing(ib).preload_z = 0;
    params.bearing(ib).operating_offset_x = 0;
    params.bearing(ib).operating_offset_y = 0;
    params.bearing(ib).subtract_preload_baseline = false;
end

params.F_ext_y = 0;
params.F_ext_z = 0;
params.static_load.nodes = [params.bearing.rotor_node];
params.static_load.Fx = cfg.static_load.Fx_N;
params.static_load.Fy = cfg.static_load.Fy_N;
params.static_load.Fz = zeros(size(cfg.static_load.Fx_N));

params.use_uploaded_rotor_model = false;
params.require_uploaded_rotor_model = cfg.require_uploaded_rotor_model;
if isfolder(cfg.newmark_model_dir)
    params.uploaded_model_dir = cfg.newmark_model_dir;
else
    params.uploaded_model_dir = cfg.newmark_model_dir;
    warning('Official Newmark model directory not found: %s', cfg.newmark_model_dir);
end

if ~preserve_output_paths
    params.output_dir = cfg.output_root;
    params.bearing_txt_dir = params.output_dir;
    params.figure_dir = fullfile(params.output_dir, 'results', 'figures');
    params.report_file = fullfile(params.output_dir, 'main_report_stage1_static_equilibrium.txt');
    params.main_report_file = params.report_file;
    params.manual_file = fullfile(params.output_dir, char([36724 25215 36716 23376 26426 21283 31995 32479 20223 30495 31243 24207 20351 29992 35828 26126 46 100 111 99 120]));
    params.result_mat_file = fullfile(params.output_dir, 'strong_coupled_all_results_stage1_static_equilibrium.mat');
    params.solver_checkpoint_file = fullfile(params.output_dir, 'strong_coupled_solver_only_stage1_static_equilibrium.mat');
end

params.Fen = cfg.solver.default_Fen;
params.timestep_convergence_Fen = cfg.solver.timestep_convergence_Fen;
params.high_frequency_results_require_timestep_convergence = cfg.solver.high_frequency_results_require_timestep_convergence;
end

function v = get_field_default(s, field, default_value)
if isfield(s, field) && ~isempty(s.(field))
    v = s.(field);
else
    v = default_value;
end
end
