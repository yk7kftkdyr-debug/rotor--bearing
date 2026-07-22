function cfg = thermal_outer_loop_config()
%THERMAL_OUTER_LOOP_CONFIG Immutable Stage9 four-case execution contract.

cfg = struct();
cfg.version = 'thermal-outer-loop-v1';
cfg.schema_version = 'case-schema-v1';
cfg.checkpoint_version = 'checkpoint-v1';
cfg.required_branch = 'thermal-outer-loop-stage1-v1';
cfg.minimum_required_commit = '66207b4932ea3005642c694a9aa06ab3361e2f08';
cfg.temperature_list_C = [20 50 80 100];
cfg.rotation_speed_rpm = 9900;
cfg.rotation_speed_rad_s = 2*pi*cfg.rotation_speed_rpm/60;
cfg.rotation_frequency_Hz = 165;

cfg.max_outer_iterations = 20;
cfg.temperature_tolerance_C = 1e-3;
cfg.power_tolerance_W = 0.12;
cfg.mechanical_tolerance = 1e-6;
cfg.axial_constraint_tolerance_m = 1e-8;
cfg.relative_load_tolerance = 1e-4;
cfg.relative_film_tolerance = 1e-3;
cfg.linearization_amplitudes_m = [1e-6 0.2e-6];
cfg.linearization_force_error_tolerance = 0.05;
cfg.edge_load_ratio = 0.01;
cfg.thermal = struct('H_ball_W_per_K', 120, 'H_roller_W_per_K', 120);
cfg.rayleigh = struct('reference_temperature_C', 20, 'target_damping_ratio', 0.01);

cfg.total_time_budget_s = 1800;
cfg.checkpoint_safety_margin_s = 120;
cfg.timing_labels = ["thermal_outer_loop"; "bearing_stiffness"; "ehl_damping"; "linearization"; ...
    "modal_reduction"; "free_decay"; "postprocess"; "checkpoint"];

cfg.modal = struct();
cfg.modal.minimum_mode_count = 12;
cfg.modal.preferred_mode_count = 22;
cfg.modal.maximum_mode_count = 30;
cfg.modal.certified_reference_count = 30;
cfg.modal.effective_mass_target = 0.95;
cfg.modal.strategy = '20C_reference_subspace_with_full_fallback';
cfg.modal.allow_full_modal_fallback = true;
cfg.modal.maximum_full_fallback_per_case = 1;
cfg.modal.ritz_backward_tolerance = 1e-10;
cfg.modal.frequency_tolerance_floor = 1e-6;
cfg.modal.frequency_tolerance_cap = 1e-4;
cfg.modal.tracked_mode_count = 6;
cfg.modal.minimum_single_mode_MAC = 0.85;
cfg.modal.minimum_cluster_subspace_MAC = 0.90;
cfg.modal.cluster_relative_frequency_gap = 0.01;

cfg.dynamics = struct('gamma', 0.5, 'beta', 0.25, 'maximum_internal_steps', 500000, ...
    'maximum_saved_steps', 30000, 'initial_node', 16, 'initial_direction', 'ux');
cfg.output = struct();
cfg.output.node_labels = ["R2"; "R10"; "R16"; "C2"; "C8"];
cfg.output.component_labels = ["x"; "y"];
cfg.result_root = fullfile('results', 'thermal_outer_loop');
cfg.checkpoint_file = fullfile(cfg.result_root, 'thermal_4cases_checkpoint.mat');
cfg.results_file = fullfile(cfg.result_root, 'thermal_4cases_results.mat');
cfg.summary_file = fullfile(cfg.result_root, 'thermal_4cases_summary.csv');
cfg.report_file = fullfile(cfg.result_root, 'thermal_4cases_report.txt');
cfg.figure_directory = fullfile(cfg.result_root, 'figures');
cfg.output.maximum_figure_count = 8;
cfg.output.mat_save_format = '-v7';
cfg.output.approved_figure_ids = ["bearing_temperature_properties"; "clearance_film_power"; ...
    "ball_load_distribution"; "roller_load_distribution"; "bearing_stiffness_damping"; ...
    "tracked_modal_frequency"; "free_decay_time_history"; "free_decay_spectrum_transfer"];

cfg.physics_signature = sprintf(['T=[20,50,80,100]|rpm=%d|omega=%.17g|Hball=%.17g|Hroller=%.17g|' ...
    'outer=%d|tolT=%.17g|tolP=%.17g|tolM=%.17g|tolAx=%.17g|tolQ=%.17g|tolH=%.17g|' ...
    'amp=[%.17g,%.17g]|forceErr=%.17g|edgeLoad=%.17g|mass=%.17g|rayleigh=%.17g'], ...
    cfg.rotation_speed_rpm, cfg.rotation_speed_rad_s, cfg.thermal.H_ball_W_per_K, cfg.thermal.H_roller_W_per_K, ...
    cfg.max_outer_iterations, cfg.temperature_tolerance_C, cfg.power_tolerance_W, cfg.mechanical_tolerance, ...
    cfg.axial_constraint_tolerance_m, cfg.relative_load_tolerance, cfg.relative_film_tolerance, ...
    cfg.linearization_amplitudes_m(1), cfg.linearization_amplitudes_m(2), cfg.linearization_force_error_tolerance, ...
    cfg.edge_load_ratio, cfg.modal.effective_mass_target, cfg.rayleigh.target_damping_ratio);
cfg.solver_signature = sprintf(['outer=%d|modal=[%d,%d,%d,%d]|strategy=%s|fallback=%d|ritz=%.17g|' ...
    'newmark=[%.17g,%.17g]|steps=%d|saved=%d'], cfg.max_outer_iterations, cfg.modal.minimum_mode_count, ...
    cfg.modal.preferred_mode_count, cfg.modal.maximum_mode_count, cfg.modal.certified_reference_count, ...
    cfg.modal.strategy, cfg.modal.allow_full_modal_fallback, cfg.modal.ritz_backward_tolerance, ...
    cfg.dynamics.gamma, cfg.dynamics.beta, cfg.dynamics.maximum_internal_steps, cfg.dynamics.maximum_saved_steps);
cfg.output_signature = sprintf('schema=%s|nodes=%s|components=%s|tracked=%d|figures=%d', cfg.schema_version, ...
    strjoin(cellstr(cfg.output.node_labels), ','), strjoin(cellstr(cfg.output.component_labels), ','), ...
    cfg.modal.tracked_mode_count, cfg.output.maximum_figure_count);
cfg.config_signature = sprintf('physics{%s}|solver{%s}|output{%s}', cfg.physics_signature, cfg.solver_signature, cfg.output_signature);
end
