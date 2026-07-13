function cfg = official_simulation_config()
%OFFICIAL_SIMULATION_CONFIG Single source for the official baseline case.
% This file fixes the paper baseline without changing the bearing, rotor,
% casing or Newmark solution logic.

root_dir = fileparts(mfilename('fullpath'));
desktop_dir = fullfile(char(java.lang.System.getProperty('user.home')), 'Desktop');

cfg.project_root = root_dir;
cfg.output_root = fullfile(desktop_dir, char([20223 30495]), 'stage1_static_equilibrium_ball_roller');
cfg.newmark_model_dir = fullfile(root_dir, char([78 101 119 109 97 114 107 35299 25925 38556 36724 25215]));
cfg.require_uploaded_rotor_model = false; % The checked-in 17-node fallback is the portable Stage-1 model.

cfg.layout = bearing_layout_config();

cfg.reference_ball_reaction = 20e3; % Result comparison only; never used in calculation.
cfg.reference_roller_reaction = 80e3; % Result comparison only; never used in calculation.
cfg.assembly.description = 'Assembly state establishes contact geometry only; radial reactions come from static equilibrium.';

cfg.static_load.description = 'steady loads act only on their declared physical rotor nodes.';
cfg.static_load.Fx_N = [0 0];
cfg.static_load.Fy_N = [0 0];

cfg.solver.default_Fen = 1024;
cfg.solver.timestep_convergence_Fen = [1024 4096 8192];
cfg.solver.high_frequency_results_require_timestep_convergence = true;
end
