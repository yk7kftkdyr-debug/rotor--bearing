function cfg = official_simulation_config()
%OFFICIAL_SIMULATION_CONFIG Single source for the official baseline case.
% This file fixes the paper baseline without changing the bearing, rotor,
% casing or Newmark solution logic.

root_dir = fileparts(mfilename('fullpath'));
desktop_dir = fullfile(char(java.lang.System.getProperty('user.home')), 'Desktop');

cfg.project_root = root_dir;
cfg.output_root = fullfile(desktop_dir, char([20223 30495]), 'baseline_ball20_roller80');
cfg.newmark_model_dir = fullfile(root_dir, char([78 101 119 109 97 114 107 35299 25925 38556 36724 25215]));
cfg.require_uploaded_rotor_model = true;

cfg.layout = bearing_layout_config();

cfg.preload.description = 'bearing quasi-dynamic preload only; not rotor-node static load';
cfg.preload.bearing1_y_N = 0;
cfg.preload.bearing1_z_N = 20e3;
cfg.preload.bearing2_y_N = 0;
cfg.preload.bearing2_z_N = 80e3;

cfg.static_load.description = 'disabled for baseline; preload is not repeated in the rotor equation';
cfg.static_load.Fx_N = [0 0];
cfg.static_load.Fy_N = [0 0];

cfg.solver.default_Fen = 1024;
cfg.solver.timestep_convergence_Fen = [1024 4096 8192];
cfg.solver.high_frequency_results_require_timestep_convergence = true;
end
