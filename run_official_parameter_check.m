function ok = run_official_parameter_check()
%RUN_OFFICIAL_PARAMETER_CHECK Check official baseline parameters only.
% This function does not run the dynamic simulation.

params = initial_conditions();
validate_official_case_config(params, 'official parameter check');

fprintf('Official baseline parameter check passed.\n');
fprintf('  bearing1: %s, rotor/case node %d/%d, preload_z %.0f N\n', ...
    params.bearing(1).name, params.bearing(1).rotor_node, params.bearing(1).case_node, params.bearing(1).preload_z);
fprintf('  bearing2: %s, rotor/case node %d/%d, preload_z %.0f N\n', ...
    params.bearing(2).name, params.bearing(2).rotor_node, params.bearing(2).case_node, params.bearing(2).preload_z);
fprintf('  static_load Fx/Fy: %s / %s N\n', mat2str(params.static_load.Fx), mat2str(params.static_load.Fy));
fprintf('  Newmark model dir: %s\n', params.uploaded_model_dir);
fprintf('  Fen default: %d; convergence Fen list: %s\n', params.Fen, mat2str(params.timestep_convergence_Fen));

if exist(fullfile(pwd, char([28909 25928 24212 30456 20851 31243 24207]), 'setup_thermal_case_common.m'), 'file')
    addpath(fullfile(pwd, char([28909 25928 24212 30456 20851 31243 24207])));
end
thermal_params = setup_thermal_case_common(params, 50, tempdir, struct());
validate_official_case_config(thermal_params, 'official thermal parameter check');
fprintf('  thermal common check: T=%.0f degC, preload_z [%.0f %.0f] N, static_load Fy=%s N\n', ...
    thermal_params.thermal_case.temperature, thermal_params.bearing(1).preload_z, ...
    thermal_params.bearing(2).preload_z, mat2str(thermal_params.static_load.Fy));

ok = true;
end
