function params = setup_rotor_bearing_input()
%SETUP_ROTOR_BEARING_INPUT Base SI-unit inputs for the coupled model.
% Most engineering working-condition changes are finalized in
% initial_conditions.m. This file keeps the compact rotor/case/bearing data.

params = struct();

% Uploaded Newmark rotor-case model. The path is assembled with Unicode
% code points to avoid Windows MATLAB source-encoding problems.
params.use_uploaded_rotor_model = true;
params.uploaded_model_dir = char([68 58 92 24120 29992 25991 20214 92 31243 24207 92 31243 24207 92 78 101 119 109 97 114 107 35299 25925 38556 36724 25215]);
params.report_file = 'D:\bearing\gunzi\2.txt';

% Rotor node positions, m.
params.node_pos = [0 cumsum([30 107 20.5 40.5 99 115.25 115.25 82 70 102.02 66.98 58 23 98.5 58.5 42.5]/1000)];

% Shaft section parameters, SI units.
params.shaft_od = 0.038;
params.shaft_id = 0.000;
params.rho = 7850;
params.E = 2.145e11;
params.G = 8.1e10;
params.nu = 0.2808;

% Discs.
params.disc_nodes = [4 6 8 14];
params.disc_od = [0.105 0.115 0.110 0.090];
params.disc_id = [0.038 0.038 0.038 0.038];
params.disc_thick = [0.025 0.030 0.028 0.022];

% Fallback case model.
params.case_node_pos = params.node_pos;
params.case_od = 0.120;
params.case_id = 0.095;
params.case_rho = 7830;
params.case_E = 2.06e11;
params.case_ground_k = 2.0e8;
params.case_ground_c = 1.5e3;

% Bearing node input: front angular-contact ball + rear cylindrical roller.
bearing(1).type = 'ball';
bearing(1).rotor_node = 2;
bearing(1).case_node = 3;
bearing(1).name = 'front angular contact ball bearing';

bearing(2).type = 'roller';
bearing(2).rotor_node = 10;
bearing(2).case_node = 10;
bearing(2).name = 'rear cylindrical roller bearing';

% Front ball bearing parameters.
bearing(1).n = 22;
bearing(1).Db = 23.812e-3;
bearing(1).Dm = 0.190;
bearing(1).clearance0 = 0.175e-3;
bearing(1).axial_clearance0 = 0.040e-3;
bearing(1).delta_fit = 0;
bearing(1).delta_thermal = 0;
bearing(1).delta_centrifugal = 0;
bearing(1).contact_angle = 14*pi/180;
bearing(1).mu = 0.08;
bearing(1).oil_viscosity = 27.6e-6*1003.5;
bearing(1).oil_density = 1003.5;
bearing(1).pressure_viscosity = 1.20e-8;
bearing(1).temperature = 20;
bearing(1).K_point = 2.443e9;
bearing(1).contact_damping = 180;
bearing(1).oil_damping = 1.2e3;
bearing(1).oil_stiffness = 8.0e6;
bearing(1).include_oil_stiffness_force = false;
bearing(1).include_oil_damping_force = true;
bearing(1).ehl_load_correction_enable = true;
bearing(1).ehl_load_exponent = -0.067;
bearing(1).ehl_load_factor_min = 0.4;
bearing(1).ehl_load_factor_max = 2.5;
bearing(1).oil_film0 = 0.12e-6;
bearing(1).roughness_rms = 0.10e-6;
bearing(1).texture_amplitude = 0.03e-6;
bearing(1).waviness_order = 2;
bearing(1).max_contact_force = 4.0e4;
bearing(1).max_delta = 1.0e-4;
bearing(1).linear_k = 1.8e7;
bearing(1).linear_c = 1.2e3;

% Rear cylindrical roller bearing parameters.
bearing(2).n = 30;
bearing(2).Dw = 0.015;
bearing(2).Dm = 0.190;
bearing(2).L = 0.015;
bearing(2).clearance0 = 0.195e-3;
bearing(2).axial_clearance0 = 0.040e-3;
bearing(2).delta_fit = 0;
bearing(2).delta_thermal = 0;
bearing(2).delta_centrifugal = 0;
bearing(2).mu = 0.10;
bearing(2).oil_viscosity = 27.6e-6*1003.5;
bearing(2).oil_density = 1003.5;
bearing(2).pressure_viscosity = 1.20e-8;
bearing(2).temperature = 20;
bearing(2).load_direction = 'radial';
bearing(2).K_line = 2.4e8;
bearing(2).contact_damping = 260;
bearing(2).oil_damping = 1.8e3;
bearing(2).oil_stiffness = 1.0e7;
bearing(2).include_oil_stiffness_force = false;
bearing(2).include_oil_damping_force = true;
bearing(2).ehl_load_correction_enable = true;
bearing(2).ehl_load_exponent = -0.067;
bearing(2).ehl_load_factor_min = 0.4;
bearing(2).ehl_load_factor_max = 2.5;
bearing(2).oil_film0 = 0.8e-6;
bearing(2).roughness_rms = 0.10e-6;
bearing(2).texture_amplitude = 0.05e-6;
bearing(2).waviness_order = 3;
bearing(2).max_contact_force = 6.0e4;
bearing(2).max_delta = 2.0e-4;
bearing(2).linear_k = 2.6e7;
bearing(2).linear_c = 1.5e3;

params.bearing = bearing;

% Unbalance input.
params.unbalance.nodes = [4 6 8 14];
params.unbalance.mass_g = [126.3 159.4 149.7 145.9];
params.unbalance.ecc_mm = [0.1206 0.1206 0.1206 0.1206];
params.unbalance.phase = [0 pi/3 2*pi/3 pi];

% Optional external radial disturbance on rotor nodes, N.
% Preloads are not repeated here.
params.static_load.nodes = [2 10];
params.static_load.Fx = [0 0];
params.static_load.Fy = [0 0];

% Speed input.
params.rpm = 9900;
params.omega = 2*pi*params.rpm/60;

% Newmark input.
params.Fen = 1024;
params.n_Fen = 20;
params.gamma = 0.65;
params.beta = 0.330625;
params.newmark_iterations = 6;
params.newmark_relaxation = 0.45;
params.startup_ramp_cycles = 3;
params.max_step_increment = inf;
params.response_limit = 2.0e-3;
params.model_case = 'full_tribology';

% Fallback model damping.
params.rayleigh_alpha = 2.0;
params.rayleigh_beta = 2.0e-6;

% Optional initial relative closure of the clearance, [x y] in m.
params.initial_bearing_offset = [0; 0];

end
