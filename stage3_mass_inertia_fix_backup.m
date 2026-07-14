function params = setup_rotor_bearing_input()
%SETUP_ROTOR_BEARING_INPUT Base SI-unit inputs for the coupled model.
% Most engineering working-condition changes are finalized in
% initial_conditions.m. This file keeps the compact rotor/case/bearing data.

params = struct();

% Uploaded Newmark rotor-case model. The path is assembled with Unicode
% code points to avoid Windows MATLAB source-encoding problems.
params.use_uploaded_rotor_model = false; % Stage-1 uses the checked-in 19/13-node model.
params.uploaded_model_dir = char([68 58 92 24120 29992 25991 20214 92 31243 24207 92 31243 24207 92 78 101 119 109 97 114 107 35299 25925 38556 36724 25215]);
params.report_file = 'D:\bearing\gunzi\2.txt';

% Rotor node positions, m: 19 nodes. The former terminal span is split so
% elements 15--17 retain the whole-disc stiffness region; mass is at R16.
params.node_pos = [0 cumsum([30 107 20.5 40.5 99 115.25 115.25 82 70 102.02 66.98 58 23 50 50 40 30 29.5]/1000)];

% Shaft section parameters, SI units.
params.shaft_od = 0.038;
params.shaft_id = 0.000;
params.rho = 7850;
params.E = 2.145e11;
params.G = 8.1e10;
params.nu = 0.2808;

% Mass model: bare shaft core remains distributed; two concentrated masses
% are at R6/R8 and the complete disk mass is concentrated only at R16.
params.mass_target.shaft_N = 798.789;
params.mass_target.concentrated_N = 4142.074;
params.mass_target.disk_N = 1439.930;
params.mass_target.case_N = 20532.445;
params.mass_target.ball_reaction_N = 1581.121;
params.mass_target.roller_reaction_N = 4799.671;
params.concentrated_mass.nodes = [6 8];
% R6/R8 split is calibrated to the supplied vertical reaction closure.
params.concentrated_mass.kg = (params.mass_target.concentrated_N/9.80665)*[0.841568 0.158432];
params.disk_mass.node = 16;
params.disk_mass.kg = params.mass_target.disk_N/9.80665;
params.shaft_mass_rho = (params.mass_target.shaft_N/9.80665)/(pi/4*(params.shaft_od^2-params.shaft_id^2)*params.node_pos(end));

% Fallback case model.
params.case_node_pos = params.node_pos([1 2 3 5 6 8 9 10 12 14 16 18 19]); % 13 nodes; C2=R2, C8=R10.
params.case_od = 0.120;
params.case_id = 0.095;
params.case_rho = 7830;
params.case_E = 2.06e11;
params.case_mass_rho = (params.mass_target.case_N/9.80665)/(pi/4*(params.case_od^2-params.case_id^2)*(params.case_node_pos(end)-params.case_node_pos(1)));
params.case_ground_nodes = [1 13];
params.case_ground_k = 5.0e8;
params.case_ground_c = 2.0e3;

% Bearing node input: front angular-contact ball + rear cylindrical roller.
bearing(1).type = 'ball';
bearing(1).rotor_node = 2;
bearing(1).case_node = 2;
bearing(1).name = 'front angular contact ball bearing';

bearing(2).type = 'roller';
bearing(2).rotor_node = 10;
bearing(2).case_node = 8;
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

% Static reconstruction has no unbalance excitation.
params.unbalance.nodes = [];
params.unbalance.mass_g = [];
params.unbalance.ecc_mm = [];
params.unbalance.phase = [];

% Optional external radial disturbance on rotor nodes, N.
% Preloads are not repeated here.
params.static_load.nodes = [2 10];
params.static_load.Fx = [0 0];
params.static_load.Fy = [0 0];
params.static_load.Fz = [0 0]; % Axial steady load at actual rotor nodes; default is zero.
params.disk_mass.inertia_kgm2 = [0.20 0.20 0.35]; % Engineering principal inertias, pending measured disk inertia data.
params.case_ground_axial_k = 1.0e8; % Configurable engineering initial values; not copied from transverse support.
params.case_ground_axial_c = 100;
params.case_ground_bending_rot_k = 1.0e6;
params.case_ground_bending_rot_c = 10;
params.case_ground_torsion_k = 1.0e6;
params.case_ground_torsion_c = 10;
params.legacy_rotor_bending_reference_k = 1.0e2; % Retains the verified 4-DOF fallback's numerical bending-rotation reference.
params.stage3_engineering_initials_pending_calibration = true; % Axial/torsional foundation values and disk principal inertias require measured calibration.

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
