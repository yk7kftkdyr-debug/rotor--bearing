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

% Document-based segmented rotor geometry. All tabulated quantities are
% diameters in mm and are converted once to SI units here.
rotor_length_mm = [30; 107; 20.5; 40.5; 99; 230.5; 230.5; 82; 70; 102.02; 66.98; 58; 23; 40; 41; 35; 41; 42.5];
rotor_id_mm = [126; 126; 126; 126; 152.4; 152.4; 152.4; 152.4; 152.4; 152.4; 152.4; 152.4; 152.4; 152.4; 135; 135; 135; 152.4];
rotor_od_mm = [160; 160; 180; 210; 178.4; 162.9; 162.9; 174.4; 194; 168; 168; 199.5; 346.5; 168; 347; 702.5; 347; 162.9];
params.rotor_elements = struct('length_m', rotor_length_mm*1e-3, 'inner_diameter_m', rotor_id_mm*1e-3, 'outer_diameter_m', rotor_od_mm*1e-3, 'count', 18, 'mass_scale', ones(18,1));
params.rotor_elements.mass_scale(15:17) = 0; % Disk geometry supplies stiffness only; disk mass is lumped at R16.
params.node_pos = [0; cumsum(params.rotor_elements.length_m)];

% Rotor material, SI units.
params.E = 210e9;
params.nu = 0.30;
params.G = params.E/(2*(1 + params.nu));
params.rho = 7800;
params.shaft_od = params.rotor_elements.outer_diameter_m(1); % Deprecated compatibility field; not used by the segmented builder.
params.shaft_id = params.rotor_elements.inner_diameter_m(1); % Deprecated compatibility field; not used by the segmented builder.

% Mass model: bare shaft core remains distributed; two concentrated masses
% are at R6/R8 and the complete disk mass is concentrated only at R16.
params.mass_target.shaft_N = 798.789;
params.mass_target.concentrated_N = 4142.074;
params.mass_target.disk_N = 1439.930;
params.mass_target.case_N = 20532.445;
params.mass_target.ball_reaction_N = 1581.121;
params.mass_target.roller_reaction_N = 4799.671;
params.concentrated_mass.nodes = [6 8];
params.concentrated_mass.kg = [251.194 171.18];
params.concentrated_mass.inertia_kgm2 = [15.17 7.8 7.8; 6.18 3.2 3.2]; % [Ix Iy Iz] -> [theta_x theta_y theta_z].
params.concentrated_mass.reference_position_m = [0.302 0.733];
params.disk_mass.node = 16;
params.disk_mass.kg = 146.832;
params.disk_mass.inertia_kgm2 = [7.4 3.78 3.78]; % [Ix Iy Iz] -> [theta_x theta_y theta_z].
params.shaft_mass_rho = params.rho; % Deprecated compatibility field; not used by the segmented builder.

% Document-based segmented casing geometry. All tabulated quantities are
% diameters in mm and are converted once to SI units here.
case_length_mm = [25; 120; 215; 20.02; 140; 218; 218; 76; 36; 240; 24; 117.16];
case_id_mm = [320; 925; 940; 725; 1050; 1265; 1265; 1265; 725; 1180; 1172; 1190];
case_od_mm = [955; 955; 1050; 1065; 1280; 1295; 1295; 1295; 1399; 1280; 1256; 1260];
params.case_elements = struct('length_m', case_length_mm*1e-3, 'inner_diameter_m', case_id_mm*1e-3, 'outer_diameter_m', case_od_mm*1e-3, 'count', 12, 'mass_scale', ones(12,1));
params.case_node_pos = [0; cumsum(params.case_elements.length_m)];
params.case_E = 193e9;
params.case_nu = 0.30;
params.case_G = params.case_E/(2*(1 + params.case_nu));
params.case_rho = 7930;
params.case_od = params.case_elements.outer_diameter_m(1); % Deprecated compatibility field; not used by the segmented builder.
params.case_id = params.case_elements.inner_diameter_m(1); % Deprecated compatibility field; not used by the segmented builder.
params.case_mass_rho = params.case_rho; % Deprecated compatibility field; not used by the segmented builder.
params.case_bearing_mass.nodes = [2 8];
params.case_bearing_mass.kg = [10.2 10.2];
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

params = bearing_stage4A_config(params);

end
