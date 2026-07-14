function params = initial_conditions()
%INITIAL_CONDITIONS Unified engineering input file for main.m.
% SI units are used throughout: m, kg, s, N, Pa, rad/s.

params = setup_rotor_bearing_input();

% Legacy output paths are overwritten at the end by
% apply_official_case_config(). They are kept here only so older scripts that
% stop before the final baseline application still have valid fields.
params.uploaded_model_dir = char([68 58 92 24120 29992 25991 20214 92 31243 24207 92 31243 24207 92 78 101 119 109 97 114 107 35299 25925 38556 36724 25215]);
params.output_dir = 'D:\bearing\gunzi';
params.bearing_txt_dir = 'D:\mixed\gun_qiu';
params.report_file = fullfile(params.output_dir, '2.txt');
params.main_report_file = params.report_file;
params.manual_file = fullfile(params.output_dir, char([36724 25215 36716 23376 26426 21283 31995 32479 20223 30495 31243 24207 20351 29992 35828 26126 46 100 111 99 120]));
params.figure_dir = fullfile(params.output_dir, 'results', 'figures');
params.result_mat_file = fullfile(params.output_dir, 'strong_coupled_all_results.mat');

% Speed.
params.rpm = 9900;
params.omega = 2*pi*params.rpm/60;

% Lubricant model: Mobil Jet Oil II equivalent 5 cSt synthetic ester
% turbine oil. Viscosity is fitted from 40 degC and 100 degC kinematic
% viscosities; pressure-viscosity coefficient is treated as an oil property
% shared by the ball and roller bearings.
params.lubricant.name = 'Mobil Jet Oil II equivalent 5 cSt synthetic ester turbine oil';
params.lubricant.nu40_cSt = 27.6;
params.lubricant.nu100_cSt = 5.1;
params.lubricant.rho15 = 1003.5;
params.lubricant.T_ref_visc = 40;
params.lubricant.alpha_p40 = 1.20e-8;
params.lubricant.k_alpha = 0.008;
params.lubricant.current_temperature = 20;
oil0 = lubricant_properties(params.lubricant.current_temperature, params);
params.lubricant.nu_cSt_current = oil0.nu_cSt;
params.lubricant.eta_current = oil0.eta;
params.lubricant.alpha_p_current = oil0.alpha_p;
params.lubricant.k_eta = oil0.k_eta;

params.thermal_model = 'fixed_inner_outer_temperature_difference';
params.temperature_list = [20 50 80 100];
params.deltaT_io_list = [0 5 10 15];

% Stage 1 static work point: unbalance force is explicitly zero.
params.unbalance.nodes = [];
params.unbalance.mass_g = [];
params.unbalance.ecc_mm = [];
params.unbalance.phase_mode = 'custom';
params.unbalance.random_seed = 20260521;
params.unbalance.phase = apply_unbalance_phase_mode(params.unbalance);

% Bearing nodes.
params.bearing(1).type = 'ball';
params.bearing(1).rotor_node = 2;
params.bearing(1).case_node = 2;
params.bearing(1).name = 'front high-speed angular contact ball bearing';

params.bearing(2).type = 'roller';
params.bearing(2).rotor_node = 10;
params.bearing(2).case_node = 8;
params.bearing(2).name = 'rear high-speed cylindrical roller bearing';

% Engineering initial clearances used in the thermal-effect study.
params.bearing(1).clearance0 = 0.175e-3;
params.bearing(1).axial_clearance0 = 0.040e-3;
params.bearing(2).clearance0 = 0.195e-3;
params.bearing(2).axial_clearance0 = 0.040e-3;

for i = 1:numel(params.bearing)
    params.bearing(i).oil_viscosity = params.lubricant.eta_current;
    params.bearing(i).pressure_viscosity = params.lubricant.alpha_p_current;
    params.bearing(i).oil_density = params.lubricant.rho15;
end

% Prescribed radial reactions are disabled. Assembly contact geometry is
% defined once here and is independent of working external loads.
params.bearing(1).preload_y = 0;
params.bearing(1).preload_z = 0;
params.bearing(2).preload_y = 0;
params.bearing(2).preload_z = 0;
params.bearing(1).support_series_kx = 8.0e7;
params.bearing(1).support_series_ky = 8.0e7;
params.bearing(2).support_series_kx = 6.0e7;
params.bearing(2).support_series_ky = 6.0e7;

% Optional external radial disturbance. Enable only for load perturbation
% studies. Program x maps to local bearing y; program y maps to local
% bearing z.
params.F_ext_y = 0;
params.F_ext_z = 0;
params.static_load.nodes = [params.bearing.rotor_node];
params.static_load.Fx = [0 0];
params.static_load.Fy = [0 0];
params.static_load.Fz = [0 0];
params.bearing_model_stage = '2D_force_on_6DOF_structure';

% Working-clearance components. Baseline keeps measured/nominal values.
% thermal_fit_centrifugal can be enabled when measured temperatures and
% fits are available; do not use assumed thermal data as the paper baseline.
params.clearance_case = 'baseline';
for i = 1:numel(params.bearing)
    params.bearing(i).clearance_fit = get_field_default(params.bearing(i), 'delta_fit', 0);
    params.bearing(i).clearance_temp = get_field_default(params.bearing(i), 'delta_thermal', 0);
    params.bearing(i).clearance_centrifugal = get_field_default(params.bearing(i), 'delta_centrifugal', 0);
    params.bearing(i).clearance_other = get_field_default(params.bearing(i), 'clearance_other', 0);
    params.bearing(i).delta_fit = params.bearing(i).clearance_fit;
    params.bearing(i).delta_thermal = params.bearing(i).clearance_temp;
    params.bearing(i).delta_centrifugal = params.bearing(i).clearance_centrifugal;
    params.bearing(i).assembly_state.radial_clearance = params.bearing(i).clearance0;
    params.bearing(i).assembly_state.clearance_change_fit = params.bearing(i).clearance_fit;
    params.bearing(i).assembly_state.clearance_change_thermal = params.bearing(i).clearance_temp;
    params.bearing(i).assembly_state.initial_interference = 0; % 待标定：缺少实测装配过盈量。
    params.bearing(i).include_oil_stiffness_force = get_field_default(params.bearing(i), 'include_oil_stiffness_force', false);
    params.bearing(i).include_oil_damping_force = get_field_default(params.bearing(i), 'include_oil_damping_force', true);
    params.bearing(i).ehl_load_correction_enable = get_field_default(params.bearing(i), 'ehl_load_correction_enable', true);
    params.bearing(i).ehl_load_exponent = get_field_default(params.bearing(i), 'ehl_load_exponent', -0.067);
    params.bearing(i).ehl_load_factor_min = get_field_default(params.bearing(i), 'ehl_load_factor_min', 0.4);
    params.bearing(i).ehl_load_factor_max = get_field_default(params.bearing(i), 'ehl_load_factor_max', 2.5);
end

% Newmark parameters tuned for stable high-speed strong coupling.
params.Fen = 1024;
params.n_Fen = 20;
params.gamma = 0.65;
params.beta = 0.330625;
params.newmark_iterations = 6;
params.newmark_relaxation = 0.45;
params.solver.max_iter = 20;
params.solver.tol_x = 1.0e-8;
params.solver.tol_R = 1.0e-6;
params.solver.tol_u = 1.0e-8;
params.solver.u_ref = 1.0e-9;
params.solver.F_ref = 1.0;
params.solver.bearing_tangent_u = 1.0e-8;
params.solver.bearing_tangent_v = 1.0e-6;
params.solver.relaxation = 0.45;
params.solver.gamma_note = 'gamma > 0.5 adds numerical damping and helps suppress nonphysical high-frequency oscillation.';
params.config.enable_residual_diagnostics = true;
params.config.use_normalized_residual_assisted_convergence = false;
params.config.allow_normalized_assisted_for_batch_screening = false;
params.startup_ramp_cycles = 3;
params.response_limit = 2.0e-2; % Must exceed the reconstructed static-workpoint displacement before dynamic screening.
params.model_case = 'fixed_nonlinear'; % Stage 2 nonlinear contact about q0 without cage-phase baseline drift.
params.initial_bearing_offset = [0; 0];
params.static_equilibrium.enable = true;
params.static_equilibrium.gravity = 9.80665;
params.static_equilibrium.vertical_direction = 'y'; % Program y is local bearing z (vertical radial direction).
params.static_equilibrium.include_case_gravity = true;
params.static_equilibrium.load_steps = 8;
params.static_equilibrium.max_iter = 35;
params.static_equilibrium.tol_R_newton = 5e-3;
params.static_equilibrium.tol_R = 1e-8;
params.static_equilibrium.refine_with_fsolve = true;
params.static_equilibrium.tol_q = 1e-8;
params.static_equilibrium.F_ref = 1.0;
params.static_equilibrium.q_ref = 1e-9;
params.static_equilibrium.tangent_step = 1e-8;
params.static_equilibrium.contact_seed = 2e-6;
params.use_legacy_linear_bearing_coupling = false; % Stage 1: avoid duplicating nonlinear contact stiffness.
params.plot_diagnostics = true;
params.save_full_bearing_state = false;
params.postprocess_in_separate_matlab = true;
params.solver_checkpoint_file = fullfile(params.output_dir, 'strong_coupled_solver_only.mat');

% Contact smoothing and normal damping. Smooth contact is used only around
% delta = 0 and keeps Hertz contact unchanged away from the clearance edge.
params.contact.smooth_enable = true;
params.contact.smooth_delta = 1.0e-7;
params.contact.damping_enable = true;
params.contact.cn_ratio = 0.05;
params.contact.stiffness_smooth_enable = true;
params.contact.alpha_k = 0.8;
params.contact.dynamic_clearance_enable = true;

% Thermal, texture, roughness, debris and waviness interfaces. They are
% off by default for the healthy baseline condition.
for i = 1:numel(params.bearing)
    params.bearing(i).contact_cn = get_field_default(params.bearing(i), 'contact_damping', 100);
    params.bearing(i).thermal.enable = false;
    params.bearing(i).thermal.T_ref = 20;
    params.bearing(i).thermal.T_inner = 20;
    params.bearing(i).thermal.T_outer = 20;
    params.bearing(i).thermal.T_roller = 20;
    params.bearing(i).thermal.T_oil = 20;
    params.bearing(i).thermal.alpha_inner = 12.5e-6;
    params.bearing(i).thermal.alpha_outer = 12.5e-6;
    params.bearing(i).thermal.alpha_roller = 12.5e-6;
    params.bearing(i).thermal.beta_eta = 0.025;

    params.bearing(i).surface.texture_enable = false;
    params.bearing(i).surface.texture_gamma = 0;
    params.bearing(i).surface.Cr = 1.0;

    params.bearing(i).roughness.enable = false;
    params.bearing(i).roughness.Rq_inner = get_field_default(params.bearing(i), 'roughness_rms', 0.1e-6);
    params.bearing(i).roughness.Rq_outer = get_field_default(params.bearing(i), 'roughness_rms', 0.1e-6);
    params.bearing(i).roughness.lambda_ratio = NaN;

    params.bearing(i).debris.enable = false;
    params.bearing(i).debris.ud = 0;

    params.bearing(i).waviness.enable = false;
    params.bearing(i).waviness.P2_amp = 0;
    params.bearing(i).waviness.order = get_field_default(params.bearing(i), 'waviness_order', 1);
end

% Apply the Stage-1 configuration with assembly and external-load inputs separated.
params = apply_official_case_config(params);

function phase = apply_unbalance_phase_mode(unbalance)
nodes = unbalance.nodes;
mode = get_field_default(unbalance, 'phase_mode', 'same_phase');
switch lower(mode)
    case 'same_phase'
        phase = zeros(size(nodes));
    case 'distributed'
        phase = [0 pi/3 2*pi/3 pi];
        if numel(phase) ~= numel(nodes)
            phase = linspace(0, pi, numel(nodes));
        end
    case 'random_seeded'
        rng(get_field_default(unbalance, 'random_seed', 1));
        phase = 2*pi*rand(size(nodes));
    case 'custom'
        phase = get_field_default(unbalance, 'phase', zeros(size(nodes)));
    otherwise
        phase = zeros(size(nodes));
end
phase = reshape(phase, size(nodes));
end

end

function v = get_field_default(s, field, default_value)
if isfield(s, field) && ~isempty(s.(field))
    v = s.(field);
else
    v = default_value;
end
end
