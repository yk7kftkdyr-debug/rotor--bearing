function thermal_result = solve_thermal_mechanical_outer_loop(model, bearing_config, T_oil_C, initial_q)
%SOLVE_THERMAL_MECHANICAL_OUTER_LOOP Two-bearing frozen-state thermal iteration.

if nargin < 4, initial_q = []; end
validateattributes(T_oil_C, {'numeric'}, {'scalar','real','finite'});
cfg = outer_loop_config();
temperature_state = struct('T_ball_C', T_oil_C, 'T_roller_C', T_oil_C);
[thermal_state, ~] = initialize_thermal_case(model, bearing_config, T_oil_C, temperature_state);
thermal_result = empty_result(thermal_state);
q_guess = initial_q;
for iteration = 1:cfg.max_outer_iter
    [thermal_state, ~] = initialize_thermal_case(model, bearing_config, T_oil_C, temperature_state);
    static_result = solve_static_equilibrium_frozen_thermal(model, bearing_config, thermal_state, q_guess);
    [film_state, Qfric_ball_W, Qfric_roller_W] = film_and_power(static_result, bearing_config, model.omega);
    thermal_state.ball.film_thickness = film_state.ball.hmin_m;
    thermal_state.roller.film_thickness = film_state.roller.hmin_m;
    T_target_ball_C = T_oil_C + Qfric_ball_W/cfg.H_ball_W_per_K;
    T_target_roller_C = T_oil_C + Qfric_roller_W/cfg.H_roller_W_per_K;
    T_next = struct('T_ball_C', (1-cfg.relaxation)*temperature_state.T_ball_C + cfg.relaxation*T_target_ball_C, ...
        'T_roller_C', (1-cfg.relaxation)*temperature_state.T_roller_C + cfg.relaxation*T_target_roller_C);
    temperature_error_C = max(abs([T_next.T_ball_C-temperature_state.T_ball_C, T_next.T_roller_C-temperature_state.T_roller_C]));
    power_residual_W = max(abs([Qfric_ball_W-cfg.H_ball_W_per_K*(temperature_state.T_ball_C-T_oil_C), ...
        Qfric_roller_W-cfg.H_roller_W_per_K*(temperature_state.T_roller_C-T_oil_C)]));
    axial_error_m = max(abs([static_result.constraint.front_axial_gap_m, static_result.constraint.numerical_axial_gauge_m]));
    thermal_result.iterations = iteration;
    thermal_result.q_static = static_result.q;
    thermal_result.static_result = static_result;
    thermal_result.thermal_state = thermal_state;
    thermal_result.Qfric_ball_W = Qfric_ball_W;
    thermal_result.Qfric_roller_W = Qfric_roller_W;
    thermal_result.T_ball_C = temperature_state.T_ball_C;
    thermal_result.T_roller_C = temperature_state.T_roller_C;
    thermal_result.contact_state = static_result.contact_state;
    thermal_result.film_state = film_state;
    thermal_result.converged = temperature_error_C <= cfg.temperature_tolerance_C && ...
        power_residual_W <= cfg.power_tolerance_W && static_result.normalized_residual <= cfg.mechanical_tolerance && ...
        axial_error_m <= cfg.axial_tolerance_m;
    if thermal_result.converged, return; end
    temperature_state = T_next;
    q_guess = static_result.q;
end
end

function cfg = outer_loop_config()
cfg = struct('H_ball_W_per_K', 120, 'H_roller_W_per_K', 120, ...
    'relaxation', 0.3, 'max_outer_iter', 20, 'temperature_tolerance_C', 1e-3, ...
    'power_tolerance_W', 0.12, 'mechanical_tolerance', 1e-6, 'axial_tolerance_m', 1e-8);
end

function result = empty_result(thermal_state)
film = struct('h_m', [], 'loaded_mask', [], 'hmin_m', NaN);
result = struct('converged', false, 'iterations', 0, 'q_static', [], ...
    'static_result', [], 'thermal_state', thermal_state, 'Qfric_ball_W', NaN, ...
    'Qfric_roller_W', NaN, 'T_ball_C', NaN, 'T_roller_C', NaN, ...
    'contact_state', [], 'film_state', struct('ball', film, 'roller', film));
end

function [film_state, Qball, Qroller] = film_and_power(static_result, bearing_config, Omega_rad_s)
ball_index = find(arrayfun(@(entry) strcmpi(entry.type, 'ball'), bearing_config), 1);
roller_index = find(arrayfun(@(entry) strcmpi(entry.type, 'roller'), bearing_config), 1);
if isempty(ball_index) || isempty(roller_index), error('ThermalOuterLoop:BearingType', 'solve_thermal_mechanical_outer_loop requires one ball and one roller bearing.'); end
[film_ball, power_ball] = one_bearing_film_and_power(static_result.contact_state.bearings(ball_index), bearing_config(ball_index), Omega_rad_s);
[film_roller, power_roller] = one_bearing_film_and_power(static_result.contact_state.bearings(roller_index), bearing_config(roller_index), Omega_rad_s);
film_state = struct('ball', film_ball, 'roller', film_roller);
Qball = power_ball.Q_fric_W; Qroller = power_roller.Q_fric_W;
if ~isfinite(Qball) || ~isfinite(Qroller), error('ThermalOuterLoop:NonFinitePower', 'solve_thermal_mechanical_outer_loop produced non-finite bearing power.'); end
end

function [film, power] = one_bearing_film_and_power(contact, bearing, Omega_rad_s)
if ~isfield(contact, 'microphysics_state') || ~isfield(contact.microphysics_state, 'temperature') || ~contact.microphysics_state.temperature.enabled
    error('ThermalOuterLoop:FrozenState', 'solve_thermal_mechanical_outer_loop requires an enabled frozen thermal contact state.');
end
temperature = contact.microphysics_state.temperature;
[film, ~] = update_contact_film_from_load(contact.Q, temperature.contact_geometry, temperature.input.eta, temperature.input.alpha_p);
power = compute_static_friction_power(bearing, contact, film, temperature.input.eta, Omega_rad_s);
end
