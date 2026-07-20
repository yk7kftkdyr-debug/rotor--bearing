function thermal_result = solve_thermal_mechanical_outer_loop(model, bearing_config, T_oil_C, initial_q)
%SOLVE_THERMAL_MECHANICAL_OUTER_LOOP Two-bearing frozen-state thermal iteration.

if nargin < 4, initial_q = []; end
validateattributes(T_oil_C, {'numeric'}, {'scalar','real','finite'});
cfg = outer_loop_config();
temperature_state = struct('T_ball_C', T_oil_C, 'T_roller_C', T_oil_C);
[thermal_state, ~] = initialize_thermal_case(model, bearing_config, T_oil_C, temperature_state);
thermal_result = empty_result(thermal_state);
q_guess = initial_q;
previous_contact_state = [];
previous_film_state = [];
for iteration = 1:cfg.max_outer_iter
    [thermal_state, ~] = initialize_thermal_case(model, bearing_config, T_oil_C, temperature_state);
    static_result = solve_static_equilibrium_frozen_thermal(model, bearing_config, thermal_state, q_guess);
    [film_state, Qfric_ball_W, Qfric_roller_W] = film_and_power(static_result, bearing_config, model.omega);
    thermal_state.ball.film_thickness = film_state.ball.hmin_m;
    thermal_state.roller.film_thickness = film_state.roller.hmin_m;
    T_target_ball_C = T_oil_C + Qfric_ball_W/cfg.H_ball_W_per_K;
    T_target_roller_C = T_oil_C + Qfric_roller_W/cfg.H_roller_W_per_K;
    metrics = acceptance_metrics(temperature_state, T_target_ball_C, T_target_roller_C, ...
        Qfric_ball_W, Qfric_roller_W, static_result, film_state, previous_contact_state, previous_film_state, cfg, T_oil_C);
    relaxation = fixed_relaxation_after_mechanical_contact_stability(metrics, cfg);
    T_next = struct('T_ball_C', (1-relaxation)*temperature_state.T_ball_C + relaxation*T_target_ball_C, ...
        'T_roller_C', (1-relaxation)*temperature_state.T_roller_C + relaxation*T_target_roller_C);
    thermal_result = assign_result(thermal_result, iteration, static_result, thermal_state, ...
        Qfric_ball_W, Qfric_roller_W, temperature_state, film_state, metrics, false, 0);
    if acceptance_passes(metrics, cfg)
        [final_static, final_thermal, final_film, final_ball_power, final_roller_power] = ...
            final_consistency_recompute(model, bearing_config, T_oil_C, temperature_state, static_result.q);
        final_metrics = acceptance_metrics(temperature_state, ...
            T_oil_C + final_ball_power/cfg.H_ball_W_per_K, T_oil_C + final_roller_power/cfg.H_roller_W_per_K, ...
            final_ball_power, final_roller_power, final_static, final_film, ...
            static_result.contact_state, film_state, cfg, T_oil_C);
        thermal_result = assign_result(thermal_result, iteration, final_static, final_thermal, ...
            final_ball_power, final_roller_power, temperature_state, final_film, final_metrics, ...
            acceptance_passes(final_metrics, cfg), 1);
        return;
    end
    previous_contact_state = static_result.contact_state;
    previous_film_state = film_state;
    temperature_state = T_next;
    q_guess = static_result.q;
end
end

function cfg = outer_loop_config()
cfg = struct('H_ball_W_per_K', 120, 'H_roller_W_per_K', 120, ...
    'relaxation', 0.3, 'relaxation_stable', 0.5, 'max_outer_iter', 20, 'temperature_tolerance_C', 1e-3, ...
    'power_tolerance_W', 0.12, 'mechanical_tolerance', 1e-6, 'axial_tolerance_m', 1e-8);
end

function result = empty_result(thermal_state)
film = struct('h_m', [], 'loaded_mask', [], 'hmin_m', NaN);
result = struct('converged', false, 'iterations', 0, 'q_static', [], ...
    'static_result', [], 'thermal_state', thermal_state, 'Qfric_ball_W', NaN, ...
    'Qfric_roller_W', NaN, 'T_ball_C', NaN, 'T_roller_C', NaN, ...
    'contact_state', [], 'film_state', struct('ball', film, 'roller', film), ...
    'outer_iterations', 0, 'temperature_residual_C', Inf, 'power_residual_W', Inf, ...
    'mechanical_residual', Inf, 'axial_constraint_error_m', Inf, ...
    'relative_Q_change', Inf, 'relative_film_change', Inf, ...
    'final_consistency_recompute_count', 0);
end

function result = assign_result(result, iteration, static_result, thermal_state, Qball, Qroller, temperature_state, film_state, metrics, converged, recompute_count)
result.iterations = iteration; result.outer_iterations = iteration;
result.q_static = static_result.q; result.static_result = static_result; result.thermal_state = thermal_state;
result.Qfric_ball_W = Qball; result.Qfric_roller_W = Qroller;
result.T_ball_C = temperature_state.T_ball_C; result.T_roller_C = temperature_state.T_roller_C;
result.contact_state = static_result.contact_state; result.film_state = film_state;
result.temperature_residual_C = metrics.temperature_residual_C;
result.power_residual_W = metrics.power_residual_W;
result.mechanical_residual = metrics.mechanical_residual;
result.axial_constraint_error_m = metrics.axial_constraint_error_m;
result.relative_Q_change = metrics.relative_Q_change;
result.relative_film_change = metrics.relative_film_change;
result.final_consistency_recompute_count = recompute_count;
result.converged = converged;
end

function metrics = acceptance_metrics(temperature_state, T_target_ball_C, T_target_roller_C, Qball, Qroller, static_result, film_state, previous_contact, previous_film, cfg, T_oil_C)
metrics.temperature_residual_C = max(abs([T_target_ball_C-temperature_state.T_ball_C, T_target_roller_C-temperature_state.T_roller_C]));
metrics.power_residual_W = max(abs([Qball-cfg.H_ball_W_per_K*(temperature_state.T_ball_C-T_oil_C), ...
    Qroller-cfg.H_roller_W_per_K*(temperature_state.T_roller_C-T_oil_C)]));
metrics.mechanical_residual = static_result.normalized_residual;
metrics.axial_constraint_error_m = max(abs([static_result.constraint.front_axial_gap_m, static_result.constraint.numerical_axial_gauge_m]));
metrics.relative_Q_change = relative_contact_change(static_result.contact_state, previous_contact, 'Q');
metrics.relative_film_change = relative_film_change(film_state, previous_film);
end

function pass = acceptance_passes(metrics, cfg)
pass = metrics.temperature_residual_C <= cfg.temperature_tolerance_C && ...
    metrics.power_residual_W <= cfg.power_tolerance_W && ...
    metrics.mechanical_residual <= cfg.mechanical_tolerance && ...
    metrics.axial_constraint_error_m <= cfg.axial_tolerance_m && ...
    metrics.relative_Q_change <= 1e-4 && metrics.relative_film_change <= 1e-3;
end

function relaxation = fixed_relaxation_after_mechanical_contact_stability(metrics, cfg)
relaxation = cfg.relaxation;
if metrics.mechanical_residual <= cfg.mechanical_tolerance && ...
        metrics.axial_constraint_error_m <= cfg.axial_tolerance_m && ...
        metrics.relative_Q_change <= 1e-4 && metrics.relative_film_change <= 1e-3
    relaxation = cfg.relaxation_stable;
end
end

function [static_result, thermal_state, film_state, Qball, Qroller] = final_consistency_recompute(model, bearing_config, T_oil_C, temperature_state, q_guess)
[thermal_state, ~] = initialize_thermal_case(model, bearing_config, T_oil_C, temperature_state);
static_result = solve_static_equilibrium_frozen_thermal(model, bearing_config, thermal_state, q_guess);
[film_state, Qball, Qroller] = film_and_power(static_result, bearing_config, model.omega);
thermal_state.ball.film_thickness = film_state.ball.hmin_m;
thermal_state.roller.film_thickness = film_state.roller.hmin_m;
end

function change = relative_contact_change(current, previous, field)
if isempty(previous), change = Inf; return; end
change = 0;
for k = 1:numel(current.bearings)
    change = max(change, relative_masked_change(current.bearings(k).(field), previous.bearings(k).(field), ...
        current.bearings(k).Q > 0, previous.bearings(k).Q > 0));
end
end

function change = relative_film_change(current, previous)
if isempty(previous), change = Inf; return; end
change = max(relative_masked_change(current.ball.h_m, previous.ball.h_m, current.ball.loaded_mask, previous.ball.loaded_mask), ...
    relative_masked_change(current.roller.h_m, previous.roller.h_m, current.roller.loaded_mask, previous.roller.loaded_mask));
end

function change = relative_masked_change(current, previous, current_mask, previous_mask)
if ~isequal(current_mask, previous_mask), change = Inf; return; end
if ~any(current_mask, 'all'), change = 0; return; end
current_value = current(current_mask); previous_value = previous(previous_mask);
change = norm(current_value-previous_value, inf)/max(norm(previous_value, inf), eps);
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
