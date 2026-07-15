function [contact_mod, thermal_state] = solve_closed_loop_thermal_contact(contact_base, operating_state, cfg)
%SOLVE_CLOSED_LOOP_THERMAL_CONTACT Deterministic single-bearing local loop.
%   No temperature history crosses a Newton call, time step or invocation.

assert(isfield(operating_state, 'evaluate_raw_contact') && ...
    isa(operating_state.evaluate_raw_contact, 'function_handle'), ...
    'solve_closed_loop_thermal_contact:MissingRawEvaluator', ...
    'A pure local evaluate_raw_contact callback is required.');
assert(isfield(operating_state, 'omega_rad_s'), ...
    'solve_closed_loop_thermal_contact:MissingOmega', 'Current omega_rad_s is required.');

T_oil_C = cfg.oil_temperature_C;
H_W_per_K = cfg.H_W_per_K;
relaxation = cfg.relaxation;
tolerance_C = cfg.temperature_tolerance_C;
max_iterations = cfg.max_iterations;
validateattributes(T_oil_C, {'numeric'}, {'real', 'finite', 'scalar'});
validateattributes(H_W_per_K, {'numeric'}, {'real', 'finite', 'scalar', 'positive'});
validateattributes(relaxation, {'numeric'}, {'real', 'finite', 'scalar', '>', 0, '<=', 1});
validateattributes(tolerance_C, {'numeric'}, {'real', 'finite', 'scalar', 'positive'});
validateattributes(max_iterations, {'numeric'}, {'real', 'finite', 'scalar', 'integer', 'positive'});

% Every invocation starts from oil temperature and one unmodified raw solve.
T_old_C = T_oil_C;
raw_result = operating_state.evaluate_raw_contact(contact_base);
local_contact_evaluations = 1;
Q_N = raw_result.Q;
converged = false;
iterations = 0;

for k = 1:max_iterations
    iterations = k;
    Q_previous_N = Q_N;
    [contact_trial, properties, film_state] = thermal_contact_trial( ...
        contact_base, operating_state.omega_rad_s, Q_N, T_old_C, cfg);
    raw_result = operating_state.evaluate_raw_contact(contact_trial);
    local_contact_evaluations = local_contact_evaluations + 1;
    friction = thermal_friction_power(contact_base.brg, operating_state.omega_rad_s, ...
        raw_result, properties.eta_Pa_s, contact_trial.film_thickness, contact_base.local);
    T_target_C = T_oil_C + friction.Q_fric_W / H_W_per_K;
    T_new_C = (1 - relaxation) * T_old_C + relaxation * T_target_C;
    Q_N = raw_result.Q;
    contact_change = max(abs(Q_N - Q_previous_N), [], 'all') / ...
        max(max(abs(Q_N), [], 'all'), 1);
    if abs(T_new_C - T_old_C) < tolerance_C && ...
            abs(T_target_C - T_new_C) < tolerance_C && contact_change < 1e-8
        T_old_C = T_new_C;
        converged = true;
        break;
    end
    T_old_C = T_new_C;
end

if ~converged && cfg.fail_on_nonconvergence
    error('solve_closed_loop_thermal_contact:Nonconvergence', ...
        'Local thermal contact did not converge within %d iterations.', max_iterations);
end

% A final full local solve prevents returning a force one iteration behind T.
T_final_C = T_old_C;
[contact_mod, properties, film_state] = thermal_contact_trial( ...
    contact_base, operating_state.omega_rad_s, Q_N, T_final_C, cfg);
raw_result = operating_state.evaluate_raw_contact(contact_mod);
local_contact_evaluations = local_contact_evaluations + 1;
friction = thermal_friction_power(contact_base.brg, operating_state.omega_rad_s, ...
    raw_result, properties.eta_Pa_s, contact_mod.film_thickness, contact_base.local);
Q_cool_W = H_W_per_K * (T_final_C - T_oil_C);
T_target_final_C = T_oil_C + friction.Q_fric_W / H_W_per_K;

thermal_state = struct('enabled', true, 'T_oil_C', T_oil_C, ...
    'T_final_C', T_final_C, 'Q_fric_W', friction.Q_fric_W, ...
    'Q_cool_W', Q_cool_W, ...
    'thermal_residual_C', abs(T_target_final_C - T_final_C), ...
    'iterations', iterations, 'converged', converged, ...
    'viscosity_Pa_s', properties.eta_Pa_s, ...
    'pressure_viscosity_Pa_inv', properties.alpha_p_Pa_inv, ...
    'working_clearance_m', contact_mod.working_clearance, ...
    'film_thickness_min_m', film_state.h_min_m, ...
    'loaded_count', raw_result.loaded_count, ...
    'max_contact_load', max(raw_result.Q, [], 'all'), ...
    'local_contact_evaluations', local_contact_evaluations);
end

function [contact_trial, properties, film_state] = thermal_contact_trial(contact_base, omega_rad_s, Q_N, temperature_C, cfg)
properties = thermal_lubricant_properties(temperature_C);
[working_clearance_m, ~] = thermal_working_clearance(contact_base.brg, ...
    temperature_C, cfg.oil_temperature_C, cfg.clearance_temperature_coefficient_m_per_C);
[film_thickness_m, film_state] = thermal_film_thickness(contact_base.brg, ...
    omega_rad_s, Q_N, properties.eta_Pa_s, properties.alpha_p_Pa_inv);
contact_trial = contact_base;
contact_trial.viscosity = properties.eta_Pa_s;
contact_trial.pressure_viscosity = properties.alpha_p_Pa_inv;
contact_trial.working_clearance = working_clearance_m;
contact_trial.film_thickness = film_thickness_m;
end
