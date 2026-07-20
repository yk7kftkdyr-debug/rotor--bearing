function [contact_mod, thermal_state] = apply_thermal_microphysics(contact_mod, operating_state, ~, module_cfg)
%APPLY_THERMAL_MICROPHYSICS Apply one externally supplied, frozen thermal state.
if ~module_cfg.enabled
    thermal_state = struct('enabled', false, 'mode', 'transparent', 'input', []);
    return;
end
if ~strcmp(module_cfg.mode, 'frozen_external')
    error('Unsupported thermal mode: %s.', module_cfg.mode);
end
if ~isfield(operating_state, 'bearing_index') || ~ismember(operating_state.bearing_index, [1 2])
    error('Frozen external thermal state requires bearing_index 1 (ball) or 2 (roller).');
end
if ~isfield(module_cfg, 'thermal_state')
    error('Frozen external thermal mode requires thermal_state input.');
end
if operating_state.bearing_index == 1, key = 'ball'; else, key = 'roller'; end
input_state = module_cfg.thermal_state.(key);
required = {'T_oil','T_final','T_film','T_inner','T_outer','T_element', ...
    'eta','alpha_p','working_clearance','film_thickness','thermal_preload'};
if ~all(isfield(input_state, required)) || ~isfinite(input_state.T_oil) || ~isfinite(input_state.T_final)
    error('Frozen thermal_state.%s must define all fields and finite T_oil/T_final.', key);
end
temperature = thermal_component_temperatures(input_state.T_oil, input_state.T_final);
property = thermal_lubricant_properties(temperature.T_film_C);
geometry = thermal_geometry(contact_mod.brg, operating_state);
clearance = thermal_working_clearance_differential(geometry.bearing_type, temperature.T_inner_C, temperature.T_outer_C, temperature.T_element_C, contact_mod.working_clearance, geometry.clearance);
resolved_state = input_state;
resolved_state.T_film = temperature.T_film_C; resolved_state.T_inner = temperature.T_inner_C; resolved_state.T_outer = temperature.T_outer_C; resolved_state.T_element = temperature.T_element_C;
resolved_state.eta = property.eta_Pa_s; resolved_state.alpha_p = property.alpha_p_Pa_inv; resolved_state.working_clearance = clearance.c_work_m; resolved_state.thermal_preload = clearance.thermal_preload;
contact_mod.viscosity = resolved_state.eta;
contact_mod.pressure_viscosity = resolved_state.alpha_p;
contact_mod.working_clearance = resolved_state.working_clearance;
contact_mod.film_thickness = resolved_state.film_thickness;
thermal_state = struct('enabled', true, 'mode', 'frozen_external', 'input', resolved_state, 'contact_geometry', geometry.film);
end

function geometry = thermal_geometry(brg, operating_state)
if ~isfield(operating_state, 'E_star') || ~isfinite(operating_state.E_star) || operating_state.E_star<=0, error('Frozen thermal mapping requires finite E_star.'); end
if ~isfield(operating_state, 'omega') || ~isfinite(operating_state.omega), error('Frozen thermal mapping requires finite omega.'); end
geometry.clearance = struct('Dm', brg.Dm, 'D_element', rolling_element_diameter(brg));
geometry.film = struct('bearing_type', lower(brg.type), 'R_effective', rolling_element_diameter(brg)/2, 'contact_length', contact_length(brg), 'E_star', operating_state.E_star, 'entrainment_velocity', max(abs(operating_state.omega)*brg.Dm/2, 1e-9), 'reference_viscosity', brg.oil_viscosity, 'film_exponent', film_exponent(brg), 'load_exponent', -0.067);
if strcmpi(brg.type, 'ball'), geometry.clearance.alpha0 = brg.assembly.contact_angle0; end
geometry.bearing_type = lower(brg.type);
end

function value = rolling_element_diameter(brg)
if strcmpi(brg.type, 'ball'), value = brg.Db; else, value = brg.Dw; end
end

function value = contact_length(brg)
if strcmpi(brg.type, 'ball'), value = brg.Db; else, value = brg.L; end
end

function value = film_exponent(brg)
if strcmpi(brg.type, 'ball'), value = 0.68; else, value = 0.70; end
end
