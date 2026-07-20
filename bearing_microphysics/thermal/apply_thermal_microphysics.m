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
if ~all(isfield(input_state, required)) || ~all(isfinite(cell2mat(struct2cell(input_state))))
    error('Frozen thermal_state.%s must define finite values for every fixed field.', key);
end
contact_mod.viscosity = input_state.eta;
contact_mod.pressure_viscosity = input_state.alpha_p;
contact_mod.working_clearance = input_state.working_clearance;
contact_mod.film_thickness = input_state.film_thickness;
thermal_state = struct('enabled', true, 'mode', 'frozen_external', 'input', input_state);
end
