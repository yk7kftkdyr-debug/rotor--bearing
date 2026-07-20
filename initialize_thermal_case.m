function [thermal_state, temperature_state] = initialize_thermal_case(model, bearing_config, T_oil_C, temperature_state)
%INITIALIZE_THERMAL_CASE Build the fixed Stage2 thermal-state contract.

validateattributes(model, {'struct'}, {'nonempty'});
validateattributes(bearing_config, {'struct'}, {'nonempty'});
validateattributes(T_oil_C, {'numeric'}, {'scalar','real','finite'});
ensure_thermal_function_path();
if nargin < 4 || isempty(temperature_state)
    temperature_state = struct('T_ball_C', T_oil_C, 'T_roller_C', T_oil_C);
end
required_temperature = {'T_ball_C','T_roller_C'};
for k = 1:numel(required_temperature)
    if ~isfield(temperature_state, required_temperature{k}) || ~isscalar(temperature_state.(required_temperature{k})) || ~isfinite(temperature_state.(required_temperature{k}))
        error('InitializeThermalCase:TemperatureState', 'initialize_thermal_case requires finite temperature_state.%s.', required_temperature{k});
    end
end
ball = bearing_by_type(bearing_config, 'ball');
roller = bearing_by_type(bearing_config, 'roller');
thermal_state = struct('ball', frozen_entry(ball, T_oil_C, temperature_state.T_ball_C), ...
    'roller', frozen_entry(roller, T_oil_C, temperature_state.T_roller_C));
end

function ensure_thermal_function_path()
thermal_path = fullfile(fileparts(mfilename('fullpath')), 'bearing_microphysics', 'thermal');
if exist('thermal_component_temperatures', 'file') ~= 2, addpath(thermal_path); end
end

function bearing = bearing_by_type(bearing_config, type)
index = find(arrayfun(@(entry) strcmpi(entry.type, type), bearing_config), 1);
if isempty(index), error('InitializeThermalCase:BearingType', 'initialize_thermal_case requires one %s bearing.', type); end
bearing = bearing_config(index);
end

function entry = frozen_entry(bearing, T_oil_C, T_final_C)
temperature = thermal_component_temperatures(T_oil_C, T_final_C);
property = thermal_lubricant_properties(temperature.T_film_C);
geometry = clearance_geometry(bearing);
clearance = thermal_working_clearance_differential(lower(bearing.type), ...
    temperature.T_inner_C, temperature.T_outer_C, temperature.T_element_C, ...
    bearing.assembly.radial_clearance, geometry);
entry = struct('T_oil', T_oil_C, 'T_final', T_final_C, ...
    'T_film', temperature.T_film_C, 'T_inner', temperature.T_inner_C, ...
    'T_outer', temperature.T_outer_C, 'T_element', temperature.T_element_C, ...
    'eta', property.eta_Pa_s, 'alpha_p', property.alpha_p_Pa_inv, ...
    'working_clearance', clearance.c_work_m, 'film_thickness', NaN, ...
    'thermal_preload', clearance.thermal_preload);
end

function geometry = clearance_geometry(bearing)
geometry = struct('Dm', bearing.Dm, 'alpha_i', 11e-6, 'alpha_o', 11e-6, 'alpha_e', 11e-6);
switch lower(bearing.type)
    case 'ball'
        geometry.D_element = bearing.Db;
        geometry.alpha0 = bearing.assembly.contact_angle0;
    case 'roller'
        geometry.D_element = bearing.Dw;
    otherwise
        error('InitializeThermalCase:BearingType', 'Unsupported bearing type: %s.', bearing.type);
end
end
