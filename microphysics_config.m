function cfg = microphysics_config(overrides)
%MICROPHYSICS_CONFIG Default local microphysics switches and settings.

cfg.thermal = struct('enabled', false, 'mode', 'frozen_external');
cfg.thermal_state = empty_thermal_state();
cfg.roughness = struct('enabled', false);
cfg.impurity = struct('enabled', false);
if nargin < 1 || isempty(overrides)
    return;
end
names = {'thermal', 'roughness', 'impurity'};
for k = 1:numel(names)
    name = names{k};
    if isfield(overrides, name) && isstruct(overrides.(name)) && ...
            isfield(overrides.(name), 'enabled')
        cfg.(name).enabled = logical(overrides.(name).enabled);
    end
end
if isfield(overrides, 'thermal') && isstruct(overrides.thermal) && isfield(overrides.thermal, 'mode')
    cfg.thermal.mode = validatestring(overrides.thermal.mode, {'frozen_external'});
end
if isfield(overrides, 'thermal_state')
    cfg.thermal_state = validate_thermal_state(overrides.thermal_state);
elseif isfield(overrides, 'thermal') && isstruct(overrides.thermal) && isfield(overrides.thermal, 'thermal_state')
    cfg.thermal_state = validate_thermal_state(overrides.thermal.thermal_state);
end
end

function thermal_state = empty_thermal_state()
fields = {'T_oil','T_final','T_film','T_inner','T_outer','T_element', ...
    'eta','alpha_p','working_clearance','film_thickness','thermal_preload'};
template = cell2struct(num2cell(nan(size(fields))), fields, 2);
thermal_state = struct('ball', template, 'roller', template);
end

function thermal_state = validate_thermal_state(thermal_state)
if ~isstruct(thermal_state) || ~all(isfield(thermal_state, {'ball','roller'}))
    error('thermal_state must contain fixed ball and roller records.');
end
required = fieldnames(empty_thermal_state().ball);
for k = 1:numel({'ball','roller'})
    name = {'ball','roller'}; name = name{k};
    if ~isstruct(thermal_state.(name)) || ~all(isfield(thermal_state.(name), required))
        error('thermal_state.%s is missing a required frozen-external field.', name);
    end
end
end
