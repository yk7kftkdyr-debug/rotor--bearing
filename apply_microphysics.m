function [contact_mod, micro_state] = apply_microphysics(contact_base, operating_state, micro_state, cfg)
%APPLY_MICROPHYSICS Sole local gateway for optional contact corrections.

if nargin < 3 || isempty(micro_state)
    micro_state = struct();
end
if nargin < 4 || isempty(cfg)
    cfg = microphysics_config();
end
micro_state = initialize_microphysics_state(micro_state);
contact_mod = contact_base;

thermal_cfg = cfg.thermal;
thermal_cfg.thermal_state = cfg.thermal_state;
[contact_mod, micro_state.temperature] = run_module(contact_mod, operating_state, micro_state, thermal_cfg, ...
    'thermal', {'viscosity', 'pressure_viscosity', 'working_clearance', ...
    'film_thickness'});
[contact_mod, micro_state.roughness_phase] = run_module(contact_mod, operating_state, micro_state, cfg.roughness, ...
    'roughness', {'surface_height', 'effective_deformation', ...
    'asperity_contact_ratio', 'contact_stiffness', 'contact_damping'});
[contact_mod, micro_state.impurity_state] = run_module(contact_mod, operating_state, micro_state, cfg.impurity, ...
    'impurity', {'characteristic_displacement', 'effective_deformation', ...
    'contact_stiffness'});
end

function state = initialize_microphysics_state(state)
required = {'temperature', 'roughness_phase', 'impurity_state', 'history'};
for k = 1:numel(required)
    if ~isfield(state, required{k})
        state.(required{k}) = [];
    end
end
if isempty(state.history)
    state.history = struct('module', {}, 'enabled', {});
end
end

function [output, module_state] = run_module(input, operating_state, micro_state, module_cfg, name, allowed)
output = input;
module_state = struct('enabled', false, 'mode', 'transparent');
if ~module_cfg.enabled
    return;
end
root = fileparts(mfilename('fullpath'));
module_path = fullfile(root, 'bearing_microphysics', name);
addpath(module_path);
before = output;
switch name
    case 'thermal'
        [output, module_state] = apply_thermal_microphysics(output, operating_state, micro_state, module_cfg);
    case 'roughness'
        [output, module_state] = apply_roughness_microphysics(output, operating_state, micro_state, module_cfg);
    case 'impurity'
        [output, module_state] = apply_impurity_microphysics(output, operating_state, micro_state, module_cfg);
end
assert_only_allowed_contact_changes(before, output, allowed, name);
end

function assert_only_allowed_contact_changes(before, after, allowed, name)
all_names = union(fieldnames(before), fieldnames(after));
for k = 1:numel(all_names)
    field = all_names{k};
    before_value = field_value(before, field);
    after_value = field_value(after, field);
    if ~isequaln(before_value, after_value) && ~ismember(field, allowed)
        error('Microphysics module %s changed forbidden field %s.', name, field);
    end
end
end

function value = field_value(s, field)
if isfield(s, field)
    value = s.(field);
else
    value = [];
end
end
