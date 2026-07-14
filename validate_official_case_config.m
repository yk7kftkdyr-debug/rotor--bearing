function validate_official_case_config(params, context)
%VALIDATE_OFFICIAL_CASE_CONFIG Stop official runs with mixed parameters.
% This is a guardrail only; it does not change the simulation model.

if nargin < 2 || isempty(context)
    context = 'official run';
end

cfg = official_simulation_config();
layout = cfg.layout;

assert(strcmp(params.bearing(1).type, layout.bearing1.type), ...
    '%s: bearing1 must be the front ball bearing.', context);
assert(strcmp(params.bearing(2).type, layout.bearing2.type), ...
    '%s: bearing2 must be the rear roller bearing.', context);
assert(params.bearing(1).rotor_node == layout.bearing1.rotor_node && ...
    params.bearing(1).case_node == layout.bearing1.case_node, ...
    '%s: bearing1 rotor/case nodes are inconsistent with the official layout.', context);
assert(params.bearing(2).rotor_node == layout.bearing2.rotor_node && ...
    params.bearing(2).case_node == layout.bearing2.case_node, ...
    '%s: bearing2 rotor/case nodes are inconsistent with the official layout.', context);

tol = 1e-9;
actual_preload = [params.bearing(1).preload_z params.bearing(2).preload_z];
assert(all(abs(actual_preload) <= tol), '%s: prescribed bearing loads are disabled.', context);

if isfield(params, 'static_load') && ~isempty(params.static_load)
    if ~isfield(params.static_load, 'Fz'), params.static_load.Fz = zeros(size(params.static_load.Fx)); end
    assert(numel(params.static_load.nodes) == numel(params.static_load.Fx) && ...
        numel(params.static_load.nodes) == numel(params.static_load.Fy) && ...
        numel(params.static_load.nodes) == numel(params.static_load.Fz), ...
        '%s: each physical steady-load node requires one Fx, Fy and Fz value.', context);
end

if isfield(params, 'require_uploaded_rotor_model') && params.require_uploaded_rotor_model
    assert(isfield(params, 'uploaded_model_dir') && isfolder(params.uploaded_model_dir), ...
        '%s: uploaded Newmark model directory is required but unavailable.', context);
end
end
