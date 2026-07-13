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
expected_preload = [cfg.preload.bearing1_z_N cfg.preload.bearing2_z_N];
actual_preload = [params.bearing(1).preload_z params.bearing(2).preload_z];
assert(all(abs(actual_preload - expected_preload) <= tol), ...
    '%s: official baseline preload must be bearing1 %.0f N and bearing2 %.0f N.', ...
    context, expected_preload(1), expected_preload(2));

if isfield(params, 'static_load') && ~isempty(params.static_load)
    assert(all(abs(params.static_load.Fx(:)) <= tol) && all(abs(params.static_load.Fy(:)) <= tol), ...
        '%s: official baseline must not apply rotor-node static_load.', context);
end

assert(isfield(params, 'uploaded_model_dir') && isfolder(params.uploaded_model_dir), ...
    '%s: official baseline requires the local uploaded Newmark model directory.', context);
end
