function validation = validate_workpoint_linearization(q0, params_frozen, stiffness_results, cfg)
%VALIDATE_WORKPOINT_LINEARIZATION Check frozen-workpoint Kb for node-16 ux perturbations.

if nargin < 4 || isempty(cfg), cfg = struct(); end
cfg = stage7_config(cfg);
validateattributes(q0, {'numeric'}, {'column','real','finite'});
validateattributes(stiffness_results, {'struct'}, {'numel',2});
validate_frozen_inputs(q0, params_frozen, stiffness_results, cfg);

q0_before = q0;
frozen_before = frozen_state(params_frozen);
[~, ~, K_structure, modelInfo] = build_rotor_case_model(params_frozen);
if size(K_structure,1) ~= numel(q0)
    error('Stage7:DimensionMismatch', 'q0 and the rebuilt structural stiffness matrix must have equal dimensions.');
end
[K_total, maps] = workpoint_total_stiffness(K_structure, params_frozen, stiffness_results, modelInfo, numel(q0));
[transverse_dofs, selected_index] = transverse_subspace(modelInfo, cfg.rotor_node);
K_t = K_total(transverse_dofs, transverse_dofs);

validation = empty_validation(cfg, frozen_before.ball.T_oil);
for candidate_index = 1:numel(cfg.amplitude_candidates_m)
    amplitude = cfg.amplitude_candidates_m(candidate_index);
    [dq_global, node_error] = coordinated_transverse_shape(K_t, transverse_dofs, selected_index, amplitude, numel(q0));
    candidate = evaluate_amplitude(q0, dq_global, params_frozen, stiffness_results, maps, cfg);
    candidate.used_amplitude_m = amplitude;
    candidate.node_displacement_error_m = node_error;
    candidate.fallback_used = candidate_index > 1;
    if candidate.pass
        validation = candidate;
        break;
    end
    validation = candidate;
end

validation.q0_unchanged = isequaln(q0, q0_before);
validation.frozen_state_unchanged = isequaln(frozen_state(params_frozen), frozen_before);
validation.pass = validation.pass && validation.q0_unchanged && validation.frozen_state_unchanged;
end

function cfg = stage7_config(cfg)
defaults = struct('rotor_node',16,'direction','ux','amplitude_candidates_m',[1e-6 0.2e-6], ...
    'force_error_tolerance',0.05,'edge_load_ratio',0.01,'equivalent_force_reference_N',1.0);
names = fieldnames(defaults);
for k = 1:numel(names)
    name = names{k};
    if ~isfield(cfg,name) || isempty(cfg.(name)), cfg.(name) = defaults.(name); end
    if ~isequaln(cfg.(name), defaults.(name))
        error('Stage7:FixedConfiguration', 'Stage7 requires the specified fixed value for cfg.%s.', name);
    end
end
end

function validate_frozen_inputs(q0, params, stiffness_results, cfg)
if ~isfield(params,'bearing') || numel(params.bearing) ~= 2
    error('Stage7:Bearings', 'Stage7 requires exactly the front ball and rear roller bearings.');
end
if ~isfield(params,'modelInfo') || ~isfield(params.modelInfo,'num_rotor_dof')
    error('Stage7:ModelInfo', 'params_frozen.modelInfo is required.');
end
if ~isfield(params,'microphysics') || ~isfield(params.microphysics,'thermal') || ...
        ~params.microphysics.thermal.enabled || ~strcmp(params.microphysics.thermal.mode,'frozen_external')
    error('Stage7:FrozenThermalState', 'Stage7 requires enabled frozen_external thermal inputs.');
end
if numel(q0) ~= params.modelInfo.num_rotor_dof + params.modelInfo.num_case_dof
    error('Stage7:q0Length', 'q0 length must match the complete 192-DOF model.');
end
if cfg.rotor_node > params.modelInfo.num_rotor_nodes
    error('Stage7:Node', 'cfg.rotor_node must identify an existing rotor node.');
end
for ib = 1:2
    if ~stiffness_results(ib).pass || ~isequal(size(stiffness_results(ib).K_local),[5 5]) || ...
            any(~isfinite(stiffness_results(ib).K_local),'all')
        error('Stage7:StiffnessGate', 'Each Stage6A workpoint stiffness must pass and be finite 5-by-5.');
    end
end
if ~strcmpi(params.bearing(1).type,'ball') || ~strcmpi(params.bearing(2).type,'roller')
    error('Stage7:BearingOrder', 'Stage7 requires ball then roller stiffness results.');
end
frozen_state(params);
end

function [K_total, maps] = workpoint_total_stiffness(K_structure, params, stiffness_results, modelInfo, n)
K_total = K_structure;
map_template = struct('B',sparse(5,n),'ir',zeros(5,1),'ic',zeros(5,1));
maps = repmat(map_template,1,2);
for ib = 1:2
    [B, ir, ic] = bearing_local_map(params.bearing(ib), modelInfo, n);
    K_total = K_total + B'*stiffness_results(ib).K_local*B;
    maps(ib) = struct('B',B,'ir',ir,'ic',ic);
end
end

function [B, ir, ic] = bearing_local_map(bearing, modelInfo, n)
ir = (6*bearing.rotor_node + (-5:-1)).';
ic = (modelInfo.num_rotor_dof + 6*bearing.case_node + (-5:-1)).';
if any(ir < 1 | ir > n) || any(ic < 1 | ic > n)
    error('Stage7:LocalMap', 'Bearing local degrees of freedom are outside the global state.');
end
B = sparse([1:5 1:5], [ir.' ic.'], [ones(1,5) -ones(1,5)], 5, n);
end

function [dofs, selected_index] = transverse_subspace(modelInfo, rotor_node)
rotor_nodes = 1:modelInfo.num_rotor_nodes;
case_nodes = 1:modelInfo.num_case_nodes;
rotor_dofs = reshape(6*rotor_nodes + [-5;-4;-2;-1],[],1);
case_dofs = modelInfo.num_rotor_dof + reshape(6*case_nodes + [-5;-4;-2;-1],[],1);
dofs = [rotor_dofs; case_dofs];
selected_global = 6*rotor_node - 5;
selected_index = find(dofs == selected_global,1);
if isempty(selected_index), error('Stage7:SelectedDof', 'Rotor node ux is absent from the transverse subspace.'); end
end

function [dq_global, node_error] = coordinated_transverse_shape(K_t, transverse_dofs, selected_index, amplitude, n)
e = sparse(selected_index,1,1,size(K_t,1),1);
A = [sparse(K_t), e; e.', sparse(1,1)];
rhs = [zeros(size(K_t,1),1); amplitude];
solution = A\rhs;
backward_error = norm(A*solution-rhs)/max(norm(A,inf)*norm(solution)+norm(rhs),1);
if any(~isfinite(solution)) || backward_error > 1e-10
    lambda_min = min(eig(full(0.5*(K_t+K_t.'))));
    error('Stage7:SingularTransverseKKT', 'Transverse KKT solve failed: rank=%d/%d, minEig=%g, backwardError=%g.', ...
        rank(full(A)), size(A,1), lambda_min, backward_error);
end
dq_t = solution(1:end-1);
node_error = abs(e.'*dq_t-amplitude);
if node_error > 1e-12
    error('Stage7:NodeConstraint', 'Node-16 transverse displacement constraint error is %.3e m.', node_error);
end
dq_global = zeros(n,1); dq_global(transverse_dofs) = dq_t;
end

function validation = evaluate_amplitude(q0, dq_global, params, stiffness_results, maps, cfg)
thermal = frozen_state(params);
validation = empty_validation(cfg, thermal.ball.T_oil);
validation.shape_method = 'minimum-strain-energy transverse KKT constrained shape';
validation.transverse_dof_count = nnz(dq_global ~= 0); % overwritten below with fixed subspace count
validation.transverse_dof_count = 4*(params.modelInfo.num_rotor_nodes + params.modelInfo.num_case_nodes);
params.current_time = 0;
qd0 = zeros(size(q0));
[F0, state0] = F_bearing(q0, qd0, params, numel(q0));
[Fplus, stateplus] = F_bearing(q0+dq_global, qd0, params, numel(q0));
[Fminus, stateminus] = F_bearing(q0-dq_global, qd0, params, numel(q0));
for ib = 1:2
    active = stiffness_results(ib).active_dof_indices(:);
    dr_plus = full(maps(ib).B*dq_global); dr_minus = -dr_plus;
    nonlinear_plus = Fplus(maps(ib).ir)-F0(maps(ib).ir);
    nonlinear_minus = Fminus(maps(ib).ir)-F0(maps(ib).ir);
    linear_plus = -stiffness_results(ib).K_local*dr_plus;
    linear_minus = -stiffness_results(ib).K_local*dr_minus;
    lever = characteristic_lever(params.bearing(ib));
    scale_full = [1;1;1;1/lever;1/lever];
    error_plus = scaled_force_error(nonlinear_plus(active), linear_plus(active), scale_full(active), cfg.equivalent_force_reference_N);
    error_minus = scaled_force_error(nonlinear_minus(active), linear_minus(active), scale_full(active), cfg.equivalent_force_reference_N);
    base_set = major_loaded_set(state0.bearings(ib), params.bearing(ib), cfg.edge_load_ratio);
    plus_set = major_loaded_set(stateplus.bearings(ib), params.bearing(ib), cfg.edge_load_ratio);
    minus_set = major_loaded_set(stateminus.bearings(ib), params.bearing(ib), cfg.edge_load_ratio);
    stable = isequal(plus_set,base_set) && isequal(minus_set,base_set);
    validation.bearing(ib) = struct('bearing_index',ib,'bearing_type',lower(params.bearing(ib).type), ...
        'active_dof_indices',active,'characteristic_lever_m',lever, ...
        'local_increment_plus',dr_plus,'local_increment_minus',dr_minus, ...
        'nonlinear_force_increment_plus',nonlinear_plus,'nonlinear_force_increment_minus',nonlinear_minus, ...
        'linear_force_prediction_plus',linear_plus,'linear_force_prediction_minus',linear_minus, ...
        'force_error_plus',error_plus,'force_error_minus',error_minus, ...
        'force_error_max',max(error_plus,error_minus),'major_loaded_set_baseline',base_set, ...
        'major_loaded_set_plus',plus_set,'major_loaded_set_minus',minus_set, ...
        'major_loaded_set_stable',stable,'edge_load_ratio',cfg.edge_load_ratio, ...
        'pass',max(error_plus,error_minus) <= cfg.force_error_tolerance && stable);
end
validation.maximum_force_error = max([validation.bearing.force_error_max]);
validation.all_major_loaded_sets_stable = all([validation.bearing.major_loaded_set_stable]);
validation.pass = validation.maximum_force_error <= cfg.force_error_tolerance && validation.all_major_loaded_sets_stable;
end

function validation = empty_validation(cfg, temperature_C)
bearing_template = struct('bearing_index',0,'bearing_type','','active_dof_indices',zeros(0,1), ...
    'characteristic_lever_m',NaN,'local_increment_plus',zeros(5,1),'local_increment_minus',zeros(5,1), ...
    'nonlinear_force_increment_plus',zeros(5,1),'nonlinear_force_increment_minus',zeros(5,1), ...
    'linear_force_prediction_plus',zeros(5,1),'linear_force_prediction_minus',zeros(5,1), ...
    'force_error_plus',Inf,'force_error_minus',Inf,'force_error_max',Inf, ...
    'major_loaded_set_baseline',false(0,1),'major_loaded_set_plus',false(0,1), ...
    'major_loaded_set_minus',false(0,1),'major_loaded_set_stable',false, ...
    'edge_load_ratio',cfg.edge_load_ratio,'pass',false);
validation = struct('temperature_C',temperature_C,'rotor_node',cfg.rotor_node,'direction',cfg.direction, ...
    'requested_amplitude_m',cfg.amplitude_candidates_m(1),'used_amplitude_m',NaN,'fallback_used',false, ...
    'transverse_dof_count',0,'node_displacement_error_m',Inf, ...
    'shape_method','minimum-strain-energy transverse KKT constrained shape', ...
    'bearing',repmat(bearing_template,1,2),'maximum_force_error',Inf, ...
    'all_major_loaded_sets_stable',false,'q0_unchanged',false,'frozen_state_unchanged',false,'pass',false);
end

function error_value = scaled_force_error(nonlinear_force, linear_force, scale, reference_N)
nonlinear_equivalent = scale(:).*nonlinear_force(:);
difference_equivalent = scale(:).*(nonlinear_force(:)-linear_force(:));
error_value = norm(difference_equivalent)/max(norm(nonlinear_equivalent), reference_N);
end

function major_set = major_loaded_set(state, bearing, edge_load_ratio)
if ~isfield(state,'Q') || isempty(state.Q) || any(~isfinite(state.Q),'all')
    error('Stage7:ContactLoad', 'Each contact state must provide finite Q for major-loaded-set validation.');
end
Q = state.Q;
if strcmpi(bearing.type,'ball')
    body_load = Q(:);
else
    body_load = sum(Q,2);
end
maximum = max(body_load);
if ~isfinite(maximum) || maximum <= 0
    error('Stage7:NoMajorLoad', 'Each perturbed bearing state requires a finite positive maximum contact load.');
end
major_set = body_load >= edge_load_ratio*maximum;
end

function lever = characteristic_lever(bearing)
if strcmpi(bearing.type,'ball')
    lever = bearing.stage4A_width_m/2;
else
    lever = bearing.L/2;
end
validateattributes(lever, {'numeric'}, {'scalar','real','finite','positive'});
end

function state = frozen_state(params)
if isfield(params,'microphysics') && isfield(params.microphysics,'thermal_state')
    state = params.microphysics.thermal_state;
elseif isfield(params,'microphysics') && isfield(params.microphysics,'thermal') && isfield(params.microphysics.thermal,'fixed_state')
    state = params.microphysics.thermal.fixed_state;
else
    error('Stage7:FrozenStateMissing', 'params_frozen must contain the fixed external thermal state.');
end
if ~isfield(state,'ball') || ~isfield(state,'roller') || ~isfinite(state.ball.T_oil)
    error('Stage7:FrozenStateInvalid', 'The frozen external thermal state is incomplete.');
end
end
