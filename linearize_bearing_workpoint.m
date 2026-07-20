function stiffness = linearize_bearing_workpoint(q0, params_frozen, bearing_index, cfg)
%LINEARIZE_BEARING_WORKPOINT Local 5-DOF workpoint stiffness at frozen temperature.

if nargin < 4 || isempty(cfg), cfg = struct(); end
cfg = linearization_config(cfg);
validateattributes(q0, {'numeric'}, {'column','real','finite'});
validateattributes(bearing_index, {'numeric'}, {'scalar','integer','positive'});
if ~isfield(params_frozen, 'bearing') || bearing_index > numel(params_frozen.bearing)
    error('WorkpointStiffness:BearingIndex', 'bearing_index must select an entry in params_frozen.bearing.');
end
if ~isfield(params_frozen, 'modelInfo') || ~isfield(params_frozen.modelInfo, 'num_rotor_dof')
    error('WorkpointStiffness:ModelInfo', 'params_frozen.modelInfo from the frozen workpoint is required.');
end
if ~isfield(params_frozen, 'microphysics') || ~isfield(params_frozen.microphysics, 'thermal') || ...
        ~params_frozen.microphysics.thermal.enabled || ~strcmp(params_frozen.microphysics.thermal.mode, 'frozen_external')
    error('WorkpointStiffness:FrozenThermalState', 'linearization requires an enabled frozen_external thermal state.');
end

params_frozen.current_time = 0;
qd0 = zeros(size(q0));
bearing = params_frozen.bearing(bearing_index);
[B, ir, ic] = local_assembly_map(bearing, params_frozen.modelInfo, numel(q0));
[Fb0, ~] = F_bearing(q0, qd0, params_frozen, numel(q0));
f0 = Fb0(ir);
action_reaction_error = norm(Fb0(ir) + Fb0(ic));
force_evaluation_count = 1;
if any(~isfinite(f0)), error('WorkpointStiffness:NonFiniteForce', 'Workpoint bearing force must be finite.'); end

h_trans = cfg.translation_step_m;
lever = rotational_lever(bearing);
h_rot = min(max(h_trans/max(lever, 1e-3), 1e-8), 1e-4);
steps = [repmat(h_trans, 3, 1); repmat(h_rot, 2, 1)];
[Jh, action_h, count_h] = central_force_jacobian(q0, qd0, params_frozen, B, ir, ic, steps);
[Jhalf, action_half, count_half] = central_force_jacobian(q0, qd0, params_frozen, B, ir, ic, steps/2);
force_evaluation_count = force_evaluation_count + count_h + count_half;
action_reaction_error = max([action_reaction_error, action_h, action_half]);
Kh = -Jh; Khalf = -Jhalf;
step_consistency_error = relative_matrix_change(Khalf, Kh);
K_raw = Khalf;
if step_consistency_error > cfg.step_target
    [Jdouble, action_double, count_double] = central_force_jacobian(q0, qd0, params_frozen, B, ir, ic, 2*steps);
    force_evaluation_count = force_evaluation_count + count_double;
    action_reaction_error = max(action_reaction_error, action_double);
    Kdouble = -Jdouble;
    double_step_error = relative_matrix_change(Kh, Kdouble);
    if double_step_error < step_consistency_error
        K_raw = Kh;
        step_consistency_error = double_step_error;
    end
end

% The front ball bearing uz coordinate is fixed by the Stage4 KKT relation.
% Its tangent is retained in K_raw_full for audit but excluded from the
% admissible mechanical subspace; the rear floating roller remains full 5-DOF.
[active_dof_indices, axial_constraint_handled_by_kkt] = admissible_dofs(bearing);
active_dof_mask = false(5,1); active_dof_mask(active_dof_indices) = true;
S = zeros(5, numel(active_dof_indices)); S(active_dof_indices,:) = eye(numel(active_dof_indices));
K_raw_full = K_raw;
K_active_raw = S'*K_raw_full*S;
K_active_use = 0.5*(K_active_raw+K_active_raw');
K_active_embedded_raw = zeros(5,5); K_active_embedded_raw(active_dof_indices, active_dof_indices) = K_active_raw;
K_active_embedded_use = zeros(5,5); K_active_embedded_use(active_dof_indices, active_dof_indices) = K_active_use;
asymmetry_full = norm(K_raw_full-K_raw_full', 'fro')/max(norm(K_raw_full, 'fro'), 1);
asymmetry_admissible = norm(K_active_raw-K_active_raw', 'fro')/max(norm(K_active_raw, 'fro'), 1);
axial_skew_share = axial_skew_fraction(K_raw_full);
eigenvalues_symmetric_part = eig(K_active_use);
minimum_eigenvalue = min(eigenvalues_symmetric_part);
maximum_eigenvalue = max(eigenvalues_symmetric_part);
symmetry_pass = asymmetry_admissible <= cfg.asymmetry_tolerance;
if axial_constraint_handled_by_kkt
    axial_constraint_pass = axial_skew_share >= 0.95;
    K_local = K_active_embedded_use;
else
    axial_constraint_pass = true;
    K_local = K_active_embedded_use;
end
restoring_work_pass = symmetry_pass && axial_constraint_pass && restoring_work_check(K_active_use, minimum_eigenvalue, maximum_eigenvalue);
step_pass = step_consistency_error <= cfg.step_target;
action_reaction_pass = action_reaction_error <= cfg.action_reaction_tolerance;
pass = step_pass && symmetry_pass && restoring_work_pass && action_reaction_pass && all(isfinite(K_local), 'all');

stiffness = struct('bearing_index', bearing_index, 'bearing_type', bearing.type, ...
    'local_dof_order', {{'ux','uy','uz','theta_x','theta_y'}}, ...
    'force_at_workpoint', f0, 'J_raw', -K_raw_full, 'K_raw', K_raw_full, ...
    'active_dof_indices', active_dof_indices, 'active_dof_mask', active_dof_mask, ...
    'K_raw_full', K_raw_full, 'K_active_embedded_raw', K_active_embedded_raw, ...
    'K_active_embedded_use', K_active_embedded_use, 'K_local', K_local, ...
    'translation_step_m', h_trans, 'rotation_step_rad', h_rot, ...
    'step_consistency_error', step_consistency_error, 'asymmetry_ratio', asymmetry_full, ...
    'asymmetry_full', asymmetry_full, 'asymmetry_admissible', asymmetry_admissible, ...
    'axial_skew_share', axial_skew_share, 'axial_constraint_handled_by_kkt', axial_constraint_handled_by_kkt, ...
    'eigenvalues_symmetric_part', eigenvalues_symmetric_part, 'minimum_eigenvalue', minimum_eigenvalue, ...
    'maximum_eigenvalue', maximum_eigenvalue, 'restoring_work_pass', restoring_work_pass, ...
    'action_reaction_error', action_reaction_error, 'force_evaluation_count', force_evaluation_count, 'pass', pass);
end

function cfg = linearization_config(cfg)
defaults = struct('translation_step_m', 1e-8, 'step_target', 1e-2, ...
    'step_failure_limit', 5e-2, 'asymmetry_tolerance', 1e-3, 'action_reaction_tolerance', 1e-10);
fields = fieldnames(defaults);
for k = 1:numel(fields)
    name = fields{k};
    if ~isfield(cfg, name) || isempty(cfg.(name)), cfg.(name) = defaults.(name); end
    validateattributes(cfg.(name), {'numeric'}, {'scalar','real','positive','finite'});
end
if cfg.step_failure_limit < cfg.step_target
    error('WorkpointStiffness:StepTolerance', 'step_failure_limit must be no smaller than step_target.');
end
end

function [B, ir, ic] = local_assembly_map(bearing, modelInfo, n)
ir = 6*bearing.rotor_node + (-5:-1);
ic = modelInfo.num_rotor_dof + 6*bearing.case_node + (-5:-1);
if any(ir < 1 | ir > n) || any(ic < 1 | ic > n), error('WorkpointStiffness:LocalMap', 'Bearing local DOFs are outside q0.'); end
B = sparse([1:5 1:5], [ir ic], [ones(1,5) -ones(1,5)], 5, n);
end

function lever = rotational_lever(bearing)
switch lower(bearing.type)
    case 'ball'
        lever = bearing.stage4A_width_m/2;
    case 'roller'
        lever = bearing.L/2;
    otherwise
        error('WorkpointStiffness:BearingType', 'Unsupported bearing type: %s.', bearing.type);
end
end

function [indices, handled_by_kkt] = admissible_dofs(bearing)
if strcmpi(bearing.type, 'ball')
    indices = [1 2 4 5]; handled_by_kkt = true;
else
    indices = 1:5; handled_by_kkt = false;
end
end

function share = axial_skew_fraction(K)
A = 0.5*(K-K');
mask = false(5,5); mask(3,:) = true; mask(:,3) = true;
share = norm(A(mask))/max(norm(A, 'fro'), eps);
end

function [J, action_error, force_count] = central_force_jacobian(q0, qd0, params_frozen, B, ir, ic, steps)
J = zeros(5,5); action_error = 0; force_count = 0;
for j = 1:5
    direction = 0.5*B'*unit_vector(5, j);
    dq = direction*steps(j);
    expected_local_change = unit_vector(5, j)*steps(j);
    if norm(B*dq-expected_local_change) > 100*eps*max(1, steps(j))
        error('WorkpointStiffness:RelativeMap', 'Local perturbation does not satisfy B*dq = e_j*h_j.');
    end
    [Fp, ~] = F_bearing(q0+dq, qd0, params_frozen, numel(q0));
    [Fm, ~] = F_bearing(q0-dq, qd0, params_frozen, numel(q0));
    if any(~isfinite(Fp(ir))) || any(~isfinite(Fm(ir)))
        error('WorkpointStiffness:NonFiniteDifference', 'Finite-difference bearing force must remain finite.');
    end
    action_error = max([action_error, norm(Fp(ir)+Fp(ic)), norm(Fm(ir)+Fm(ic))]);
    J(:,j) = (Fp(ir)-Fm(ir))/(2*steps(j));
    force_count = force_count + 2;
end
end

function pass = restoring_work_check(K, lambda_min, lambda_max)
tolerance = 1e-8*max(1, lambda_max);
if lambda_min < -tolerance, pass = false; return; end
n = size(K,1); directions = eye(n);
if n >= 2, directions = [directions, ([1;1;zeros(n-2,1)])/sqrt(2)]; end
if n >= 3, directions = [directions, ([zeros(n-2,1);1;1])/sqrt(2)]; end
work = diag(directions'*K*directions);
pass = all(work >= -1e-8*max(1, norm(K,2)*sum(directions.^2,1)).');
end

function value = relative_matrix_change(current, previous)
value = norm(current-previous, 'fro')/max(norm(current, 'fro'), 1);
end

function v = unit_vector(n, index)
v = zeros(n,1); v(index) = 1;
end
