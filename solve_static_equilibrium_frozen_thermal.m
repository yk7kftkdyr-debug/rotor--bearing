function static_result = solve_static_equilibrium_frozen_thermal(model, bearing_config, thermal_state, initial_q)
%SOLVE_STATIC_EQUILIBRIUM_FROZEN_THERMAL Static work point at a fixed thermal state.
% The formal residual is K_s*q - F_gravity - F_steady - F_bearing(q) + C'*lambda.
% C contains the front locating relation and a numerical axial gauge only.

if nargin < 4 || isempty(initial_q), initial_q = []; end
params = model;
validateattributes(bearing_config, {'struct'}, {'nonempty'});
params.bearing = bearing_config;
params.microphysics = microphysics_config(struct('thermal', ...
    struct('enabled', true, 'mode', 'frozen_external'), 'thermal_state', thermal_state));
[MM, ~, K_s, modelInfo] = build_rotor_case_model(params);
params.modelInfo = modelInfo;
params.num_rotor_dof = modelInfo.num_rotor_dof;
params.num_case_dof = modelInfo.num_case_dof;
cfg = frozen_static_config(params);
F_gravity = gravity_load(MM, modelInfo, cfg);
F_static = F_gravity + steady_load(params, size(MM, 1));
C = axial_constraints(params, modelInfo, size(MM, 1));
use_initial_state = ~isempty(initial_q);
if use_initial_state, q = initial_q(:); else, q = structural_initial_seed(params, F_static, size(MM, 1), modelInfo, cfg); end
if numel(q) ~= size(MM, 1) || any(~isfinite(q)), error('FrozenThermalStatic:InitialState', 'Initial q must be a finite %d-vector.', size(MM, 1)); end
lambda = zeros(size(C, 1), 1);
if use_initial_state, load_factor = 1; load_increment = 0; else, load_factor = 0; load_increment = 0.25; end
total_iterations = 0;
if use_initial_state
    [q, lambda, info] = newton_kkt(q, lambda, K_s, F_static, params, C, cfg, thermal_state);
    total_iterations = info.iterations;
    if ~info.converged
        error('FrozenThermalStatic:NewtonFailure', ['Frozen thermal static Newton failed from supplied initial_q after %d iterations; ', ...
            'normalized residual %.3e; ball T_final=%.6g C; roller T_final=%.6g C.'], ...
            info.iterations, info.normalized_residual, thermal_state.ball.T_final, thermal_state.roller.T_final);
    end
end
while load_factor < 1
    trial_factor = min(load_factor + load_increment, 1);
    [trial_q, trial_lambda, info] = newton_kkt(q, lambda, K_s, trial_factor*F_static, params, C, cfg, thermal_state);
    total_iterations = total_iterations + info.iterations;
    if info.converged
        q = trial_q; lambda = trial_lambda; load_factor = trial_factor;
        load_increment = min(2*load_increment, 1-load_factor);
    else
        load_increment = load_increment/2;
        if load_increment < 1/32
            error('FrozenThermalStatic:NewtonFailure', ['Frozen thermal static Newton failed at load factor %.6f after %d iterations; ', ...
                'normalized residual %.3e; ball T_final=%.6g C; roller T_final=%.6g C.'], ...
                trial_factor, info.iterations, info.normalized_residual, thermal_state.ball.T_final, thermal_state.roller.T_final);
        end
    end
end
[R_unconstrained, Fb, contact_state] = mechanical_residual(q, K_s, F_static, params);
R = R_unconstrained + C'*lambda;
kkt_residual = [R; C*q];
normalized_residual = norm(kkt_residual)/max(norm(F_static), cfg.F_ref);
converged = normalized_residual <= cfg.tol_R;
if ~converged
    error('FrozenThermalStatic:FinalResidual', ['Frozen thermal static solve ended with normalized residual %.3e; ', ...
        'ball T_final=%.6g C; roller T_final=%.6g C.'], normalized_residual, thermal_state.ball.T_final, thermal_state.roller.T_final);
end
static_result = struct('q', q, 'residual', R, 'bearing_force', Fb, ...
    'contact_state', contact_state, 'converged', converged, 'iterations', total_iterations);
static_result.normalized_residual = normalized_residual;
static_result.constraint = struct('front_axial_gap_m', C(1,:)*q, ...
    'numerical_axial_gauge_m', C(2,:)*q, 'matrix', C, 'lambda_N', lambda, ...
    'numerical_axial_gauge_constraint', 'u_z,C1 = 0; not physical load path');
static_result.reaction_force = reaction_force(q, K_s, params, modelInfo, lambda);
static_result.action_reaction_error = bearing_action_reaction_error(Fb, params, modelInfo);
static_result.F_gravity = F_gravity;
static_result.F_static = F_static;
end

function [q, lambda, info] = newton_kkt(q, lambda, K_s, F_step, params, C, cfg, thermal_state)
info = struct('converged', false, 'iterations', 0, 'normalized_residual', Inf);
for iter = 1:cfg.max_iter
    [R, ~, ~] = mechanical_residual(q, K_s, F_step, params);
    residual = [R + C'*lambda; C*q];
    scale = max(norm(F_step), cfg.F_ref);
    info.normalized_residual = norm(residual)/scale;
    info.iterations = iter;
    if info.normalized_residual <= cfg.tol_R, info.converged = true; return; end
    J = local_bearing_static_tangent(q, K_s, params, cfg);
    A = [J, C'; C, sparse(size(C,1), size(C,1))];
    correction = solve_kkt(A, -residual, numel(q));
    dq = correction(1:numel(q)); dlambda = correction(numel(q)+1:end);
    [accepted, alpha] = residual_line_search(q, lambda, dq, dlambda, K_s, F_step, params, C, norm(residual));
    if ~accepted
        error('FrozenThermalStatic:LineSearchFailure', ['Frozen thermal Newton line search failed at iteration %d; residual %.3e; ', ...
            'ball T_final=%.6g C; roller T_final=%.6g C.'], iter, info.normalized_residual, thermal_state.ball.T_final, thermal_state.roller.T_final);
    end
    q = q + alpha*dq; lambda = lambda + alpha*dlambda;
end
end

function [R, Fb, state] = mechanical_residual(q, K_s, F_static, params)
params.current_time = 0;
[Fb, state] = F_bearing(q, zeros(size(q)), params, numel(q));
R = K_s*q - F_static - Fb;
end

function J = local_bearing_static_tangent(q, K_s, params, cfg)
n = numel(q); J = K_s;
for ib = 1:numel(params.bearing)
    B = bearing_local_map(params.bearing(ib), params.modelInfo, n);
    r = B*q; Jb = zeros(5, 5);
    for k = 1:5
        h = local_difference_step(r(k), k, cfg.tangent_step);
        direction = 0.5*B'*unit_vector(5, k);
        state_plus = local_bearing_state(q + h*direction, params, ib);
        state_minus = local_bearing_state(q - h*direction, params, ib);
        Jb(:,k) = (bearing_force_vector(state_plus) - bearing_force_vector(state_minus))/(2*h);
    end
    J = J - B'*Jb*B;
end

function state = local_bearing_state(q, params, bearing_index)
% The contact kernels are independent by bearing; preserve the externally frozen
% thermal record while avoiding evaluation of the uninvolved bearing.
local_params = params;
local_params.bearing = params.bearing(bearing_index);
if bearing_index == 2
    local_params.microphysics.thermal_state.ball = params.microphysics.thermal_state.roller;
end
[~, bearing_state] = nonlinear_bearing_force(q, zeros(size(q)), local_params, local_params.bearing, numel(q));
state = bearing_state.bearings(1);
end
end

function v = unit_vector(n, k)
v = zeros(n, 1); v(k) = 1;
end

function h = local_difference_step(value, index, base_step)
reference_scale = [1e-6; 1e-6; 1e-6; 1e-5; 1e-5];
minimum_step = [base_step; base_step; base_step; 10*base_step; 10*base_step];
h = max(minimum_step(index), sqrt(eps)*max(abs(value), reference_scale(index)));
end

function f = bearing_force_vector(state)
f = [state.Fx; state.Fy; state.Fz; state.Mx; state.My];
end

function B = bearing_local_map(bearing, modelInfo, n)
ir = 6*bearing.rotor_node + (-5:-1);
ic = modelInfo.num_rotor_dof + 6*bearing.case_node + (-5:-1);
B = sparse([1:5 1:5], [ir ic], [ones(1,5) -ones(1,5)], 5, n);
end

function [accepted, alpha] = residual_line_search(q, lambda, dq, dlambda, K_s, F_static, params, C, residual_norm)
accepted = false; alpha = 1;
for attempt = 1:12
    q_trial = q + alpha*dq;
    [R_trial, ~, ~] = mechanical_residual(q_trial, K_s, F_static, params);
    trial_norm = norm([R_trial + C'*(lambda + alpha*dlambda); C*q_trial]);
    if isfinite(trial_norm) && trial_norm < residual_norm
        accepted = true; return;
    end
    alpha = 0.5*alpha;
end
end

function correction = solve_kkt(A, rhs, mechanical_dof)
% Numerical regularization is confined to the mechanical tangent block; the
% Lagrange-multiplier zero block remains exact so constraint reactions are not damped.
mechanical_block = A(1:mechanical_dof, 1:mechanical_dof);
scale = max(1, norm(mechanical_block, inf));
if rcond(full(A)) < 1e-12
    A(1:mechanical_dof, 1:mechanical_dof) = mechanical_block + 1e-10*scale*speye(mechanical_dof);
end
correction = A\rhs;
if any(~isfinite(correction)), error('FrozenThermalStatic:KKTCorrection', 'Frozen thermal KKT produced a non-finite Newton correction.'); end
end

function C = axial_constraints(params, modelInfo, n)
front = params.bearing(1);
ir = 6*front.rotor_node - 3;
ic = modelInfo.num_rotor_dof + 6*front.case_node - 3;
gauge = modelInfo.num_rotor_dof + 6*1 - 3;
C = sparse([1 1 2], [ir ic gauge], [1 -1 1], 2, n);
end

function F = gravity_load(MM, modelInfo, cfg)
a = zeros(size(MM,1), 1);
vertical = [2:6:modelInfo.num_rotor_dof, modelInfo.num_rotor_dof+2:6:modelInfo.num_rotor_dof+modelInfo.num_case_dof].';
if ~cfg.include_case_gravity, vertical = vertical(vertical <= modelInfo.num_rotor_dof); end
a(vertical) = -cfg.gravity;
F = MM*a;
end

function F = steady_load(params, n)
F = zeros(n,1);
if ~isfield(params, 'static_load') || isempty(params.static_load), return; end
loads = params.static_load;
if ~isfield(loads, 'Fz'), loads.Fz = zeros(size(loads.Fx)); end
for k = 1:numel(loads.nodes)
    node = loads.nodes(k); index = 6*node + (-5:-3);
    F(index) = F(index) + [loads.Fx(k); loads.Fy(k); loads.Fz(k)];
end
end

function q = structural_initial_seed(params, F_static, n, modelInfo, cfg)
q = zeros(n,1);
if norm(F_static) == 0, return; end
vertical_load = sum(F_static([2:6:modelInfo.num_rotor_dof, modelInfo.num_rotor_dof+2:6:n]));
clearances = arrayfun(@(bearing) bearing.assembly.radial_clearance, params.bearing);
q(2:6:modelInfo.num_rotor_dof) = sign(vertical_load)*(max(clearances) + cfg.contact_seed);
end

function cfg = frozen_static_config(params)
cfg = params.static_equilibrium;
required = {'gravity','include_case_gravity','max_iter','tol_R','F_ref','tangent_step','contact_seed'};
for k = 1:numel(required)
    if ~isfield(cfg, required{k}), error('FrozenThermalStatic:Configuration', 'static_equilibrium.%s is required.', required{k}); end
end
cfg.tol_R = 1e-6; % Stage4 frozen-thermal mechanical acceptance criterion.
cfg.max_iter = max(cfg.max_iter, 80); % Local Newton budget for clearance traversal.
end

function reactions = reaction_force(q, K_s, params, modelInfo, lambda)
foundation = 0;
for node = params.case_ground_nodes
    iy = modelInfo.num_rotor_dof + 6*node - 4;
    foundation = foundation - params.case_ground_k*q(iy);
end
reactions = struct('foundation_vertical_N', foundation, 'front_axial_constraint_lambda_N', lambda(1), ...
    'numerical_axial_gauge_lambda_N', lambda(2), 'numerical_axial_gauge_constraint', ...
    'u_z,C1 = 0; not physical load path', 'structural_reaction_vector', -K_s*q);
end

function error_value = bearing_action_reaction_error(Fb, params, modelInfo)
error_value = 0;
for ib = 1:numel(params.bearing)
    b = params.bearing(ib); ir = 6*b.rotor_node + (-5:-1); ic = modelInfo.num_rotor_dof + 6*b.case_node + (-5:-1);
    error_value = max(error_value, norm(Fb(ir) + Fb(ic)));
end
end
