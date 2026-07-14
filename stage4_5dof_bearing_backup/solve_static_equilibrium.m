function result = solve_static_equilibrium(params, run_acceptance_tests)
%SOLVE_STATIC_EQUILIBRIUM Two-dimensional rotor-bearing-case static work point.
% R_static(q) = K_s*q - F_static - F_bearing_static(q) = 0.

if nargin < 2, run_acceptance_tests = true; end
result = solve_core(params);
if run_acceptance_tests
    result.acceptance = run_acceptance(params, result);
end
end

function result = solve_core(params)
[MM, ~, K_s, modelInfo] = build_rotor_case_model(params);
params.modelInfo = modelInfo;
params.num_rotor_dof = modelInfo.num_rotor_dof;
params.num_case_dof = modelInfo.num_case_dof;
cfg = static_config(params);
verify_assembly_state(params);
F_gravity = build_gravity_load(MM, modelInfo, cfg);
F_steady = build_steady_load(params, size(MM,1));
F_static = F_gravity + F_steady;
q = initial_contact_seed(params, F_static, size(MM,1), modelInfo, cfg);
converged = false; history = zeros(cfg.load_steps*cfg.max_iter, 4); history_count = 0;
for step = 1:cfg.load_steps
    F_step = (step/cfg.load_steps)*F_static;
    for iter = 1:cfg.max_iter
        [R, ~, ~] = static_residual(q, K_s, F_step, params);
        eps_R = norm(R)/max(norm(F_step), cfg.F_ref);
        if eps_R <= cfg.tol_R_newton
            converged = true; break;
        end
        J = static_tangent(q, K_s, params, cfg.tangent_step);
        dq = -safe_solve(J, R);
        eps_q = norm(dq)/max(norm(q), cfg.q_ref);
        lambda = line_search(q, dq, K_s, F_step, params, norm(R));
        q = q + lambda*dq;
        history_count = history_count + 1; history(history_count,:) = [step iter eps_R eps_q];
        if eps_q <= cfg.tol_q
            [R, ~, ~] = static_residual(q, K_s, F_step, params);
            converged = norm(R)/max(norm(F_step), cfg.F_ref) <= cfg.tol_R_newton;
            if converged, break; end
        end
    end
    if ~converged
        error('Static equilibrium failed at load substep %d: normalized residual %.3e.', step, eps_R);
    end
    converged = false;
end
[R, Fb, state] = static_residual(q, K_s, F_static, params);
if cfg.refine_with_fsolve && norm(F_static) > 0
    q = refine_static_root(q, K_s, F_static, params, cfg);
    [R, Fb, state] = static_residual(q, K_s, F_static, params);
end
eps_R = norm(R)/max(norm(F_static), cfg.F_ref);
result.q0 = q;
result.K_s = K_s;
result.MM = MM;
result.modelInfo = modelInfo;
result.F_gravity = F_gravity;
result.F_steady = F_steady;
result.F_static = F_static;
result.F_bearing_static = Fb;
result.bearing_state = state;
result.residual = R;
result.normalized_residual = eps_R;
result.converged = eps_R <= cfg.tol_R;
result.history = history(1:history_count,:);
result.bearing = bearing_summary(state, Fb, modelInfo, params);
result.rotor_static_deflection = q(1:modelInfo.num_rotor_dof);
result.case_static_deformation = q(modelInfo.num_rotor_dof+1:end);
vertical_dofs = vertical_translation_dofs(modelInfo, cfg.vertical_direction);
Kq = K_s*q;
result.boundary_reaction_vertical = -sum(Kq(vertical_dofs));
result.total_gravity_vertical = sum(F_gravity(vertical_dofs));
result.total_steady_vertical = sum(F_steady(vertical_dofs));
result.force_balance_error_vertical = result.total_gravity_vertical + result.total_steady_vertical + result.boundary_reaction_vertical;
result.force_balance_error_norm = abs(result.force_balance_error_vertical)/max(norm(F_static), cfg.F_ref);
result.gravity_components = gravity_components(modelInfo);
[result.foundation_reaction_vertical, result.other_constraint_reaction_vertical] = foundation_reactions(q, params, modelInfo, result.boundary_reaction_vertical);
result.configuration = cfg;
end

function [R, Fb, state] = static_residual(q, K_s, F_static, params)
params.current_time = 0;
[Fb, state] = F_bearing(q, zeros(size(q)), params, numel(q));
R = K_s*q - F_static - Fb;
end

function J = static_tangent(q, K_s, params, h)
n = numel(q); J = K_s; params.current_time = 0;
for j = 1:n
    dq = zeros(n,1); dq(j) = h;
    [Fbp, ~] = F_bearing(q + dq, zeros(n,1), params, n);
    [Fbm, ~] = F_bearing(q - dq, zeros(n,1), params, n);
    J(:,j) = J(:,j) - (Fbp - Fbm)/(2*h);
end
end

function lambda = line_search(q, dq, K_s, F_static, params, Rnorm)
lambda = 1;
for k = 1:12
    [Rtrial, ~, ~] = static_residual(q + lambda*dq, K_s, F_static, params);
    if norm(Rtrial) <= (1 - 1e-4*lambda)*Rnorm, return; end
    lambda = 0.5*lambda;
end
end

function dq = safe_solve(J, R)
scale = max(1, norm(J, inf));
if rcond(full(J)) < 1e-12, J = J + 1e-10*scale*speye(size(J)); end
dq = J \ R;
if any(~isfinite(dq)), error('Static Newton tangent produced a non-finite correction.'); end
end

function F = build_gravity_load(MM, modelInfo, cfg)
a = zeros(size(MM,1),1); vertical = vertical_translation_dofs(modelInfo, cfg.vertical_direction);
if ~cfg.include_case_gravity, vertical = vertical(vertical <= modelInfo.num_rotor_dof); end
a(vertical) = -cfg.gravity;
F = MM*a;
end

function F = build_steady_load(params, n)
F = zeros(n,1);
if ~isfield(params, 'static_load') || isempty(params.static_load), return; end
loads = params.static_load;
if ~isfield(loads,'Fz'), loads.Fz=zeros(size(loads.Fx)); end
if ~(numel(loads.nodes) == numel(loads.Fx) && numel(loads.nodes) == numel(loads.Fy) && numel(loads.nodes) == numel(loads.Fz)), error('static_load nodes, Fx, Fy and Fz must have equal lengths.'); end
for k = 1:numel(loads.nodes)
    node = loads.nodes(k); F(6*node-5) = F(6*node-5) + loads.Fx(k); F(6*node-4) = F(6*node-4) + loads.Fy(k); F(6*node-3) = F(6*node-3) + loads.Fz(k);
end
end

function q = initial_contact_seed(params, F_static, n, modelInfo, cfg)
q = zeros(n,1);
if norm(F_static) == 0, return; end
for ib = 1:numel(params.bearing)
    b = params.bearing(ib); [c, ~] = calc_working_clearance(b, params.omega);
    iy = 6*b.rotor_node - 4; q(iy) = sign(sum(F_static(vertical_translation_dofs(modelInfo, cfg.vertical_direction))))*(max(c,0) + cfg.contact_seed);
end
end

function dofs = vertical_translation_dofs(modelInfo, direction)
if ~strcmpi(direction, 'y'), error('Stage 1 supports only the verified program y / local z vertical direction.'); end
dofs = [2:6:modelInfo.num_rotor_dof, modelInfo.num_rotor_dof+2:6:modelInfo.num_rotor_dof+modelInfo.num_case_dof].';
end

function out = bearing_summary(state, Fb, modelInfo, params)
out = repmat(struct('Fx',0,'Fy',0,'Fr',0,'action_reaction_error',0), 1, numel(params.bearing));
for ib = 1:numel(params.bearing)
    b = params.bearing(ib); irx = 6*b.rotor_node-5; iry = irx+1; icx = modelInfo.num_rotor_dof+6*b.case_node-5; icy = icx+1;
    out(ib).Fx = state.bearings(ib).Fx; out(ib).Fy = state.bearings(ib).Fy; out(ib).Fr = hypot(out(ib).Fx, out(ib).Fy);
    out(ib).action_reaction_error = norm([Fb(irx)+Fb(icx); Fb(iry)+Fb(icy)]);
end
end

function cfg = static_config(params)
cfg = params.static_equilibrium;
fields = {'gravity','vertical_direction','include_case_gravity','load_steps','max_iter','tol_R_newton','tol_R','tol_q','F_ref','q_ref','tangent_step','contact_seed','refine_with_fsolve'};
for k = 1:numel(fields), if ~isfield(cfg, fields{k}), error('static_equilibrium.%s must be defined centrally.', fields{k}); end, end
end

function q = refine_static_root(q, K_s, F_static, params, cfg)
options = optimoptions('fsolve', 'Display', 'off', 'FunctionTolerance', cfg.tol_R*max(norm(F_static), cfg.F_ref), 'StepTolerance', cfg.q_ref, 'MaxIterations', 150, 'MaxFunctionEvaluations', 20000);
[q, ~, exitflag] = fsolve(@(x) static_residual_vector(x, K_s, F_static, params), q, options);
if exitflag <= 0, warning('Static root refinement stopped with exit flag %d.', exitflag); end
end

function R = static_residual_vector(q, K_s, F_static, params)
[R, ~, ~] = static_residual(q, K_s, F_static, params);
end

function verify_assembly_state(params)
required = {'radial_clearance','clearance_change_fit','clearance_change_thermal','initial_interference'};
for ib = 1:numel(params.bearing)
    if ~isfield(params.bearing(ib), 'assembly_state'), warning('Bearing %d assembly_state is missing; zero-interference defaults are pending calibration.', ib); continue; end
    for k = 1:numel(required)
        if ~isfield(params.bearing(ib).assembly_state, required{k}), warning('Bearing %d assembly_state.%s is missing; default is pending calibration.', ib, required{k}); end
    end
end
end

function checks = run_acceptance(params, result)
zero = params; zero.static_equilibrium.gravity = 0; zero.static_load.Fx(:) = 0; zero.static_load.Fy(:) = 0; if isfield(zero.static_load,'Fz'), zero.static_load.Fz(:) = 0; end
for ib = 1:numel(zero.bearing), zero.bearing(ib).assembly_state.initial_interference = 0; end
zero_result = solve_core(zero);
checks.test1_zero_load_q0_norm = norm(zero_result.q0); checks.test1_zero_load_bearing_norm = norm(zero_result.F_bearing_static);
checks.test2_force_balance_error = result.force_balance_error_vertical;
checks.test3_normalized_residual = result.normalized_residual;
checks.test4_ball_reaction = result.bearing(1).Fr; checks.test4_roller_reaction = result.bearing(2).Fr;
checks.test5_action_reaction_error = max([result.bearing.action_reaction_error]);
checks.unbalance_force_norm = norm(unbalance_force(0, params, numel(result.q0)));
if isfield(params, 'mass_target') && isfield(params.mass_target, 'ball_reaction_N')
    checks.ball_reaction_error = result.bearing(1).Fy - params.mass_target.ball_reaction_N;
    checks.roller_reaction_error = result.bearing(2).Fy - params.mass_target.roller_reaction_N;
end
end

function g = gravity_components(modelInfo)
m = modelInfo.mass_components;
g.shaft = -m.shaft_N; g.concentrated = -m.concentrated_N; g.disk = -m.disk_N; g.case = -m.case_N;
end

function [foundation, other] = foundation_reactions(q, params, modelInfo, boundary_total)
foundation = 0;
if isfield(params, 'case_ground_nodes')
    for node = params.case_ground_nodes
        iy = modelInfo.num_rotor_dof + 6*node - 4;
        foundation = foundation - params.case_ground_k*q(iy);
    end
end
other = boundary_total - foundation;
end
