function sim = newmark_newton_multi(params)
%NEWMARK_NEWTON_MULTI Strong-coupled Newmark-beta solver with relaxed
%nonlinear bearing-force iteration.
% Equation form:
%   M*xddot + C*xdot + K*x = Fu(t) + Fbearing(x,xdot,t)
% Fbearing is returned as the signed restoring contact force assembled on
% rotor and case DOFs, so no additional sign flip is applied here.

if isfield(params, 'case_definition') && isfield(params.case_definition, 'name') && ...
        strcmp(params.case_definition.name, 'baseline_ball20_roller80')
    validate_official_case_config(params, 'newmark official baseline');
end

[MM, CC, KK, modelInfo] = build_rotor_case_model(params);
params.modelInfo = modelInfo;
params.num_rotor_dof = modelInfo.num_rotor_dof;
params.num_case_dof = modelInfo.num_case_dof;

% Add equivalent linear bearing support [k -k; -k k] and [c -c; -c c].
% Mapping note: program x = bearing local y; program y = bearing local z.
[KK, CC] = add_bearing_linear_coupling(KK, CC, params, modelInfo.num_rotor_dof);

N3 = size(MM, 1);
wi = params.omega;
Fen = params.Fen;
n_Fen = params.n_Fen;
gamma = params.gamma;
beta = params.beta;
ht = 2*pi/wi/Fen;
total_steps = Fen*n_Fen;
time = (0:total_steps)*ht;
nb = numel(params.bearing);
max_iter = get_solver_field(params, 'max_iter', get_field_default(params, 'newmark_iterations', 20));
tol_x = get_solver_field(params, 'tol_x', 1e-8);
tol_R = get_solver_field(params, 'tol_R', 1e-6);
relax = get_solver_field(params, 'relaxation', get_field_default(params, 'newmark_relaxation', 0.45));
enable_residual_diagnostics = get_config_field(params, 'enable_residual_diagnostics', true);
use_norm_assisted = get_config_field(params, 'use_normalized_residual_assisted_convergence', false);
allow_norm_batch = get_config_field(params, 'allow_normalized_assisted_for_batch_screening', false);
if use_norm_assisted && is_batch_screening_context(params) && ~allow_norm_batch
    warning('Normalized residual assisted convergence is enabled but not authorized for batch screening. Falling back to original strict criterion.');
    use_norm_assisted = false;
end

yn = zeros(N3, total_steps + 1);
dyn = zeros(N3, total_steps + 1);
ddyn = zeros(N3, total_steps + 1);
F_b_hist = zeros(2*nb, total_steps + 1);
Fb_global_hist = zeros(N3, total_steps + 1);
loaded_count_hist = zeros(nb, total_steps + 1);
slip_hist = zeros(nb, total_steps + 1);
save_full_state = get_field_default(params, 'save_full_bearing_state', false);
if save_full_state
    bearingStateHist = cell(total_steps + 1, 1);
else
    bearingStateHist = {};
end
delta_max_hist = zeros(nb, total_steps + 1);
oil_film_min_hist = zeros(nb, total_steps + 1);
contact_stiffness_hist = zeros(nb, total_steps + 1);
clearance_work_hist = zeros(nb, total_steps + 1);
err_x_hist = zeros(1, total_steps + 1);
err_R_hist = zeros(1, total_steps + 1);
iter_hist = zeros(1, total_steps + 1);
converged_hist = true(1, total_steps + 1);
assisted_converged_hist = false(1, total_steps + 1);
if enable_residual_diagnostics
    residual_diag = init_residual_diag(total_steps + 1);
else
    residual_diag = struct();
end

% Optional external disturbance only. Bearing preloads are not repeated.
F_ext = build_external_force(params, N3);

params.current_time = 0;
[Fb0, state0] = F_bearing(yn(:,1), dyn(:,1), params, N3);
Fu0 = unbalance_force(0, params, N3);
ddyn(:,1) = MM \ (Fu0 + F_ext + Fb0 - CC*dyn(:,1) - KK*yn(:,1));
Fb_global_hist(:,1) = Fb0;
if save_full_state
    bearingStateHist{1} = state0;
end
for ib = 1:nb
    F_b_hist(2*ib-1:2*ib,1) = [state0.bearings(ib).Fx; state0.bearings(ib).Fy];
    loaded_count_hist(ib,1) = state0.bearings(ib).loaded_count;
    slip_hist(ib,1) = state0.bearings(ib).slip_ratio;
    [delta_max_hist(ib,1), oil_film_min_hist(ib,1), contact_stiffness_hist(ib,1), clearance_work_hist(ib,1)] = state_diagnostics(state0.bearings(ib));
end

a0 = 1/(beta*ht^2);
a1 = gamma/(beta*ht);
a2 = 1/(beta*ht);
a3 = 1/(2*beta) - 1;
a4 = gamma/beta - 1;
a5 = ht*(gamma/(2*beta) - 1);
Keff = KK + a0*MM + a1*CC;

for n = 1:total_steps
    t = time(n+1);
    params.current_time = t;
    ramp = startup_ramp_factor(t, wi, params);

    qn = yn(:,n);
    vn = dyn(:,n);
    an = ddyn(:,n);
    q_iter = qn + ht*vn + ht^2*(0.5 - beta)*an;
    a_iter = a0*(q_iter - qn) - a2*vn - a3*an;
    v_iter = vn + ht*((1 - gamma)*an + gamma*a_iter);

    q_new = q_iter;
    v_new = v_iter;
    a_new = a_iter;
    bearingState = state0;
    err_x = inf;
    err_R = inf;
    err_R_norm_ref = inf;
    residual_metrics = empty_residual_metrics();
    step_converged = false;
    assisted_converged = false;

    for iter = 1:max_iter
        Fu = unbalance_force(t, params, N3);
        [Fb, bearingState] = F_bearing(q_iter, v_iter, params, N3);
        F_drive = ramp*(Fu + F_ext);
        Feff = F_drive + Fb ...
            + MM*(a0*qn + a2*vn + a3*an) ...
            + CC*(a1*qn + a4*vn + a5*an);
        q_trial = Keff \ Feff;
        a_trial = a0*(q_trial - qn) - a2*vn - a3*an;
        v_trial = vn + ht*((1 - gamma)*an + gamma*a_trial);

        [Fb_trial, bearingStateTrial] = F_bearing(q_trial, v_trial, params, N3);
        R = MM*a_trial + CC*v_trial + KK*q_trial - F_drive - Fb_trial;
        err_x = vecnorm2(q_trial - q_iter)/max(1, vecnorm2(q_trial));
        err_R = vecnorm2(R)/max(1, vecnorm2(F_drive));
        residual_metrics = build_residual_metrics(R, Feff, Fb_trial, Fu, err_R, err_x);
        err_R_norm_ref = residual_metrics.err_R_norm_ref;

        if err_x < tol_x && err_R < tol_R
            q_iter = q_trial;
            v_iter = v_trial;
            a_iter = a_trial;
            bearingState = bearingStateTrial;
            step_converged = true;
            break;
        end
        if use_norm_assisted && err_R <= 1e-5 && err_R_norm_ref <= 1e-6 && err_x <= 1e-10
            q_iter = q_trial;
            v_iter = v_trial;
            a_iter = a_trial;
            bearingState = bearingStateTrial;
            step_converged = true;
            assisted_converged = true;
            break;
        end

        q_iter = relax*q_trial + (1 - relax)*q_iter;
        a_iter = a0*(q_iter - qn) - a2*vn - a3*an;
        v_iter = vn + ht*((1 - gamma)*an + gamma*a_iter);
        bearingState = bearingStateTrial;
    end

    q_new = q_iter;
    a_new = a0*(q_new - qn) - a2*vn - a3*an;
    v_new = vn + ht*((1 - gamma)*an + gamma*a_new);
    yn(:,n+1) = q_new;
    dyn(:,n+1) = v_new;
    ddyn(:,n+1) = a_new;
    err_x_hist(n+1) = err_x;
    err_R_hist(n+1) = err_R;
    iter_hist(n+1) = iter;
    converged_hist(n+1) = step_converged;
    assisted_converged_hist(n+1) = assisted_converged;
    if enable_residual_diagnostics
        residual_diag = save_residual_metrics(residual_diag, n+1, residual_metrics, iter, step_converged, assisted_converged);
    end

    [Fb_save, bearingState] = F_bearing(q_new, v_new, params, N3);
    Fb_global_hist(:,n+1) = Fb_save;
    if save_full_state
        bearingStateHist{n+1} = bearingState;
    end
    for ib = 1:nb
        F_b_hist(2*ib-1:2*ib,n+1) = [bearingState.bearings(ib).Fx; bearingState.bearings(ib).Fy];
        loaded_count_hist(ib,n+1) = bearingState.bearings(ib).loaded_count;
        slip_hist(ib,n+1) = bearingState.bearings(ib).slip_ratio;
        [delta_max_hist(ib,n+1), oil_film_min_hist(ib,n+1), contact_stiffness_hist(ib,n+1), clearance_work_hist(ib,n+1)] = state_diagnostics(bearingState.bearings(ib));
    end

    if any(~isfinite(yn(:,n+1))) || max(abs(yn(:,n+1))) > params.response_limit
        warning('Response became abnormal at step %d. Simulation stopped early.', n);
        time = time(1:n+1);
        yn = yn(:,1:n+1);
        dyn = dyn(:,1:n+1);
        ddyn = ddyn(:,1:n+1);
        F_b_hist = F_b_hist(:,1:n+1);
        Fb_global_hist = Fb_global_hist(:,1:n+1);
        loaded_count_hist = loaded_count_hist(:,1:n+1);
        slip_hist = slip_hist(:,1:n+1);
        delta_max_hist = delta_max_hist(:,1:n+1);
        oil_film_min_hist = oil_film_min_hist(:,1:n+1);
        contact_stiffness_hist = contact_stiffness_hist(:,1:n+1);
        clearance_work_hist = clearance_work_hist(:,1:n+1);
        err_x_hist = err_x_hist(1:n+1);
        err_R_hist = err_R_hist(1:n+1);
        iter_hist = iter_hist(1:n+1);
        converged_hist = converged_hist(1:n+1);
        assisted_converged_hist = assisted_converged_hist(1:n+1);
        if enable_residual_diagnostics
            residual_diag = trim_residual_diag(residual_diag, n+1);
        end
        if save_full_state
            bearingStateHist = bearingStateHist(1:n+1);
        end
        break;
    end
end

sim.MM = MM;
sim.CC = CC;
sim.KK = KK;
sim.modelInfo = modelInfo;
sim.yn = yn;
sim.dyn = dyn;
sim.ddyn = ddyn;
sim.time = time;
sim.F_b_hist = F_b_hist;
sim.Fb_global_hist = Fb_global_hist;
sim.loaded_count_hist = loaded_count_hist;
sim.slip_hist = slip_hist;
sim.bearingStateHist = bearingStateHist;
sim.delta_max_hist = delta_max_hist;
sim.oil_film_min_hist = oil_film_min_hist;
sim.contact_stiffness_hist = contact_stiffness_hist;
sim.clearance_work_hist = clearance_work_hist;
sim.solver.err_x_hist = err_x_hist;
sim.solver.err_R_hist = err_R_hist;
sim.solver.iter_hist = iter_hist;
sim.solver.converged_hist = converged_hist;
sim.solver.assisted_converged_hist = assisted_converged_hist;
sim.solver.max_err_x = max(err_x_hist);
sim.solver.max_err_R = max(err_R_hist);
sim.solver.unconverged_steps = sum(~converged_hist);
sim.solver.max_iter = max_iter;
iter_used = iter_hist(2:end);
if isempty(iter_used)
    sim.solver.avg_iter = 0;
    sim.solver.max_used_iter = 0;
else
    sim.solver.avg_iter = sum(iter_used)/numel(iter_used);
    sim.solver.max_used_iter = max(iter_used);
end
sim.solver.tol_x = tol_x;
sim.solver.tol_R = tol_R;
sim.solver.dt = ht;
sim.solver.gamma = gamma;
sim.solver.beta = beta;
sim.solver.method = 'Newmark-beta with relaxed nonlinear bearing-force iteration';
sim.solver.residual_diagnostics_enabled = enable_residual_diagnostics;
sim.solver.use_normalized_residual_assisted_convergence = use_norm_assisted;
sim.solver.allow_normalized_assisted_for_batch_screening = allow_norm_batch;
if enable_residual_diagnostics
    residual_diag = trim_residual_diag(residual_diag, numel(time));
    sim.solver.residual_diagnostics = residual_diag;
end
sim.convergence = build_convergence_summary(sim, tol_x, tol_R);
if sim.solver.unconverged_steps > 0
    warning('Newmark-beta relaxed nonlinear force iteration had %d unconverged steps. Max err_x=%.3e, max err_R=%.3e. See report for details.', ...
        sim.solver.unconverged_steps, sim.solver.max_err_x, sim.solver.max_err_R);
end
sim.params = params;
end

function [KK, CC] = add_bearing_linear_coupling(KK, CC, params, num_rotor)
for ib = 1:numel(params.bearing)
    b = params.bearing(ib);
    irx = 4*b.rotor_node - 3;
    iry = 4*b.rotor_node - 2;
    icx = num_rotor + 4*b.case_node - 3;
    icy = num_rotor + 4*b.case_node - 2;
    KK = add_pair(KK, irx, icx, b.kx);
    KK = add_pair(KK, iry, icy, b.ky);
    CC = add_pair(CC, irx, icx, b.cx);
    CC = add_pair(CC, iry, icy, b.cy);
end
end

function A = add_pair(A, i, j, val)
A(i,i) = A(i,i) + val;
A(j,j) = A(j,j) + val;
A(i,j) = A(i,j) - val;
A(j,i) = A(j,i) - val;
end

function F_ext = build_external_force(params, N3)
F_ext = zeros(N3, 1);
if isfield(params, 'static_load') && ~isempty(params.static_load)
    for k = 1:numel(params.static_load.nodes)
        nd = params.static_load.nodes(k);
        F_ext(4*nd - 3) = F_ext(4*nd - 3) + params.static_load.Fx(k);
        F_ext(4*nd - 2) = F_ext(4*nd - 2) + params.static_load.Fy(k);
    end
end
end

function ramp = startup_ramp_factor(t, omega, params)
cycles = get_field_default(params, 'startup_ramp_cycles', 0);
if cycles <= 0
    ramp = 1;
    return;
end
T = 2*pi/omega;
s = min(max(t/(cycles*T), 0), 1);
ramp = 0.5 - 0.5*cos(pi*s);
end

function v = get_field_default(s, field, default_value)
if isfield(s, field) && ~isempty(s.(field))
    v = s.(field);
else
    v = default_value;
end
end

function v = get_solver_field(params, field, default_value)
if isfield(params, 'solver') && isfield(params.solver, field) && ~isempty(params.solver.(field))
    v = params.solver.(field);
else
    v = default_value;
end
end

function v = get_config_field(params, field, default_value)
if isfield(params, 'config') && isfield(params.config, field) && ~isempty(params.config.(field))
    v = params.config.(field);
elseif isfield(params, field) && ~isempty(params.(field))
    v = params.(field);
else
    v = default_value;
end
end

function tf = is_batch_screening_context(params)
tf = false;
if isfield(params, 'is_batch_screening') && params.is_batch_screening
    tf = true;
elseif isfield(params, 'screening_case_id') || isfield(params, 'batch_screening')
    tf = true;
elseif isfield(params, 'output_dir') && contains(lower(char(params.output_dir)), 'screen')
    tf = true;
end
end

function d = init_residual_diag(n)
d.err_R_original = zeros(1,n);
d.R_abs_norm = zeros(1,n);
d.R_inf_norm = zeros(1,n);
d.F_eff_norm = zeros(1,n);
d.F_bearing_norm = zeros(1,n);
d.F_unbalance_norm = zeros(1,n);
d.F_reference = ones(1,n);
d.err_R_norm_ref = zeros(1,n);
d.err_x = zeros(1,n);
d.iter_count = zeros(1,n);
d.is_original_converged = true(1,n);
d.is_assisted_converged = false(1,n);
end

function m = empty_residual_metrics()
m.err_R_original = NaN;
m.R_abs_norm = NaN;
m.R_inf_norm = NaN;
m.F_eff_norm = NaN;
m.F_bearing_norm = NaN;
m.F_unbalance_norm = NaN;
m.F_reference = NaN;
m.err_R_norm_ref = NaN;
m.err_x = NaN;
end

function m = build_residual_metrics(R, Feff, Fb, Fu, err_R, err_x)
m.err_R_original = err_R;
m.R_abs_norm = vecnorm2(R);
m.R_inf_norm = norm(R, inf);
m.F_eff_norm = vecnorm2(Feff);
m.F_bearing_norm = vecnorm2(Fb);
m.F_unbalance_norm = vecnorm2(Fu);
m.F_reference = max([m.F_eff_norm, m.F_bearing_norm, m.F_unbalance_norm, 1.0]);
m.err_R_norm_ref = m.R_abs_norm/m.F_reference;
m.err_x = err_x;
end

function d = save_residual_metrics(d, idx, m, iter, is_converged, is_assisted)
d.err_R_original(idx) = m.err_R_original;
d.R_abs_norm(idx) = m.R_abs_norm;
d.R_inf_norm(idx) = m.R_inf_norm;
d.F_eff_norm(idx) = m.F_eff_norm;
d.F_bearing_norm(idx) = m.F_bearing_norm;
d.F_unbalance_norm(idx) = m.F_unbalance_norm;
d.F_reference(idx) = m.F_reference;
d.err_R_norm_ref(idx) = m.err_R_norm_ref;
d.err_x(idx) = m.err_x;
d.iter_count(idx) = iter;
d.is_original_converged(idx) = is_converged && ~is_assisted;
d.is_assisted_converged(idx) = is_assisted;
end

function d = trim_residual_diag(d, n)
fields = fieldnames(d);
for i = 1:numel(fields)
    value = d.(fields{i});
    if isnumeric(value) || islogical(value)
        d.(fields{i}) = value(:,1:min(n,size(value,2)));
    end
end
end

function c = build_convergence_summary(sim, tol_x, tol_R)
err_R = sim.solver.err_R_hist(2:end);
err_x = sim.solver.err_x_hist(2:end);
if isfield(sim.solver, 'residual_diagnostics')
    diag = sim.solver.residual_diagnostics;
    err_norm_ref = diag.err_R_norm_ref(2:end);
else
    err_norm_ref = NaN(size(err_R));
end
n = max(numel(err_R), 1);
strict = err_R <= tol_R & err_x <= tol_x;
engineering = err_R > 1e-6 & err_R <= 1e-5 & err_x <= 1e-10;
mild = err_R > 1e-5 & err_R <= 1e-4 & err_x <= 1e-10;
serious = ~(strict | engineering | mild);
norm_strict = err_norm_ref <= 1e-6;
norm_engineering = err_norm_ref > 1e-6 & err_norm_ref <= 1e-5;
norm_mild = err_norm_ref > 1e-5 & err_norm_ref <= 1e-4;
norm_serious = err_norm_ref > 1e-4;
c.strict_count = sum(strict);
c.engineering_count = sum(engineering);
c.mild_count = sum(mild);
c.serious_count = sum(serious);
c.strict_ratio = c.strict_count/n;
c.engineering_ratio = c.engineering_count/n;
c.mild_ratio = c.mild_count/n;
c.serious_ratio = c.serious_count/n;
c.strict_plus_engineering_ratio = (c.strict_count + c.engineering_count)/n;
c.reported_unconverged_ratio = get_field_default(sim.solver, 'unconverged_steps', 0)/n;
c.err_R_original_max = max_or_nan(err_R);
c.err_R_original_median = median_or_nan(err_R);
c.err_R_original_p95 = percentile_or_nan(err_R, 95);
c.err_R_original_p99 = percentile_or_nan(err_R, 99);
c.err_R_norm_ref_max = max_or_nan(err_norm_ref);
c.err_R_norm_ref_median = median_or_nan(err_norm_ref);
c.err_R_norm_ref_p95 = percentile_or_nan(err_norm_ref, 95);
c.err_R_norm_ref_p99 = percentile_or_nan(err_norm_ref, 99);
c.err_x_max = max_or_nan(err_x);
c.err_x_median = median_or_nan(err_x);
c.normalized_strict_ratio = sum(norm_strict)/n;
c.normalized_engineering_ratio = sum(norm_engineering)/n;
c.normalized_mild_ratio = sum(norm_mild)/n;
c.normalized_serious_ratio = sum(norm_serious)/n;
c.assisted_count = sum(get_field_default(sim.solver, 'assisted_converged_hist', false(size(sim.solver.converged_hist))));
c.original_strict_criterion_retained = true;
c.normalized_assisted_enabled = get_field_default(sim.solver, 'use_normalized_residual_assisted_convergence', false);
c.batch_screening_authorized = get_field_default(sim.solver, 'allow_normalized_assisted_for_batch_screening', false);
end

function v = max_or_nan(x)
x = x(isfinite(x));
if isempty(x), v = NaN; else, v = max(x); end
end

function v = median_or_nan(x)
x = x(isfinite(x));
if isempty(x), v = NaN; else, v = median(x); end
end

function v = percentile_or_nan(x, p)
x = sort(x(isfinite(x)));
if isempty(x)
    v = NaN;
    return;
end
idx = max(1, min(numel(x), ceil(p/100*numel(x))));
v = x(idx);
end

function [delta_max, oil_min, k_eff, c_work] = state_diagnostics(st)
if isfield(st, 'delta') && ~isempty(st.delta)
    delta_max = max(st.delta);
else
    delta_max = 0;
end
if isfield(st, 'h') && ~isempty(st.h)
    oil_min = min(st.h);
else
    oil_min = NaN;
end
if isfield(st, 'k_contact_eff') && ~isempty(st.k_contact_eff)
    k_eff = st.k_contact_eff;
else
    k_eff = 0;
end
if isfield(st, 'c_work') && ~isempty(st.c_work)
    c_work = st.c_work;
else
    c_work = NaN;
end
end

function v = vecnorm2(x)
v = sqrt(sum(x(:).^2));
end
