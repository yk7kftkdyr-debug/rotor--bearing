function [result, bearing] = bearing_quasi_dynamic_roller(bearing, params)
%BEARING_QUASI_DYNAMIC_ROLLER Simplified high-speed cylindrical roller solver.
% Inputs:
%   bearing - roller-bearing parameter structure.
%   params  - global working condition.
% Outputs:
%   result  - quasi-dynamic contact/lubrication/stiffness state.
%   bearing - bearing structure with updated kx, ky, cx, cy.

omega = params.omega;
n = bearing.n;
Dw = bearing.Dw;
Dm = bearing.Dm;
L = bearing.L;
Cb = bearing.K_line;
expn = 10/9;
[cw, clearInfo] = calc_working_clearance(bearing, omega);

Fy0 = get_field_default(bearing, 'preload_y', 0);
Fz0 = get_field_default(bearing, 'preload_z', 0);
Fpre = hypot(Fy0, Fz0);
load_angle = atan2(Fz0, Fy0 + eps);
theta = 2*pi*(0:n-1)/n;
omega_cage = 0.5*omega*(1 - Dw/Dm);

r_eq = solve_radial_closure(Fpre, load_angle, theta, cw, Cb, expn);
[Fy_contact, Fz_contact, Q, delta] = contact_sum(r_eq*cos(load_angle), r_eq*sin(load_angle), theta, cw, Cb, expn);

U = max(abs(omega)*Dm/2, 1e-9);
h = oil_film_estimate(bearing, U, Q);
contact_area = max(L*Dw, 1e-12);
pressure = Q/contact_area;
PV = pressure.*abs(0.02*omega_cage*Dw);
rough = max(get_field_default(bearing, 'roughness_rms', 0.1e-6), 1e-9);
lambda = h/rough;
mu_eff = bearing.mu*(1 + 0.40*exp(-lambda));
unloaded_fraction = sum(Q == 0)/numel(Q);
slip_ratio = min(0.55, max(0.03, 0.16 + 0.10*unloaded_fraction));

K = numerical_stiffness(r_eq*cos(load_angle), r_eq*sin(load_angle), theta, cw, Cb, expn);
K = max(K, 0);

% Mapping: program x = local bearing y, program y = local bearing z.
bearing.kx = max(K(1,1), get_field_default(bearing, 'linear_k', 1e7));
bearing.ky = max(K(2,2), get_field_default(bearing, 'linear_k', 1e7));
bearing.cx = get_field_default(bearing, 'linear_c', get_field_default(bearing, 'oil_damping', 1e3));
bearing.cy = bearing.cx;

result.type = 'rear high-speed cylindrical roller bearing';
result.clearInfo = clearInfo;
result.clearance_work = cw;
result.theta = theta;
result.delta = delta;
result.Q = Q;
result.loaded_count = sum(Q > 0);
result.Fy_contact = Fy_contact;
result.Fz_contact = Fz_contact;
result.max_contact_load = max(Q);
result.max_contact_stress = max(pressure);
result.oil_film = h;
result.min_oil_film = min(h);
result.PV = PV;
result.mu_eff = mu_eff;
result.lambda = lambda;
result.stiffness_matrix = K;
result.kx = bearing.kx;
result.ky = bearing.ky;
result.cx = bearing.cx;
result.cy = bearing.cy;
result.cage_speed = omega_cage;
result.slip_ratio = slip_ratio;
result.preload_y = Fy0;
result.preload_z = Fz0;
result.radial_closure = r_eq;
result.load_angle = load_angle;
result.operating_offset_x = r_eq*cos(load_angle);
result.operating_offset_y = r_eq*sin(load_angle);
end

function r = solve_radial_closure(Fpre, a, theta, cw, Cb, expn)
if Fpre <= 0
    r = 0;
    return;
end
lo = 0;
hi = max(cw + (Fpre/max(Cb,eps))^(1/expn)*5, cw + 5e-6);
for k = 1:80
    mid = 0.5*(lo + hi);
    [Fy, Fz] = contact_sum(mid*cos(a), mid*sin(a), theta, cw, Cb, expn);
    Fdir = abs(Fy*cos(a) + Fz*sin(a));
    if Fdir < Fpre
        lo = mid;
    else
        hi = mid;
    end
end
r = 0.5*(lo + hi);
end

function [Fy, Fz, Q, delta] = contact_sum(Y, Z, theta, cw, Cb, expn)
delta = Y*cos(theta) + Z*sin(theta) - cw;
delta(delta <= 0) = 0;
Q = Cb*delta.^expn;
Fy = sum(Q.*cos(theta));
Fz = sum(Q.*sin(theta));
end

function K = numerical_stiffness(Y, Z, theta, cw, Cb, expn)
epsd = 1e-7;
[Fy0, Fz0] = contact_sum(Y, Z, theta, cw, Cb, expn);
[FyY, FzY] = contact_sum(Y + epsd, Z, theta, cw, Cb, expn);
[FyZ, FzZ] = contact_sum(Y, Z + epsd, theta, cw, Cb, expn);
K = [(FyY-Fy0)/epsd, (FyZ-Fy0)/epsd; (FzY-Fz0)/epsd, (FzZ-Fz0)/epsd];
end

function h = oil_film_estimate(bearing, U, Q)
mu0 = get_field_default(bearing, 'oil_viscosity', 3.18e-2);
alpha_p = get_field_default(bearing, 'pressure_viscosity', 1.25e-8);
h0 = get_field_default(bearing, 'oil_film0', 0.8e-6);
load_factor = (max(Q, 1)./max(max(Q), 1)).^(-0.073);
h = h0 + 2.0e-8*(mu0/3.18e-2)^0.67*(alpha_p/1.25e-8)^0.49*(U/10)^0.67.*load_factor;
h = min(max(h, 0.05e-6), 5e-6);
end

function v = get_field_default(s, field, default_value)
if isfield(s, field) && ~isempty(s.(field))
    v = s.(field);
else
    v = default_value;
end
end
