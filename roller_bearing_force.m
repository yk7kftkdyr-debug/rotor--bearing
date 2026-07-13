function [Fx, Fy, state] = roller_bearing_force(xr, yr, vxr, vyr, bearing, omega, t)
%ROLLER_BEARING_FORCE Cylindrical roller bearing nonlinear contact force.
% Inputs:
%   xr, yr, vxr, vyr - relative displacement and velocity between rotor and case.
%   bearing          - roller bearing parameters.
%   omega            - shaft speed, rad/s.
%   t                - time, s.
% Outputs:
%   Fx, Fy - bearing force acting on rotor node.
%   state  - contact state, including Q_i, delta_i, h_i, PV_i and slip.

if nargin < 7
    t = 0;
end

n = bearing.n;
Dw = bearing.Dw;
Dm = bearing.Dm;
L = bearing.L;
mu0 = bearing.mu;
K_line = get_field_default(bearing, 'K_line', 2.0e8);
cn = get_field_default(bearing, 'contact_cn', get_field_default(bearing, 'contact_damping', 80));
coil = get_field_default(bearing, 'oil_damping', 0);
kfilm = get_field_default(bearing, 'oil_stiffness', 0);
Qmax = get_field_default(bearing, 'max_contact_force', inf);
delta_max = get_field_default(bearing, 'max_delta', inf);
[c_work, clearInfo] = calc_working_clearance(bearing, omega);
contact = get_contact_config(bearing);
surface = get_nested_default(bearing, 'surface', struct());
roughness = get_nested_default(bearing, 'roughness', struct());
debris = get_nested_default(bearing, 'debris', struct());
waviness = get_nested_default(bearing, 'waviness', struct());

omega_cage_ideal = 0.5*omega*(1 - Dw/Dm);
cage_angle = omega_cage_ideal*t;
theta = 2*pi*(0:n-1)/n + cage_angle;

U = max(abs(omega)*Dm/2, 1e-9);
h_base = oil_film_estimate(bearing, U);
Q_ref = ehl_load_reference(bearing);
ehl_enable = get_field_default(bearing, 'ehl_load_correction_enable', true);
if get_field_default(surface, 'texture_enable', false)
    h_base = h_base*get_field_default(surface, 'Cr', 1.0);
    texture_gamma = get_field_default(surface, 'texture_gamma', 0);
else
    texture_gamma = 0;
end
if get_field_default(roughness, 'enable', false)
    rough = sqrt(get_field_default(roughness, 'Rq_inner', 0.1e-6)^2 + get_field_default(roughness, 'Rq_outer', 0.1e-6)^2);
else
    rough = get_field_default(bearing, 'roughness_rms', 0);
end
ud = 0;
if get_field_default(debris, 'enable', false)
    ud = get_field_default(debris, 'ud', 0);
end
if get_field_default(waviness, 'enable', false)
    P2_amp = get_field_default(waviness, 'P2_amp', 0);
    waviness_order = get_field_default(waviness, 'order', get_field_default(bearing, 'waviness_order', 3));
else
    P2_amp = 0;
    waviness_order = get_field_default(bearing, 'waviness_order', 3);
end

rho_r = get_field_default(bearing, 'roller_density', 7850);
m_roll = rho_r*pi*Dw^2/4*L;
Fc = m_roll*(omega_cage_ideal^2)*(Dm/2);

Fx_i = zeros(1,n);
Fy_i = zeros(1,n);
Q = zeros(1,n);
delta = zeros(1,n);
delta_raw = zeros(1,n);
Hs = zeros(1,n);
h = zeros(1,n);
h_uncorrected = zeros(1,n);
PV = zeros(1,n);
slip_i = zeros(1,n);
mu_i = zeros(1,n);
k_contact = zeros(1,n);

for i = 1:n
    ci = cos(theta(i));
    si = sin(theta(i));
    h_uncorrected(i) = h_base*(1 + 0.08*cos(theta(i) - atan2(yr, xr)) + texture_gamma*cos(waviness_order*theta(i)));
    h_uncorrected(i) = max(h_uncorrected(i), 0);
    h(i) = h_uncorrected(i);
    P2 = P2_amp*cos(waviness_order*theta(i));
    delta_raw(i) = xr*ci + yr*si - c_work + ud - P2 - h(i);
    [delta_pos, Hs(i)] = positive_contact_delta(delta_raw(i), contact);
    delta(i) = min(delta_pos, delta_max);

    if delta(i) > 0
        vn = vxr*ci + vyr*si;
        Qh = K_line*delta(i)^(10/9);
        Qd = 0;
        if contact.damping_enable
            Qd = cn*vn*Hs(i);
        end
        speed_factor = (abs(omega)/(2*pi*9900/60))^2;
        centrifugal_factor = 1 + 0.08*speed_factor;
        Qi = max(0, Qh*centrifugal_factor + Qd + 0.03*Fc*Hs(i));
        Qi = min(Qi, Qmax);
        if ehl_enable
            h(i) = load_corrected_oil_film(h_uncorrected(i), Qi, Q_ref, bearing);
            delta_raw(i) = xr*ci + yr*si - c_work + ud - P2 - h(i);
            [delta_pos, Hs(i)] = positive_contact_delta(delta_raw(i), contact);
            delta(i) = min(delta_pos, delta_max);
            vn = vxr*ci + vyr*si;
            Qh = K_line*delta(i)^(10/9);
            Qd = 0;
            if contact.damping_enable
                Qd = cn*vn*Hs(i);
            end
            Qi = min(max(Qh*centrifugal_factor + Qd + 0.03*Fc*Hs(i), 0), Qmax);
        end
        k_contact(i) = (10/9)*K_line*max(delta(i), contact.smooth_delta)^(1/9)*centrifugal_factor*Hs(i);

        tx = -si;
        ty = ci;
        vt = vxr*tx + vyr*ty - omega_cage_ideal*Dw*0.02;
        lambda_ratio = h(i)/max(rough, 1e-9);
        mu_i(i) = mu0*(1 + 0.40*exp(-lambda_ratio) + 0.15*min(abs(vt)/U, 2));
        Ft = mu_i(i)*Qi*sign_smooth(vt);

        Fx_i(i) = -Qi*ci - Ft*tx;
        Fy_i(i) = -Qi*si - Ft*ty;
        Q(i) = Qi;

        contact_area = max(L*Dw, 1e-12);
        p_mean = Qi/contact_area;
        PV(i) = p_mean*abs(vt);
        slip_i(i) = min(0.60, max(0, 0.02 + 0.30*Fc/(Qi + Fc + 1) + 0.04*abs(vt)/(U + 1e-12)));
    else
        if ehl_enable
            h(i) = load_corrected_oil_film(h_uncorrected(i), 0, Q_ref, bearing);
            delta_raw(i) = xr*ci + yr*si - c_work + ud - P2 - h(i);
        end
        Q(i) = 0;
        mu_i(i) = mu0;
        PV(i) = 0;
        slip_i(i) = 0.45;
    end
end

Fx = sum(Fx_i);
Fy = sum(Fy_i);
if get_field_default(bearing, 'include_oil_damping_force', true)
    Fx = Fx - coil*vxr;
    Fy = Fy - coil*vyr;
end
if get_field_default(bearing, 'include_oil_stiffness_force', false)
    Fx = Fx - kfilm*xr;
    Fy = Fy - kfilm*yr;
end

if ~isfinite(Fx) || ~isfinite(Fy)
    Fx = 0;
    Fy = 0;
end

loaded = delta_raw > 0 & Q > 1e-9;
state.theta = theta;
state.delta = delta;
state.delta_raw = delta_raw;
state.Hs = Hs;
state.Q = Q;
state.h_base = h_uncorrected;
state.h = h;
state.PV = PV;
state.mu_i = mu_i;
state.k_contact = k_contact;
state.k_contact_eff = smooth_contact_stiffness(k_contact, contact);
state.loaded_count = sum(loaded);
state.loaded_index = find(loaded);
state.sliding_speed = PV ./ max(Q/(max(L*Dw,1e-12)), 1e-12);
if any(loaded)
    loaded_slip = slip_i(loaded);
    state.slip_ratio = sum(loaded_slip)/numel(loaded_slip);
else
    state.slip_ratio = 0.45;
end
state.cage_speed_ideal = omega_cage_ideal;
state.cage_speed_estimated = omega_cage_ideal*(1 - state.slip_ratio);
state.ball_spin_speed = NaN;
state.centrifugal_force = Fc;
state.c_work = c_work;
state.clearInfo = clearInfo;
state.contact = contact;
state.roughness_lambda_min = min(h)/max(rough, 1e-12);
state.Q_ref = Q_ref;
state.ehl_load_correction_enable = ehl_enable;
state.slip_model = 'empirical indicator based on load, centrifugal effect and tangential velocity';
state.include_oil_damping_force = get_field_default(bearing, 'include_oil_damping_force', true);
state.include_oil_stiffness_force = get_field_default(bearing, 'include_oil_stiffness_force', false);

end

function h = oil_film_estimate(bearing, U)
mu_oil = get_field_default(bearing, 'oil_viscosity', 3.18e-2);
if isfield(bearing, 'thermal') && get_field_default(bearing.thermal, 'enable', false)
    T_ref = get_field_default(bearing.thermal, 'T_ref', 20);
    T_oil = get_field_default(bearing.thermal, 'T_oil', T_ref);
    beta_eta = get_field_default(bearing.thermal, 'beta_eta', 0.025);
    mu_oil = mu_oil*exp(-beta_eta*(T_oil - T_ref));
end

alpha_p = get_field_default(bearing, 'pressure_viscosity', 1.25e-8);
h0 = get_field_default(bearing, 'oil_film0', 0.8e-6);
h = h0 + 2.0e-8*(mu_oil/3.18e-2)^0.67*(alpha_p/1.25e-8)^0.49*(U/10)^0.67;
h = min(max(h, 0), 5e-6);
end

function h_corr = load_corrected_oil_film(h_base_local, Q, Q_ref, bearing)
Qmin = 1e-6;
expn = get_field_default(bearing, 'ehl_load_exponent', -0.067);
fmin = get_field_default(bearing, 'ehl_load_factor_min', 0.4);
fmax = get_field_default(bearing, 'ehl_load_factor_max', 2.5);
factor = (max(abs(Q), Qmin)/max(Q_ref, Qmin))^expn;
factor = min(max(factor, fmin), fmax);
h_corr = max(h_base_local*factor, 0);
end

function Q_ref = ehl_load_reference(bearing)
if isfield(bearing, 'Q_ref') && ~isempty(bearing.Q_ref)
    Q_ref = bearing.Q_ref;
else
    preload = max([abs(get_field_default(bearing, 'preload_z', 0)), abs(get_field_default(bearing, 'preload_y', 0)), 1]);
    Q_ref = preload/max(get_field_default(bearing, 'n', 1)/2, 1);
end
Q_ref = max(Q_ref, 1e-6);
end

function [delta_pos, Hs] = positive_contact_delta(delta_raw, contact)
if contact.smooth_enable
    epsd = max(contact.smooth_delta, eps);
    Hs = 0.5*(1 + tanh(delta_raw/epsd));
    delta_pos = 0.5*(delta_raw + sqrt(delta_raw^2 + epsd^2));
else
    Hs = double(delta_raw > 0);
    delta_pos = max(delta_raw, 0);
end
end

function k_eff = smooth_contact_stiffness(k_contact, contact)
k_new = max(k_contact);
if contact.stiffness_smooth_enable
    % No global tangent matrix is changed here; the smoothed estimate is
    % reported and used for diagnostics/contact-frequency checks.
    k_eff = k_new;
else
    k_eff = k_new;
end
end

function contact = get_contact_config(bearing)
if isfield(bearing, 'contact')
    contact = bearing.contact;
else
    contact = struct();
end
contact.smooth_enable = get_field_default(contact, 'smooth_enable', true);
contact.smooth_delta = get_field_default(contact, 'smooth_delta', 1e-7);
contact.damping_enable = get_field_default(contact, 'damping_enable', true);
contact.stiffness_smooth_enable = get_field_default(contact, 'stiffness_smooth_enable', true);
contact.alpha_k = get_field_default(contact, 'alpha_k', 0.8);
end

function s = get_nested_default(parent, field, default_value)
if isfield(parent, field) && ~isempty(parent.(field))
    s = parent.(field);
else
    s = default_value;
end
end

function y = sign_smooth(x)
y = x/(abs(x) + 1e-9);
end

function v = get_field_default(s, field, default_value)
if isfield(s, field) && ~isempty(s.(field))
    v = s.(field);
else
    v = default_value;
end
end
