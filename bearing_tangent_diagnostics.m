function diag = bearing_tangent_diagnostics(bearing, state)
%BEARING_TANGENT_DIAGNOSTICS Explicit local nonlinear tangent stiffness.
% This is a post-processing diagnostic. It is not assembled into the global
% Newmark effective stiffness matrix unless a solver explicitly does so.

n = numel(state.theta);
theta = state.theta(:).';
delta = get_vec(state, 'delta', n);
delta_raw = get_vec(state, 'delta_raw', n);
Q = get_vec(state, 'Q', n);
h = get_vec_first(state, {'h','oil_film'}, n);
lambda = get_vec_first(state, {'lambda','roughness_lambda'}, n);
slip = get_vec_first(state, {'slip_i','slip_ratio'}, n);
Hs = get_vec(state, 'Hs', n);
contact = get_contact_from_state(state);

if strcmpi(bearing.type, 'ball')
    C = get_field_default(bearing, 'K_point', 2.443e9);
    expn = 1.5;
    radial_factor = cos(get_field_default(bearing, 'contact_angle', 0));
    centrifugal_factor = 1;
else
    C = get_field_default(bearing, 'K_line', 2.4e8);
    expn = 10/9;
    radial_factor = 1;
    omega_ref = 2*pi*9900/60;
    omega_case = get_field_default(state, 'omega', omega_ref);
    centrifugal_factor = (1 + 0.08*(abs(omega_case)/omega_ref)^2);
end

kt = zeros(1,n);
kt_fd = zeros(1,n);
rel_diff = zeros(1,n);
for i = 1:n
    dpos_draw = smooth_delta_derivative(delta_raw(i), contact);
    if delta(i) > 0 && Q(i) > 1e-9
        kt(i) = expn*C*max(delta(i), eps)^(expn-1)*dpos_draw*radial_factor*centrifugal_factor;
        eps_fd = max(1e-9, 1e-5*abs(delta_raw(i)));
        qp = elastic_load_from_raw(delta_raw(i) + eps_fd, C, expn, radial_factor, centrifugal_factor, contact, bearing);
        qm = elastic_load_from_raw(delta_raw(i) - eps_fd, C, expn, radial_factor, centrifugal_factor, contact, bearing);
        kt_fd(i) = (qp - qm)/(2*eps_fd);
        rel_diff(i) = abs(kt(i)-kt_fd(i))/max(abs(kt_fd(i)), 1e-12);
    else
        kt(i) = 0;
        kt_fd(i) = 0;
        rel_diff(i) = 0;
    end
end

K = zeros(2,2);
for i = 1:n
    ni = [cos(theta(i)); sin(theta(i))];
    K = K + kt(i)*(ni*ni.');
end

diag.element_id = 1:n;
diag.theta = theta;
diag.theta_deg = mod(theta*180/pi, 360);
diag.loaded = Q > 1e-9 & delta > 0;
diag.delta = delta;
diag.Q = Q;
diag.kt_analytical = kt;
diag.kt_finite_difference = kt_fd;
diag.kt_relative_difference = rel_diff;
diag.h = h;
diag.lambda = lambda;
diag.slip = slip;
diag.K_local = K;
diag.trace = trace(K);
diag.eigenvalues = sort(eig(K)).';
diag.max_contact_force = max(Q);
diag.contact_force_rms = sqrt(mean(Q(:).^2));
diag.note = 'Explicit nonlinear tangent stiffness is a post-processing quantity and is not assembled into the global Newmark effective stiffness matrix in the current implementation.';
end

function q = elastic_load_from_raw(raw, C, expn, radial_factor, centrifugal_factor, contact, bearing)
[dpos, ~] = positive_contact_delta_local(raw, contact);
dpos = min(dpos, get_field_default(bearing, 'max_delta', inf));
if dpos <= 0
    q = 0;
else
    q = C*dpos^expn*radial_factor*centrifugal_factor;
end
end

function d = smooth_delta_derivative(raw, contact)
if contact.smooth_enable
    epsd = max(contact.smooth_delta, eps);
    d = 0.5*(1 + raw/sqrt(raw^2 + epsd^2));
else
    d = double(raw > 0);
end
end

function [delta_pos, Hs] = positive_contact_delta_local(delta_raw, contact)
if contact.smooth_enable
    epsd = max(contact.smooth_delta, eps);
    Hs = 0.5*(1 + tanh(delta_raw/epsd));
    delta_pos = 0.5*(delta_raw + sqrt(delta_raw^2 + epsd^2));
else
    Hs = double(delta_raw > 0);
    delta_pos = max(delta_raw, 0);
end
end

function contact = get_contact_from_state(state)
if isfield(state, 'contact') && ~isempty(state.contact)
    contact = state.contact;
else
    contact = struct();
end
contact.smooth_enable = get_field_default(contact, 'smooth_enable', true);
contact.smooth_delta = get_field_default(contact, 'smooth_delta', 1e-7);
end

function v = get_vec(state, field, n)
if isfield(state, field) && ~isempty(state.(field))
    v = state.(field)(:).';
else
    v = zeros(1,n);
end
end

function v = get_vec_first(state, fields, n)
v = [];
for i = 1:numel(fields)
    if isfield(state, fields{i}) && ~isempty(state.(fields{i}))
        raw = state.(fields{i});
        if isscalar(raw)
            v = repmat(raw, 1, n);
        else
            v = raw(:).';
        end
        break;
    end
end
if isempty(v)
    v = NaN(1,n);
end
if numel(v) ~= n
    v = repmat(v(1), 1, n);
end
end

function v = get_field_default(s, field, default_value)
if isfield(s, field) && ~isempty(s.(field))
    v = s.(field);
else
    v = default_value;
end
end
