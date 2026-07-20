function power = compute_static_friction_power(bearing, contact_state, film_state, eta_Pa_s, Omega_rad_s)
%COMPUTE_STATIC_FRICTION_POWER Two-sided EHL drag power at a static work point.
% Reads the frozen Stage4 contact state only; it never updates temperature or force.

validateattributes(eta_Pa_s, {'numeric'}, {'scalar','real','finite','positive'});
validateattributes(Omega_rad_s, {'numeric'}, {'scalar','real','finite'});
required_contact = {'Q','delta'};
for k = 1:numel(required_contact)
    if ~isfield(contact_state, required_contact{k})
        error('StaticFrictionPower:ContactState', 'compute_static_friction_power requires contact_state.%s.', required_contact{k});
    end
end
if ~isfield(film_state, 'h_m')
    error('StaticFrictionPower:FilmState', 'compute_static_friction_power requires film_state.h_m.');
end
Q = contact_state.Q; delta = contact_state.delta; h = film_state.h_m;
if ~isequal(size(Q), size(delta)) || ~isequal(size(Q), size(h))
    error('StaticFrictionPower:Dimensions', 'Q, delta and h_m must have identical contact-array dimensions.');
end
[rolling_diameter, alpha, contact_length] = bearing_kinematics_input(bearing, contact_state);
if ~isfinite(bearing.Dm) || bearing.Dm <= 0 || rolling_diameter <= 0 || contact_length <= 0
    error('StaticFrictionPower:Geometry', 'compute_static_friction_power requires finite positive Dm, rolling diameter and contact length.');
end
[omega_c, omega_element] = standard_rolling_kinematics(bearing.type, Omega_rad_s, bearing.Dm, rolling_diameter, alpha);
[v_inner, v_outer] = two_sided_slip_speed(Omega_rad_s, omega_c, omega_element, bearing.Dm, rolling_diameter, alpha);
loaded_mask = Q > 0 & delta > 0 & h > 0;
if any(~isfinite(Q(loaded_mask))) || any(~isfinite(delta(loaded_mask))) || any(~isfinite(h(loaded_mask)))
    error('StaticFrictionPower:NonFiniteContact', 'compute_static_friction_power found non-finite Q, delta or h_m in a loaded contact.');
end
area = contact_area(bearing.type, rolling_diameter/2, contact_length, h);
F_drag_inner = zeros(size(Q)); F_drag_outer = zeros(size(Q));
F_drag_inner(loaded_mask) = eta_Pa_s*area(loaded_mask)*v_inner./h(loaded_mask);
F_drag_outer(loaded_mask) = eta_Pa_s*area(loaded_mask)*v_outer./h(loaded_mask);
P_inner = zeros(size(Q)); P_outer = zeros(size(Q));
P_inner(loaded_mask) = abs(F_drag_inner(loaded_mask)*v_inner);
P_outer(loaded_mask) = abs(F_drag_outer(loaded_mask)*v_outer);
if any(~isfinite(P_inner(loaded_mask))) || any(~isfinite(P_outer(loaded_mask)))
    error('StaticFrictionPower:NonFinitePower', 'compute_static_friction_power produced non-finite drag power.');
end
power = struct('Q_fric_W', sum(P_inner(:)) + sum(P_outer(:)), ...
    'Q_fric_inner_W', sum(P_inner(:)), 'Q_fric_outer_W', sum(P_outer(:)), ...
    'loaded_mask', loaded_mask, 'loaded_count', nnz(loaded_mask), ...
    'contact_area_m2', area, 'F_drag_inner_N', F_drag_inner, ...
    'F_drag_outer_N', F_drag_outer, 'v_slip_inner_mps', v_inner, ...
    'v_slip_outer_mps', v_outer, 'omega_c_rad_s', omega_c, ...
    'omega_element_rad_s', omega_element, 'valid', true);
end

function [diameter, alpha, length] = bearing_kinematics_input(bearing, contact_state)
switch lower(bearing.type)
    case 'ball'
        diameter = bearing.Db; length = bearing.Db;
        if isfield(contact_state, 'contact_angle') && isfinite(contact_state.contact_angle)
            alpha = contact_state.contact_angle;
        else
            alpha = bearing.assembly.contact_angle0;
        end
    case 'roller'
        diameter = bearing.Dw; alpha = 0; length = bearing.L;
    otherwise
        error('StaticFrictionPower:BearingType', 'Unsupported bearing type: %s.', bearing.type);
end
end

function [omega_c, omega_element] = standard_rolling_kinematics(type, Omega, Dm, d, alpha)
switch lower(type)
    case 'ball'
        ratio = d/Dm*cos(alpha);
        omega_c = 0.5*Omega*(1 - ratio);
        omega_element = Dm/(2*d)*(1 - ratio^2)*Omega;
    case 'roller'
        ratio = d/Dm;
        omega_c = 0.5*Omega*(1 - ratio);
        omega_element = Dm/(2*d)*(1 - ratio^2)*Omega;
    otherwise
        error('StaticFrictionPower:BearingType', 'Unsupported bearing type: %s.', type);
end
end

function [v_inner, v_outer] = two_sided_slip_speed(Omega, omega_c, omega_element, Dm, d, alpha)
v_cage = omega_c*Dm/2;
v_roll_inner = v_cage + omega_element*d/2;
v_roll_outer = v_cage - omega_element*d/2;
v_race_inner = Omega*(Dm - d*cos(alpha))/2;
v_race_outer = 0;
v_inner = abs(v_race_inner - v_roll_inner);
v_outer = abs(v_roll_outer - v_race_outer);
end

function area = contact_area(type, R_effective, contact_length, h)
switch lower(type)
    case 'ball'
        area = pi*R_effective*h;
    case 'roller'
        half_width = sqrt(R_effective*h);
        area = 2*half_width*contact_length;
    otherwise
        error('StaticFrictionPower:BearingType', 'Unsupported bearing type: %s.', type);
end
end
