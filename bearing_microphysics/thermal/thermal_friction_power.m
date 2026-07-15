function friction = thermal_friction_power(brg, omega_rad_s, raw_contact, eta_Pa_s, film_thickness_m, local)
%THERMAL_FRICTION_POWER Local algebraic drag/slip power for loaded contacts.
%   This reduced thermo-viscous-contact model has no cage ODE or history.

validateattributes(omega_rad_s, {'numeric'}, {'real', 'finite', 'scalar'});
validateattributes(eta_Pa_s, {'numeric'}, {'real', 'finite', 'scalar', 'nonnegative'});
required = {'Q', 'delta', 'element_angle', 'contact_angle'};
assert(all(isfield(raw_contact, required)), ...
    'thermal_friction_power:IncompleteRawContact', 'raw_contact lacks required local fields.');
Q_N = raw_contact.Q;
delta_m = raw_contact.delta;
assert(isequal(size(Q_N), size(delta_m), size(film_thickness_m)), ...
    'thermal_friction_power:ContactGridSize', 'Q, delta and film_thickness must share one contact grid.');

[cage_speed, element_spin_speed, element_diameter_m] = ideal_speeds(brg, omega_rad_s, raw_contact.contact_angle);
entrainment_speed = abs(omega_rad_s) * brg.Dm / 4;
element_angle = raw_contact.element_angle;
local_tangential_velocity = local_contact_velocity(local, element_angle, size(Q_N));
surface_sliding_velocity = abs(omega_rad_s) * brg.Dm / 2 - ...
    cage_speed * brg.Dm / 2 - element_spin_speed * element_diameter_m / 2;
sliding_velocity = abs(local_tangential_velocity + surface_sliding_velocity);

loaded = Q_N > 0 & delta_m > 0 & film_thickness_m > 0;
contact_area_m2 = local_contact_area(brg, film_thickness_m, element_diameter_m);
drag_force = zeros(size(Q_N));
drag_force(loaded) = eta_Pa_s * entrainment_speed .* ...
    contact_area_m2(loaded) ./ film_thickness_m(loaded);
valid_loaded_mask = loaded & isfinite(drag_force) & isfinite(sliding_velocity);
Q_fric_W = sum(abs(drag_force(valid_loaded_mask)) .* ...
    sliding_velocity(valid_loaded_mask));

friction = struct('drag_force', drag_force, ...
    'sliding_velocity', sliding_velocity, ...
    'valid_loaded_mask', valid_loaded_mask, 'Q_fric_W', Q_fric_W, ...
    'cage_speed', cage_speed, 'element_spin_speed', element_spin_speed, ...
    'entrainment_speed', entrainment_speed);
end

function [cage_speed, element_spin_speed, diameter_m] = ideal_speeds(brg, omega_rad_s, contact_angle)
if strcmpi(brg.type, 'ball')
    diameter_m = brg.Db;
    cage_speed = 0.5 * omega_rad_s * (1 - brg.Db / brg.Dm * cos(contact_angle));
    element_spin_speed = brg.Dm / (2 * brg.Db) * ...
        (1 - (brg.Db * cos(contact_angle) / brg.Dm)^2) * omega_rad_s;
elseif strcmpi(brg.type, 'roller')
    diameter_m = brg.Dw;
    cage_speed = 0.5 * omega_rad_s * (1 - brg.Dw / brg.Dm);
    element_spin_speed = brg.Dm / (2 * brg.Dw) * ...
        (1 - (brg.Dw / brg.Dm)^2) * omega_rad_s;
else
    error('thermal_friction_power:UnknownBearingType', 'Unknown bearing type: %s.', brg.type);
end
end

function tangential = local_contact_velocity(local, element_angle, contact_size)
assert(isfield(local, 'rdot') && numel(local.rdot) >= 2, ...
    'thermal_friction_power:MissingLocalVelocity', 'local.rdot must contain local x/y velocity.');
v_tangent = -local.rdot(1) .* sin(element_angle) + local.rdot(2) .* cos(element_angle);
if isvector(v_tangent) && contact_size(1) > 1
    tangential = repmat(v_tangent(:), 1, contact_size(2));
else
    tangential = reshape(v_tangent, contact_size);
end
end

function area_m2 = local_contact_area(brg, film_thickness_m, diameter_m)
half_width_m = sqrt(diameter_m .* film_thickness_m);
if strcmpi(brg.type, 'ball')
    area_m2 = pi .* half_width_m.^2;
else
    area_m2 = 2 * brg.L .* half_width_m;
end
end
