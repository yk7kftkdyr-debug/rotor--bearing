function local = bearing_relative_state(q, qd, bearing, num_rotor)
%BEARING_RELATIVE_STATE Relative rotor-to-case motion at one bearing seat.

irx = 4*bearing.rotor_node - 3; iry = 4*bearing.rotor_node - 2;
icx = num_rotor + 4*bearing.case_node - 3; icy = num_rotor + 4*bearing.case_node - 2;
local.xr = q(irx) - q(icx); local.yr = q(iry) - q(icy);
local.vxr = qd(irx) - qd(icx); local.vyr = qd(iry) - qd(icy);
end
