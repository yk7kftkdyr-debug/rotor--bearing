function F = assemble_bearing_force(F, Fx, Fy, rotor_node, case_node, num_rotor)
%ASSEMBLE_BEARING_FORCE Assemble bearing action and reaction.
% Inputs:
%   F          - global force vector to be updated.
%   Fx, Fy     - feedback force acting on rotor node, N; the case receives
%                the equal and opposite reaction force.
%   rotor_node - rotor bearing node number.
%   case_node  - case bearing seat node number.
%   num_rotor  - number of rotor DOFs.
% Output:
%   F - updated global force vector.

irx = 4*rotor_node - 3;
iry = 4*rotor_node - 2;
icx = num_rotor + 4*case_node - 3;
icy = num_rotor + 4*case_node - 2;

F(irx) = F(irx) + Fx;
F(iry) = F(iry) + Fy;
F(icx) = F(icx) - Fx;
F(icy) = F(icy) - Fy;

end
