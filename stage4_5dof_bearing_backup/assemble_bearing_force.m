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

% Stage 3 compatibility: 2-D bearing force acts only on ux/uy of the
% 6-DOF structure [ux uy uz theta_x theta_y theta_z].
irx = 6*rotor_node - 5;
iry = 6*rotor_node - 4;
icx = num_rotor + 6*case_node - 5;
icy = num_rotor + 6*case_node - 4;

F(irx) = F(irx) + Fx;
F(iry) = F(iry) + Fy;
F(icx) = F(icx) - Fx;
F(icy) = F(icy) - Fy;

end
