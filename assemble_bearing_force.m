function F = assemble_bearing_force(F, force_or_fx, varargin)
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

if numel(force_or_fx)==5
    f=force_or_fx(:); rotor_node=varargin{1}; case_node=varargin{2}; num_rotor=varargin{3}; ir=6*rotor_node+(-5:-1); ic=num_rotor+6*case_node+(-5:-1); F(ir)=F(ir)+f; F(ic)=F(ic)-f;
else
    Fx=force_or_fx; Fy=varargin{1}; rotor_node=varargin{2}; case_node=varargin{3}; num_rotor=varargin{4}; irx=6*rotor_node-5; iry=irx+1; icx=num_rotor+6*case_node-5; icy=icx+1; F(irx)=F(irx)+Fx; F(iry)=F(iry)+Fy; F(icx)=F(icx)-Fx; F(icy)=F(icy)-Fy;
end

end
