function Fu_global = unbalance_force(t, params, N_dof)
%UNBALANCE_FORCE Assemble rotating unbalance force.
% Inputs:
%   t      - time, s.
%   params - structure containing omega and unbalance fields.
%   N_dof  - optional global DOF count.
% Output:
%   Fu_global - global unbalance force vector, N.

if nargin < 3 || isempty(N_dof)
    N_dof = params.modelInfo.num_rotor_dof + params.modelInfo.num_case_dof;
end

omega = params.omega;
unbalance = params.unbalance;
if ~isfield(unbalance, 'phase') || isempty(unbalance.phase)
    unbalance.phase = zeros(size(unbalance.nodes));
end
Fu_global = zeros(N_dof, 1);

for k = 1:numel(unbalance.nodes)
    nd = unbalance.nodes(k);
    me = unbalance.mass_g(k)*1e-3;
    ecc = unbalance.ecc_mm(k)*1e-3;
    phase = unbalance.phase(k);
    F0 = me*ecc*omega^2;
    ix = 4*nd - 3;
    iy = 4*nd - 2;
    Fu_global(ix) = Fu_global(ix) + F0*cos(omega*t + phase);
    Fu_global(iy) = Fu_global(iy) + F0*sin(omega*t + phase);
end

end
