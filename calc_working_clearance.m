function [c_work, clearInfo] = calc_working_clearance(bearing, omega)
%CALC_WORKING_CLEARANCE Calculate working radial clearance.
% Inputs:
%   bearing - bearing parameter structure.
%   omega   - shaft speed, rad/s.
% Outputs:
%   c_work    - working clearance, m.
%   clearInfo - component contributions.
%
% c_work = c0 + clearance_fit + clearance_temp +
%          clearance_centrifugal + clearance_other

if nargin < 2
    omega = 0;
end

assembly = get_field_default(bearing, 'assembly_state', struct());
c0 = get_field_default(assembly, 'radial_clearance', get_field_default(bearing, 'clearance0', 0));
delta_fit = get_field_default(assembly, 'clearance_change_fit', get_field_default(bearing, 'clearance_fit', get_field_default(bearing, 'delta_fit', 0)));
delta_thermal = get_field_default(assembly, 'clearance_change_thermal', get_field_default(bearing, 'clearance_temp', get_field_default(bearing, 'delta_thermal', 0)));
initial_interference = get_field_default(assembly, 'initial_interference', 0);
delta_centrifugal = get_field_default(bearing, 'clearance_centrifugal', get_field_default(bearing, 'delta_centrifugal', 0));
delta_other = get_field_default(bearing, 'clearance_other', 0);

if isfield(bearing, 'thermal') && get_field_default(bearing.thermal, 'enable', false)
    T_ref = get_field_default(bearing.thermal, 'T_ref', 20);
    T_inner = get_field_default(bearing.thermal, 'T_inner', T_ref);
    T_outer = get_field_default(bearing.thermal, 'T_outer', T_ref);
    alpha_inner = get_field_default(bearing.thermal, 'alpha_inner', 11.5e-6);
    alpha_outer = get_field_default(bearing.thermal, 'alpha_outer', 11.5e-6);
    R_inner = get_field_default(bearing, 'R_inner', get_field_default(bearing, 'Dm', 0)/2);
    R_outer = get_field_default(bearing, 'R_outer', get_field_default(bearing, 'Dm', 0)/2);
    delta_r_inner = alpha_inner*R_inner*(T_inner - T_ref);
    delta_r_outer = alpha_outer*R_outer*(T_outer - T_ref);
    delta_thermal = delta_thermal + delta_r_outer - delta_r_inner;
else
    delta_r_inner = 0;
    delta_r_outer = 0;
    T_ref = 20;
    T_inner = T_ref;
    T_outer = T_ref;
end

% Optional compact centrifugal estimate if user provides geometry data.
if ~isfield(bearing, 'delta_centrifugal') && isfield(bearing, 'Dm')
    rho = get_field_default(bearing, 'ring_density', 7850);
    E = get_field_default(bearing, 'ring_E', 2.1e11);
    r = bearing.Dm/2;
    delta_centrifugal = 2*rho*r^3*omega^2/E;
end

c_work = c0 + delta_fit + delta_thermal + delta_centrifugal + delta_other - initial_interference;

clearInfo.c0 = c0;
clearInfo.delta_fit = delta_fit;
clearInfo.delta_thermal = delta_thermal;
clearInfo.delta_centrifugal = delta_centrifugal;
clearInfo.delta_other = delta_other;
clearInfo.initial_interference = initial_interference;
clearInfo.delta_r_inner_thermal = delta_r_inner;
clearInfo.delta_r_outer_thermal = delta_r_outer;
clearInfo.T_ref = T_ref;
clearInfo.T_inner = T_inner;
clearInfo.T_outer = T_outer;
clearInfo.deltaT_io = T_inner - T_outer;
clearInfo.c_work = c_work;

end

function v = get_field_default(s, field, default_value)
if isfield(s, field) && ~isempty(s.(field))
    v = s.(field);
else
    v = default_value;
end
end
