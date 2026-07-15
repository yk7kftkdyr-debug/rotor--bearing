function [working_clearance_m, state] = thermal_working_clearance(brg, temperature_C, oil_temperature_C, clearance_temperature_coefficient_m_per_C)
%THERMAL_WORKING_CLEARANCE Local thermal clearance referenced to this bearing.

if nargin < 3 || isempty(oil_temperature_C), oil_temperature_C = 20; end
if nargin < 4 || isempty(clearance_temperature_coefficient_m_per_C)
    clearance_temperature_coefficient_m_per_C = 1.5e-6;
end
validateattributes(temperature_C, {'numeric'}, {'real', 'finite'});
validateattributes(oil_temperature_C, {'numeric'}, {'real', 'finite', 'scalar'});
validateattributes(clearance_temperature_coefficient_m_per_C, {'numeric'}, {'real', 'finite', 'scalar'});
assert(isfield(brg, 'assembly') && isfield(brg.assembly, 'radial_clearance'), ...
    'thermal_working_clearance:MissingReferenceClearance', ...
    'brg.assembly.radial_clearance is required.');

c_reference_m = brg.assembly.radial_clearance;
validateattributes(c_reference_m, {'numeric'}, {'real', 'finite', 'scalar', 'nonnegative'});
working_clearance_m = c_reference_m + clearance_temperature_coefficient_m_per_C .* ...
    (temperature_C - oil_temperature_C);
state = struct('c_reference_m', c_reference_m, ...
    'oil_temperature_C', oil_temperature_C, ...
    'clearance_temperature_coefficient_m_per_C', clearance_temperature_coefficient_m_per_C);
end
