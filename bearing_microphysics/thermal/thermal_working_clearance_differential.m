function clearance = thermal_working_clearance_differential(bearing_type, T_inner_C, T_outer_C, T_element_C, c_ref_m, geometry)
%THERMAL_WORKING_CLEARANCE_DIFFERENTIAL Differential thermal radial clearance.
validateattributes(T_inner_C, {'numeric'}, {'scalar','real','finite'});
validateattributes(T_outer_C, {'numeric'}, {'scalar','real','finite'});
validateattributes(T_element_C, {'numeric'}, {'scalar','real','finite'});
validateattributes(c_ref_m, {'numeric'}, {'scalar','real','finite'});
required = {'Dm','D_element'};
if ~isstruct(geometry) || ~all(isfield(geometry, required)), error('Thermal clearance geometry requires Dm and D_element.'); end
validateattributes(geometry.Dm, {'numeric'}, {'scalar','positive','finite'});
validateattributes(geometry.D_element, {'numeric'}, {'scalar','positive','finite'});
switch lower(bearing_type)
    case 'ball'
        if ~isfield(geometry, 'alpha0'), error('Ball thermal clearance geometry requires alpha0.'); end
        validateattributes(geometry.alpha0, {'numeric'}, {'scalar','real','finite'});
        Dri = geometry.Dm - geometry.D_element*cos(geometry.alpha0);
    case 'roller'
        Dri = geometry.Dm - geometry.D_element;
    otherwise
        error('Unsupported bearing type: %s.', bearing_type);
end
if Dri <= 0, error('Thermal clearance geometry has nonpositive inner-race diameter.'); end
clearance_reduction = 11e-6*(Dri*(T_inner_C - T_outer_C) + 2*geometry.D_element*(T_element_C - T_outer_C));
clearance = struct('c_work_m', c_ref_m - clearance_reduction, 'thermal_preload', c_ref_m - clearance_reduction < 0);
end
