function temperature = thermal_component_temperatures(T_oil_C, T_final_C)
%THERMAL_COMPONENT_TEMPERATURES Deterministic bearing component temperatures.
validateattributes(T_oil_C, {'numeric'}, {'scalar','real','finite'});
validateattributes(T_final_C, {'numeric'}, {'scalar','real','finite'});
deltaT_mean = T_final_C - T_oil_C;
deltaT_io = 0.5*deltaT_mean;
temperature = struct('T_inner_C', T_final_C + 0.5*deltaT_io, ...
    'T_outer_C', T_final_C - 0.5*deltaT_io, 'T_element_C', T_final_C, ...
    'T_film_C', 0.5*T_oil_C + 0.5*T_final_C);
end
