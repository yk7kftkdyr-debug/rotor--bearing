function property = thermal_lubricant_properties(T_film_C)
%THERMAL_LUBRICANT_PROPERTIES Single reduced lubricant property mapping.
validateattributes(T_film_C, {'numeric'}, {'scalar','real','finite'});
property.nu_cSt = 27.6*exp(-0.02815*(T_film_C - 40));
property.eta_Pa_s = 1003.5*property.nu_cSt*1e-6;
property.alpha_p_Pa_inv = 1.28e-8*exp(-0.010*(T_film_C - 20));
end
