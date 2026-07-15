function properties = thermal_lubricant_properties(temperature_C)
%THERMAL_LUBRICANT_PROPERTIES Deterministic local lubricant properties.
%   This stage supplies temperature-dependent properties only; it does not
%   perform a thermal balance, store history, or read an operating case.

validateattributes(temperature_C, {'numeric'}, {'real', 'finite'});
properties.temperature_C = temperature_C;
properties.rho_oil_kg_m3 = 1003.5;
properties.nu_cSt = 27.6 .* exp(-0.02815 .* (temperature_C - 40));
properties.eta_Pa_s = properties.rho_oil_kg_m3 .* properties.nu_cSt .* 1e-6;
properties.alpha_p_Pa_inv = 1.28e-8 .* exp(-0.010 .* (temperature_C - 20));
end
