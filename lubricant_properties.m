function oil = lubricant_properties(T, params)
%LUBRICANT_PROPERTIES Mobil Jet Oil II equivalent oil properties.
% Inputs:
%   T      - oil temperature, degC.
%   params - global parameter structure with params.lubricant.
% Output:
%   oil.nu_cSt  - kinematic viscosity, cSt.
%   oil.eta     - dynamic viscosity, Pa*s.
%   oil.alpha_p - pressure-viscosity coefficient, Pa^-1.
%   oil.rho     - density used in the present sensitivity model, kg/m^3.
%
% The viscosity model is fitted by 40 degC and 100 degC kinematic
% viscosities. Density is kept constant in this small-scope engineering
% sensitivity model.

if nargin < 2 || ~isfield(params, 'lubricant')
    params.lubricant = default_lubricant();
end

lub = params.lubricant;
nu40 = get_field_default(lub, 'nu40_cSt', 27.6);
nu100 = get_field_default(lub, 'nu100_cSt', 5.1);
rho = get_field_default(lub, 'rho15', 1003.5);
alpha_p40 = get_field_default(lub, 'alpha_p40', 1.20e-8);
k_alpha = get_field_default(lub, 'k_alpha', 0.008);

k_eta = log(nu40/nu100)/(100 - 40);
nu_cSt = nu40*exp(-k_eta*(T - 40));
eta = nu_cSt*1e-6*rho;
alpha_p = alpha_p40*exp(-k_alpha*(T - 40));

oil.name = get_field_default(lub, 'name', 'Mobil Jet Oil II equivalent 5 cSt synthetic ester turbine oil');
oil.T = T;
oil.nu_cSt = nu_cSt;
oil.eta = eta;
oil.alpha_p = alpha_p;
oil.rho = rho;
oil.k_eta = k_eta;
oil.k_alpha = k_alpha;
oil.nu40_cSt = nu40;
oil.nu100_cSt = nu100;
oil.alpha_p40 = alpha_p40;
end

function lub = default_lubricant()
lub.name = 'Mobil Jet Oil II equivalent 5 cSt synthetic ester turbine oil';
lub.nu40_cSt = 27.6;
lub.nu100_cSt = 5.1;
lub.rho15 = 1003.5;
lub.T_ref_visc = 40;
lub.alpha_p40 = 1.20e-8;
lub.k_alpha = 0.008;
end

function v = get_field_default(s, field, default_value)
if isfield(s, field) && ~isempty(s.(field))
    v = s.(field);
else
    v = default_value;
end
end
