function cfg = build_equivalent_damping_frequency_domain_config()
%BUILD_EQUIVALENT_DAMPING_FREQUENCY_DOMAIN_CONFIG Stage A mainline contract.

cfg.mainline_name = ...
    'thermal_equivalent_damping_frequency_domain';

cfg.analysis_type = ...
    'FULL_ORDER_1X_FREQUENCY_DOMAIN_WITH_CONTACT_NONLINEARITY_GATE';

cfg.damping_model = 'fixed_equivalent_system';
cfg.legacy_damping_mode = 'audit_only';

cfg.dynamic_ehl_used = false;
cfg.old_ehl_used_in_formal_response = false;
cfg.newmark_used = false;

cfg.temperature_cases_C = [20, 50, 80, 100];

cfg.rotational_speed_rpm = 9900;
cfg.rotational_speed_rad_s = 2*pi*cfg.rotational_speed_rpm/60;
cfg.frequency_1X_Hz = cfg.rotational_speed_rpm/60;
cfg.omega_1X_rad_s = 2*pi*cfg.frequency_1X_Hz;

cfg.damping_scenarios.names = {'LOW','NOMINAL','HIGH'};
cfg.damping_scenarios.target_zeta = [0.005, 0.010, 0.020];
cfg.damping_reference_temperature_C = 20;
cfg.freeze_formal_damping_across_temperatures = true;

cfg.unbalance.rotor_node = 16;
cfg.unbalance.mass_kg = 1e-3;
cfg.unbalance.eccentricity_m = 1e-3;
cfg.unbalance.phase_rad = 0;
cfg.unbalance.unbalance_kg_m = ...
    cfg.unbalance.mass_kg*cfg.unbalance.eccentricity_m;

cfg.nonlinearity_gate.phase_count = 32;
cfg.nonlinearity_gate.strong_ratio = 0.10;
cfg.nonlinearity_gate.caution_ratio = 0.20;
cfg.nonlinearity_gate.error_limit = 0.10;
cfg.nonlinearity_gate.harmonic_ratio_limit = 0.10;

cfg.result_directory = ...
    fullfile('results', ...
    'thermal_equivalent_damping_frequency_domain');

cfg.result_mat_name = ...
    'thermal_4T_frequency_domain_results.mat';

cfg.result_txt_name = ...
    'thermal_4T_frequency_domain_summary.txt';

cfg.mapping_validation_required = true;
end
