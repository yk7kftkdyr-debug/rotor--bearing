function params = bearing_stage4A_config(params)
%BEARING_STAGE4A_CONFIG Stage4-A parameter provenance; no unknown preload is inferred.
params.bearing_model_stage = '5DOF_static_interface'; params.stage4A.enable = true; params.stage4A.static_physical_pass = false; params.stage4A.dynamic_allowed = false;
params.stage4B = struct('enable',false,'mode','B0_zero_dynamic_load','local_dof_count',5);
params.stage4A.axial_location = struct('enabled',true,'rotor_node',2,'case_node',2,'clearance_m',0.020e-3,'constraint','uz_rotor_minus_uz_case_equals_zero','front_role','locating','rear_role','floating');
params.stage4A.confirmed_parameters.front_ball = struct('contact_angle_deg',14,'ball_number',22,'ball_diameter_m',23.812e-3,'groove_curvature_inner',0.515,'groove_curvature_outer',0.515,'pitch_diameter_m',0.190,'radial_clearance_m',0.175e-3,'axial_clearance_m',0.040e-3,'width_m',0.038);
params.stage4A.confirmed_parameters.rear_roller = struct('radial_clearance_m',0.195e-3,'axial_clearance_m',0.040e-3,'circumferential_clearance_m',0.300e-3,'roller_number',30,'roller_diameter_m',0.015,'roller_length_m',0.015,'width_m',0.028);
params.stage4A.engineering_assumption_parameters = struct('front_preload_displacement_m',0,'front_axial_force_N',0,'rear_type','NU_equivalent','rear_axial_mode','floating','roller_crowning','unknown','roller_slice_count',9,'roller_slice_validation_count',13);
params.stage4A.future_required_parameters = struct('actual_preload_displacement',[],'actual_axial_force',[],'axial_force_direction',[],'rotor_force_application_node',[],'front_bearing_pairing',[],'rear_bearing_type',[],'roller_profile',[]);
params.thermal_model = false; params.lubrication_model = false; params.roughness_model = false; params.inclusion_model = false;
for k=1:numel(params.bearing)
    params.bearing(k).temperature = []; params.bearing(k).viscosity = []; params.bearing(k).roughness = struct('enable',false); params.bearing(k).inclusion_state = struct('enable',false); params.bearing(k).lubrication_state = struct('enable',false); params.bearing(k).include_oil_damping_force = false; params.bearing(k).include_oil_stiffness_force = false; params.bearing(k).ehl_load_correction_enable = false;
end
params.bearing(1).assembly.preload_mode = 'unknown_not_calibrated'; params.bearing(1).assembly.preload_displacement = 0; params.bearing(1).assembly.radial_clearance = 0.175e-3; params.bearing(1).assembly.axial_clearance = 0.040e-3; params.bearing(1).assembly.contact_angle0 = 14*pi/180; params.bearing(1).assembly.contact_angle_max = 32*pi/180; params.bearing(1).stage4A_width_m = 0.038;
params.bearing(2).assembly.preload_mode = 'not_applicable_floating'; params.bearing(2).assembly.preload_displacement = 0; params.bearing(2).assembly.radial_clearance = 0.195e-3; params.bearing(2).assembly.axial_clearance = 0.040e-3; params.bearing(2).assembly.circumferential_clearance = 0.300e-3; params.bearing(2).axial_mode = 'floating'; params.bearing(2).stage4A_slice_count = 9; params.bearing(2).stage4A_slice_validation_count = 13;
end
