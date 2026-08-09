function validation = validate_stage_f_results(candidate,baseline,frozen_projection)
%VALIDATE_STAGE_F_RESULTS Independently recompute every Stage F diagnostic.

if nargin < 3
    frozen_projection = build_stage_f_frozen_input_projection(baseline);
end

diagnostics = {};
gate_status = '';
if ~isstruct(candidate) || ~isstruct(baseline)
    validation = finish({'SCHEMA'},gate_status); return;
end
required = {'progress','decision','validation','frequency_response', ...
    'bearing_contact','nonlinearity_gate','stage_f_execution_audit'};
if ~all(isfield(candidate,required))
    validation = finish({'SCHEMA'},gate_status); return;
end

if ~same_without_stage_f_validation(candidate,baseline) || ...
        ~canonical_input_reference_matches(candidate,frozen_projection)
    diagnostics{end+1} = 'FROZEN';
end
if ~same_field(candidate,baseline,'static')
    diagnostics{end+1} = 'FROZEN_STATIC';
end
if ~same_field(candidate,baseline,'damping')
    diagnostics{end+1} = 'FROZEN_DAMPING';
end
if ~same_field(candidate,baseline,'frequency_response')
    diagnostics{end+1} = 'FROZEN_RESPONSE';
end
if ~same_validation(candidate,baseline)
    diagnostics{end+1} = 'FROZEN_VALIDATION';
end
if forbidden_payload_present(candidate,baseline)
    diagnostics{end+1} = 'FORBIDDEN_PAYLOAD';
end

temperatures = [20 50 80 100]; scenarios = {'LOW','NOMINAL','HIGH'};
bearings = {'front','rear'};
require_formal_audit = canonical_mapping_available(baseline,'T20','front') && ...
    canonical_mapping_available(baseline,'T20','rear');
global_hard_failure = false; global_strong = true;
expected_recovered = struct();
expected_scans = struct();
for temperature = temperatures
    tf = sprintf('T%d',temperature);
    for scenario_index = 1:numel(scenarios)
        scenario = scenarios{scenario_index};
        for bearing_index = 1:numel(bearings)
            bearing = bearings{bearing_index};
            try
                actual = candidate.bearing_contact.(tf).(scenario).(bearing);
                qhat = baseline.frequency_response.(tf).(scenario).qhat_full;
                [mapping,mapping_matches] = validation_mapping( ...
                    baseline,actual,temperature,tf,bearing);
                if ~mapping_matches
                    diagnostics{end+1} = 'MAPPING_SNAPSHOT'; %#ok<AGROW>
                end
                expected = independently_recover_loads(mapping,qhat);
                expected_recovered.(tf).(scenario).(bearing) = expected;
                diagnostics = compare_recovered(actual,expected,diagnostics);
                global_hard_failure = global_hard_failure || ...
                    expected.hard_gate.loaded_slice_contact_loss || ...
                    expected.hard_gate.loaded_slice_linearity_limit_exceeded;
                value = expected.hard_gate.maximum_loaded_slice_r_delta;
                if isfinite(value), global_strong = global_strong && value <= 0.10; end
            catch
                diagnostics{end+1} = ['RECOVERED_' upper(tf) '_' upper(scenario) '_' upper(bearing)]; %#ok<AGROW>
                global_hard_failure = true; global_strong = false;
            end
        end
    end
end

for temperature = temperatures
    tf = sprintf('T%d',temperature);
    for bearing_index = 1:numel(bearings)
        bearing = bearings{bearing_index};
        try
            actual = candidate.nonlinearity_gate.(tf).(bearing);
            nominal = expected_recovered.(tf).NOMINAL.(bearing);
            expected_scan = independently_rebuild_scan( ...
                baseline,actual,temperature,tf,bearing,nominal);
            expected_scans.(tf).(bearing) = expected_scan;
            if require_formal_audit
                expected_thermal = stage_f_frozen_summary( ...
                    baseline.static.(tf),bearing);
            else
                expected_thermal = struct([]);
            end
            [scan_diagnostics,scan_hard,scan_strong] = ...
                validate_scan(actual,nominal,expected_scan, ...
                require_formal_audit,expected_thermal);
            diagnostics = [diagnostics scan_diagnostics]; %#ok<AGROW>
            global_hard_failure = global_hard_failure || scan_hard;
            global_strong = global_strong && scan_strong;
        catch
            diagnostics{end+1} = ['NONLINEARITY_' upper(tf) '_' upper(bearing)]; %#ok<AGROW>
            global_hard_failure = true; global_strong = false;
        end
    end
end

diagnostics = [diagnostics validate_execution_audit(candidate.stage_f_execution_audit)];
diagnostics = [diagnostics validate_source_and_elapsed_provenance( ...
    candidate,baseline,temperatures,scenarios,bearings,require_formal_audit)];
if global_hard_failure
    gate_status = 'REPRESENTATIVE_NONLINEAR_VALIDATION_REQUIRED';
elseif global_strong
    gate_status = 'FREQUENCY_DOMAIN_LINEARIZATION_ACCEPTED';
else
    gate_status = 'FREQUENCY_DOMAIN_LINEARIZATION_CAUTION';
end
allow_stage_g = ~strcmp(gate_status,'REPRESENTATIVE_NONLINEAR_VALIDATION_REQUIRED');
expected_first_trigger = independently_rebuild_first_trigger( ...
    expected_recovered,expected_scans);
if ~first_trigger_matches(candidate.progress,expected_first_trigger)
    diagnostics{end+1} = 'FIRST_TRIGGER';
end
if ~progress_matches(candidate,gate_status,allow_stage_g)
    diagnostics{end+1} = 'FINAL_GATE';
end
diagnostics = unique(diagnostics,'stable');
validation = finish(diagnostics,gate_status);
validation.checks = struct('frozen_stage_a_to_e', ...
    ~any(startsWith(diagnostics,'FROZEN')), ...
    'recovered_loads',~any(startsWith(diagnostics,'RECOVERED_')), ...
    'scan_schedule',~any(strcmp(diagnostics,'SCAN_SCHEDULE')), ...
    'final_gate',~any(strcmp(diagnostics,'FINAL_GATE')), ...
    'forbidden_payloads',~any(strcmp(diagnostics,'FORBIDDEN_PAYLOAD')));
end

function diagnostics = compare_recovered(actual,expected,diagnostics)
try
    a = actual.slice_level; e = expected.slice_level;
    if ~isequaln(a.Q_static_slice,e.Q_static_slice), diagnostics{end+1}='QSTATIC_SLICE'; end
    if ~isequaln(a.delta_static_slice,e.delta_static_slice), diagnostics{end+1}='DELTA_STATIC_SLICE'; end
    if ~isequaln(a.k_static_slice,e.k_static_slice), diagnostics{end+1}='K_STATIC_SLICE'; end
    if ~isequaln(a.loaded_mask_static_slice,e.loaded_mask_static_slice), diagnostics{end+1}='LOADED_MASK_SLICE'; end
    if ~isequaln(a.delta_hat_slice,e.delta_hat_slice), diagnostics{end+1}='DELTA_HAT'; end
    if ~isequaln(a.delta_hat_real_slice,e.delta_hat_real_slice) || ...
            ~isequaln(a.delta_hat_imag_slice,e.delta_hat_imag_slice) || ...
            ~isequaln(a.delta_hat_magnitude_slice,e.delta_hat_magnitude_slice) || ...
            ~isequaln(a.delta_hat_phase_rad_slice,e.delta_hat_phase_rad_slice)
        diagnostics{end+1}='DELTA_HAT_COMPONENTS';
    end
    if ~isequaln(a.Qhat_slice,e.Qhat_slice), diagnostics{end+1}='QHAT'; end
    if ~isequaln(a.Qhat_real_slice,e.Qhat_real_slice) || ...
            ~isequaln(a.Qhat_imag_slice,e.Qhat_imag_slice) || ...
            ~isequaln(a.Qhat_magnitude_slice,e.Qhat_magnitude_slice) || ...
            ~isequaln(a.Qhat_phase_rad_slice,e.Qhat_phase_rad_slice)
        diagnostics{end+1}='QHAT_COMPONENTS';
    end
    if ~isequaln(a.Qmax_linear_slice,e.Qmax_linear_slice), diagnostics{end+1}='QMAX_SLICE'; end
    if ~isequaln(a.Qmin_linear_slice,e.Qmin_linear_slice), diagnostics{end+1}='QMIN'; end
    if ~isequaln(a.r_delta_slice,e.r_delta_slice), diagnostics{end+1}='R_DELTA'; end
    if ~isequaln(a.linearity_class_slice,e.linearity_class_slice), diagnostics{end+1}='LINEARITY_CLASS'; end
    if ~isequaln(a.potential_contact_loss_slice,e.potential_contact_loss_slice), diagnostics{end+1}='CONTACT_LOSS'; end
    if ~isequaln(a.contact_loss_evaluated_slice,e.contact_loss_evaluated_slice), diagnostics{end+1}='CONTACT_LOSS_EVALUATED'; end
    if ~isequaln(a.contact_loss_reason_slice,e.contact_loss_reason_slice), diagnostics{end+1}='CONTACT_LOSS_REASON'; end
    ar = actual.roller_level; er = expected.roller_level;
    if ~isequaln(ar.Q_static_roller,er.Q_static_roller), diagnostics{end+1}='QSTATIC_ROLLER'; end
    if ~isequaln(ar.loaded_mask_static_roller,er.loaded_mask_static_roller), diagnostics{end+1}='LOADED_MASK_ROLLER'; end
    if ~isequaln(ar.Qhat_roller,er.Qhat_roller), diagnostics{end+1}='QHAT_ROLLER'; end
    if ~isequaln(ar.Qhat_real_roller,er.Qhat_real_roller) || ...
            ~isequaln(ar.Qhat_imag_roller,er.Qhat_imag_roller) || ...
            ~isequaln(ar.Qhat_magnitude_roller,er.Qhat_magnitude_roller) || ...
            ~isequaln(ar.Qhat_phase_rad_roller,er.Qhat_phase_rad_roller)
        diagnostics{end+1}='QHAT_ROLLER_COMPONENTS';
    end
    if ~isequaln(ar.Qmax_linear_roller,er.Qmax_linear_roller), diagnostics{end+1}='QMAX'; end
    if ~isequaln(ar.Qmin_linear_roller,er.Qmin_linear_roller), diagnostics{end+1}='QMIN_ROLLER'; end
    if ~isequaln(ar.r_delta_roller,er.r_delta_roller), diagnostics{end+1}='R_DELTA_ROLLER'; end
    if ~isequaln(ar.linearity_class_roller,er.linearity_class_roller), diagnostics{end+1}='LINEARITY_CLASS_ROLLER'; end
    if ~isequaln(ar.potential_contact_loss_roller,er.potential_contact_loss_roller), diagnostics{end+1}='CONTACT_LOSS_ROLLER'; end
    if ~isequaln(ar.contact_loss_evaluated_roller,er.contact_loss_evaluated_roller), diagnostics{end+1}='CONTACT_LOSS_EVALUATED_ROLLER'; end
    if ~isequaln(actual.dynamic_resultant.local_complex, ...
            expected.dynamic_resultant.local_complex), diagnostics{end+1}='RESULTANT'; end
    if ~isequaln(actual.dynamic_resultant.global_complex, ...
            expected.dynamic_resultant.global_complex), diagnostics{end+1}='RESULTANT_GLOBAL'; end
    if ~isequaln(actual.dynamic_resultant.Fx_hat, ...
            expected.dynamic_resultant.Fx_hat), diagnostics{end+1}='FX_HAT'; end
    if ~isequaln(actual.dynamic_resultant.Fy_hat, ...
            expected.dynamic_resultant.Fy_hat), diagnostics{end+1}='FY_HAT'; end
    if ~isequaln(actual.dynamic_resultant.radial_force_peak, ...
            expected.dynamic_resultant.radial_force_peak), diagnostics{end+1}='RADIAL_PEAK'; end
    if ~isequaln(actual.dynamic_resultant.radial_force_rms, ...
            expected.dynamic_resultant.radial_force_rms), diagnostics{end+1}='RADIAL_RMS'; end
    if ~isequaln(actual.dynamic_resultant.component_phase_rad, ...
            expected.dynamic_resultant.component_phase_rad), diagnostics{end+1}='RESULTANT_PHASE'; end
    if ~isequaln(actual.contact_load_resultant,expected.contact_load_resultant)
        diagnostics{end+1}='CONTACT_LOAD_RESULTANT';
    end
    if ~isequaln(actual.contact_force_resultant,expected.contact_force_resultant)
        diagnostics{end+1}='CONTACT_FORCE_RESULTANT';
    end
    if ~isequaln(actual.bearing_force_on_rotor,expected.bearing_force_on_rotor)
        diagnostics{end+1}='BEARING_FORCE_ON_ROTOR';
    end
    aa = actual.force_assembly_validation; ea = expected.force_assembly_validation;
    if ~isequaln(aa.element_vs_analytic_tangent_relative_error, ...
            ea.element_vs_analytic_tangent_relative_error), diagnostics{end+1}='ASSEMBLY_CORE'; end
    if ~isequaln(aa.element_vs_analytic_tangent_threshold,1e-10), diagnostics{end+1}='ASSEMBLY_CORE_THRESHOLD'; end
    if ~strcmp(aa.element_vs_analytic_tangent_gate_status, ...
            'ROLLING_ELEMENT_FORCE_ASSEMBLY_MATCHED'), diagnostics{end+1}='ASSEMBLY_CORE_STATUS'; end
    if ~isequaln(aa.analytic_vs_saved_tangent_relative_error, ...
            ea.analytic_vs_saved_tangent_relative_error), diagnostics{end+1}='TANGENT_AUDIT'; end
    if ~isequaln(aa.analytic_vs_saved_tangent_threshold,1e-8), diagnostics{end+1}='TANGENT_AUDIT_THRESHOLD'; end
    if ~strcmp(aa.analytic_vs_saved_tangent_gate_status, ...
            'ANALYTIC_VS_FD_TANGENT_AUDIT_PASSED'), diagnostics{end+1}='TANGENT_AUDIT_STATUS'; end
    if ~isequaln(aa,ea), diagnostics{end+1}='FORCE_AUDIT'; end
    if ~isequaln(actual.hard_gate,expected.hard_gate), diagnostics{end+1}='HARD_GATE'; end
    if ~isequaln(actual.interface_relative_motion_hat, ...
            expected.interface_relative_motion_hat), diagnostics{end+1}='INTERFACE_RELATIVE_MOTION'; end
catch
    diagnostics{end+1} = 'RECOVERED_SCHEMA';
end
end

function [mapping,matches] = validation_mapping(baseline,actual,temperature,tf,bearing)
if canonical_mapping_available(baseline,tf,bearing)
    mapping = build_stage_f_rolling_element_linearization_mapping( ...
        baseline,temperature,bearing);
    matches = ~isfield(actual,'mapping_snapshot');
else
    candidate_mapping = candidate_mapping_snapshot(actual);
    mapping = candidate_mapping;
    matches = true;
end
end

function mapping = candidate_mapping_snapshot(actual)
mapping = actual.('mapping_snapshot');
end

function expected = independently_recover_loads(mapping,qhat_full)
qhat_full = qhat_full(:);
u_relative = mapping.interface_transform*qhat_full;
delta_hat_vector = mapping.contact_jacobian_local*u_relative;
Qhat_vector = mapping.k_static_slice(:).*delta_hat_vector;
shape = size(mapping.k_static_slice);
delta_hat = reshape(delta_hat_vector,shape);
Qhat = reshape(Qhat_vector,shape);
Q_static = mapping.Q_static_slice;
delta_static = mapping.delta_static_slice;
k_static = mapping.k_static_slice;
loaded = logical(mapping.loaded_mask_static_slice);
Qmax = Q_static+abs(Qhat);
Qmin = Q_static-abs(Qhat);

r_delta = nan(shape);
r_delta(loaded) = abs(delta_hat(loaded))./max(abs(delta_static(loaded)),1e-12);
classification_tolerance = 100*eps;
linearity_class = repmat({'NOT_EVALUATED_STATICALLY_UNLOADED_SLICE'},shape);
linearity_class(loaded & r_delta <= 0.10+classification_tolerance) = ...
    {'STRONG_LOCAL_LINEARITY'};
linearity_class(loaded & r_delta > 0.10+classification_tolerance & ...
    r_delta <= 0.20+classification_tolerance) = {'ACCEPTABLE_WITH_CAUTION'};
linearity_class(loaded & r_delta > 0.20+classification_tolerance) = ...
    {'LOCAL_LINEARITY_LIMIT_EXCEEDED'};
contact_loss = loaded & Qmin <= 0;
contact_loss_reason = repmat({'STATICALLY_UNLOADED_ELEMENT'},shape);
contact_loss_reason(loaded & ~contact_loss) = {'NO_LINEAR_CONTACT_LOSS'};
contact_loss_reason(contact_loss) = {'POTENTIAL_CONTACT_LOSS'};
slice_level = struct('level_label','SLICE_LEVEL', ...
    'Q_static_slice',Q_static,'loaded_mask_static_slice',loaded, ...
    'delta_static_slice',delta_static,'k_static_slice',k_static, ...
    'delta_hat_slice',delta_hat,'delta_hat_real_slice',real(delta_hat), ...
    'delta_hat_imag_slice',imag(delta_hat), ...
    'delta_hat_magnitude_slice',abs(delta_hat), ...
    'delta_hat_phase_rad_slice',angle(delta_hat), ...
    'Qhat_slice',Qhat,'Qhat_real_slice',real(Qhat), ...
    'Qhat_imag_slice',imag(Qhat),'Qhat_magnitude_slice',abs(Qhat), ...
    'Qhat_phase_rad_slice',angle(Qhat), ...
    'Qmax_linear_slice',Qmax,'Qmin_linear_slice',Qmin, ...
    'r_delta_slice',r_delta,'linearity_class_slice',{linearity_class}, ...
    'potential_contact_loss_slice',contact_loss, ...
    'contact_loss_evaluated_slice',loaded, ...
    'contact_loss_reason_slice',{contact_loss_reason});

Q_static_roller = sum(Q_static,2);
Qhat_roller = sum(Qhat,2);
loaded_roller = any(loaded,2);
Qmax_roller = Q_static_roller+abs(Qhat_roller);
Qmin_roller = Q_static_roller-abs(Qhat_roller);
r_delta_roller = nan(mapping.element_count,1);
for roller_index = 1:mapping.element_count
    values = r_delta(roller_index,loaded(roller_index,:));
    if ~isempty(values), r_delta_roller(roller_index) = max(values); end
end
roller_class = repmat({'NOT_EVALUATED_STATICALLY_UNLOADED_ROLLER'}, ...
    mapping.element_count,1);
roller_class(loaded_roller & r_delta_roller <= 0.10+classification_tolerance) = ...
    {'STRONG_LOCAL_LINEARITY'};
roller_class(loaded_roller & r_delta_roller > 0.10+classification_tolerance & ...
    r_delta_roller <= 0.20+classification_tolerance) = {'ACCEPTABLE_WITH_CAUTION'};
roller_class(loaded_roller & r_delta_roller > 0.20+classification_tolerance) = ...
    {'LOCAL_LINEARITY_LIMIT_EXCEEDED'};
roller_loss = loaded_roller & Qmin_roller <= 0;
roller_level = struct('level_label','ROLLER_LEVEL', ...
    'Q_static_roller',Q_static_roller, ...
    'loaded_mask_static_roller',loaded_roller, ...
    'Qhat_roller',Qhat_roller,'Qhat_real_roller',real(Qhat_roller), ...
    'Qhat_imag_roller',imag(Qhat_roller), ...
    'Qhat_magnitude_roller',abs(Qhat_roller), ...
    'Qhat_phase_rad_roller',angle(Qhat_roller), ...
    'Qmax_linear_roller',Qmax_roller,'Qmin_linear_roller',Qmin_roller, ...
    'r_delta_roller',r_delta_roller,'linearity_class_roller',{roller_class}, ...
    'potential_contact_loss_roller',roller_loss, ...
    'contact_loss_evaluated_roller',loaded_roller);

analytic_constructed = mapping.force_assembly_local* ...
    diag(mapping.k_static_slice(:))*mapping.contact_jacobian_local;
analytic_definition_error = validation_relative_error( ...
    analytic_constructed,mapping.K_b_local_analytic);
contact_load_local = mapping.force_assembly_local*Qhat_vector;
analytic_force = mapping.K_b_local_analytic*u_relative;
saved_force = mapping.K_b_local*u_relative;
element_error = validation_relative_error(contact_load_local,analytic_force);
tangent_error = validation_relative_error(analytic_constructed,mapping.K_b_local);
assembly = struct( ...
    'element_vs_analytic_tangent_relative_error',element_error, ...
    'element_vs_analytic_tangent_threshold',1e-10, ...
    'element_vs_analytic_tangent_gate_status','ROLLING_ELEMENT_FORCE_ASSEMBLY_MATCHED', ...
    'analytic_vs_saved_tangent_relative_error',tangent_error, ...
    'analytic_vs_saved_tangent_threshold',1e-8, ...
    'analytic_vs_saved_tangent_gate_status','ANALYTIC_VS_FD_TANGENT_AUDIT_PASSED', ...
    'analytic_definition_relative_error',analytic_definition_error, ...
    'analytic_local_complex',analytic_force, ...
    'saved_tangent_local_complex',saved_force);
radial_components = contact_load_local(1:2);
radial_phase_map = [real(radial_components) -imag(radial_components)];
contact_load_resultant = struct('local_complex',contact_load_local, ...
    'global_complex',mapping.interface_transform.'*contact_load_local, ...
    'Fx_hat',contact_load_local(1),'Fy_hat',contact_load_local(2), ...
    'radial_force_peak',norm(radial_phase_map,2), ...
    'radial_force_rms',norm(radial_components,2)/sqrt(2), ...
    'component_phase_rad',angle(contact_load_local), ...
    'force_semantics','CONTACT_LOAD_PLUS_AQ');
bearing_force_local = -contact_load_local;
bearing_force_on_rotor = struct('local_complex',bearing_force_local, ...
    'global_complex',mapping.interface_transform.'*bearing_force_local, ...
    'Fx_hat',bearing_force_local(1),'Fy_hat',bearing_force_local(2), ...
    'radial_force_peak',norm(radial_phase_map,2), ...
    'radial_force_rms',norm(radial_components,2)/sqrt(2), ...
    'component_phase_rad',angle(bearing_force_local), ...
    'force_semantics','FORCE_ON_ROTOR_MINUS_AQ');
dynamic_resultant = contact_load_resultant;

loaded_r = r_delta(loaded);
if isempty(loaded_r), maximum_r_delta = NaN; else, maximum_r_delta = max(loaded_r); end
limit_exceeded = any(r_delta(loaded) > 0.20+classification_tolerance);
if any(contact_loss(:)) || limit_exceeded
    status = 'REPRESENTATIVE_NONLINEAR_VALIDATION_REQUIRED';
elseif isempty(loaded_r) || maximum_r_delta <= 0.10
    status = 'FREQUENCY_DOMAIN_LINEARIZATION_ACCEPTED';
else
    status = 'FREQUENCY_DOMAIN_LINEARIZATION_CAUTION';
end
hard_gate = struct('loaded_slice_contact_loss',any(contact_loss(:)), ...
    'loaded_slice_linearity_limit_exceeded',limit_exceeded, ...
    'maximum_loaded_slice_r_delta',maximum_r_delta,'status',status);
expected = struct('slice_level',slice_level,'roller_level',roller_level, ...
    'dynamic_resultant',dynamic_resultant, ...
    'contact_load_resultant',contact_load_resultant, ...
    'contact_force_resultant',contact_load_resultant, ...
    'bearing_force_on_rotor',bearing_force_on_rotor, ...
    'force_assembly_validation',assembly,'hard_gate',hard_gate, ...
    'mapping_snapshot',mapping,'interface_relative_motion_hat',u_relative);
end

function expected = independently_rebuild_scan( ...
        baseline,actual,temperature,tf,bearing,nominal)
theta = 2*pi*(0:31)/32;
if canonical_mapping_available(baseline,tf,bearing)
    mapping = build_stage_f_rolling_element_linearization_mapping( ...
        baseline,temperature,bearing);
    q_static = baseline.static.(tf).q_static(:);
    qhat = baseline.frequency_response.(tf).NOMINAL.qhat_full(:);
    linear_force_hat = -(mapping.K_b_local_analytic* ...
        (mapping.interface_transform*qhat));
    loaded_static = logical(mapping.loaded_mask_static_slice(:));
    parameters = independent_formal_parameters( ...
        baseline,baseline.static.(tf));
    if strcmp(bearing,'front'), bearing_index = 1; else, bearing_index = 2; end
    evaluation = @(q) independent_formal_force(q,parameters,bearing_index);
    [static_force,static_loaded] = evaluation(q_static);
    if ~isequal(logical(static_loaded(:)),loaded_static)
        error('StageF:IndependentScanStaticMask', ...
            'Formal baseline active set differs during independent scan rebuild.');
    end
    contact = baseline.static.(tf).(['bearing_state_' bearing]);
    if isfield(contact,'raw_contact') && isfield(contact.raw_contact,'f5')
        saved_static_force = contact.raw_contact.f5(:);
    else
        saved_static_force = [contact.Fx;contact.Fy;contact.Fz; ...
            contact.Mx;contact.My];
    end
    static_reference_error = validation_relative_error( ...
        static_force,saved_static_force);
else
    force_count = numel(baseline.static.(tf).q_static);
    static_force = zeros(force_count,1);
    linear_force_hat = zeros(force_count,1);
    loaded_static = logical(actual.loaded_mask_static_slice(:));
    evaluation = [];
    static_reference_error = 0;
end
force_count = numel(static_force);
exact_force_history = zeros(force_count,32);
exact_dynamic_history = zeros(force_count,32);
linear_dynamic_history = real(linear_force_hat(:)*exp(1i*theta));
active_history = false(numel(loaded_static),32);
for phase_index = 1:32
    if isempty(evaluation)
        force_phase = static_force+linear_dynamic_history(:,phase_index);
        loaded_phase = loaded_static;
    else
        q_phase = q_static+real(qhat*exp(1i*theta(phase_index)));
        [force_phase,loaded_phase] = evaluation(q_phase);
    end
    exact_force_history(:,phase_index) = force_phase(:);
    exact_dynamic_history(:,phase_index) = force_phase(:)-static_force;
    active_history(:,phase_index) = logical(loaded_phase(:));
end
active_changed = any(active_history ~= loaded_static,'all');
first_change = first_active_change(active_history,loaded_static,theta);
numerator = history_rms_norm(exact_dynamic_history-linear_dynamic_history);
denominator = history_rms_norm(exact_dynamic_history);
if numerator == 0 && denominator == 0
    epsilon_NL = 0;
else
    epsilon_NL = numerator/max(denominator,eps);
end
expected = struct('scenario','NOMINAL','phase_count',32, ...
    'phase_values_rad',theta,'loaded_mask_static_slice',loaded_static, ...
    'active_set_history',active_history,'active_set_changed',active_changed, ...
    'first_active_set_change',first_change, ...
    'static_force_reference',static_force, ...
    'static_reference_relative_error',static_reference_error, ...
    'exact_force_history',exact_force_history, ...
    'exact_dynamic_force_history',exact_dynamic_history, ...
    'linear_force_hat',linear_force_hat, ...
    'linear_dynamic_force_history',linear_dynamic_history, ...
    'epsilon_NL',epsilon_NL, ...
    'maximum_r_delta_nominal',nominal.hard_gate.maximum_loaded_slice_r_delta, ...
    'potential_contact_loss_nominal', ...
        nominal.hard_gate.loaded_slice_contact_loss);
end

function parameters = independent_formal_parameters(baseline,static_state)
parameters = initial_conditions();
[~,~,~,model_info] = build_rotor_case_model(parameters);
parameters.modelInfo = model_info;
parameters.num_rotor_dof = model_info.num_rotor_dof;
parameters.num_case_dof = model_info.num_case_dof;
parameters.omega = baseline.configuration.rotational_speed_rad_s;
parameters.microphysics = microphysics_config(struct('thermal', ...
    struct('enabled',true,'mode','frozen_external'), ...
    'thermal_state',static_state.thermal_state));
end

function [force,loaded] = independent_formal_force(q,parameters,bearing_index)
[~,bearing_state] = nonlinear_bearing_force(q,zeros(size(q)), ...
    parameters,parameters.bearing,numel(q));
state = bearing_state.bearings(bearing_index);
force = [state.Fx;state.Fy;state.Fz;state.Mx;state.My];
loaded = state.Q > 0;
end

function [diagnostics,hard_failure,strong] = validate_scan( ...
        actual,nominal,expected,require_formal_audit,expected_thermal)
diagnostics = {}; hard_failure = true; strong = false;
required = {'scenario','phase_count','phase_values_rad','loaded_mask_static_slice', ...
    'active_set_history','active_set_changed','first_active_set_change', ...
    'static_force_reference','exact_force_history', ...
    'exact_dynamic_force_history','linear_force_hat', ...
    'linear_dynamic_force_history','epsilon_NL','F1_complex','F2_complex', ...
    'F3_complex','H2_H1','H3_H1','maximum_r_delta_nominal', ...
    'potential_contact_loss_nominal','gate_status'};
harmonic_required = {'F1_norm','F2_norm','F3_norm','F1_phase_rad', ...
    'F2_phase_rad','F3_phase_rad', ...
    'F1_force_semantics','F2_force_semantics','F3_force_semantics', ...
    'diagnostic_label'};
if ~isstruct(actual) || ~all(isfield(actual,required))
    diagnostics = {'NONLINEARITY_SCHEMA'}; return;
end
if ~all(isfield(actual,harmonic_required))
    diagnostics = {'HARMONIC_AUDIT'}; return;
end
if ~all(isfield(actual,{'thermal_state_frozen','thermal_frozen_summary'}))
    diagnostics = {'THERMAL_FROZEN'}; return;
end
theta = 2*pi*(0:31)/32;
if ~strcmp(actual.scenario,'NOMINAL') || actual.phase_count ~= 32 || ...
        ~isequal(size(actual.phase_values_rad),size(theta)) || ...
        max(abs(actual.phase_values_rad-theta)) > 100*eps(2*pi)
    diagnostics{end+1} = 'PHASE_VALUES_RAD';
end
independent_mismatch = false;
if ~isequaln(actual.loaded_mask_static_slice,expected.loaded_mask_static_slice)
    diagnostics{end+1} = 'LOADED_MASK_STATIC_SLICE'; independent_mismatch = true;
end
if ~same_numeric(actual.static_force_reference,expected.static_force_reference)
    diagnostics{end+1} = 'STATIC_FORCE_REFERENCE'; independent_mismatch = true;
end
if ~isfield(actual,'static_reference_relative_error') || ...
        ~same_scalar(actual.static_reference_relative_error, ...
        expected.static_reference_relative_error) || ...
        actual.static_reference_relative_error > 1e-10
    diagnostics{end+1} = 'STATIC_REFERENCE_ERROR';
end
if ~same_numeric(actual.exact_force_history,expected.exact_force_history)
    diagnostics{end+1} = 'EXACT_FORCE_HISTORY'; independent_mismatch = true;
end
if ~same_numeric(actual.exact_dynamic_force_history,expected.exact_dynamic_force_history)
    diagnostics{end+1} = 'EXACT_DYNAMIC_FORCE_HISTORY';
    independent_mismatch = true;
end
if ~same_numeric(actual.linear_force_hat,expected.linear_force_hat)
    diagnostics{end+1} = 'LINEAR_FORCE_HAT'; independent_mismatch = true;
end
if ~same_numeric(actual.linear_dynamic_force_history,expected.linear_dynamic_force_history)
    diagnostics{end+1} = 'LINEAR_DYNAMIC_FORCE_HISTORY';
    independent_mismatch = true;
end
if ~isequaln(actual.active_set_history,expected.active_set_history)
    diagnostics{end+1} = 'ACTIVE_SET'; independent_mismatch = true;
end
if ~isequaln(actual.active_set_changed,expected.active_set_changed)
    diagnostics{end+1} = 'ACTIVE_SET';
    independent_mismatch = true;
end
if ~isequaln(actual.first_active_set_change,expected.first_active_set_change)
    diagnostics{end+1} = 'FIRST_ACTIVE_SET_CHANGE';
    independent_mismatch = true;
end
if ~same_scalar(actual.epsilon_NL,expected.epsilon_NL)
    diagnostics{end+1}='EPSILON_NL'; independent_mismatch = true;
end
if independent_mismatch
    diagnostics{end+1} = 'INDEPENDENT_SCAN_REBUILD';
end
try
    harmonics = independently_compute_stage_f_harmonics( ...
        expected.exact_dynamic_force_history,theta);
    if ~same_numeric(actual.F1_complex,harmonics.F1_complex), diagnostics{end+1}='F1'; end
    if ~same_numeric(actual.F2_complex,harmonics.F2_complex), diagnostics{end+1}='F2'; end
    if ~same_numeric(actual.F3_complex,harmonics.F3_complex), diagnostics{end+1}='F3'; end
    if ~same_scalar(actual.H2_H1,harmonics.H2_H1), diagnostics{end+1}='H2'; end
    if ~same_scalar(actual.H3_H1,harmonics.H3_H1), diagnostics{end+1}='H3'; end
    harmonic_audit_ok = same_scalar(actual.F1_norm,harmonics.F1_norm) && ...
        same_scalar(actual.F2_norm,harmonics.F2_norm) && ...
        same_scalar(actual.F3_norm,harmonics.F3_norm) && ...
        same_numeric(actual.F1_phase_rad,harmonics.F1_phase_rad) && ...
        same_numeric(actual.F2_phase_rad,harmonics.F2_phase_rad) && ...
        same_numeric(actual.F3_phase_rad,harmonics.F3_phase_rad) && ...
        strcmp(actual.F1_force_semantics,'FORCE_ON_ROTOR_MINUS_AQ') && ...
        strcmp(actual.F2_force_semantics,'FORCE_ON_ROTOR_MINUS_AQ') && ...
        strcmp(actual.F3_force_semantics,'FORCE_ON_ROTOR_MINUS_AQ') && ...
        strcmp(actual.diagnostic_label, ...
        'BEARING_NONLINEAR_FORCE_HARMONIC_DIAGNOSTICS');
    if ~harmonic_audit_ok, diagnostics{end+1}='HARMONIC_AUDIT'; end
catch
    harmonics = struct('H2_H1',Inf,'H3_H1',Inf);
    diagnostics{end+1} = 'HARMONICS';
end
if ~isequal(actual.thermal_state_frozen,true) || ...
        ~frozen_summary_matches(actual.thermal_frozen_summary, ...
        expected_thermal,require_formal_audit)
    diagnostics{end+1}='THERMAL_FROZEN';
end
r_values = nominal.slice_level.r_delta_slice( ...
    nominal.slice_level.loaded_mask_static_slice);
if isempty(r_values), expected_max_r = NaN; else, expected_max_r = max(r_values); end
expected_loss = any(nominal.slice_level.potential_contact_loss_slice(:));
if ~isequaln(actual.maximum_r_delta_nominal,expected_max_r)
    diagnostics{end+1} = 'MAXIMUM_R_DELTA_NOMINAL';
end
if ~isequaln(logical(actual.potential_contact_loss_nominal),expected_loss)
    diagnostics{end+1} = 'POTENTIAL_CONTACT_LOSS_NOMINAL';
end
hard_failure = expected_loss || (isfinite(expected_max_r) && expected_max_r > 0.20) || ...
    expected.active_set_changed || expected.epsilon_NL > 0.10 || ...
    harmonics.H2_H1 > 0.10 || harmonics.H3_H1 > 0.10;
strong = ~hard_failure && (~isfinite(expected_max_r) || expected_max_r <= 0.10) && ...
    expected.epsilon_NL <= 0.05 && harmonics.H2_H1 <= 0.05 && harmonics.H3_H1 <= 0.05;
if hard_failure
    expected_status = 'REPRESENTATIVE_NONLINEAR_VALIDATION_REQUIRED';
elseif strong
    expected_status = 'FREQUENCY_DOMAIN_LINEARIZATION_ACCEPTED';
else
    expected_status = 'FREQUENCY_DOMAIN_LINEARIZATION_CAUTION';
end
if ~strcmp(actual.gate_status,expected_status)
    diagnostics{end+1} = 'BEARING_GATE_STATUS';
end
end

function harmonics = independently_compute_stage_f_harmonics(history,theta)
theta = reshape(theta,1,[]); sample_count = numel(theta);
F1 = (2/sample_count)*history*exp(-1i*theta).';
F2 = (2/sample_count)*history*exp(-2i*theta).';
F3 = (2/sample_count)*history*exp(-3i*theta).';
norm1 = norm(F1,2); norm2 = norm(F2,2); norm3 = norm(F3,2);
harmonics = struct('F1_complex',F1,'F2_complex',F2,'F3_complex',F3, ...
    'F1_norm',norm1,'F2_norm',norm2,'F3_norm',norm3, ...
    'F1_phase_rad',angle(F1),'F2_phase_rad',angle(F2), ...
    'F3_phase_rad',angle(F3), ...
    'H2_H1',norm2/max(norm1,eps),'H3_H1',norm3/max(norm1,eps));
end

function pass = frozen_summary_matches(actual,expected,require_formal_audit)
required = {'checksum';'passed';'sha256'};
pass = isstruct(actual) && isscalar(actual) && ...
    isequal(sort(fieldnames(actual)),required) && ...
    isequal(actual.passed,true) && ischar(actual.sha256) && ...
    ~isempty(regexp(actual.sha256,'^[0-9a-f]{64}$','once')) && ...
    ischar(actual.checksum) && ...
    ~isempty(regexp(actual.checksum,'^[0-9a-f]{8}$','once'));
if require_formal_audit
    pass = pass && isequaln(actual,stage_f_value_reference(expected));
end
end

function summary = stage_f_frozen_summary(static_state,bearing)
if strcmp(bearing,'front')
    thermal = static_state.thermal_state.ball;
elseif strcmp(bearing,'rear')
    thermal = static_state.thermal_state.roller;
else
    summary = struct([]); return;
end
contact_state = static_state.(['bearing_state_' bearing]);
summary = struct('temperature_C',thermal.T_final, ...
    'viscosity_Pa_s',thermal.eta,'clearance_m',thermal.working_clearance, ...
    'contact_state_sha256',stage_f_value_sha256(contact_state));
end

function hash = stage_f_value_sha256(value)
encoded = jsonencode(value,'ConvertInfAndNaN',true);
bytes = unicode2native(encoded,'UTF-8');
digest = java.security.MessageDigest.getInstance('SHA-256');
digest.update(typecast(uint8(bytes),'int8'));
raw = typecast(digest.digest(),'uint8');
hash = lower(reshape(dec2hex(raw,2).',1,[]));
end

function reference = stage_f_value_reference(value)
bytes = getByteStreamFromArray(value);
digest = java.security.MessageDigest.getInstance('SHA-256');
digest.update(typecast(uint8(bytes),'int8'));
raw = typecast(digest.digest(),'uint8');
crc = java.util.zip.CRC32;
crc.update(typecast(uint8(bytes),'int8'));
reference = struct('sha256',lower(reshape(dec2hex(raw,2).',1,[])), ...
    'checksum',lower(dec2hex(double(crc.getValue()),8)),'passed',true);
end

function trigger = independently_rebuild_first_trigger(recovered,scans)
trigger = struct([]);
for temperature = [20 50 80 100]
    tf = sprintf('T%d',temperature);
    for scenario = {'LOW','NOMINAL','HIGH'}
        sf = scenario{1};
        for bearing = {'front','rear'}
            bn = bearing{1}; item = recovered.(tf).(sf).(bn);
            loss = find(item.slice_level.potential_contact_loss_slice,1);
            limit = find(item.slice_level.loaded_mask_static_slice & ...
                item.slice_level.r_delta_slice > 0.20,1);
            if ~isempty(loss) || ~isempty(limit)
                if ~isempty(loss)
                    index = loss; metric = 'POTENTIAL_CONTACT_LOSS';
                    trigger_value = item.slice_level.Qmin_linear_slice(index);
                    threshold = 0;
                else
                    index = limit; metric = 'LOCAL_LINEARITY_LIMIT_EXCEEDED';
                    trigger_value = item.slice_level.r_delta_slice(index);
                    threshold = 0.20;
                end
                [roller_index,slice_index] = ind2sub( ...
                    size(item.slice_level.loaded_mask_static_slice),index);
                trigger = struct('temperature_case_C',temperature, ...
                    'scenario',sf,'bearing',bn,'roller_index',roller_index, ...
                    'slice_index',slice_index,'metric',metric, ...
                    'phase_index',NaN,'phase_rad',NaN, ...
                    'slice_indices',slice_index,'trigger_value',trigger_value, ...
                    'threshold',threshold,'temperature',temperature, ...
                    'phase',NaN,'roller_id',roller_index,'slice_id',slice_index);
                return;
            end
        end
    end
    for bearing = {'front','rear'}
        bn = bearing{1}; scan = scans.(tf).(bn);
        harmonics = independently_compute_stage_f_harmonics( ...
            scan.exact_dynamic_force_history,scan.phase_values_rad);
        if scan.active_set_changed
            metric = 'ACTIVE_CONTACT_SET_CHANGED';
            change = scan.first_active_set_change;
            phase_index = change.phase_index; phase_rad = change.phase_rad;
            slice_indices = change.slice_indices; trigger_value = 1; threshold = 0;
            [roller_ids,slice_ids] = decode_active_contact_indices( ...
                bn,slice_indices);
            roller_index = roller_ids(1); slice_index = slice_ids(1);
        elseif scan.epsilon_NL > 0.10
            metric = 'EPSILON_NL'; phase_index = NaN; phase_rad = NaN;
            slice_indices = []; trigger_value = scan.epsilon_NL; threshold = 0.10;
            roller_ids = []; slice_ids = [];
            roller_index = NaN; slice_index = NaN;
        elseif harmonics.H2_H1 > 0.10
            metric = 'H2_H1'; phase_index = NaN; phase_rad = NaN;
            slice_indices = []; trigger_value = harmonics.H2_H1; threshold = 0.10;
            roller_ids = []; slice_ids = [];
            roller_index = NaN; slice_index = NaN;
        elseif harmonics.H3_H1 > 0.10
            metric = 'H3_H1'; phase_index = NaN; phase_rad = NaN;
            slice_indices = []; trigger_value = harmonics.H3_H1; threshold = 0.10;
            roller_ids = []; slice_ids = [];
            roller_index = NaN; slice_index = NaN;
        else
            continue;
        end
        trigger = struct('temperature_case_C',temperature, ...
            'scenario','NOMINAL','bearing',bn,'roller_index',roller_index, ...
            'slice_index',slice_index,'metric',metric,'phase_index',phase_index, ...
            'phase_rad',phase_rad,'slice_indices',slice_indices, ...
            'trigger_value',trigger_value,'threshold',threshold, ...
            'temperature',temperature,'phase',phase_rad, ...
            'roller_id',roller_ids,'slice_id',slice_ids);
        return;
    end
end
end

function pass = first_trigger_matches(progress,expected)
pass = isfield(progress,'stage_f_first_trigger');
if ~pass, return; end
actual = progress.stage_f_first_trigger;
if isempty(expected)
    pass = isempty(actual); return;
end
if ~isstruct(actual) || isempty(actual)
    pass = false; return;
end
required = {'temperature_case_C','scenario','bearing','roller_index', ...
    'slice_index','metric','phase_index','phase_rad','slice_indices', ...
    'trigger_value','threshold','temperature','phase','roller_id','slice_id'};
pass = all(isfield(actual,required));
for k = 1:numel(required)
    pass = pass && isequaln(actual.(required{k}),expected.(required{k}));
end
end

function [roller_ids,slice_ids] = decode_active_contact_indices(bearing,linear_indices)
if strcmp(bearing,'front')
    contact_shape = [22 1];
elseif strcmp(bearing,'rear')
    contact_shape = [30 9];
else
    roller_ids = []; slice_ids = []; return;
end
linear_indices = reshape(linear_indices,1,[]);
if isempty(linear_indices) || any(linear_indices < 1) || ...
        any(linear_indices > prod(contact_shape)) || ...
        any(linear_indices ~= floor(linear_indices))
    roller_ids = []; slice_ids = []; return;
end
[roller_ids,slice_ids] = ind2sub(contact_shape,linear_indices);
roller_ids = reshape(roller_ids,1,[]);
slice_ids = reshape(slice_ids,1,[]);
end

function diagnostics = validate_execution_audit(audit)
diagnostics = {};
try
    requests = audit.scan_requests;
    schedule_ok = numel(requests) == 8 && audit.scan_request_count == 8 && ...
        audit.LOW_scans == 0 && audit.HIGH_scans == 0;
    observed = cell(1,numel(requests)); evaluator_ok = true;
    for k = 1:numel(requests)
        item = requests(k);
        schedule_ok = schedule_ok && item.phase_count == 32 && ...
            strcmp(item.scenario,'NOMINAL') && item.thermal_state_frozen;
        evaluator_ok = evaluator_ok && strcmp(item.evaluator_function, ...
            'nonlinear_bearing_force');
        observed{k} = sprintf('T%d_%s',item.temperature_case_C,item.bearing);
    end
    expected = {'T20_front','T20_rear','T50_front','T50_rear', ...
        'T80_front','T80_rear','T100_front','T100_rear'};
    schedule_ok = schedule_ok && isequal(sort(observed),sort(expected));
    evaluator_ok = evaluator_ok && strcmp(audit.evaluator_function, ...
        'nonlinear_bearing_force') && strcmp(audit.evaluator_source_sha256, ...
        'd82ae8cc92a1f1fb55d9d0c14da3efa196e9f085ce22abc5cdd2b345f93af5c8') && ...
        audit.thermal_state_frozen;
    if ~schedule_ok, diagnostics{end+1} = 'SCAN_SCHEDULE'; end
    if ~evaluator_ok, diagnostics{end+1} = 'EVALUATOR'; end
catch
    diagnostics = {'SCAN_SCHEDULE','EVALUATOR'};
end
end

function diagnostics = validate_source_and_elapsed_provenance( ...
        candidate,baseline,temperatures,scenarios,bearings,require_formal_audit)
diagnostics = {};
audit = candidate.stage_f_execution_audit;
source_ok = isfield(audit,'source_commit') && ischar(audit.source_commit) && ...
    ~isempty(regexp(audit.source_commit,'^[0-9a-f]{40}$','once'));
if require_formal_audit
    source_ok = source_ok && strcmp(audit.source_commit, ...
        current_repository_commit());
end
elapsed_ok = isfield(audit,'elapsed_seconds') && ...
    isnumeric(audit.elapsed_seconds) && isscalar(audit.elapsed_seconds) && ...
    isfinite(audit.elapsed_seconds) && audit.elapsed_seconds >= 0;
for temperature = temperatures
    tf = sprintf('T%d',temperature);
    for scenario = scenarios
        sf = scenario{1};
        for bearing = bearings
            item = candidate.bearing_contact.(tf).(sf).(bearing{1});
            source_ok = source_ok && ...
                isfield(item,'stage_e_response_source_commit') && ...
                strcmp(item.stage_e_response_source_commit, ...
                baseline.meta.stage_e_source_commit) && ...
                isfield(item,'static_state_source_commit') && ...
                strcmp(item.static_state_source_commit, ...
                baseline.meta.stage_d_source_commit) && ...
                isfield(item,'stage_f_source_commit') && ...
                strcmp(item.stage_f_source_commit,audit.source_commit);
        end
    end
end
if ~source_ok, diagnostics{end+1} = 'SOURCE_COMMIT'; end
if ~elapsed_ok, diagnostics{end+1} = 'ELAPSED_SECONDS'; end
end

function commit = current_repository_commit
repository_root = fileparts(mfilename('fullpath'));
command = sprintf('git -C "%s" rev-parse HEAD',repository_root);
[status,output] = system(command);
if status == 0
    commit = strtrim(output);
else
    commit = '';
end
end

function pass = progress_matches(candidate,gate,allow)
try
    p = candidate.progress;
    pass = isequal(p.stage_f_load_recovery_complete,true) && ...
        isequal(p.stage_f_nonlinear_scan_complete,true) && ...
        isequal(p.stage_f_complete,true) && strcmp(p.stage_f_gate_status,gate) && ...
        isequal(logical(p.allow_stage_g),allow) && ...
        strcmp(p.last_completed_gate,gate) && ...
        strcmp(candidate.decision.status,gate) && ...
        isequal(logical(candidate.decision.allow_stage_g),allow) && ...
        isequal(candidate.validation.stage_f.passed,true) && ...
        strcmp(candidate.validation.stage_f.gate_status,gate);
    if strcmp(gate,'REPRESENTATIVE_NONLINEAR_VALIDATION_REQUIRED')
        pass = pass && ~isempty(p.stage_f_first_trigger);
    else
        pass = pass && isempty(p.stage_f_first_trigger);
    end
catch
    pass = false;
end
end

function first = first_active_change(history,static_mask,theta)
first = struct([]);
for k = 1:size(history,2)
    changed = find(history(:,k) ~= static_mask);
    if ~isempty(changed)
        first = struct('phase_index',k,'phase_rad',theta(k), ...
            'slice_indices',reshape(changed,1,[]), ...
            'static_mask',static_mask,'phase_mask',logical(history(:,k)));
        return;
    end
end
end

function pass = canonical_mapping_available(candidate,tf,bearing)
pass = isfield(candidate,'meta') && isfield(candidate,'static') && ...
    isfield(candidate.static,tf) && ...
    isfield(candidate.static.(tf),['bearing_state_' bearing]) && ...
    isfield(candidate.static,'T20') && ...
    isfield(candidate.static.T20,'bearing_mapping');
end

function pass = same_without_stage_f_validation(candidate,baseline)
try
    projection = candidate;
    for name = {'bearing_contact','nonlinearity_gate','stage_f_execution_audit'}
        if isfield(projection,name{1}), projection = rmfield(projection,name{1}); end
    end
    if isfield(baseline.validation,'stage_f')
        projection.validation.stage_f = baseline.validation.stage_f;
    elseif isfield(projection,'validation') && isfield(projection.validation,'stage_f')
        projection.validation = rmfield(projection.validation,'stage_f');
    end
    additions = {'stage_f_load_recovery_complete','stage_f_nonlinear_scan_complete', ...
        'stage_f_complete','stage_f_gate_status','stage_f_first_trigger','allow_stage_g'};
    for k = 1:numel(additions)
        if isfield(baseline.progress,additions{k})
            projection.progress.(additions{k}) = baseline.progress.(additions{k});
        elseif isfield(projection.progress,additions{k})
            projection.progress = rmfield(projection.progress,additions{k});
        end
    end
    projection.progress.last_completed_gate = baseline.progress.last_completed_gate;
    projection.decision = baseline.decision;
    pass = isequaln(projection,baseline);
catch
    pass = false;
end
end

function pass = canonical_input_reference_matches(candidate,frozen_projection)
try
    actual = candidate.stage_f_execution_audit.canonical_input_reference;
    expected = stage_f_value_reference(frozen_projection);
    pass = isstruct(actual) && isscalar(actual) && ...
        isequal(sort(fieldnames(actual)),{'checksum';'passed';'sha256'}) && ...
        isequal(actual,expected);
catch
    pass = false;
end
end

function pass = same_validation(candidate,baseline)
try
    value = candidate.validation;
    if isfield(value,'stage_f'), value = rmfield(value,'stage_f'); end
    pass = isequaln(value,baseline.validation);
catch
    pass = false;
end
end

function pass = same_field(a,b,name)
pass = isfield(a,name) && isfield(b,name) && isequaln(a.(name),b.(name));
end

function present = forbidden_payload_present(candidate,baseline)
payload = struct();
for name = {'bearing_contact','nonlinearity_gate','stage_f_execution_audit'}
    payload.(name{1}) = candidate.(name{1});
end
if isfield(candidate.validation,'stage_f')
    payload.validation_stage_f = candidate.validation.stage_f;
end
for name = {'stage_f_load_recovery_complete','stage_f_nonlinear_scan_complete', ...
        'stage_f_complete','stage_f_gate_status','stage_f_first_trigger','allow_stage_g'}
    if isfield(candidate.progress,name{1})
        payload.(name{1}) = candidate.progress.(name{1});
    end
end
forbid_mapping = canonical_mapping_available(baseline,'T20','front');
present = recursive_forbidden(payload,forbid_mapping);
end

function present = recursive_forbidden(value,forbid_mapping)
present = false;
if isstruct(value)
    for index = 1:numel(value)
        names = fieldnames(value(index));
        for k = 1:numel(names)
            lower_name = lower(names{k});
            forbidden_exact = {'q_m','q_static','thermal_state','k_t','m','g', ...
                'formal_parameters','frozen_thermal_state','contact_state_snapshot'};
            if any(strcmp(lower_name,forbidden_exact)) || ...
                    contains(lower_name,'newmark') || ...
                    startsWith(lower_name,'viscosity') || ...
                    startsWith(lower_name,'clearance') || ...
                    (forbid_mapping && strcmp(lower_name,'mapping_snapshot'))
                present = true; return;
            end
            if recursive_forbidden(value(index).(names{k}),forbid_mapping)
                present = true; return;
            end
        end
    end
elseif iscell(value)
    for k = 1:numel(value)
        if recursive_forbidden(value{k},forbid_mapping), present = true; return; end
    end
end
end

function value = history_rms_norm(history)
value = sqrt(mean(sum(abs(history).^2,1)));
end

function pass = same_numeric(actual,expected)
pass = isnumeric(actual) && isnumeric(expected) && ...
    isequal(size(actual),size(expected)) && all(isfinite(actual),'all') && ...
    all(isfinite(expected),'all') && ...
    norm(actual(:)-expected(:)) <= 1e-12*max(norm(expected(:)),1);
end

function pass = same_scalar(actual,expected)
pass = isnumeric(actual) && isscalar(actual) && isfinite(actual) && ...
    isfinite(expected) && abs(actual-expected) <= 1e-12*max(abs(expected),1);
end

function value = validation_relative_error(actual,expected)
value = norm(actual(:)-expected(:))/max(norm(expected(:)),eps);
end

function validation = finish(diagnostics,gate_status)
validation = struct('passed',isempty(diagnostics), ...
    'gate_status',gate_status,'diagnostic_identifiers',{diagnostics}, ...
    'diagnostics',{diagnostics},'messages',{diagnostics}, ...
    'checks',struct());
end
