function validation = validate_stage_f_representative_results(candidate,baseline)
%VALIDATE_STAGE_F_REPRESENTATIVE_RESULTS Validate only the two-case addendum.

diagnostics = {};
if ~isstruct(candidate) || ~isstruct(baseline) || ...
        ~same_without_representative(candidate,baseline)
    diagnostics{end+1} = 'FROZEN_BASELINE';
end
try
    representative = candidate.nonlinear_validation.representative;
catch
    validation = finish({'SCHEMA'},false,''); return;
end
required = {'schema_version','selection','cases','gate_status', ...
    'allow_stage_g','source_commit'};
if ~isstruct(representative) || ~all(isfield(representative,required)) || ...
        ~strcmp(representative.schema_version, ...
        'stage-f-representative-nonlinear-validation-v1') || ...
        isempty(regexp(representative.source_commit,'^[0-9a-f]{40}$','once'))
    diagnostics{end+1} = 'SCHEMA';
end

projection = build_stage_f_frozen_input_projection(baseline);
selection = expected_selection(baseline);
try
    selection_ok = strcmp(representative.selection.case_A, ...
        'T20_NOMINAL_front') && ...
        strcmp(representative.selection.case_B,selection.label) && ...
        strcmp(representative.selection.selection_metric, ...
        'MAXIMUM_SAVED_EPSILON_NL');
catch
    selection_ok = false;
end
if ~selection_ok, diagnostics{end+1} = 'CASE_SELECTION'; end

try
    case_A = representative.cases.case_A;
    case_B = representative.cases.case_B;
    diagnostics = [diagnostics validate_case( ...
        case_A,baseline,projection,20,'front','A')];
    diagnostics = [diagnostics validate_case( ...
        case_B,baseline,projection,selection.temperature, ...
        selection.bearing,'B')];
    significant = case_A.active_set_changed || case_A.contact_loss_detected || ...
        case_B.active_set_changed || case_B.contact_loss_detected;
catch
    diagnostics{end+1} = 'CASE_SCHEMA';
    significant = true;
end
if significant
    expected_gate = 'REPRESENTATIVE_NONLINEAR_EFFECT_SIGNIFICANT';
    expected_allow = false;
else
    expected_gate = 'WORKPOINT_LINEARIZATION_LOCAL_DEVIATION_EXPLAINED';
    expected_allow = true;
end
if ~strcmp(representative.gate_status,expected_gate) || ...
        ~isequal(logical(representative.allow_stage_g),expected_allow)
    diagnostics{end+1} = 'FINAL_GATE';
end
diagnostics = unique(diagnostics,'stable');
validation = finish(diagnostics,expected_allow,expected_gate);
end

function diagnostics = validate_case(value,baseline,projection,temperature,bearing,role)
diagnostics = {};
required = {'temperature_case_C','scenario','bearing','selection_role', ...
    'phase_count','phase_values_rad','F_exact','F_linear', ...
    'static_force_reference','linear_force_hat','force_semantics', ...
    'epsilon_NL','loaded_mask_static','loaded_mask_history', ...
    'active_set_changed','Q_min','minimum_loaded_Q', ...
    'contact_loss_detected','force_history_sha256', ...
    'active_history_sha256','Q_min_sha256','F1_complex','F2_complex', ...
    'F3_complex','H2_H1','H3_H1','frozen_input_audit'};
if ~isstruct(value) || ~all(isfield(value,required))
    diagnostics = {'CASE_SCHEMA'}; return;
end
theta = 2*pi*(0:31)/32;
if value.temperature_case_C ~= temperature || ...
        ~strcmp(value.scenario,'NOMINAL') || ~strcmp(value.bearing,bearing) || ...
        ~strcmp(value.selection_role,role) || value.phase_count ~= 32 || ...
        ~isequal(size(value.phase_values_rad),size(theta)) || ...
        max(abs(value.phase_values_rad-theta)) > 100*eps(2*pi) || ...
        ~strcmp(value.force_semantics,'FORCE_ON_ROTOR_MINUS_AQ')
    diagnostics{end+1} = 'CASE_IDENTITY';
end
if any(~isfinite(value.F_exact),'all') || ...
        any(~isfinite(value.F_linear),'all') || ...
        size(value.F_exact,2) ~= 32 || ...
        ~isequal(size(value.F_exact),size(value.F_linear)) || ...
        ~strcmp(value.force_history_sha256,serialized_sha256(value.F_exact))
    diagnostics{end+1} = 'FORCE_HISTORY';
end
expected_linear = value.static_force_reference(:)+ ...
    real(value.linear_force_hat(:)*exp(1i*theta));
if ~same_numeric(value.F_linear,expected_linear)
    diagnostics{end+1} = 'LINEAR_PREDICTION';
end
exact_dynamic = value.F_exact-value.static_force_reference(:);
linear_dynamic = value.F_linear-value.static_force_reference(:);
expected_epsilon = relative_history_error(exact_dynamic,linear_dynamic);
if ~same_scalar(value.epsilon_NL,expected_epsilon)
    diagnostics{end+1} = 'NONLINEAR_ERROR';
end
loaded_static = logical(value.loaded_mask_static(:));
history = logical(value.loaded_mask_history);
Q_min = value.Q_min(:);
expected_changed = any(history ~= loaded_static,'all');
expected_loss = any(loaded_static & Q_min <= 0);
if size(history,2) ~= 32 || size(history,1) ~= numel(loaded_static) || ...
        numel(Q_min) ~= numel(loaded_static) || ...
        ~strcmp(value.active_history_sha256,serialized_sha256(history)) || ...
        ~strcmp(value.Q_min_sha256,serialized_sha256(Q_min)) || ...
        ~isequal(logical(value.active_set_changed),expected_changed) || ...
        ~isequal(logical(value.contact_loss_detected),expected_loss) || ...
        ~same_scalar_or_nan(value.minimum_loaded_Q,min_loaded(Q_min,loaded_static))
    diagnostics{end+1} = 'CONTACT_STATE';
end
harmonics = independent_harmonics(exact_dynamic,theta);
if ~same_numeric(value.F1_complex,harmonics.F1_complex) || ...
        ~same_numeric(value.F2_complex,harmonics.F2_complex) || ...
        ~same_numeric(value.F3_complex,harmonics.F3_complex) || ...
        ~same_scalar(value.H2_H1,harmonics.H2_H1) || ...
        ~same_scalar(value.H3_H1,harmonics.H3_H1)
    diagnostics{end+1} = 'HARMONICS';
end
if ~frozen_audit_matches(value.frozen_input_audit,baseline, ...
        projection,temperature,bearing)
    diagnostics{end+1} = 'FROZEN_INPUT';
end
if formal_baseline_available(baseline)
    expected = rebuild_formal_case_from_baseline( ...
        baseline,projection,temperature,bearing,role);
    diagnostics = [diagnostics compare_formal_rebuild(value,expected)];
end
end

function diagnostics = compare_formal_rebuild(value,expected)
diagnostics = {};
force_fields = {'phase_values_rad','F_exact','F_linear', ...
    'static_force_reference','linear_force_hat','epsilon_NL', ...
    'F1_complex','F2_complex','F3_complex','H2_H1','H3_H1'};
force_match = true;
for index = 1:numel(force_fields)
    name = force_fields{index};
    force_match = force_match && same_numeric(value.(name),expected.(name));
end
if ~force_match, diagnostics{end+1} = 'FORMAL_FORCE_RECOMPUTATION'; end
contact_fields = {'loaded_mask_static','loaded_mask_history', ...
    'active_set_changed','Q_min','minimum_loaded_Q','contact_loss_detected'};
contact_match = true;
for index = 1:numel(contact_fields)
    name = contact_fields{index};
    contact_match = contact_match && same_value(value.(name),expected.(name));
end
if ~contact_match, diagnostics{end+1} = 'FORMAL_CONTACT_RECOMPUTATION'; end
exact_fields = {'phase_count','force_semantics','force_history_sha256', ...
    'active_history_sha256','Q_min_sha256','frozen_input_audit'};
audit_match = true;
for index = 1:numel(exact_fields)
    name = exact_fields{index};
    audit_match = audit_match && isequaln(value.(name),expected.(name));
end
if ~audit_match, diagnostics{end+1} = 'FORMAL_AUDIT_RECOMPUTATION'; end
end

function expected = rebuild_formal_case_from_baseline( ...
        baseline,projection,temperature,bearing,role)
tf = sprintf('T%d',temperature);
static_state = baseline.static.(tf);
mapping = build_stage_f_rolling_element_linearization_mapping( ...
    baseline,temperature,bearing);
parameters = independent_formal_parameters(baseline,static_state);
if strcmp(bearing,'front'), bearing_index = 1; else, bearing_index = 2; end
q_static = static_state.q_static(:);
qhat = baseline.frequency_response.(tf).NOMINAL.qhat_full(:);
linear_force_hat = -(mapping.K_b_local_analytic* ...
    (mapping.interface_transform*qhat));
[static_force,static_loaded,static_Q] = independent_formal_state( ...
    q_static,parameters,bearing_index);
loaded_static = logical(mapping.loaded_mask_static_slice(:));
if ~isequal(static_loaded,loaded_static) || numel(static_Q) ~= numel(loaded_static)
    error('StageFRepresentative:FormalStaticReferenceMismatch', ...
        'REPRESENTATIVE_NONLINEAR_VALIDATION_FAILED: formal static reference changed.');
end
theta = 2*pi*(0:31)/32;
F_exact = zeros(5,32); loaded_history = false(numel(loaded_static),32);
Q_history = zeros(numel(loaded_static),32);
for phase_index = 1:32
    q_phase = q_static+real(qhat*exp(1i*theta(phase_index)));
    [force,loaded,Q] = independent_formal_state( ...
        q_phase,parameters,bearing_index);
    F_exact(:,phase_index) = force;
    loaded_history(:,phase_index) = loaded;
    Q_history(:,phase_index) = Q;
end
F_linear = static_force+real(linear_force_hat*exp(1i*theta));
exact_dynamic = F_exact-static_force;
linear_dynamic = F_linear-static_force;
Q_min = min(Q_history,[],2);
harmonics = independent_harmonics(exact_dynamic,theta);
expected = struct('temperature_case_C',temperature, ...
    'scenario','NOMINAL','bearing',bearing,'selection_role',role, ...
    'phase_count',32,'phase_values_rad',theta,'F_exact',F_exact, ...
    'F_linear',F_linear,'static_force_reference',static_force, ...
    'linear_force_hat',linear_force_hat, ...
    'force_semantics','FORCE_ON_ROTOR_MINUS_AQ', ...
    'epsilon_NL',relative_history_error(exact_dynamic,linear_dynamic), ...
    'loaded_mask_static',loaded_static, ...
    'loaded_mask_history',loaded_history, ...
    'active_set_changed',any(loaded_history ~= loaded_static,'all'), ...
    'Q_min',Q_min,'minimum_loaded_Q',min_loaded(Q_min,loaded_static), ...
    'contact_loss_detected',any(loaded_static & Q_min <= 0), ...
    'force_history_sha256',serialized_sha256(F_exact), ...
    'active_history_sha256',serialized_sha256(loaded_history), ...
    'Q_min_sha256',serialized_sha256(Q_min), ...
    'F1_complex',harmonics.F1_complex,'F2_complex',harmonics.F2_complex, ...
    'F3_complex',harmonics.F3_complex,'H2_H1',harmonics.H2_H1, ...
    'H3_H1',harmonics.H3_H1, ...
    'frozen_input_audit',expected_frozen_audit( ...
    baseline,projection,temperature,bearing));
end

function [force,loaded,Q] = independent_formal_state(q,parameters,bearing_index)
[~,bearing_state] = nonlinear_bearing_force(q,zeros(size(q)), ...
    parameters,parameters.bearing,numel(q));
state = bearing_state.bearings(bearing_index);
force = [state.Fx;state.Fy;state.Fz;state.Mx;state.My];
Q = state.Q(:); loaded = Q > 0;
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

function pass = formal_baseline_available(baseline)
pass = isfield(baseline,'configuration') && ...
    isfield(baseline.configuration,'rotational_speed_rad_s') && ...
    isfield(baseline,'static') && isfield(baseline.static,'T20') && ...
    isfield(baseline.static.T20,'bearing_mapping') && ...
    isfield(baseline,'frequency_response');
end

function selection = expected_selection(baseline)
best = -Inf; selection = struct('temperature',NaN,'bearing','','label','');
for temperature = [50 80 100]
    tf = sprintf('T%d',temperature);
    for bearing = {'front','rear'}
        name = bearing{1}; value = baseline.nonlinearity_gate.(tf).(name).epsilon_NL;
        if value > best
            best = value; selection.temperature = temperature;
            selection.bearing = name;
        end
    end
end
selection.label = sprintf('T%d_NOMINAL_%s', ...
    selection.temperature,selection.bearing);
end

function pass = frozen_audit_matches(audit,baseline,projection,temperature,bearing)
expected = expected_frozen_audit(baseline,projection,temperature,bearing);
pass = isequaln(audit,expected);
end

function expected = expected_frozen_audit(baseline,projection,temperature,bearing)
tf = sprintf('T%d',temperature); item = projection.temperature_cases.(tf);
if strcmp(bearing,'front')
    contact_hash = item.contact_state_front_sha256;
    viscosity = viscosity_source(baseline.static.(tf),'front');
else
    contact_hash = item.contact_state_rear_sha256;
    viscosity = viscosity_source(baseline.static.(tf),'rear');
end
expected = struct('static_state_summary_sha256', ...
    item.static_state_summary_sha256,'thermal_state_sha256', ...
    item.thermal_state_sha256,'viscosity_sha256',serialized_sha256(viscosity), ...
    'clearance_sha256',item.clearance_sha256, ...
    'contact_state_sha256',contact_hash, ...
    'q_static_sha256',serialized_sha256(baseline.static.(tf).q_static), ...
    'passed',true);
end

function value = viscosity_source(state,bearing)
value = [];
if strcmp(bearing,'front') && isfield(state.thermal_state,'ball') && ...
        isfield(state.thermal_state.ball,'eta')
    value = state.thermal_state.ball.eta;
elseif strcmp(bearing,'rear') && isfield(state.thermal_state,'roller') && ...
        isfield(state.thermal_state.roller,'eta')
    value = state.thermal_state.roller.eta;
end
end

function pass = same_without_representative(candidate,baseline)
try
    projection = candidate;
    if isfield(projection,'nonlinear_validation') && ...
            isstruct(projection.nonlinear_validation) && ...
            isfield(projection.nonlinear_validation,'representative')
        projection.nonlinear_validation = rmfield( ...
            projection.nonlinear_validation,'representative');
        if isempty(fieldnames(projection.nonlinear_validation)) && ...
                ~isfield(baseline,'nonlinear_validation')
            projection = rmfield(projection,'nonlinear_validation');
        end
    end
    pass = isequaln(projection,baseline);
catch
    pass = false;
end
end

function value = relative_history_error(exact_dynamic,linear_dynamic)
numerator = sqrt(mean(sum(abs(exact_dynamic-linear_dynamic).^2,1)));
denominator = sqrt(mean(sum(abs(exact_dynamic).^2,1)));
if numerator == 0 && denominator == 0
    value = 0;
else
    value = numerator/max(denominator,eps);
end
end

function out = independent_harmonics(history,theta)
n = numel(theta); F1 = (2/n)*history*exp(-1i*theta).';
F2 = (2/n)*history*exp(-2i*theta).';
F3 = (2/n)*history*exp(-3i*theta).'; norm1 = norm(F1,2);
out = struct('F1_complex',F1,'F2_complex',F2,'F3_complex',F3, ...
    'H2_H1',norm(F2,2)/max(norm1,eps), ...
    'H3_H1',norm(F3,2)/max(norm1,eps));
end

function value = min_loaded(Q_min,loaded)
values = Q_min(loaded);
if isempty(values), value = NaN; else, value = min(values); end
end

function pass = same_numeric(a,b)
pass = isnumeric(a) && isnumeric(b) && isequal(size(a),size(b)) && ...
    all(isfinite(a),'all') && all(isfinite(b),'all') && ...
    norm(a(:)-b(:)) <= 1e-12*max(norm(b(:)),1);
end

function pass = same_scalar(a,b)
pass = isnumeric(a) && isscalar(a) && isfinite(a) && isfinite(b) && ...
    abs(a-b) <= 1e-12*max(abs(b),1);
end

function pass = same_scalar_or_nan(a,b)
pass = (isnan(a) && isnan(b)) || same_scalar(a,b);
end

function pass = same_value(a,b)
if isnumeric(a) && isnumeric(b)
    if isscalar(a) && isscalar(b) && (isnan(a) || isnan(b))
        pass = same_scalar_or_nan(a,b);
    else
        pass = same_numeric(a,b);
    end
else
    pass = isequaln(a,b);
end
end

function hash = serialized_sha256(value)
bytes = getByteStreamFromArray(value);
engine = java.security.MessageDigest.getInstance('SHA-256');
engine.update(typecast(uint8(bytes),'int8'));
digest = typecast(engine.digest(),'uint8');
hash = lower(reshape(dec2hex(digest,2).',1,[]));
end

function validation = finish(diagnostics,allow,gate)
diagnostics = unique(diagnostics,'stable');
validation = struct('passed',isempty(diagnostics), ...
    'diagnostics',{diagnostics},'gate_status',gate,'allow_stage_g',allow);
end
