function metrics = extract_stage_e_frequency_response_metrics(qhat_full,omega,mapping)
%EXTRACT_STAGE_E_FREQUENCY_RESPONSE_METRICS Derive SI 1X response quantities.

qhat_full = qhat_full(:);
if ~isscalar(omega) || ~isfinite(omega) || omega <= 0 || ...
        ~isnumeric(qhat_full) || any(~isfinite(qhat_full))
    error('StageE:Metrics','A finite response vector and positive frequency are required.');
end
required = {'rotor','casing','front_relative_transform','rear_relative_transform'};
if ~isstruct(mapping) || ~all(isfield(mapping,required)) || ...
        ~isfield(mapping,'mapping_validation_required') || ...
        ~isfield(mapping,'validated') || ...
        ~isequal(mapping.mapping_validation_required,true) || ~isequal(mapping.validated,true)
    error('StageE:MetricsMapping','Mapping is incomplete.');
end
validate_mapping(mapping,numel(qhat_full));

metrics = struct('omega_rad_s',omega,'frequency_Hz',omega/(2*pi));
for kind = {'rotor','casing'}
    group = kind{1}; names = fieldnames(mapping.(group));
    for j = 1:numel(names)
        name = names{j}; dofs = mapping.(group).(name);
        component = component_metrics(qhat_full(dofs(:)),omega);
        metrics.([group '_metrics']).(name) = component;
        metrics.phase_metrics.(name) = phase_metrics(component);
        metrics.orbit_metrics.(name) = orbit_metrics(component.displacement_complex_m);
    end
end

metrics.bearing_interface_metrics.front = interface_metrics( ...
    mapping.front_relative_transform*qhat_full,omega);
metrics.bearing_interface_metrics.rear = interface_metrics( ...
    mapping.rear_relative_transform*qhat_full,omega);
R2 = metrics.rotor_metrics.R2; R10 = metrics.rotor_metrics.R10;
C2 = metrics.casing_metrics.C2; C8 = metrics.casing_metrics.C8;
denominator_threshold_m = 1e-12;
metrics.transfer_metrics.front = transmissibility(C2.amplitude_vector_m,R2.amplitude_vector_m,denominator_threshold_m);
metrics.transfer_metrics.rear = transmissibility(C8.amplitude_vector_m,R10.amplitude_vector_m,denominator_threshold_m);
metrics.transfer_metrics.front_to_rear_casing = ratio_or_nan(C8.resultant_peak_m,C2.resultant_peak_m,denominator_threshold_m);
metrics.transfer_metrics.front_to_rear_rotor = ratio_or_nan(R10.resultant_peak_m,R2.resultant_peak_m,denominator_threshold_m);
metrics.transfer_metrics.velocity_equals_displacement_at_single_frequency = true;
metrics.transfer_metrics.acceleration_equals_displacement_at_single_frequency = true;
metrics.transfer_metrics.denominator_threshold_m = denominator_threshold_m;
end

function value = component_metrics(q,omega)
if numel(q) ~= 2, error('StageE:MetricsMapping','Each response point needs x and y DOFs.'); end
q = q(:);
amplitude = abs(q);
resultant = hypot(amplitude(1),amplitude(2));
radial_rms = resultant/sqrt(2);
value = struct('displacement_complex_m',q,'amplitude_vector_m',amplitude, ...
    'x_peak_m',amplitude(1),'y_peak_m',amplitude(2), ...
    'x_rms_m',amplitude(1)/sqrt(2),'y_rms_m',amplitude(2)/sqrt(2), ...
    'resultant_peak_m',resultant,'radial_peak_m',resultant, ...
    'radial_rms_m',radial_rms, ...
    'velocity_complex_m_s',1i*omega*q, ...
    'acceleration_complex_m_s2',-omega^2*q, ...
    'x_velocity_peak_m_s',omega*amplitude(1), ...
    'y_velocity_peak_m_s',omega*amplitude(2), ...
    'x_velocity_rms_m_s',omega*amplitude(1)/sqrt(2), ...
    'y_velocity_rms_m_s',omega*amplitude(2)/sqrt(2), ...
    'x_acceleration_peak_m_s2',omega^2*amplitude(1), ...
    'y_acceleration_peak_m_s2',omega^2*amplitude(2), ...
    'x_acceleration_rms_m_s2',omega^2*amplitude(1)/sqrt(2), ...
    'y_acceleration_rms_m_s2',omega^2*amplitude(2)/sqrt(2), ...
    'radial_velocity_peak_m_s',omega*resultant, ...
    'radial_velocity_rms_m_s',omega*radial_rms, ...
    'radial_acceleration_peak_m_s2',omega^2*resultant, ...
    'radial_acceleration_rms_m_s2',omega^2*radial_rms, ...
    'x_phase_rad',angle(q(1)),'y_phase_rad',angle(q(2)));
end

function value = phase_metrics(component)
threshold = 1e-12;
value = struct('x_phase_rad',component.x_phase_rad,'y_phase_rad',component.y_phase_rad, ...
    'x_near_zero_amplitude',component.x_peak_m <= threshold, ...
    'y_near_zero_amplitude',component.y_peak_m <= threshold);
value.x_status = phase_status(value.x_near_zero_amplitude);
value.y_status = phase_status(value.y_near_zero_amplitude);
value.x_phase_reliable = ~value.x_near_zero_amplitude;
value.y_phase_reliable = ~value.y_near_zero_amplitude;
end

function status = phase_status(near_zero)
if near_zero
    status = 'PHASE_UNRELIABLE_DUE_TO_NEAR_ZERO_AMPLITUDE';
else
    status = 'PHASE_VALID';
end
end

function value = orbit_metrics(q)
A = [real(q(1)) -imag(q(1)); real(q(2)) -imag(q(2))];
[U,S] = svd(A); s = diag(S);
principal_axis_angle_rad = atan2(U(2,1),U(1,1));
near_zero_threshold_m = 100*eps;
linear_axis_ratio_threshold = 1e-8;
near_circular_axis_ratio_threshold = 1-1e-8;
if s(1) <= near_zero_threshold_m
    axis_ratio = 0;
    rotation_direction = 'LINEAR';
    rotation_status = 'ROTATION_DIRECTION_UNRELIABLE_DUE_TO_NEAR_ZERO_ORBIT';
    rotation_reliable = false;
    principal_axis_valid = false;
    principal_axis_status = 'PRINCIPAL_AXIS_UNDEFINED_NEAR_ZERO_ORBIT';
else
    axis_ratio = s(2)/s(1);
    relative_determinant = det(A)/(s(1)*s(2));
    if axis_ratio <= linear_axis_ratio_threshold || ...
            abs(relative_determinant) <= linear_axis_ratio_threshold
        rotation_direction = 'LINEAR';
        rotation_status = 'ROTATION_DIRECTION_UNRELIABLE_DUE_TO_LINEAR_ORBIT';
        rotation_reliable = false;
        principal_axis_valid = true;
        principal_axis_status = 'PRINCIPAL_AXIS_VALID';
    elseif relative_determinant < 0
        rotation_direction = 'CLOCKWISE';
        rotation_status = 'ROTATION_DIRECTION_VALID';
        rotation_reliable = true;
        principal_axis_valid = axis_ratio < near_circular_axis_ratio_threshold;
        principal_axis_status = principal_axis_label(principal_axis_valid);
    else
        rotation_direction = 'COUNTERCLOCKWISE';
        rotation_status = 'ROTATION_DIRECTION_VALID';
        rotation_reliable = true;
        principal_axis_valid = axis_ratio < near_circular_axis_ratio_threshold;
        principal_axis_status = principal_axis_label(principal_axis_valid);
    end
end
value = struct('semi_major_axis_m',s(1),'semi_minor_axis_m',s(2), ...
    'axis_ratio',axis_ratio,'time_domain_map',A, ...
    'principal_axis_angle_rad',principal_axis_angle_rad, ...
    'principal_axis_valid',principal_axis_valid, ...
    'principal_axis_status',principal_axis_status, ...
    'rotation_direction',rotation_direction,'rotation_status',rotation_status, ...
    'rotation_reliable',rotation_reliable);
end

function status = principal_axis_label(valid)
if valid
    status = 'PRINCIPAL_AXIS_VALID';
else
    status = 'PRINCIPAL_AXIS_UNDEFINED_NEAR_CIRCULAR';
end
end

function value = interface_metrics(relative,omega)
value = component_metrics(relative,omega);
value.relative_displacement_complex_m = relative(:);
value.relative_velocity_complex_m_s = 1i*omega*relative(:);
value.relative_acceleration_complex_m_s2 = -omega^2*relative(:);
phase = phase_metrics(value);
value.x_phase_reliable = phase.x_phase_reliable;
value.y_phase_reliable = phase.y_phase_reliable;
value.x_phase_status = phase.x_status;
value.y_phase_status = phase.y_status;
end

function value = transmissibility(numerator,denominator,threshold)
denominator_norm = norm(denominator);
if denominator_norm <= threshold
    value = struct('displacement_transmissibility',NaN, ...
        'velocity_transmissibility',NaN,'acceleration_transmissibility',NaN, ...
        'valid',false,'status','TRANSMISSIBILITY_DENOMINATOR_NEAR_ZERO');
else
    ratio = norm(numerator)/denominator_norm;
    value = struct('displacement_transmissibility',ratio, ...
        'velocity_transmissibility',ratio,'acceleration_transmissibility',ratio, ...
        'valid',true,'status','TRANSMISSIBILITY_VALID');
end
end

function value = ratio_or_nan(numerator,denominator,threshold)
if denominator <= threshold
    value = struct('value',NaN,'valid',false, ...
        'status','TRANSMISSIBILITY_DENOMINATOR_NEAR_ZERO');
else
    value = struct('value',numerator/denominator,'valid',true, ...
        'status','TRANSMISSIBILITY_VALID');
end
end

function validate_mapping(mapping,n)
required_points = {'R2','R10'};
for group = {'rotor','casing'}
    name = group{1};
    if ~isstruct(mapping.(name))
        error('StageE:MetricsMapping','Rotor and casing mappings must be structs.');
    end
    if strcmp(name,'casing'), required_points = {'C2','C8'}; end
    for k = 1:numel(required_points)
        point = required_points{k};
        if ~isfield(mapping.(name),point) || ~valid_point_dofs(mapping.(name).(point),n)
            error('StageE:MetricsMapping','Each mapped point requires two unique in-range integer DOFs.');
        end
    end
end
transforms = {mapping.front_relative_transform,mapping.rear_relative_transform};
if any(cellfun(@(T) ~isnumeric(T) || ~isreal(T) || ~isequal(size(T),[2 n]) || ...
        any(~isfinite(T),'all'),transforms))
    error('StageE:MetricsMapping','Interface transforms must be finite real 2-by-n matrices.');
end
end

function valid = valid_point_dofs(dofs,n)
valid = isnumeric(dofs) && isreal(dofs) && isequal(size(dofs(:)),[2 1]) && ...
    all(isfinite(dofs(:))) && all(dofs(:) == floor(dofs(:))) && ...
    all(dofs(:) >= 1 & dofs(:) <= n) && numel(unique(dofs(:))) == 2;
end
