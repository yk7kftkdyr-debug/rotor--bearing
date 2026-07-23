function audit = build_thermal_validity_audit(results, cfg)
%BUILD_THERMAL_VALIDITY_AUDIT Pure in-memory audit of four saved thermal cases.
% This function intentionally does not call solvers, create files, or alter results.

results_before = results;
validate_config(cfg);
records = require_records(results);
temperatures = arrayfun(@(r) require_scalar(r.meta, 'T_oil_C', 'meta'), records);
if ~isequal(temperatures, [20 50 80 100])
    error('ThermalValidityAudit:TemperatureOrder', 'case_records must be ordered at 20/50/80/100 C.');
end

case_template = empty_case_audit();
case_audit = repmat(case_template, 4, 1);
for k = 1:4
    record = records(k);
    case_audit(k).case_index = k;
    case_audit(k).T_oil_C = temperatures(k);
    case_audit(k).cold_start_fallback_used = logical_field(record.initialization, 'cold_start_fallback_used', false);
    case_audit(k).path_independence_verified = false;
    case_audit(k).ball = bearing_audit(record, 'ball', cfg, cfg.model.H_ball_W_K);
    case_audit(k).roller = bearing_audit(record, 'roller', cfg, cfg.model.H_roller_W_K);
    case_audit(k).modal = modal_audit(record, cfg);
    case_audit(k).initial_condition = initial_condition_audit(record, case_audit(k).modal, cfg);
end

for k = 1:4
    case_audit(k).ball.Cxx_ratio_to_20C = ratio_to_reference( ...
        case_audit(k).ball.Cxx_Ns_m, case_audit(1).ball.Cxx_Ns_m);
    case_audit(k).ball.Cyy_ratio_to_20C = ratio_to_reference( ...
        case_audit(k).ball.Cyy_Ns_m, case_audit(1).ball.Cyy_Ns_m);
    case_audit(k).roller.Cxx_ratio_to_20C = ratio_to_reference( ...
        case_audit(k).roller.Cxx_Ns_m, case_audit(1).roller.Cxx_Ns_m);
    case_audit(k).roller.Cyy_ratio_to_20C = ratio_to_reference( ...
        case_audit(k).roller.Cyy_Ns_m, case_audit(1).roller.Cyy_Ns_m);
end

audit = struct();
audit.version = "thermal-validity-audit-v1";
audit.case_audit = case_audit;
audit.input_manifest = struct('temperature_list_C', temperatures, 'case_count', 4, ...
    'roughness_source_note', string(cfg.roughness.source_note), ...
    'rotation_speed_rpm', cfg.model.rotation_speed_rpm, ...
    'H_ball_W_K', cfg.model.H_ball_W_K, 'H_roller_W_K', cfg.model.H_roller_W_K, ...
    'file_output_attempted', false, 'solver_called', false);
audit.memory_gate = struct('results_unchanged', isequaln(results_before, results), ...
    'all_lambda_computed', all_lambda_computed(case_audit), ...
    'all_friction_heat_finite', all_friction_heat_finite(case_audit), ...
    'low_frequency_modal_peak_rejected', low_frequency_rejected(case_audit), ...
    'unreliable_damping_is_nan', unreliable_damping_is_nan(case_audit), ...
    'ehl_validity_flags_correct', ehl_flags_correct(case_audit));
audit.pass = audit.memory_gate.results_unchanged && audit.memory_gate.all_lambda_computed && ...
    audit.memory_gate.all_friction_heat_finite && audit.memory_gate.low_frequency_modal_peak_rejected && ...
    audit.memory_gate.unreliable_damping_is_nan && audit.memory_gate.ehl_validity_flags_correct;
end

function validate_config(cfg)
if ~isstruct(cfg) || ~isscalar(cfg), error('ThermalValidityAudit:Config', 'cfg must be a scalar struct.'); end
required = {'model','roughness','lambda','spectrum','damping','export'};
if ~all(isfield(cfg, required)), error('ThermalValidityAudit:Config', 'cfg must come from thermal_validity_audit_config.'); end
positive_scalar(cfg.model.rotation_speed_rpm, 'model.rotation_speed_rpm');
positive_scalar(cfg.model.H_ball_W_K, 'model.H_ball_W_K');
positive_scalar(cfg.model.H_roller_W_K, 'model.H_roller_W_K');
positive_scalar(cfg.roughness.ball.composite_rq_m, 'roughness.ball.composite_rq_m');
positive_scalar(cfg.roughness.roller.composite_rq_m, 'roughness.roller.composite_rq_m');
if ~(ischar(cfg.roughness.source_note) || isstring(cfg.roughness.source_note)) || strlength(string(cfg.roughness.source_note)) == 0
    error('ThermalValidityAudit:RoughnessSource', 'roughness.source_note must be explicitly supplied.');
end
end

function positive_scalar(value, name)
if ~(isnumeric(value) && isscalar(value) && isfinite(value) && value > 0)
    error('ThermalValidityAudit:MissingPhysicalMetadata', '%s must be a finite positive explicit value.', name);
end
end

function records = require_records(results)
if ~isstruct(results) || ~isscalar(results) || ~isfield(results, 'case_records') || numel(results.case_records) ~= 4
    error('ThermalValidityAudit:ResultsSchema', 'results.case_records must contain exactly four cases.');
end
records = results.case_records(:).';
for k = 1:4
    if ~isfield(records(k), 'meta') || ~isfield(records(k), 'thermal') || ~isfield(records(k), 'bearing') || ...
            ~isfield(records(k), 'modal') || ~isfield(records(k), 'response')
        error('ThermalValidityAudit:ResultsSchema', 'Case %d lacks the required saved-result schema.', k);
    end
end
end

function entry = empty_case_audit()
entry = struct('case_index', NaN, 'T_oil_C', NaN, 'cold_start_fallback_used', false, ...
    'path_independence_verified', false, 'ball', struct(), 'roller', struct(), ...
    'modal', repmat(empty_modal_entry(), 1, 6), 'initial_condition', struct());
end

function value = require_scalar(s, name, scope)
if ~isfield(s, name) || ~(isnumeric(s.(name)) && isscalar(s.(name)) && isfinite(s.(name)))
    error('ThermalValidityAudit:MissingField', 'Missing finite %s.%s.', scope, name);
end
value = s.(name);
end

function value = logical_field(s, name, fallback)
if isfield(s, name) && islogical(s.(name)) && isscalar(s.(name)), value = s.(name); else, value = fallback; end
end

function entry = bearing_audit(record, type, cfg, H_W_K)
thermal = require_field(record.thermal, type, 'thermal');
bearing = require_field(record.bearing, type, 'bearing');
roughness = cfg.roughness.(type).composite_rq_m;
film = require_positive(thermal, 'minimum_loaded_film_m', ['thermal.' type]);
power = require_finite(thermal, 'friction_power_W', ['thermal.' type]);
Tfinal = require_finite(thermal, 'T_final_C', ['thermal.' type]);
Tinner = require_finite(thermal, 'T_inner_C', ['thermal.' type]);
Touter = require_finite(thermal, 'T_outer_C', ['thermal.' type]);
Telement = require_finite(thermal, 'T_element_C', ['thermal.' type]);
working = require_finite(thermal, 'working_clearance_m', ['thermal.' type]);
change = require_finite(thermal, 'clearance_change_m', ['thermal.' type]);
lambda = film / roughness;
[regime, full_film, mixed] = classify_lambda(lambda, cfg.lambda);
omega = 2*pi*cfg.model.rotation_speed_rpm/60;
deltaT = Tfinal - record.meta.T_oil_C;
heat_error = power - H_W_K * deltaT;
entry = struct('bearing_type', string(type), 'composite_rq_m', roughness, 'lambda_ratio', lambda, ...
    'lubrication_regime', regime, 'full_film_assumption_plausible', full_film, ...
    'mixed_lubrication_warning', mixed, 'friction_power_W', power, ...
    'equivalent_friction_torque_Nm', power / omega, 'heat_balance_error_W', heat_error, ...
    'heat_balance_relative_error', abs(heat_error) / max(abs(power), 1), ...
    'implied_heat_transfer_W_K', safe_ratio(power, deltaT), ...
    'friction_absolute_value_calibrated', false, 'reference_clearance_m', working - change, ...
    'working_clearance_m', working, 'clearance_change_m', change, 'clearance_model_calibrated', false, ...
    'T_inner_C', Tinner, 'T_outer_C', Touter, 'T_element_C', Telement, ...
    'inner_outer_temperature_difference_C', Tinner - Touter, ...
    'clearance_uses_reduced_temperature_partition', true, 'fit_and_housing_expansion_included', false, ...
    'clearance_absolute_trend_calibrated', false, 'C_local_original', original_matrix(bearing), ...
    'C_local_available', ~isempty(original_matrix(bearing)), ...
    'Cxx_Ns_m', optional_finite(bearing, 'radial_Cxx_Ns_m'), ...
    'Cyy_Ns_m', optional_finite(bearing, 'radial_Cyy_Ns_m'), ...
    'Cxx_ratio_to_20C', NaN, 'Cyy_ratio_to_20C', NaN, ...
    'ehl_damping_validity', damping_validity(lambda, cfg.lambda.full_film_min));
end

function value = require_field(s, name, scope)
if ~isfield(s, name), error('ThermalValidityAudit:MissingField', 'Missing %s.%s.', scope, name); end
value = s.(name);
end

function value = require_finite(s, name, scope)
value = require_field(s, name, scope);
if ~(isnumeric(value) && isscalar(value) && isfinite(value))
    error('ThermalValidityAudit:MissingField', '%s.%s must be a finite scalar.', scope, name);
end
end

function value = require_positive(s, name, scope)
value = require_finite(s, name, scope);
if value <= 0, error('ThermalValidityAudit:InvalidFilm', '%s.%s must be positive.', scope, name); end
end

function value = optional_finite(s, name)
if isfield(s, name) && isnumeric(s.(name)) && isscalar(s.(name)) && isfinite(s.(name)), value = s.(name); else, value = NaN; end
end

function value = original_matrix(s)
if isfield(s, 'C_ehl_local'), value = s.C_ehl_local; else, value = []; end
end

function value = safe_ratio(numerator, denominator)
if isfinite(denominator) && denominator ~= 0, value = numerator / denominator; else, value = NaN; end
end

function [regime, full_film, mixed] = classify_lambda(lambda, thresholds)
if lambda >= thresholds.full_film_min
    regime = "FULL_FILM_PLAUSIBLE"; full_film = true; mixed = false;
elseif lambda >= thresholds.transition_min
    regime = "TRANSITION"; full_film = false; mixed = true;
elseif lambda >= thresholds.mixed_min
    regime = "MIXED_LUBRICATION"; full_film = false; mixed = true;
else
    regime = "SEVERE_MIXED_BOUNDARY_RISK"; full_film = false; mixed = true;
end
end

function value = damping_validity(lambda, full_film_min)
if lambda < full_film_min, value = "MODEL_SENSITIVITY_ONLY"; else, value = "FULL_FILM_MODEL_WITHIN_LAMBDA_SCOPE"; end
end

function value = ratio_to_reference(value, reference)
if isfinite(value) && isfinite(reference) && reference ~= 0, value = value/reference; else, value = NaN; end
end

function modes = modal_audit(record, cfg)
response = record.response;
time = column_vector(require_field(response, 'time_s', 'response'));
X = require_field(response, 'displacement_m', 'response');
if ~isnumeric(X) || size(X,1) ~= numel(time) || size(X,2) ~= 10 || any(~isfinite(X), 'all')
    error('ThermalValidityAudit:ResponseSchema', 'response.displacement_m must be finite N-by-10 data.');
end
[is_uniform, dt] = uniform_time(time, cfg.spectrum.uniform_time_relative_tolerance);
tracked = require_field(record.modal, 'tracked_frequency_Hz', 'modal');
if ~isnumeric(tracked) || numel(tracked) < cfg.spectrum.tracked_mode_count || any(~isfinite(tracked(1:6))) || any(tracked(1:6) <= 0)
    error('ThermalValidityAudit:TrackedModes', 'The first six tracked modal frequencies must be finite and positive.');
end
modes = repmat(empty_modal_entry(), 1, 6);
if numel(time) < cfg.spectrum.minimum_samples || ~is_uniform
    reason = "NONUNIFORM_OR_INSUFFICIENT_TIME_SAMPLES";
    for m = 1:6
        modes(m).tracked_mode_index = m; modes(m).tracked_frequency_Hz = tracked(m); modes(m).failure_reason = reason;
    end
    return
end

[frequency, amplitude, detrended] = spectra_for_channels(time, X, dt);
labels = channel_labels(response);
for m = 1:6
    fi = tracked(m); half_width = modal_half_width(tracked(1:6), m, 1/(numel(time)*dt), cfg.spectrum);
    modes(m) = select_modal_peak(frequency, amplitude, detrended, dt, labels, fi, half_width, tracked(1), m, cfg);
end
end

function entry = empty_modal_entry()
entry = struct('tracked_mode_index', NaN, 'tracked_frequency_Hz', NaN, 'selected_response_channel', "", ...
    'selected_response_channel_index', NaN, 'dominant_modal_frequency_Hz', NaN, ...
    'frequency_error_to_tracked_mode', NaN, 'modal_peak_to_floor_ratio', NaN, 'modal_peak_valid', false, ...
    'identified_modal_frequency_Hz', NaN, 'identified_modal_damping_ratio', NaN, ...
    'log_decrement', NaN, 'fit_R2', NaN, 'attenuation_rate_per_s', NaN, ...
    'same_direction_peak_count', 0, 'high_damping_warning', false, 'failure_reason', "NOT_EVALUATED");
end

function vector = column_vector(value)
if ~isnumeric(value) || ~isvector(value) || any(~isfinite(value)), error('ThermalValidityAudit:Time', 'response.time_s must be a finite numeric vector.'); end
vector = value(:);
if any(diff(vector) <= 0), error('ThermalValidityAudit:Time', 'response.time_s must increase strictly.'); end
end

function [ok, dt] = uniform_time(time, tolerance)
d = diff(time); dt = median(d); ok = isfinite(dt) && dt > 0 && max(abs(d-dt)) <= tolerance * max(dt, 1);
end

function [frequency, amplitude, detrended] = spectra_for_channels(time, X, dt)
n = numel(time); detrended = detrend(X - mean(X,1));
window = 0.5 - 0.5*cos(2*pi*(0:n-1)'/(n-1)); coherent_gain = mean(window);
Y = fft(detrended .* window, [], 1); n_one = floor(n/2)+1;
frequency = (0:n_one-1)'/(n*dt); amplitude = abs(Y(1:n_one,:))/(n*coherent_gain);
if n_one > 2, amplitude(2:end-1,:) = 2*amplitude(2:end-1,:); end
end

function labels = channel_labels(response)
nodes = string(require_field(response, 'node_labels', 'response')); components = string(require_field(response, 'component_labels', 'response'));
if numel(nodes) ~= 5 || numel(components) ~= 2, error('ThermalValidityAudit:ResponseLabels', 'Saved response labels must contain five nodes and two components.'); end
labels = strings(1,10);
for k = 1:5
    labels(2*k-1) = nodes(k) + "-" + components(1); labels(2*k) = nodes(k) + "-" + components(2);
end
end

function half_width = modal_half_width(tracked, index, df, settings)
half_width = max(settings.half_width_bins*df, settings.half_width_fraction*tracked(index));
if index == 1, neighbor_gap = tracked(2)-tracked(1); elseif index == numel(tracked), neighbor_gap = tracked(end)-tracked(end-1); else, neighbor_gap = min(tracked(index)-tracked(index-1), tracked(index+1)-tracked(index)); end
if ~(isfinite(neighbor_gap) && neighbor_gap > 0), half_width = NaN; else, half_width = min(half_width, settings.maximum_neighbor_fraction*neighbor_gap); end
end

function entry = select_modal_peak(frequency, amplitude, detrended, dt, labels, fi, half_width, f1, index, cfg)
entry = empty_modal_entry(); entry.tracked_mode_index = index; entry.tracked_frequency_Hz = fi;
if ~isfinite(half_width) || half_width <= 0
    entry.failure_reason = "INVALID_MODAL_WINDOW"; return
end
mask = frequency >= fi-half_width & frequency <= fi+half_width & frequency >= cfg.spectrum.minimum_modal_fraction_of_f1*f1 & frequency > 0;
if nnz(mask) < 3, entry.failure_reason = "MODAL_WINDOW_HAS_TOO_FEW_FFT_BINS"; return; end
best_ratio = -Inf; best = struct('channel', NaN, 'bin', NaN, 'ratio', NaN);
for ch = 1:size(amplitude,2)
    [peak, local] = max(amplitude(mask,ch)); bins = find(mask); bin = bins(local);
    floor_value = local_floor(amplitude(:,ch), bins, bin);
    ratio = peak / floor_value;
    if ratio > best_ratio, best_ratio = ratio; best = struct('channel',ch,'bin',bin,'ratio',ratio); end
end
entry.selected_response_channel = labels(best.channel); entry.selected_response_channel_index = best.channel;
entry.dominant_modal_frequency_Hz = frequency(best.bin); entry.frequency_error_to_tracked_mode = frequency(best.bin)-fi;
entry.modal_peak_to_floor_ratio = best.ratio;
entry.modal_peak_valid = isfinite(best.ratio) && best.ratio >= cfg.spectrum.minimum_peak_to_floor_ratio && ...
    frequency(best.bin) >= cfg.spectrum.minimum_modal_fraction_of_f1*f1;
if ~entry.modal_peak_valid
    entry.failure_reason = "MODAL_PEAK_TO_FLOOR_BELOW_THRESHOLD"; return
end
entry.identified_modal_frequency_Hz = entry.dominant_modal_frequency_Hz;
[zeta, delta, R2, rate, count, reason] = identify_damping(detrended(:,best.channel), dt, entry.dominant_modal_frequency_Hz, half_width, best.ratio, cfg.damping, cfg.spectrum.minimum_peak_to_floor_ratio);
entry.same_direction_peak_count = count; entry.fit_R2 = R2; entry.attenuation_rate_per_s = rate;
if isfinite(zeta)
    entry.identified_modal_damping_ratio = zeta; entry.log_decrement = delta; entry.high_damping_warning = zeta > cfg.damping.high_damping_warning_ratio; entry.failure_reason = "";
else
    entry.failure_reason = reason;
end
end

function floor_value = local_floor(A, bins, peak_bin)
exclude = abs(bins-peak_bin) <= 1; candidate = A(bins(~exclude)); candidate = candidate(isfinite(candidate) & candidate >= 0);
if isempty(candidate), candidate = A(bins); end
floor_value = max(median(candidate), eps(max(1, A(peak_bin))));
end

function [zeta, delta, R2, rate, count, reason] = identify_damping(x, dt, frequency_Hz, half_width, peak_ratio, settings, min_ratio)
zeta = NaN; delta = NaN; R2 = NaN; rate = NaN; count = 0; reason = "DAMPING_NOT_IDENTIFIED";
if peak_ratio < min_ratio, reason = "MODAL_PEAK_TO_FLOOR_BELOW_THRESHOLD"; return; end
n = numel(x); fs = 1/dt; signed_frequency = (0:n-1)'*fs/n; signed_frequency(signed_frequency > fs/2) = signed_frequency(signed_frequency > fs/2)-fs;
distance = abs(abs(signed_frequency)-frequency_Hz); inner = settings.cosine_taper_inner_fraction*half_width;
weight = zeros(n,1); weight(distance <= inner) = 1; taper = distance > inner & distance <= half_width;
weight(taper) = 0.5*(1+cos(pi*(distance(taper)-inner)/(half_width-inner)));
narrow = real(ifft(fft(x).*weight)); edge = max(1, ceil(5/(frequency_Hz*dt)));
if n <= 2*edge+2, reason = "RESPONSE_TOO_SHORT_AFTER_CYCLE_EXCLUSION"; return; end
valid = (edge+1):(n-edge); [locations, values] = same_direction_peaks(narrow(valid)); locations = locations + edge;
count = numel(values);
if count < settings.minimum_same_direction_peaks, reason = "INSUFFICIENT_SAME_DIRECTION_PEAKS"; return; end
if values(end) >= values(1), reason = "PEAKS_NOT_OVERALL_DECREASING"; return; end
t = (locations-1)*dt; log_values = log(values); coefficient = polyfit(t, log_values, 1); fitted = polyval(coefficient, t);
R2 = 1-sum((log_values-fitted).^2)/max(sum((log_values-mean(log_values)).^2), eps);
rate = -coefficient(1);
if ~(isfinite(R2) && R2 >= settings.minimum_fit_R2), reason = "LOG_PEAK_FIT_R2_BELOW_THRESHOLD"; return; end
if ~(isfinite(rate) && rate > 0), reason = "NONPOSITIVE_ATTENUATION_RATE"; return; end
delta = rate*median(diff(t)); zeta = delta/sqrt((2*pi)^2+delta^2);
if ~(zeta > settings.minimum_damping_ratio && zeta < settings.maximum_damping_ratio)
    zeta = NaN; delta = NaN; reason = "DAMPING_RATIO_OUTSIDE_VALID_RANGE";
end
end

function [locations, values] = same_direction_peaks(x)
positive = find(x(2:end-1) > x(1:end-2) & x(2:end-1) >= x(3:end)) + 1;
negative = find(x(2:end-1) < x(1:end-2) & x(2:end-1) <= x(3:end)) + 1;
if numel(positive) >= numel(negative), locations = positive; values = x(positive); else, locations = negative; values = -x(negative); end
locations = locations(values > 0); values = values(values > 0);
end

function entry = initial_condition_audit(record, modes, ~)
response = record.response; time = column_vector(response.time_s); X = response.displacement_m; labels = channel_labels(response);
index = find(labels == "R16-x", 1); f1 = modes(1).tracked_frequency_Hz;
entry = struct('R16_peak_is_initial_condition_controlled', true, 'first_cycle_time_s', 1/f1, ...
    'peak_after_first_cycle_m', NaN, 'rms_after_first_cycle_m', NaN, ...
    'energy_decay_to_10pct_time_s', NaN, 'energy_decay_to_1pct_time_s', NaN, ...
    'energy_decay_method', "FIT_BASED_NARROWBAND_ENERGY");
after = time >= 1/f1;
if any(after), entry.peak_after_first_cycle_m = max(abs(X(after,index))); entry.rms_after_first_cycle_m = sqrt(mean(X(after,index).^2)); end
if isfinite(modes(1).attenuation_rate_per_s) && modes(1).attenuation_rate_per_s > 0
    entry.energy_decay_to_10pct_time_s = log(10)/(2*modes(1).attenuation_rate_per_s);
    entry.energy_decay_to_1pct_time_s = log(100)/(2*modes(1).attenuation_rate_per_s);
end
end

function value = all_lambda_computed(cases)
value = all(arrayfun(@(c) isfinite(c.ball.lambda_ratio) && isfinite(c.roller.lambda_ratio), cases));
end

function value = all_friction_heat_finite(cases)
value = all(arrayfun(@(c) all(isfinite([c.ball.equivalent_friction_torque_Nm c.ball.heat_balance_error_W c.roller.equivalent_friction_torque_Nm c.roller.heat_balance_error_W])), cases));
end

function value = low_frequency_rejected(cases)
value = true;
for k = 1:numel(cases)
    f1 = cases(k).modal(1).tracked_frequency_Hz;
    for m = 1:numel(cases(k).modal)
        item = cases(k).modal(m);
        if item.modal_peak_valid && item.dominant_modal_frequency_Hz < 0.5*f1, value = false; return; end
    end
end
end

function value = unreliable_damping_is_nan(cases)
value = true;
for k = 1:numel(cases)
    for m = 1:numel(cases(k).modal)
        item = cases(k).modal(m);
        if strlength(item.failure_reason) > 0 && isfinite(item.identified_modal_damping_ratio), value = false; return; end
    end
end
end

function value = ehl_flags_correct(cases)
value = true;
for k = 1:numel(cases)
    for type = ["ball" "roller"]
        entry = cases(k).(type);
        expected = damping_validity(entry.lambda_ratio, 1.2);
        if entry.ehl_damping_validity ~= expected, value = false; return; end
    end
end
end
