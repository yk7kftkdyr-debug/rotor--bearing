function [record_out, derived] = postprocess_thermal_case(record_in, cfg)
%POSTPROCESS_THERMAL_CASE Compute journal metrics from one saved free-decay case.

validate_record(record_in, cfg);
record_out = record_in;
[time_s, sample_indices, sample_dt_s] = uniform_samples(record_in.response, record_in.dynamics.output_stride);
N = numel(time_s);
if N < 1024, error('Stage9C1:FFTLength', 'At least 1024 uniformly spaced saved samples are required.'); end
displacement = record_in.response.displacement_m(sample_indices, :);
velocity = record_in.response.velocity_m_s(sample_indices, :);
acceleration = record_in.response.acceleration_m_s2(sample_indices, :);
frequency_Hz = (0:floor(N / 2)).' / (N * sample_dt_s);
ceiling_Hz = max(record_in.modal.tracked_frequency_Hz);
if ~isfinite(ceiling_Hz) || ceiling_Hz <= 0, error('Stage9C1:FrequencyCeiling', 'The saved tracked modal frequencies do not provide a positive analysis ceiling.'); end

derived = empty_derived(frequency_Hz, time_s, sample_indices, sample_dt_s, ceiling_Hz);
metrics = record_in.paper_metrics;
for column = 1:10
    node_index = ceil(column / 2); component_index = mod(column - 1, 2) + 1;
    metrics.peak_displacement_m(node_index, component_index) = max(abs(displacement(:, column)));
    metrics.peak_to_peak_displacement_m(node_index, component_index) = max(displacement(:, column)) - min(displacement(:, column));
    metrics.rms_displacement_m(node_index, component_index) = sqrt(mean(displacement(:, column).^2));
    metrics.peak_velocity_m_s(node_index, component_index) = max(abs(velocity(:, column)));
    metrics.rms_velocity_m_s(node_index, component_index) = sqrt(mean(velocity(:, column).^2));
    metrics.peak_acceleration_m_s2(node_index, component_index) = max(abs(acceleration(:, column)));
    metrics.rms_acceleration_m_s2(node_index, component_index) = sqrt(mean(acceleration(:, column).^2));
    [derived.displacement_spectrum(:, column), dominant_frequency_Hz, peak_ratio] = one_sided_spectrum(displacement(:, column), sample_dt_s, ceiling_Hz);
    [derived.velocity_spectrum(:, column), ~, ~] = one_sided_spectrum(velocity(:, column), sample_dt_s, ceiling_Hz);
    [derived.acceleration_spectrum(:, column), ~, ~] = one_sided_spectrum(acceleration(:, column), sample_dt_s, ceiling_Hz);
    metrics.dominant_frequency_Hz(node_index, component_index) = dominant_frequency_Hz;
    derived.dominant_to_second_peak_ratio(node_index, component_index) = peak_ratio;
    [metrics.logarithmic_decrement(node_index, component_index), metrics.identified_damping_ratio(node_index, component_index), ...
        derived.damping_fit_R2(node_index, component_index), derived.damping_fit_valid(node_index, component_index), warning_text] = ...
        identify_single_mode_decay(displacement(:, column), time_s, dominant_frequency_Hz, peak_ratio);
    if strlength(warning_text) > 0
        derived.warnings(end + 1, 1) = sprintf('%s-%s: %s', record_in.response.node_labels(node_index), ...
            record_in.response.component_labels(component_index), warning_text);
    end
end
[metrics, ratio_warnings] = casing_rotor_ratios(metrics, displacement);
derived.warnings = [derived.warnings; ratio_warnings];
derived.warnings(end + 1, 1) = sprintf(['FFT peak search is conservatively limited to %.6g Hz, the largest of the six saved tracked frequencies; ' ...
    'the checkpoint does not retain the highest frequency of all %d reduced modes.'], ceiling_Hz, record_in.modal.retained_mode_count);
record_out.paper_metrics = metrics;
end

function derived = empty_derived(frequency_Hz, time_s, sample_indices, sample_dt_s, ceiling_Hz)
count = numel(frequency_Hz);
derived = struct('frequency_Hz', frequency_Hz, 'displacement_spectrum', nan(count, 10), ...
    'velocity_spectrum', nan(count, 10), 'acceleration_spectrum', nan(count, 10), ...
    'uniform_time_s', time_s, 'uniform_sample_indices', sample_indices, 'uniform_sample_dt_s', sample_dt_s, ...
    'damping_fit_R2', nan(5, 2), 'damping_fit_valid', false(5, 2), ...
    'dominant_to_second_peak_ratio', nan(5, 2), 'analysis_frequency_ceiling_Hz', ceiling_Hz, 'warnings', strings(0, 1));
end

function validate_record(record, cfg)
required = {'meta', 'convergence', 'bearing', 'linearization', 'modal', 'dynamics', 'response', 'paper_metrics'};
if ~isstruct(record) || ~all(isfield(record, required)), error('Stage9C1:RecordSchema', 'The case record does not satisfy the Stage9 schema.'); end
if record.meta.status ~= "completed" || ~record.convergence.pass || ~record.bearing.ball.stiffness_pass || ...
        ~record.bearing.roller.stiffness_pass || ~record.bearing.ball.damping_pass || ~record.bearing.roller.damping_pass || ...
        ~record.linearization.pass || ~record.modal.pass || ~record.dynamics.pass
    error('Stage9C1:RecordGate', 'Only a completed case that passes every Stage9 gate may be postprocessed.');
end
response = record.response;
if ~isequal(string(response.node_labels(:)), string(cfg.output.node_labels(:))) || ...
        ~isequal(string(response.component_labels(:)), string(cfg.output.component_labels(:)))
    error('Stage9C1:ResponseLabels', 'The saved response labels are not R2, R10, R16, C2, C8 with x/y components.');
end
matrices = {response.displacement_m, response.velocity_m_s, response.acceleration_m_s2};
if any(cellfun(@(value) size(value, 2) ~= 10 || size(value, 1) ~= numel(response.time_s) || any(~isfinite(value), 'all'), matrices)) || ...
        numel(response.saved_internal_step_index) ~= numel(response.time_s) || any(~isfinite(response.time_s)) || any(diff(response.time_s) <= 0)
    error('Stage9C1:ResponseData', 'The saved ten-channel response must be finite and strictly time increasing.');
end
end

function [time_s, sample_indices, sample_dt_s] = uniform_samples(response, output_stride)
N = response.fft_valid_sample_count;
if ~isscalar(N) || N ~= floor(N) || N < 2 || N > numel(response.time_s), error('Stage9C1:FFTValidCount', 'fft_valid_sample_count is invalid.'); end
sample_indices = (1:N).'; time_s = response.time_s(sample_indices);
sample_dt_s = response.uniform_sample_dt_s;
if ~isfinite(sample_dt_s) || sample_dt_s <= 0 || any(abs(diff(time_s) - sample_dt_s) > max(1e-12, 100 * eps(sample_dt_s))) || ...
        any(diff(response.saved_internal_step_index(sample_indices)) ~= output_stride)
    error('Stage9C1:UniformSampling', 'The FFT samples are not the contiguous uniform saved-output sequence at output_stride.');
end
end

function [amplitude, dominant_frequency_Hz, peak_ratio] = one_sided_spectrum(signal, sample_dt_s, ceiling_Hz)
N = numel(signal); window = 0.5 - 0.5 * cos(2 * pi * (0:N - 1).' / (N - 1));
amplitude = abs(fft((signal - mean(signal)) .* window)) / sum(window);
amplitude = amplitude(1:floor(N / 2) + 1);
if numel(amplitude) > 2, amplitude(2:end - 1) = 2 * amplitude(2:end - 1); end
frequency_Hz = (0:floor(N / 2)).' / (N * sample_dt_s);
band = frequency_Hz > 0 & frequency_Hz <= ceiling_Hz;
if ~any(band), error('Stage9C1:FrequencyBand', 'No positive FFT bin lies inside the permitted modal-frequency band.'); end
candidate = amplitude(band); candidate_frequency = frequency_Hz(band);
[largest, index] = max(candidate); dominant_frequency_Hz = candidate_frequency(index);
if numel(candidate) < 2, peak_ratio = Inf; else, candidate(index) = -Inf; peak_ratio = largest / max(candidate); end
end

function [logarithmic_decrement, damping_ratio, fit_R2, fit_valid, warning_text] = identify_single_mode_decay(signal, time_s, dominant_frequency_Hz, peak_ratio)
logarithmic_decrement = NaN; damping_ratio = NaN; fit_R2 = NaN; fit_valid = false; warning_text = "";
if ~isfinite(dominant_frequency_Hz) || dominant_frequency_Hz <= 0
    warning_text = "没有正的主频，未识别阻尼。"; return;
end
if ~isfinite(peak_ratio) || peak_ratio < 3
    warning_text = "主频与第二谱峰幅值之比小于 3，拒绝将多模态响应识别为单模态阻尼。"; return;
end
peaks = find(signal(2:end-1) > signal(1:end-2) & signal(2:end-1) >= signal(3:end)) + 1;
peaks = peaks(signal(peaks) >= 0.05 * max(abs(signal)));
if numel(peaks) < 6
    warning_text = "同向有效峰值少于 6 个，未识别阻尼。"; return;
end
amplitude = signal(peaks); peak_time = time_s(peaks);
if amplitude(end) >= amplitude(1)
    warning_text = "有效峰值未总体衰减，未识别阻尼。"; return;
end
coefficients = polyfit(peak_time, log(amplitude), 1); fitted = polyval(coefficients, peak_time);
sum_total = sum((log(amplitude) - mean(log(amplitude))).^2); fit_R2 = 1 - sum((log(amplitude) - fitted).^2) / max(sum_total, realmin);
sigma = -coefficients(1);
if ~isfinite(fit_R2) || fit_R2 < 0.90 || ~isfinite(sigma) || sigma <= 0
    warning_text = "峰值包络对数线性拟合未满足 R²≥0.90 且负斜率条件，未识别阻尼。"; return;
end
omega_d = 2 * pi * dominant_frequency_Hz;
logarithmic_decrement = sigma / dominant_frequency_Hz;
damping_ratio = sigma / sqrt(sigma^2 + omega_d^2);
fit_valid = true;
end

function [metrics, warnings] = casing_rotor_ratios(metrics, displacement)
radial = sqrt(displacement(:, 1:2:end).^2 + displacement(:, 2:2:end).^2);
[metrics.front_casing_rotor_peak_ratio, warning_front_peak] = ratio(max(radial(:, 4)), max(radial(:, 1)));
[metrics.rear_casing_rotor_peak_ratio, warning_rear_peak] = ratio(max(radial(:, 5)), max(radial(:, 2)));
[metrics.front_casing_rotor_rms_ratio, warning_front_rms] = ratio(sqrt(mean(radial(:, 4).^2)), sqrt(mean(radial(:, 1).^2)));
[metrics.rear_casing_rotor_rms_ratio, warning_rear_rms] = ratio(sqrt(mean(radial(:, 5).^2)), sqrt(mean(radial(:, 2).^2)));
warnings = [warning_front_peak; warning_rear_peak; warning_front_rms; warning_rear_rms];
warnings = warnings(strlength(warnings) > 0);
end

function [value, warning_text] = ratio(numerator, denominator)
warning_text = "";
if ~isfinite(numerator) || ~isfinite(denominator) || denominator <= 0
    value = NaN; warning_text = "转子径向响应分母非正或非有限，未计算自由衰减响应比。";
else
    value = numerator / denominator;
end
end
