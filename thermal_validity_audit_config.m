function cfg = thermal_validity_audit_config(overrides)
%THERMAL_VALIDITY_AUDIT_CONFIG Explicit contract for the in-memory V1 audit.
% Physical metadata intentionally has no defaults: it must be supplied by
% the caller before build_thermal_validity_audit is allowed to run.

if nargin < 1, overrides = struct(); end
if ~isstruct(overrides) || ~isscalar(overrides)
    error('ThermalValidityAudit:ConfigOverrides', 'overrides must be a scalar struct.');
end

cfg = struct();
cfg.version = "thermal-validity-audit-v1";
cfg.model = struct('rotation_speed_rpm', NaN, 'H_ball_W_K', NaN, 'H_roller_W_K', NaN);
cfg.roughness = struct('ball', struct('composite_rq_m', NaN), ...
    'roller', struct('composite_rq_m', NaN), 'source_note', "");
cfg.lambda = struct('full_film_min', 1.2, 'transition_min', 0.6, ...
    'mixed_min', 0.05);
cfg.spectrum = struct('tracked_mode_count', 6, 'minimum_samples', 32, ...
    'uniform_time_relative_tolerance', 1e-8, 'half_width_bins', 2, ...
    'half_width_fraction', 0.03, 'maximum_neighbor_fraction', 0.45, ...
    'minimum_peak_to_floor_ratio', 3, 'minimum_modal_fraction_of_f1', 0.5);
cfg.damping = struct('minimum_same_direction_peaks', 6, 'minimum_fit_R2', 0.90, ...
    'minimum_damping_ratio', 0, 'maximum_damping_ratio', 1, ...
    'high_damping_warning_ratio', 0.2, 'cosine_taper_inner_fraction', 0.75);
cfg.export = struct('ball_detail_rows', 22, 'roller_detail_rows', 30, ...
    'not_computed_reason', "NOT_COMPUTED_BY_CURRENT_REDUCED_MODEL");
cfg = merge_known_fields(cfg, overrides, 'cfg');
end

function target = merge_known_fields(target, source, path_name)
names = fieldnames(source);
for k = 1:numel(names)
    name = names{k};
    if ~isfield(target, name)
        error('ThermalValidityAudit:UnknownConfigField', 'Unknown configuration field %s.%s.', path_name, name);
    end
    if isstruct(target.(name))
        if ~isstruct(source.(name)) || ~isscalar(source.(name))
            error('ThermalValidityAudit:ConfigStruct', '%s.%s must be a scalar struct.', path_name, name);
        end
        target.(name) = merge_known_fields(target.(name), source.(name), [path_name '.' name]);
    else
        target.(name) = source.(name);
    end
end
end
