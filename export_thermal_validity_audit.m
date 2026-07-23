function package = export_thermal_validity_audit(audit, cfg)
%EXPORT_THERMAL_VALIDITY_AUDIT Assemble a fixed in-memory package without I/O.

if ~isstruct(audit) || ~isscalar(audit) || ~isfield(audit, 'case_audit') || numel(audit.case_audit) ~= 4
    error('ThermalValidityAudit:ExportAudit', 'audit must be the four-case output of build_thermal_validity_audit.');
end
if ~isstruct(cfg) || ~isfield(cfg, 'export'), error('ThermalValidityAudit:ExportConfig', 'cfg must be a thermal validity audit configuration.'); end
cases = audit.case_audit(:);
package = struct();
package.summary_table = make_summary(cases);
package.case_audit = cases;
package.ball_bearing_detail = unavailable_detail(cases, cfg.export.ball_detail_rows, "ball", cfg.export.not_computed_reason);
package.roller_bearing_detail = unavailable_detail(cases, cfg.export.roller_detail_rows, "roller", cfg.export.not_computed_reason);
package.report_sections = report_sections(cases);
package.input_manifest = audit.input_manifest;
package.pass = logical(audit.pass);
end

function summary = make_summary(cases)
n = numel(cases); T = zeros(n,1); ball_lambda = zeros(n,1); roller_lambda = zeros(n,1); ball_power = zeros(n,1); roller_power = zeros(n,1);
valid_modes = zeros(n,1); valid_damping = zeros(n,1);
for k = 1:n
    T(k) = cases(k).T_oil_C; ball_lambda(k) = cases(k).ball.lambda_ratio; roller_lambda(k) = cases(k).roller.lambda_ratio;
    ball_power(k) = cases(k).ball.friction_power_W; roller_power(k) = cases(k).roller.friction_power_W;
    valid_modes(k) = nnz([cases(k).modal.modal_peak_valid]); valid_damping(k) = nnz(isfinite([cases(k).modal.identified_modal_damping_ratio]));
end
summary = table(T, ball_lambda, roller_lambda, ball_power, roller_power, valid_modes, valid_damping, ...
    'VariableNames', {'T_oil_C','Ball_lambda_ratio','Roller_lambda_ratio','Ball_friction_power_W','Roller_friction_power_W','Valid_modal_peak_count','Valid_damping_count'});
end

function detail = unavailable_detail(cases, row_count, type, reason)
if ~(isnumeric(row_count) && isscalar(row_count) && row_count >= 1 && row_count == floor(row_count))
    error('ThermalValidityAudit:ExportDetailRows', 'detail row counts must be positive integers.');
end
template = struct('case_index', NaN, 'T_oil_C', NaN, 'bearing_type', string(type), 'row_index', NaN, ...
    'value', NaN, 'available', false, 'reason', string(reason));
detail = repmat(template, numel(cases), row_count);
for k = 1:numel(cases)
    for row = 1:row_count
        detail(k,row).case_index = cases(k).case_index; detail(k,row).T_oil_C = cases(k).T_oil_C; detail(k,row).row_index = row;
    end
end
end

function sections = report_sections(cases)
sections = strings(4,1);
sections(1) = "IN_MEMORY_ONLY: no MAT, CSV, TXT, image, directory, log, or plot was created.";
sections(2) = "LUBRICATION_AND_EHL: lambda classifications are applicability flags and do not modify the saved workpoint.";
sections(3) = "SPECTRUM_AND_DAMPING: modal peaks use tracked-mode windows; failed damping conditions are reported as NaN.";
sections(4) = "INITIAL_CONDITION: R16-x full-time peak is excluded from temperature-effect comparison.";
if any(~arrayfun(@(c) c.path_independence_verified, cases)), sections(4) = sections(4) + " 100 C path independence remains unverified."; end
end
