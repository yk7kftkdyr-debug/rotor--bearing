function sample = dynamic_ehl_sample_schema(sample)
%DYNAMIC_EHL_SAMPLE_SCHEMA Canonical offline dynamic-EHL sample contract.
%
% Dynamic damping is defined by the local harmonic response
%
%     DeltaF = (K + i*w*C)*DeltaX,
%     C = imag(DeltaF/DeltaX)/w.
%
% This schema is reserved for replacing a traditional squeeze-film
% approximation with traceable dynamic-EHL data.  It neither evaluates a
% damping formula nor creates physical samples.  A finite C_dynamic_Ns_m is
% accepted only when SOURCE_TYPE is literature, dynamic_ehl_solver, or
% experimental_fit; empirical scaling and manually generated values are not
% valid sources.
%
% With no input, this function returns an unavailable in-memory template.
% With a candidate sample, it validates required fields, SI-unit metadata,
% and source provenance, then returns the validated candidate unchanged.

if nargin == 0
    sample = empty_sample();
    return;
end

validateattributes(sample, {'struct'}, {'scalar'}, mfilename, 'sample', 1);
required = {'X','Y','solver_status','source_type','validation_error','timestamp','units'};
if ~all(isfield(sample, required))
    error('DynamicEHLSample:MissingField', 'Sample must contain every canonical schema field.');
end
require_fields(sample.X, {'Q_N','T_K','U_m_s','Rx_m','Ry_m','eta_Pa_s','alpha_p_Pa_inv'}, 'X');
require_fields(sample.Y, {'C_dynamic_Ns_m'}, 'Y');
validate_units(sample.units);

positive = {'Q_N','T_K','Rx_m','Ry_m','eta_Pa_s','alpha_p_Pa_inv'};
for k = 1:numel(positive)
    validateattributes(sample.X.(positive{k}), {'numeric'}, ...
        {'scalar','real','finite','positive'}, mfilename, ['X.' positive{k}]);
end
validateattributes(sample.X.U_m_s, {'numeric'}, ...
    {'scalar','real','finite','nonnegative'}, mfilename, 'X.U_m_s');
validateattributes(sample.Y.C_dynamic_Ns_m, {'numeric'}, ...
    {'scalar','real'}, mfilename, 'Y.C_dynamic_Ns_m');
if ~(isstring(sample.solver_status) || ischar(sample.solver_status)) || ~isscalar(string(sample.solver_status))
    error('DynamicEHLSample:SolverStatus', 'solver_status must be a scalar string.');
end
if ~(isstring(sample.source_type) || ischar(sample.source_type)) || ~isscalar(string(sample.source_type))
    error('DynamicEHLSample:SourceType', 'source_type must be a scalar string.');
end

if isfinite(sample.Y.C_dynamic_Ns_m)
    if sample.Y.C_dynamic_Ns_m <= 0
        error('DynamicEHLSample:InvalidDamping', 'C_dynamic_Ns_m must be positive when available.');
    end
    allowed = ["literature","dynamic_ehl_solver","experimental_fit"];
    if ~ismember(string(sample.source_type), allowed)
        error('DynamicEHLSample:InvalidSource', 'Available C_dynamic_Ns_m requires an approved physical source_type.');
    end
    if ~isdatetime(sample.timestamp) || ~isscalar(sample.timestamp) || isnat(sample.timestamp)
        error('DynamicEHLSample:Timestamp', 'Available samples require a finite scalar timestamp.');
    end
elseif ~isnan(sample.Y.C_dynamic_Ns_m)
    error('DynamicEHLSample:InvalidDamping', 'C_dynamic_Ns_m must be positive or NaN.');
end
end

function sample = empty_sample()
sample = struct( ...
    'X', struct('Q_N',NaN,'T_K',NaN,'U_m_s',NaN,'Rx_m',NaN,'Ry_m',NaN, ...
        'eta_Pa_s',NaN,'alpha_p_Pa_inv',NaN), ...
    'Y', struct('C_dynamic_Ns_m',NaN), ...
    'solver_status', "UNAVAILABLE", ...
    'source_type', "", ...
    'validation_error', "NO_VALIDATED_DYNAMIC_EHL_VALUE", ...
    'timestamp', NaT, ...
    'units', canonical_units());
end

function units = canonical_units()
units = struct('Q_N',"N",'T_K',"K",'U_m_s',"m/s",'Rx_m',"m",'Ry_m',"m", ...
    'eta_Pa_s',"Pa.s",'alpha_p_Pa_inv',"Pa^-1",'C_dynamic_Ns_m',"N.s/m");
end

function require_fields(s, names, context)
if ~isstruct(s) || ~isscalar(s) || ~all(isfield(s, names))
    error('DynamicEHLSample:MissingInput', '%s fields are incomplete.', context);
end
end

function validate_units(units)
expected = canonical_units();
names = fieldnames(expected);
if ~isstruct(units) || ~isscalar(units) || ~all(isfield(units, names))
    error('DynamicEHLSample:Units', 'SI-unit metadata is incomplete.');
end
for k = 1:numel(names)
    if ~strcmp(string(units.(names{k})), expected.(names{k}))
        error('DynamicEHLSample:Units', 'Unit for %s must be %s.', names{k}, expected.(names{k}));
    end
end
end
