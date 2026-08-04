function schedule = build_stage_e_case_schedule(temperatures_C, scenarios)
%BUILD_STAGE_E_CASE_SCHEDULE Return the frozen Stage E temperature/scenario order.

canonical_temperatures = [20 50 80 100];
canonical_scenarios = {'LOW','NOMINAL','HIGH'};
if nargin < 1 || isempty(temperatures_C), temperatures_C = canonical_temperatures; end
if nargin < 2 || isempty(scenarios), scenarios = canonical_scenarios; end
if ~isequal(reshape(temperatures_C,1,[]),canonical_temperatures) || ...
        ~iscellstr(scenarios) || ~isequal(reshape(scenarios,1,[]),canonical_scenarios)
    error('StageE:Schedule','Stage E requires T20, T50, T80, T100 and LOW, NOMINAL, HIGH.');
end

schedule = repmat(struct('temperature_case_C',0,'scenario','','case_id','', ...
    'temperature_index',0,'scenario_index',0),1,12);
k = 0;
for t = 1:numel(canonical_temperatures)
    for s = 1:numel(canonical_scenarios)
        k = k+1;
        schedule(k).temperature_case_C = canonical_temperatures(t);
        schedule(k).scenario = canonical_scenarios{s};
        schedule(k).case_id = sprintf('T%d_%s',canonical_temperatures(t),canonical_scenarios{s});
        schedule(k).temperature_index = t;
        schedule(k).scenario_index = s;
    end
end
end
