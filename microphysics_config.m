function cfg = microphysics_config(overrides)
%MICROPHYSICS_CONFIG Default local microphysics switches and settings.

cfg.thermal = struct('enabled', false);
cfg.roughness = struct('enabled', false);
cfg.impurity = struct('enabled', false);
if nargin < 1 || isempty(overrides)
    return;
end
names = {'thermal', 'roughness', 'impurity'};
for k = 1:numel(names)
    name = names{k};
    if isfield(overrides, name) && isstruct(overrides.(name)) && ...
            isfield(overrides.(name), 'enabled')
        cfg.(name).enabled = logical(overrides.(name).enabled);
    end
end
end
