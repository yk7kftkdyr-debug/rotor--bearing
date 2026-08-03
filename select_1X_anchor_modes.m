function selection = select_1X_anchor_modes( ...
    natural_frequencies_Hz, mass_normalized_modes, observation_dofs, ...
    frequency_1X_Hz, participation_threshold)
%SELECT_1X_ANCHOR_MODES Select two traceable physical modes near 1X.

if nargin < 5 || isempty(participation_threshold)
    participation_threshold = 1e-10;
end
frequency = natural_frequencies_Hz(:);
if ~isnumeric(frequency) || numel(frequency) < 2 || ...
        any(~isfinite(frequency)) || any(frequency <= 0) || ...
        any(diff(frequency) < 0)
    error('StageC:AnchorFrequency', ...
        'At least two sorted finite positive physical frequencies are required.');
end
if ~isnumeric(mass_normalized_modes) || ~isreal(mass_normalized_modes) || ...
        size(mass_normalized_modes,2) ~= numel(frequency) || ...
        any(~isfinite(mass_normalized_modes), 'all')
    error('StageC:AnchorModes', ...
        'Mode columns must match the physical frequency vector.');
end
n = size(mass_normalized_modes,1);
observation_dofs = unique(observation_dofs(:).', 'stable');
if isempty(observation_dofs) || any(~isfinite(observation_dofs)) || ...
        any(observation_dofs ~= floor(observation_dofs)) || ...
        any(observation_dofs < 1 | observation_dofs > n)
    error('StageC:ObservationMapping', ...
        'Observation DOFs must be valid unique physical-coordinate indices.');
end
if ~isscalar(frequency_1X_Hz) || ~isfinite(frequency_1X_Hz) || ...
        frequency_1X_Hz <= 0 || ~isscalar(participation_threshold) || ...
        ~isfinite(participation_threshold) || participation_threshold < 0
    error('StageC:AnchorConfiguration', ...
        'The 1X frequency and participation threshold must be valid scalars.');
end

participation = vecnorm(mass_normalized_modes(observation_dofs,:),2,1)./ ...
    max(vecnorm(mass_normalized_modes,2,1), eps);
eligible = find(participation > participation_threshold);
relaxed = false;
if numel(eligible) < 2
    eligible = (1:numel(frequency)).';
    relaxed = true;
end

below = eligible(frequency(eligible) < frequency_1X_Hz);
above = eligible(frequency(eligible) > frequency_1X_Hz);
if ~isempty(below) && ~isempty(above)
    [~, ib] = min(frequency_1X_Hz-frequency(below));
    [~, ia] = min(frequency(above)-frequency_1X_Hz);
    anchors = [below(ib) above(ia)];
    bracketing = true;
else
    [~, order] = sort(abs(frequency(eligible)-frequency_1X_Hz), 'ascend');
    anchors = sort(eligible(order(1:2))).';
    bracketing = frequency(anchors(1)) < frequency_1X_Hz && ...
        frequency(anchors(2)) > frequency_1X_Hz;
end
anchors = reshape(anchors,1,2);

selection = struct();
selection.anchor_indices = anchors;
selection.anchor_modes = anchors;
selection.anchor_frequencies_Hz = frequency(anchors).';
selection.anchor_distance_from_1X_Hz = ...
    abs(frequency(anchors).'-frequency_1X_Hz);
selection.anchor_participation = participation(anchors);
selection.all_mode_participation = participation;
selection.bracketing_flag = bracketing;
selection.anchor_modes_not_bracketing_1X = ~bracketing;
selection.participation_filter_relaxed = relaxed;
selection.participation_threshold = participation_threshold;
selection.observation_dofs = observation_dofs;
selection.frequency_1X_Hz = frequency_1X_Hz;
end
