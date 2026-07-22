function tracking = track_thermal_modes(previous_modes, current_modes, M_t, previous_frequency_Hz, current_frequency_Hz, cfg)
%TRACK_THERMAL_MODES Mass-weighted six-mode tracking with cluster handling.

count = cfg.modal.tracked_mode_count;
validate_inputs(previous_modes, current_modes, M_t, previous_frequency_Hz, current_frequency_Hz, count);
previous_modes = previous_modes(:,1:count); current_modes = current_modes(:,1:count);
previous_frequency_Hz = previous_frequency_Hz(1:count); current_frequency_Hz = current_frequency_Hz(1:count);
MAC = mac_matrix(previous_modes, current_modes, M_t);
assignment = frequency_aware_assignment(MAC, previous_frequency_Hz, current_frequency_Hz);
[cluster_id, cluster_used] = cluster_groups(previous_frequency_Hz, current_frequency_Hz(assignment), cfg.modal.cluster_relative_frequency_gap);
MAC_diagonal = zeros(1,count); cluster_scores = nan(1,count); pass_by_mode = false(1,count);
for group = unique(cluster_id)
    members = find(cluster_id == group);
    if isscalar(members)
        member = members(1); MAC_diagonal(member) = MAC(member, assignment(member));
        pass_by_mode(member) = MAC_diagonal(member) >= cfg.modal.minimum_single_mode_MAC;
    else
        ordered = sort(assignment(members));
        assignment(members) = ordered;
        for member = members, MAC_diagonal(member) = MAC(member, assignment(member)); end
        score = cluster_subspace_mac(previous_modes(:,members), current_modes(:,assignment(members)), M_t);
        cluster_scores(members) = score;
        pass_by_mode(members) = score >= cfg.modal.minimum_cluster_subspace_MAC;
    end
end
tracked_frequency_Hz = current_frequency_Hz(assignment).';
tracking = struct('tracked_mode_indices', assignment(:).', 'tracked_modes', current_modes(:,assignment), ...
    'tracked_frequency_Hz', tracked_frequency_Hz, 'MAC_diagonal', MAC_diagonal, ...
    'minimum_MAC', min([MAC_diagonal, cluster_scores(isfinite(cluster_scores))]), ...
    'cluster_tracking_used', cluster_used, 'cluster_id', cluster_id(:).', 'cluster_MAC', cluster_scores, ...
    'MAC_matrix', MAC, 'tracking_pass', all(pass_by_mode) && all(isfinite(tracked_frequency_Hz)));
end

function validate_inputs(previous, current, M, previous_frequency, current_frequency, count)
if ~isequal(size(M,1), size(M,2)) || any(~isfinite(M), 'all') || norm(M-M.', 'fro')/max(norm(M, 'fro'), 1) > 1e-10
    error('Stage9B:TrackingMass', 'M_t must be finite and symmetric for mass-weighted MAC tracking.');
end
if size(previous,1) ~= size(M,1) || size(current,1) ~= size(M,1) || size(previous,2) < count || size(current,2) < count || ...
        numel(previous_frequency) < count || numel(current_frequency) < count || any(~isfinite([previous(:); current(:); previous_frequency(:); current_frequency(:)]))
    error('Stage9B:TrackingInputs', 'Six finite compatible transverse modes and frequencies are required.');
end
end

function values = mac_matrix(previous, current, M)
count = size(previous,2); values = zeros(count,count);
for i = 1:count
    previous_norm = real(previous(:,i)'*M*previous(:,i));
    for j = 1:count
        current_norm = real(current(:,j)'*M*current(:,j));
        values(i,j) = abs(previous(:,i)'*M*current(:,j))^2/max(previous_norm*current_norm, realmin);
    end
end
if any(~isfinite(values), 'all') || any(values(:) < -1e-12)
    error('Stage9B:TrackingMAC', 'Mass-weighted MAC values must be finite and nonnegative.');
end
end

function assignment = frequency_aware_assignment(MAC, previous_frequency, current_frequency)
count = size(MAC,1); assignment = zeros(1,count); available = true(1,count);
for i = 1:count
    relative_gap = abs(current_frequency(:).'-previous_frequency(i))./max(previous_frequency(i), realmin);
    score = MAC(i,:)./(1+relative_gap); score(~available) = -Inf;
    [~, selected] = max(score);
    if ~isfinite(score(selected)), error('Stage9B:TrackingAssignment', 'No unused current mode is available for MAC tracking.'); end
    assignment(i) = selected; available(selected) = false;
end
end

function [identifier, used] = cluster_groups(previous_frequency, current_frequency, gap_limit)
count = numel(previous_frequency); edges = false(1,max(count-1,0));
for i = 1:numel(edges)
    previous_gap = abs(previous_frequency(i+1)-previous_frequency(i))/max(previous_frequency(i), realmin);
    current_gap = abs(current_frequency(i+1)-current_frequency(i))/max(current_frequency(i), realmin);
    edges(i) = previous_gap < gap_limit || current_gap < gap_limit;
end
identifier = ones(1,count); for i = 2:count, identifier(i) = identifier(i-1) + ~edges(i-1); end
used = any(edges);
end

function value = cluster_subspace_mac(previous, current, M)
previous = mass_normalize(previous, M); current = mass_normalize(current, M);
singular_values = svd(previous'*M*current);
value = min(abs(singular_values).^2);
end

function modes = mass_normalize(modes, M)
for i = 1:size(modes,2)
    value = real(modes(:,i)'*M*modes(:,i));
    if ~isfinite(value) || value <= 0, error('Stage9B:TrackingNormalization', 'Tracked modal mass must be finite and positive.'); end
    modes(:,i) = modes(:,i)/sqrt(value);
end
end
