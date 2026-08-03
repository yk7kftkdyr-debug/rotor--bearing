function statistics = compute_stage_d_bearing_statistics(Q, element_angle_rad)
%COMPUTE_STAGE_D_BEARING_STATISTICS Static load-zone statistics by element.

if ~isnumeric(Q) || ~isreal(Q) || isempty(Q) || any(~isfinite(Q),'all') || any(Q < 0,'all')
    error('StageD:ContactLoads','Contact loads must be finite and nonnegative.');
end
angles = reshape(element_angle_rad,1,[]);
if ~isnumeric(angles) || ~isreal(angles) || any(~isfinite(angles)) || isempty(angles)
    error('StageD:ElementAngles','Element angles must be a finite nonempty vector.');
end
if isvector(Q)
    loads = reshape(Q,1,[]);
elseif size(Q,1) == numel(angles)
    loads = reshape(sum(Q,2),1,[]);
elseif size(Q,2) == numel(angles)
    loads = reshape(sum(Q,1),1,[]);
else
    error('StageD:ContactLoadDimensions', ...
        'A contact-load matrix must map one dimension to element angles.');
end
if numel(loads) ~= numel(angles)
    error('StageD:ContactLoadDimensions', ...
        'Rolling-element loads and angles must have the same count.');
end

loaded_mask = loads > 0;
loaded_indices = find(loaded_mask);
loaded = loads(loaded_mask);
if isempty(loaded)
    Q_max = 0; Q_mean = 0; Q_sum = 0; Q_std = 0; CV = 0;
    max_index = NaN; zone = 0;
else
    [Q_max,local_max] = max(loaded);
    max_index = loaded_indices(local_max);
    Q_mean = mean(loaded); Q_sum = sum(loaded);
    if isscalar(loaded), Q_std = 0; else, Q_std = std(loaded,0); end
    CV = Q_std/max(Q_mean,eps);
    loaded_angles = sort(mod(angles(loaded_mask),2*pi));
    if isscalar(loaded_angles)
        zone = 0;
    else
        gaps = diff([loaded_angles loaded_angles(1)+2*pi]);
        zone = 2*pi-max(gaps);
    end
end

statistics = struct('rolling_element_loads_N',loads, ...
    'loaded_mask',loaded_mask,'loaded_indices',loaded_indices, ...
    'loaded_count',numel(loaded_indices),'load_zone_angle_rad',zone, ...
    'load_zone_angle_deg',zone*180/pi, ...
    'max_load_element_index',max_index,'Q_max_N',Q_max, ...
    'Q_mean_loaded_N',Q_mean,'Q_sum_N',Q_sum, ...
    'Q_std_loaded_N',Q_std,'CV_Q',CV, ...
    'element_angle_rad',angles,'source','FORMAL_ACCEPTED_CONTACT_STATE');
end
