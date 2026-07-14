%% Compare front and rear rotor bearing-node responses for the latest report cases.
% This post-processing script reads existing MAT simulation outputs only.
% It does not rerun dynamics.

clearvars
clc

cases = {};

% Latest filtered report cases:
% Unbalance: scale = 1, 2, 5, 8
unb_scales = [1, 2, 5, 8];
unb_tags = {'1', '2', '5', '8'};
for ii = 1:numel(unb_scales)
    cases(end+1, :) = {'unbalance', sprintf('scale=%g', unb_scales(ii)), ...
        fullfile(pwd, sprintf('results_unbalance_scale_%s.mat', unb_tags{ii}))}; %#ok<SAGROW>
end

% External load: all original loads
loads = [0, 1000, 2500, 5000, 7500];
for ii = 1:numel(loads)
    cases(end+1, :) = {'external_load', sprintf('L=%gN', loads(ii)), ...
        fullfile(pwd, sprintf('results_external_load_%gN.mat', loads(ii)))}; %#ok<SAGROW>
end

% Bearing stiffness: K3, K4, K6, K7, K8
stiff_ids = {'K3','K4','K6','K7','K8'};
for ii = 1:numel(stiff_ids)
    cases(end+1, :) = {'bearing_stiffness', stiff_ids{ii}, ...
        fullfile(pwd, sprintf('results_bearing_stiffness_%s.mat', stiff_ids{ii}))}; %#ok<SAGROW>
end

summary = table();
for ii = 1:size(cases, 1)
    data_in = load(cases{ii, 3}, 'result');
    result = data_in.result;
    row = compare_one_result(result, cases{ii, 1}, cases{ii, 2});
    summary = [summary; row]; %#ok<AGROW>
end

out_csv = fullfile(pwd, 'front_rear_bearing_comparison_report_cases.csv');
out_xlsx = fullfile(pwd, 'front_rear_bearing_comparison_report_cases.xlsx');
writetable(summary, out_csv);
writetable(summary, out_xlsx);
disp(summary);
fprintf('\nGenerated:\n  %s\n  %s\n', out_csv, out_xlsx);


function row = compare_one_result(result, study_type, case_label)
    idx = result.idx_plot;
    fs = result.fs;
    nodes = result.loc_rub(:).';
    front = node_metrics(result, nodes(1), idx, fs);
    rear = node_metrics(result, nodes(2), idx, fs);

    row = table( ...
        {study_type}, {case_label}, nodes(1), nodes(2), ...
        front.disp_pp, rear.disp_pp, rel_diff(rear.disp_pp, front.disp_pp), ...
        front.vel_rms, rear.vel_rms, rel_diff(rear.vel_rms, front.vel_rms), ...
        front.acc_rms, rear.acc_rms, rel_diff(rear.acc_rms, front.acc_rms), ...
        front.orbit_radius_max, rear.orbit_radius_max, rel_diff(rear.orbit_radius_max, front.orbit_radius_max), ...
        front.amp_1X, rear.amp_1X, rel_diff(rear.amp_1X, front.amp_1X), ...
        'VariableNames', {'study_type','case_label','front_node','rear_node', ...
        'front_disp_pp','rear_disp_pp','disp_rel_diff', ...
        'front_vel_rms','rear_vel_rms','vel_rel_diff', ...
        'front_acc_rms','rear_acc_rms','acc_rel_diff', ...
        'front_orbit_radius_max','rear_orbit_radius_max','orbit_rel_diff', ...
        'front_amp_1X','rear_amp_1X','amp_1X_rel_diff'});
end


function m = node_metrics(result, node, idx, fs)
    x_idx = 4 * node - 3;
    y_idx = 4 * node - 2;
    x = result.yn(x_idx, idx).';
    y = result.yn(y_idx, idx).';
    vx = result.dyn(x_idx, idx).';
    vy = result.dyn(y_idx, idx).';
    ax = result.ddyn(x_idx, idx).';
    ay = result.ddyn(y_idx, idx).';
    x0 = x - local_mean(x);
    y0 = y - local_mean(y);
    vx0 = vx - local_mean(vx);
    vy0 = vy - local_mean(vy);
    ax0 = ax - local_mean(ax);
    ay0 = ay - local_mean(ay);
    orbit_radius = sqrt(x0.^2 + y0.^2);
    [freq_ax, acc_amp_x] = single_sided_spectrum(ax0, fs);
    m.disp_pp = max([local_peak2peak(x0), local_peak2peak(y0)]);
    m.vel_rms = sqrt(local_mean(vx0.^2 + vy0.^2));
    m.acc_rms = sqrt(local_mean(ax0.^2 + ay0.^2));
    m.orbit_radius_max = max(orbit_radius);
    m.amp_1X = harmonic_amp(freq_ax, acc_amp_x, 165);
end


function r = rel_diff(rear_value, front_value)
    r = (rear_value - front_value) / max(abs(front_value), eps);
end


function [freq, amp] = single_sided_spectrum(x, fs)
    x = x(:);
    x = x - local_mean(x);
    n = numel(x);
    if n < 4
        freq = 0;
        amp = 0;
        return
    end
    w = hann(n);
    xw = x .* w;
    y = fft(xw);
    p2 = abs(y / sum(w) * 2);
    p1 = p2(1:floor(n/2)+1);
    p1(1) = p1(1) / 2;
    freq = fs * (0:floor(n/2)).' / n;
    amp = p1(:);
end


function amp = harmonic_amp(freq, spec, target)
    [~, idx] = min(abs(freq - target));
    amp = spec(idx);
end


function p = local_peak2peak(x)
    x = x(:);
    p = max(x) - min(x);
end


function m = local_mean(x)
    x = x(:);
    m = sum(x) / numel(x);
end
