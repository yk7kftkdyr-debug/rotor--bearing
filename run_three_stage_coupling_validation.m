function validation = run_three_stage_coupling_validation()
%RUN_THREE_STAGE_COUPLING_VALIDATION Compare three coupling levels.
% Stage 1: linear bearing baseline.
% Stage 2: fixed rolling-element nonlinear contact.
% Stage 3: full tribology coupling with oil film and slip indicators.

cases = {'linear', 'fixed_nonlinear', 'full_tribology'};
labels = {'linear baseline', 'fixed nonlinear contact', 'full tribology coupling'};
out_dir = 'D:\bearing\gunzi';
if ~isfolder(out_dir)
    mkdir(out_dir);
end

validation = struct([]);
for ic = 1:numel(cases)
    report_file = fullfile(out_dir, ['2_' cases{ic} '.txt']);
    fprintf('\n===== Three-stage validation: %s =====\n', labels{ic});
    result = run_strong_coupled_rotor_response(report_file, false, cases{ic});
    validation(ic).case = cases{ic}; %#ok<AGROW>
    validation(ic).label = labels{ic};
    validation(ic).report_file = report_file;
    validation(ic).summary = summarize_case_result(result);
end

summary_file = fullfile(out_dir, 'three_stage_validation_summary.txt');
summary_csv = fullfile(out_dir, 'three_stage_validation_summary.csv');
write_validation_summary(summary_file, validation);
write_validation_csv(summary_csv, validation);
save(fullfile(out_dir, 'three_stage_validation.mat'), 'validation');
fprintf('\nThree-stage validation summary saved: %s\n', summary_csv);
end

function summary = summarize_case_result(result)
idx = result.rotorResponse.idx_plot;
tr = result.modelInfo.rotor_trans_dof;
summary.max_disp = max(abs(reshape(result.yn(tr,idx), [], 1)));
summary.max_vel = max(abs(reshape(result.dyn(tr,idx), [], 1)));
summary.max_acc = max(abs(reshape(result.ddyn(tr,idx), [], 1)));
fr = result.params.rpm/60;

for ib = 1:numel(result.params.bearing)
    rn = result.params.bearing(ib).rotor_node;
    x = result.yn(6*rn-5, idx);
    y = result.yn(6*rn-4, idx);
    vx = result.dyn(6*rn-5, idx);
    vy = result.dyn(6*rn-4, idx);
    ax = result.ddyn(6*rn-5, idx);
    ay = result.ddyn(6*rn-4, idx);
    Fx = result.F_b_hist(2*ib-1, idx);
    Fy = result.F_b_hist(2*ib, idx);

    summary.bearing(ib).max_disp = max([abs(x(:)); abs(y(:))]);
    summary.bearing(ib).max_vel = max([abs(vx(:)); abs(vy(:))]);
    summary.bearing(ib).max_acc = max([abs(ax(:)); abs(ay(:))]);
    summary.bearing(ib).force_peak = max(hypot(Fx, Fy));
    loaded_vec = result.loaded_count_hist(ib,idx);
    summary.bearing(ib).loaded_mean = sum(loaded_vec)/max(numel(loaded_vec), 1);
    summary.bearing(ib).slip_indicator_max = max(abs(result.slip_hist(ib,idx)));
    summary.bearing(ib).one_x_amp = max(first_harmonic_amplitude(result.time(idx), x, fr), first_harmonic_amplitude(result.time(idx), y, fr));
    summary.bearing(ib).two_x_amp = max(first_harmonic_amplitude(result.time(idx), x, 2*fr), first_harmonic_amplitude(result.time(idx), y, 2*fr));
    fcage = estimate_cage_frequency(result.params.bearing(ib), result.params.omega);
    summary.bearing(ib).cage_freq_amp = max(first_harmonic_amplitude(result.time(idx), x, fcage), first_harmonic_amplitude(result.time(idx), y, fcage));
    [summary.bearing(ib).oil_film_min, summary.bearing(ib).k_contact_mean, summary.bearing(ib).k_contact_pp] = state_metrics(result, ib, idx);
end

summary.bearing1_loaded_mean = summary.bearing(1).loaded_mean;
summary.bearing2_loaded_mean = summary.bearing(2).loaded_mean;
summary.bearing1_slip_max = summary.bearing(1).slip_indicator_max;
summary.bearing2_slip_max = summary.bearing(2).slip_indicator_max;
end

function write_validation_summary(file_name, validation)
fid = fopen(file_name, 'wt');
if fid < 0
    warning('Cannot write validation summary: %s', file_name);
    return;
end
cleanupObj = onCleanup(@() fclose(fid));
fprintf(fid, 'Three-stage bearing-rotor-case coupling validation\n');
fprintf(fid, 'case, max_disp(m), max_vel(m/s), max_acc(m/s2), loaded_front, loaded_rear, slip_indicator_front, slip_indicator_rear\n');
for i = 1:numel(validation)
    s = validation(i).summary;
    fprintf(fid, '%s, %.6e, %.6e, %.6e, %.6f, %.6f, %.6f, %.6f\n', ...
        validation(i).case, s.max_disp, s.max_vel, s.max_acc, ...
        s.bearing1_loaded_mean, s.bearing2_loaded_mean, ...
        s.bearing1_slip_max, s.bearing2_slip_max);
end
end

function write_validation_csv(file_name, validation)
fid = fopen(file_name, 'wt');
if fid < 0
    warning('Cannot write validation CSV: %s', file_name);
    return;
end
cleanupObj = onCleanup(@() fclose(fid));
fprintf(fid, 'case,bearing,max_disp_m,max_vel_mps,max_acc_mps2,force_peak_N,oil_film_min_m,slip_indicator_max,k_contact_mean_Npm,k_contact_pp_Npm,amp_1x_m,amp_2x_m,amp_cage_m,loaded_mean\n');
for ic = 1:numel(validation)
    for ib = 1:numel(validation(ic).summary.bearing)
        s = validation(ic).summary.bearing(ib);
        fprintf(fid, '%s,%d,%.9e,%.9e,%.9e,%.9e,%.9e,%.9e,%.9e,%.9e,%.9e,%.9e,%.9e,%.9e\n', ...
            validation(ic).case, ib, s.max_disp, s.max_vel, s.max_acc, s.force_peak, s.oil_film_min, ...
            s.slip_indicator_max, s.k_contact_mean, s.k_contact_pp, s.one_x_amp, s.two_x_amp, s.cage_freq_amp, s.loaded_mean);
    end
end
end

function A = first_harmonic_amplitude(t, x, f)
x = x(:);
x = x - sum(x)/max(numel(x), 1);
t = t(:);
if numel(t) < 3 || f <= 0 || ~isfinite(f)
    A = 0;
    return;
end
c = cos(2*pi*f*t);
s = sin(2*pi*f*t);
A = hypot(2/numel(t)*sum(x.*c), 2/numel(t)*sum(x.*s));
end

function f = estimate_cage_frequency(b, omega)
if strcmpi(b.type, 'ball')
    alpha = get_field_default(b, 'contact_angle', 0);
    d = b.Db;
    f = 0.5*omega*(1 - d/b.Dm*cos(alpha))/(2*pi);
else
    d = b.Dw;
    f = 0.5*omega*(1 - d/b.Dm)/(2*pi);
end
end

function [oil_min, k_mean, k_pp] = state_metrics(result, ib, idx)
oil = [];
kk = [];
if isfield(result, 'bearingStateHist') && ~isempty(result.bearingStateHist)
    for ii = idx
        st = result.bearingStateHist{ii};
        if ~isempty(st) && isfield(st, 'bearings') && numel(st.bearings) >= ib
            bstate = st.bearings(ib);
            if isfield(bstate, 'h') && ~isempty(bstate.h)
                oil(end+1) = min(bstate.h); %#ok<AGROW>
            end
            if isfield(bstate, 'k_contact_eff') && ~isempty(bstate.k_contact_eff)
                kk(end+1) = bstate.k_contact_eff; %#ok<AGROW>
            end
        end
    end
end
if isempty(oil)
    oil_min = NaN;
else
    oil_min = min(oil);
end
if isempty(kk)
    k_mean = NaN;
    k_pp = NaN;
else
    k_mean = sum(kk)/max(numel(kk), 1);
    k_pp = max(kk) - min(kk);
end
end

function v = get_field_default(s, field, default_value)
if isfield(s, field) && ~isempty(s.(field))
    v = s.(field);
else
    v = default_value;
end
end
