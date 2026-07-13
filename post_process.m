function post = post_process(sim, params, bearingQD)
%POST_PROCESS Statistics, figures and response CSV.

if nargin < 3
    bearingQD = {};
end
if ~isfolder(params.output_dir)
    mkdir(params.output_dir);
end
if ~isfolder(params.figure_dir)
    mkdir(params.figure_dir);
end

time = sim.time;
idx = max(1, numel(time) - 5*params.Fen + 1):numel(time);
tplot = time(idx) - time(idx(1));
nb = numel(params.bearing);

post.idx_plot = idx;
post.time_plot = tplot;
post.figure_dir = params.figure_dir;
post.bearing = struct([]);

for ib = 1:nb
    b = params.bearing(ib);
    ix = 4*b.rotor_node - 3;
    iy = 4*b.rotor_node - 2;

    x = sim.yn(ix, idx);
    y = sim.yn(iy, idx);
    vx = sim.dyn(ix, idx);
    vy = sim.dyn(iy, idx);
    ax = sim.ddyn(ix, idx);
    ay = sim.ddyn(iy, idx);
    Fx = sim.F_b_hist(2*ib-1, idx);
    Fy = sim.F_b_hist(2*ib, idx);
    Fmag = sqrt(Fx.^2 + Fy.^2);

    s.xmax = max(abs(x));
    s.ymax = max(abs(y));
    s.xpp = max(x) - min(x);
    s.ypp = max(y) - min(y);
    s.vxmax = max(abs(vx));
    s.vymax = max(abs(vy));
    s.axmax = max(abs(ax));
    s.aymax = max(abs(ay));
    s.Fxmax = max(abs(Fx));
    s.Fymax = max(abs(Fy));
    s.Fmax = max(Fmag);
    s.loaded_mean = sum(sim.loaded_count_hist(ib,idx))/numel(idx);
    s.slip_max = max(abs(sim.slip_hist(ib,idx)));
    post.bearing(ib).summary = s;

    diag = extract_contact_diagnostics(sim, ib, idx);
    post.bearing(ib).delta_max = diag.delta_max;
    post.bearing(ib).clearance_work = diag.clearance_work;
    post.bearing(ib).oil_film_min = diag.oil_film_min;
    post.bearing(ib).contact_stiffness = diag.contact_stiffness;

    save_contact_force(tplot, Fx, Fy, Fmag, title_bearing(ib, 'force_time'), ...
        fullfile(params.figure_dir, sprintf('bearing%d_contact_force.png', ib)));
    save_single_time(tplot, diag.contact_stiffness, zh('stiffness_unit'), title_bearing(ib, 'stiffness_change'), ...
        fullfile(params.figure_dir, sprintf('bearing%d_contact_stiffness.png', ib)));
    save_single_time(tplot, diag.clearance_work*1e6, zh('clearance_unit'), title_bearing(ib, 'clearance_change'), ...
        fullfile(params.figure_dir, sprintf('bearing%d_clearance_work.png', ib)));
    save_single_time(tplot, diag.oil_film_min*1e6, zh('oil_unit'), title_bearing(ib, 'oil_change'), ...
        fullfile(params.figure_dir, sprintf('bearing%d_oil_film.png', ib)));
    save_single_time(tplot, sim.slip_hist(ib,idx), zh('slip'), title_bearing(ib, 'slip_change'), ...
        fullfile(params.figure_dir, sprintf('bearing%d_slip_ratio.png', ib)));

    save_xy_time(tplot, x, y, zh('disp_unit'), title_rotor(ib, 'disp'), ...
        fullfile(params.figure_dir, sprintf('rotor_bearing%d_displacement.png', ib)));
    save_xy_time(tplot, vx*1e3, vy*1e3, zh('vel_unit'), title_rotor(ib, 'vel'), ...
        fullfile(params.figure_dir, sprintf('rotor_bearing%d_velocity.png', ib)));
    save_xy_time(tplot, ax, ay, zh('acc_unit'), title_rotor(ib, 'acc'), ...
        fullfile(params.figure_dir, sprintf('rotor_bearing%d_acceleration.png', ib)));
    save_orbit(x - sum(x)/numel(x), y - sum(y)/numel(y), title_rotor(ib, 'orbit_demean'), ...
        fullfile(params.figure_dir, sprintf('rotor_bearing%d_orbit_demean.png', ib)));
    save_spectrum(tplot, x, params, b, title_rotor(ib, 'spec_x'), ...
        fullfile(params.figure_dir, sprintf('rotor_bearing%d_spectrum_x.png', ib)));
    save_spectrum(tplot, y, params, b, title_rotor(ib, 'spec_y'), ...
        fullfile(params.figure_dir, sprintf('rotor_bearing%d_spectrum_y.png', ib)));
end

save_bearing_baseline_figures(sim, params, bearingQD, idx, tplot);
save_optional_interface_compare(params, post);

post.csv_file = fullfile(params.output_dir, 'coupled_response_last5cycles.csv');
write_response_csv(post.csv_file, sim, params, idx, tplot);
end

function diag = extract_contact_diagnostics(sim, ib, idx)
if isfield(sim, 'delta_max_hist') && ~isempty(sim.delta_max_hist)
    diag.delta_max = sim.delta_max_hist(ib,idx);
    diag.clearance_work = sim.clearance_work_hist(ib,idx);
    diag.oil_film_min = sim.oil_film_min_hist(ib,idx);
    diag.contact_stiffness = sim.contact_stiffness_hist(ib,idx);
    return;
end
n = numel(idx);
diag.delta_max = zeros(1,n);
diag.clearance_work = zeros(1,n);
diag.oil_film_min = zeros(1,n);
diag.contact_stiffness = zeros(1,n);
for k = 1:n
    st = sim.bearingStateHist{idx(k)};
    if isempty(st) || ~isfield(st, 'bearings') || numel(st.bearings) < ib
        diag.delta_max(k) = NaN;
        diag.clearance_work(k) = NaN;
        diag.oil_film_min(k) = NaN;
        diag.contact_stiffness(k) = NaN;
        continue;
    end
    bs = st.bearings(ib);
    diag.delta_max(k) = max_or_zero(bs, 'delta');
    diag.clearance_work(k) = scalar_or_nan(bs, 'c_work');
    if isfield(bs, 'h') && ~isempty(bs.h)
        diag.oil_film_min(k) = min(bs.h);
    else
        diag.oil_film_min(k) = NaN;
    end
    diag.contact_stiffness(k) = scalar_or_nan(bs, 'k_contact_eff');
end
end

function v = max_or_zero(s, field)
if isfield(s, field) && ~isempty(s.(field))
    v = max(s.(field));
else
    v = 0;
end
end

function v = scalar_or_nan(s, field)
if isfield(s, field) && ~isempty(s.(field))
    v = s.(field);
else
    v = NaN;
end
end

function save_load_distribution(qd, ttl, file_name)
fig = new_fig();
bar(1:numel(qd.Q), qd.Q, 0.75, 'FaceColor', [0.18 0.18 0.18], 'EdgeColor', 'k');
grid on; box on;
set_axis_style();
xlabel(zh('element_id'), 'FontName', font_cn(), 'FontSize', 14, 'Interpreter','none');
ylabel(zh('load_unit'), 'FontName', font_cn(), 'FontSize', 14, 'Interpreter','none');
title(ttl, 'FontName', font_cn(), 'FontSize', 18, 'Interpreter','none');
save_png(fig, file_name);
close(fig);
end

function save_bearing_baseline_figures(sim, params, bearingQD, idx, tplot)
if ~get_field_default(params, 'plot_bearing_baseline_figures', true)
    return;
end
out_dir = fullfile(params.figure_dir, 'bearing_baseline');
if ~isfolder(out_dir)
    mkdir(out_dir);
end
nb = numel(params.bearing);
for ib = 1:nb
    if nargin >= 3 && numel(bearingQD) >= ib && ~isempty(bearingQD{ib})
        qd = bearingQD{ib};
        save_baseline_distribution(qd, ib, 'load', ...
            fullfile(out_dir, sprintf('bearing%d_load_distribution', ib)));
        save_baseline_distribution(qd, ib, 'oil', ...
            fullfile(out_dir, sprintf('bearing%d_oil_film_distribution', ib)));
    end
end
save_loaded_count_history(tplot, sim.loaded_count_hist(:,idx), ...
    fullfile(out_dir, 'loaded_count_history'));
save_slip_ratio_history(tplot, sim.slip_hist(:,idx), ...
    fullfile(out_dir, 'slip_ratio_history'));
save_bearing_force_history(tplot, sim.F_b_hist(:,idx), nb, ...
    fullfile(out_dir, 'bearing_force_history'));
end

function save_baseline_distribution(qd, ib, kind, base_name)
if ~isfield(qd, 'Q') || isempty(qd.Q)
    return;
end
Q = qd.Q(:).';
loaded = Q > 0;
x = 1:numel(Q);
fig = new_fig();
if strcmp(kind, 'load')
    y = Q;
    ttl = sprintf('Bearing %d Contact Load Distribution', ib);
    ylab = 'Contact load Q_i / N';
else
    if isfield(qd, 'oil_film') && ~isempty(qd.oil_film)
        y = qd.oil_film(:).';
    elseif isfield(qd, 'h') && ~isempty(qd.h)
        y = qd.h(:).';
    else
        close(fig);
        return;
    end
    ttl = sprintf('Bearing %d Oil Film Thickness Distribution', ib);
    ylab = 'Oil film thickness h_i / m';
end
bar(x(~loaded), y(~loaded), 0.75, 'FaceColor', [0.78 0.78 0.78], 'EdgeColor', [0.35 0.35 0.35]); hold on;
bar(x(loaded), y(loaded), 0.75, 'FaceColor', [0.12 0.31 0.55], 'EdgeColor', 'k');
grid on; box on;
set_axis_style();
xlabel('Rolling element index', 'FontName','Times New Roman', 'FontSize', 14);
ylabel(ylab, 'FontName','Times New Roman', 'FontSize', 14);
title(ttl, 'FontName','Times New Roman', 'FontSize', 16, 'Interpreter','none');
legend('Unloaded element', 'Loaded element', 'FontName','Times New Roman', 'FontSize', 11, 'Location','best');
save_figure_pair(fig, base_name);
close(fig);
end

function save_loaded_count_history(t, loaded_count, base_name)
if isempty(loaded_count)
    return;
end
fig = new_fig();
plot(t, loaded_count.', 'LineWidth', 1.5);
grid on; box on; xlim([0 t(end)]);
set_axis_style();
xlabel('Time / s', 'FontName','Times New Roman', 'FontSize', 14);
ylabel('Loaded rolling-element count', 'FontName','Times New Roman', 'FontSize', 14);
title('Loaded Rolling-Element Count History', 'FontName','Times New Roman', 'FontSize', 16);
legend(make_bearing_labels(size(loaded_count,1)), 'FontName','Times New Roman', 'FontSize', 11, 'Location','best');
save_figure_pair(fig, base_name);
close(fig);
end

function save_slip_ratio_history(t, slip_hist, base_name)
if isempty(slip_hist)
    return;
end
fig = new_fig();
plot(t, slip_hist.', 'LineWidth', 1.5);
grid on; box on; xlim([0 t(end)]);
set_axis_style();
xlabel('Time / s', 'FontName','Times New Roman', 'FontSize', 14);
ylabel('Cage relative slip index', 'FontName','Times New Roman', 'FontSize', 14);
title('Cage Relative Slip Index History', 'FontName','Times New Roman', 'FontSize', 16);
legend(make_bearing_labels(size(slip_hist,1)), 'FontName','Times New Roman', 'FontSize', 11, 'Location','best');
save_figure_pair(fig, base_name);
close(fig);
end

function save_bearing_force_history(t, F_b_hist, nb, base_name)
if isempty(F_b_hist)
    return;
end
fig = new_fig();
for ib = 1:nb
    Fx = F_b_hist(2*ib-1,:);
    Fy = F_b_hist(2*ib,:);
    Fmag = sqrt(Fx.^2 + Fy.^2);
    subplot(nb,1,ib);
    plot(t, Fx, 'LineWidth', 1.1); hold on;
    plot(t, Fy, '--', 'LineWidth', 1.1);
    plot(t, Fmag, 'k-', 'LineWidth', 1.3);
    grid on; box on; xlim([0 t(end)]);
    set_axis_style();
    xlabel('Time / s', 'FontName','Times New Roman', 'FontSize', 12);
    ylabel('Bearing force / N', 'FontName','Times New Roman', 'FontSize', 12);
    title(sprintf('Bearing %d Nonlinear Force History', ib), 'FontName','Times New Roman', 'FontSize', 14);
    legend('Fx','Fy','Resultant', 'FontName','Times New Roman', 'FontSize', 10, 'Location','best');
end
save_figure_pair(fig, base_name);
close(fig);
end

function save_contact_force(t, Fx, Fy, Fmag, ttl, file_name)
fig = new_fig();
plot(t, Fx, 'k-', 'LineWidth', 1.5); hold on;
plot(t, Fy, 'Color', [0.25 0.25 0.25], 'LineStyle', '--', 'LineWidth', 1.5);
plot(t, Fmag, 'Color', [0 0.25 0.55], 'LineWidth', 1.5);
grid on; box on; xlim([0 t(end)]);
set_axis_style();
xlabel(zh('time_unit'), 'FontName', font_cn(), 'FontSize', 14, 'Interpreter','none');
ylabel(zh('force_unit'), 'FontName', font_cn(), 'FontSize', 14, 'Interpreter','none');
title(ttl, 'FontName', font_cn(), 'FontSize', 18, 'Interpreter','none');
legend(zh('xdir'), zh('ydir'), zh('resultant'), 'FontName', font_cn(), 'FontSize', 12, 'Location', 'best', 'Interpreter','none');
save_png(fig, file_name);
close(fig);
end

function save_orbit(x, y, ttl, file_name)
fig = new_fig();
plot(x, y, 'k', 'LineWidth', 1.5);
axis equal; grid on; box on;
set_axis_style();
xlabel('x / m', 'FontName','Times New Roman', 'FontSize', 14);
ylabel('y / m', 'FontName','Times New Roman', 'FontSize', 14);
title(ttl, 'FontName', font_cn(), 'FontSize', 18, 'Interpreter','none');
save_png(fig, file_name);
close(fig);
end

function save_xy_time(t, x, y, ylab, ttl, file_name)
fig = new_fig();
plot(t, x, 'k-', 'LineWidth', 1.5); hold on;
plot(t, y, 'Color', [0.25 0.25 0.25], 'LineStyle', '--', 'LineWidth', 1.5);
grid on; box on; xlim([0 t(end)]);
set_axis_style();
xlabel(zh('time_unit'), 'FontName', font_cn(), 'FontSize', 14, 'Interpreter','none');
ylabel(ylab, 'FontName', font_cn(), 'FontSize', 14, 'Interpreter','none');
title(ttl, 'FontName', font_cn(), 'FontSize', 18, 'Interpreter','none');
legend(zh('xdir'), zh('ydir'), 'FontName', font_cn(), 'FontSize', 12, 'Location', 'best', 'Interpreter','none');
save_png(fig, file_name);
close(fig);
end

function save_single_time(t, x, ylab, ttl, file_name)
fig = new_fig();
plot(t, x, 'k-', 'LineWidth', 1.5);
grid on; box on; xlim([0 t(end)]);
set_axis_style();
xlabel(zh('time_unit'), 'FontName', font_cn(), 'FontSize', 14, 'Interpreter','none');
ylabel(ylab, 'FontName', font_cn(), 'FontSize', 14, 'Interpreter','none');
title(ttl, 'FontName', font_cn(), 'FontSize', 18, 'Interpreter','none');
save_png(fig, file_name);
close(fig);
end

function save_spectrum(t, x, params, b, ttl, file_name)
x = x - sum(x)/numel(x);
dt = t(2) - t(1);
N = numel(x);
Y = abs(fft(x))/N*2;
f = (0:N-1)/(N*dt);
half = 1:floor(N/2);
fig = new_fig();
plot(f(half), Y(half), 'k-', 'LineWidth', 1.5);
grid on; box on;
fr = params.rpm/60;
xlim([0 max(5*fr, 1000)]);
set_axis_style();
xlabel(zh('freq_unit'), 'FontName', font_cn(), 'FontSize', 14, 'Interpreter','none');
ylabel(zh('amp_unit'), 'FontName', font_cn(), 'FontSize', 14, 'Interpreter','none');
title(ttl, 'FontName', font_cn(), 'FontSize', 18, 'Interpreter','none');
add_freq_marker(fr, zh('one_x'));
add_freq_marker(2*fr, zh('two_x'));
add_freq_marker(estimate_cage_frequency(params, b), zh('cage_freq'));
save_png(fig, file_name);
close(fig);
end

function f = estimate_cage_frequency(params, b)
if strcmpi(b.type, 'ball')
    alpha = get_field_default(b, 'contact_angle', 0);
    d = b.Db;
    f = 0.5*params.omega*(1 - d/b.Dm*cos(alpha))/(2*pi);
else
    d = b.Dw;
    f = 0.5*params.omega*(1 - d/b.Dm)/(2*pi);
end
end

function add_freq_marker(freq, label_text)
if ~isfinite(freq) || freq <= 0
    return;
end
yl = ylim;
line([freq freq], yl, 'Color', [0.55 0.55 0.55], 'LineStyle', ':', 'LineWidth', 1.2);
text(freq, yl(2)*0.92, label_text, 'Rotation', 90, 'FontName', font_cn(), 'FontSize', 11, ...
    'HorizontalAlignment','right', 'VerticalAlignment','top');
end

function save_optional_interface_compare(params, post)
if any_interface_enabled(params, 'thermal')
    save_interface_compare(params, post, zh('thermal_compare'), 'thermal_effect_compare.png');
end
if any_interface_enabled(params, 'surface')
    save_interface_compare(params, post, zh('texture_compare'), 'texture_effect_compare.png');
end
if any_interface_enabled(params, 'roughness')
    save_interface_compare(params, post, zh('roughness_compare'), 'roughness_effect_compare.png');
end
if any_clearance_variation(post)
    save_interface_compare(params, post, zh('clearance_compare'), 'clearance_effect_compare.png');
end
if any_interface_enabled(params, 'debris') || any_interface_enabled(params, 'waviness')
    save_interface_compare(params, post, zh('debris_compare'), 'debris_waviness_effect_compare.png');
end
end

function tf = any_interface_enabled(params, field)
tf = false;
for i = 1:numel(params.bearing)
    if isfield(params.bearing(i), field)
        s = params.bearing(i).(field);
        if isfield(s, 'enable') && s.enable
            tf = true;
        elseif strcmp(field, 'surface') && isfield(s, 'texture_enable') && s.texture_enable
            tf = true;
        end
    end
end
end

function tf = any_clearance_variation(post)
tf = false;
for i = 1:numel(post.bearing)
    c = post.bearing(i).clearance_work;
    if max(c) - min(c) > 1e-12
        tf = true;
    end
end
end

function save_interface_compare(params, post, ttl, file_name)
fig = new_fig();
names = {zh('bearing1'), zh('bearing2')};
vals = zeros(numel(post.bearing),4);
for i = 1:numel(post.bearing)
    vals(i,1) = max(post.bearing(i).clearance_work)*1e6;
    vals(i,2) = min(post.bearing(i).oil_film_min)*1e6;
    vals(i,3) = max(post.bearing(i).contact_stiffness)/1e7;
    vals(i,4) = max(post.bearing(i).summary.xmax, post.bearing(i).summary.ymax)*1e6;
end
bar(vals);
grid on; box on;
set(gca, 'XTickLabel', names(1:numel(post.bearing)), 'FontName', font_cn(), 'FontSize', 13);
ylabel(zh('compare_y'), 'FontName', font_cn(), 'FontSize', 14);
title(ttl, 'FontName', font_cn(), 'FontSize', 18);
legend(zh('clearance_short'), zh('oil_short'), zh('stiffness_short'), zh('rotor_disp_short'), ...
    'FontName', font_cn(), 'FontSize', 11, 'Location', 'best');
save_png(fig, fullfile(params.figure_dir, file_name));
close(fig);
end

function fig = new_fig()
fig = figure('Visible','off', 'Color','w');
end

function set_axis_style()
set(gca, 'FontName','Times New Roman', 'FontSize', 13, 'LineWidth', 1.0);
end

function save_png(fig, file_name)
set(fig, 'PaperPositionMode', 'auto');
print(fig, file_name, '-dpng', '-r180');
end

function save_figure_pair(fig, base_name)
set(fig, 'PaperPositionMode', 'auto');
print(fig, [base_name '.png'], '-dpng', '-r300');
try
    savefig(fig, [base_name '.fig'], 'compact');
catch
    savefig(fig, [base_name '.fig']);
end
end

function labels = make_bearing_labels(nb)
labels = cell(1, nb);
for i = 1:nb
    labels{i} = sprintf('Bearing %d', i);
end
end

function write_response_csv(file_name, sim, params, idx, tplot)
fid = fopen(file_name, 'wt');
if fid < 0
    warning('Cannot write CSV: %s', file_name);
    return;
end
cleanupObj = onCleanup(@() fclose(fid));
fprintf(fid, 't_s');
for ib = 1:numel(params.bearing)
    fprintf(fid, ',b%d_x_m,b%d_y_m,b%d_vx_mps,b%d_vy_mps,b%d_ax_mps2,b%d_ay_mps2,b%d_Fx_N,b%d_Fy_N', ib,ib,ib,ib,ib,ib,ib,ib);
end
fprintf(fid, '\n');
for k = 1:numel(idx)
    fprintf(fid, '%.12e', tplot(k));
    for ib = 1:numel(params.bearing)
        rn = params.bearing(ib).rotor_node;
        ii = idx(k);
        fprintf(fid, ',%.12e,%.12e,%.12e,%.12e,%.12e,%.12e,%.12e,%.12e', ...
            sim.yn(4*rn-3,ii), sim.yn(4*rn-2,ii), ...
            sim.dyn(4*rn-3,ii), sim.dyn(4*rn-2,ii), ...
            sim.ddyn(4*rn-3,ii), sim.ddyn(4*rn-2,ii), ...
            sim.F_b_hist(2*ib-1,ii), sim.F_b_hist(2*ib,ii));
    end
    fprintf(fid, '\n');
end
end

function s = title_bearing(ib, kind)
s = [zh('bearing') num2str(ib) zh(kind)];
end

function s = title_rotor(ib, kind)
s = [zh('rotor_bearing') num2str(ib) zh(kind)];
end

function f = font_cn()
f = char([24494 36719 38597 40657]);
end

function s = zh(key)
switch key
    case 'element_id', s = char([28378 21160 20307 32534 21495]);
    case 'load_unit', s = char([25509 35302 36733 33655 32 47 32 78]);
    case 'time_unit', s = char([26102 38388 32 47 32 115]);
    case 'force_unit', s = char([25509 35302 21147 32 47 32 78]);
    case 'xdir', s = char([120 26041 21521]);
    case 'ydir', s = char([121 26041 21521]);
    case 'resultant', s = char([21512 21147]);
    case 'stiffness_unit', s = char([25509 35302 21018 24230 32 47 32 40 78 47 109 41]);
    case 'clearance_unit', s = char([24037 20316 28216 38553 32 47 32 956 109]);
    case 'oil_unit', s = char([26368 23567 27833 33180 21402 24230 32 47 32 956 109]);
    case 'slip', s = char([25171 28369 25351 26631]);
    case 'loaded_count', s = char([25215 36733 28378 21160 20307 25968 37327]);
    case 'disp_unit', s = char([20301 31227 32 47 32 109]);
    case 'vel_unit', s = char([36895 24230 32 47 32 40 109 109 47 115 41]);
    case 'acc_unit', s = char([21152 36895 24230 32 47 32 40 109 47 115 178 41]);
    case 'freq_unit', s = char([39057 29575 32 47 32 72 122]);
    case 'amp_unit', s = char([24133 20540 32 47 32 109]);
    case 'one_x', s = char([49 88 36716 39057]);
    case 'two_x', s = char([50 88 36716 39057]);
    case 'cage_freq', s = char([20445 25345 26550 39057 29575]);
    case 'bearing', s = char([36724 25215]);
    case 'loaddist', s = char([28378 21160 20307 25215 36733 20998 24067]);
    case 'force_time', s = char([21160 24577 22686 37327 25509 35302 21147 21709 24212]);
    case 'stiffness_change', s = char([23616 37096 25509 35302 21018 24230 35786 26029 20540]);
    case 'clearance_change', s = char([31561 25928 24037 20316 28216 38553]);
    case 'oil_change', s = char([26368 23567 31561 25928 27833 33180 21402 24230 21464 21270]);
    case 'slip_change', s = char([20445 25345 26550 25171 28369 25351 26631 21464 21270]);
    case 'loaded_change', s = char([25215 36733 28378 21160 20307 25968 37327 21464 21270]);
    case 'rotor_bearing', s = char([36716 23376 36724 25215]);
    case 'disp', s = char([22788 120 47 121 26041 21521 20301 31227 21709 24212]);
    case 'vel', s = char([22788 120 47 121 26041 21521 36895 24230 21709 24212]);
    case 'acc', s = char([22788 120 47 121 26041 21521 21152 36895 24230 21709 24212]);
    case 'orbit', s = char([22788 36724 24515 36712 36857]);
    case 'orbit_demean', s = char([22788 21435 22343 20540 36724 24515 36712 36857]);
    case 'spec_x', s = char([22788 120 26041 21521 39057 35889]);
    case 'spec_y', s = char([22788 121 26041 21521 39057 35889]);
    case 'bearing1', s = char([36724 25215 49]);
    case 'bearing2', s = char([36724 25215 50]);
    case 'thermal_compare', s = char([28909 25928 24212 23545 36724 25215 21160 24577 24615 33021 21644 36716 23376 25391 21160 21709 24212 30340 24433 21709]);
    case 'texture_compare', s = char([34920 38754 32441 29702 23545 36724 25215 21160 24577 24615 33021 21644 36716 23376 25391 21160 21709 24212 30340 24433 21709]);
    case 'roughness_compare', s = char([34920 38754 31895 31961 24230 23545 36724 25215 28070 28369 29366 24577 21644 36716 23376 21709 24212 30340 24433 21709]);
    case 'clearance_compare', s = char([24037 20316 28216 38553 23545 36724 25215 25509 35302 29366 24577 21644 36716 23376 25391 21160 21709 24212 30340 24433 21709]);
    case 'debris_compare', s = char([26434 36136 47 27874 32441 24230 23545 36724 25215 25509 35302 21147 21644 36716 23376 25391 21160 21709 24212 30340 24433 21709]);
    case 'compare_y', s = char([24402 19968 21270 47 24037 31243 37327 32423 25351 26631]);
    case 'clearance_short', s = char([24037 20316 28216 38553 47 956 109]);
    case 'oil_short', s = char([26368 23567 27833 33180 47 956 109]);
    case 'stiffness_short', s = char([25509 35302 21018 24230 47 49 101 55 40 78 47 109 41]);
    case 'rotor_disp_short', s = char([36716 23376 20301 31227 23792 20540 47 956 109]);
    otherwise, s = key;
end
end

function v = get_field_default(s, field, default_value)
if isfield(s, field) && ~isempty(s.(field))
    v = s.(field);
else
    v = default_value;
end
end
