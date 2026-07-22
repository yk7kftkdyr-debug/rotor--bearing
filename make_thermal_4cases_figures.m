function figure_files = make_thermal_4cases_figures(records, derived, summary_table, output_root)
%MAKE_THERMAL_4CASES_FIGURES Create the eight controlled journal figure classes.

if numel(records) ~= 4 || numel(derived) ~= 4 || height(summary_table) ~= 4 || ~isfolder(output_root)
    error('Stage9C1:FigureInputs', 'Four completed records, four derived structures, a four-row summary, and an output directory are required.');
end
font_name = chinese_font_name(); colors = [0 114 178; 230 159 0; 0 158 115; 204 121 167] / 255;
labels = compose('%g ℃', summary_table.T_oil_C); figure_files = strings(8, 2);
figure_files(1, :) = export_pair(figure_temperature_viscosity(summary_table, colors, labels, font_name), output_root, 'Fig01_Temperature_Viscosity');
figure_files(2, :) = export_pair(figure_clearance_film_power(summary_table, colors, labels, font_name), output_root, 'Fig02_Clearance_Film_Power');
figure_files(3, :) = export_pair(figure_load_distribution(records, colors, labels, font_name, 'ball'), output_root, 'Fig03_Ball_Load_Distribution');
figure_files(4, :) = export_pair(figure_load_distribution(records, colors, labels, font_name, 'roller'), output_root, 'Fig04_Roller_Load_Distribution');
figure_files(5, :) = export_pair(figure_stiffness_damping(summary_table, colors, labels, font_name), output_root, 'Fig05_Bearing_Stiffness_Damping');
figure_files(6, :) = export_pair(figure_modal(summary_table, colors, labels, font_name), output_root, 'Fig06_Tracked_Modal_Frequency_MAC');
figure_files(7, :) = export_pair(figure_free_decay(records, colors, labels, font_name), output_root, 'Fig07_Free_Decay_Response');
figure_files(8, :) = export_pair(figure_spectrum_ratio(derived, summary_table, colors, labels, font_name), output_root, 'Fig08_Spectrum_Response_Ratio');
end

function fig = figure_temperature_viscosity(summary, colors, ~, font)
fig = base_figure(font, [7.2 3.1]); layout = tiledlayout(fig, 1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
nexttile(layout); hold on; plot(summary.T_oil_C, summary.Ball_T_final_C, '-o', 'Color', colors(1,:), 'LineWidth', 1.4, 'DisplayName', '前球轴承'); plot(summary.T_oil_C, summary.Roller_T_final_C, '-s', 'Color', colors(2,:), 'LineWidth', 1.4, 'DisplayName', '后滚子轴承'); plot(summary.T_oil_C, summary.T_oil_C, '--', 'Color', [0.25 0.25 0.25], 'LineWidth', 1.0, 'DisplayName', '入口油温'); xlabel('入口油温 (℃)'); ylabel('温度 (℃)'); legend('Location', 'northwest'); grid on;
nexttile(layout); hold on; semilogy(summary.T_oil_C, summary.Ball_viscosity_Pa_s, '-o', 'Color', colors(1,:), 'LineWidth', 1.4, 'DisplayName', '前球轴承'); semilogy(summary.T_oil_C, summary.Roller_viscosity_Pa_s, '-s', 'Color', colors(2,:), 'LineWidth', 1.4, 'DisplayName', '后滚子轴承'); xlabel('入口油温 (℃)'); ylabel('黏度 (Pa·s)'); legend('Location', 'southwest'); grid on; title(layout, '温度与润滑黏度');
end

function fig = figure_clearance_film_power(summary, colors, ~, font)
fig = base_figure(font, [7.2 6.0]); layout = tiledlayout(fig, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
plot_pair(nexttile(layout), summary.T_oil_C, summary.Ball_working_clearance_m * 1e6, summary.Roller_working_clearance_m * 1e6, colors, '工作游隙 (μm)');
plot_pair(nexttile(layout), summary.T_oil_C, summary.Ball_minimum_film_m * 1e9, summary.Roller_minimum_film_m * 1e9, colors, '最小承载油膜 (nm)');
plot_pair(nexttile(layout), summary.T_oil_C, summary.Ball_friction_power_W, summary.Roller_friction_power_W, colors, '摩擦功率 (W)'); xlabel('入口油温 (℃)'); title(layout, '工作游隙、油膜与摩擦功率');
end

function fig = figure_load_distribution(records, colors, labels, font, bearing_name)
fig = base_figure(font, [7.2 3.4]); axes(fig); hold on;
for index = 1:4
    load_N = records(index).contact.(bearing_name).body_load_N;
    plot(1:numel(load_N), load_N, '-o', 'Color', colors(index,:), 'LineWidth', 1.15, 'MarkerSize', 3.5, 'DisplayName', char(labels(index)));
end
xlabel('滚动体编号'); ylabel('滚动体合力 (N)'); grid on; legend('Location', 'northeast');
if strcmp(bearing_name, 'ball'), title('前球轴承载荷分布'); else, title('后滚子轴承载荷分布'); end
end

function fig = figure_stiffness_damping(summary, colors, ~, font)
fig = base_figure(font, [7.2 6.0]); layout = tiledlayout(fig, 2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
plot_pair(nexttile(layout), summary.T_oil_C, summary.Ball_Kxx_N_m, summary.Roller_Kxx_N_m, colors, 'K_{xx} (N/m)');
plot_pair(nexttile(layout), summary.T_oil_C, summary.Ball_Kyy_N_m, summary.Roller_Kyy_N_m, colors, 'K_{yy} (N/m)');
plot_pair(nexttile(layout), summary.T_oil_C, summary.Ball_Cxx_Ns_m, summary.Roller_Cxx_Ns_m, colors, 'C_{xx} (N·s/m)');
plot_pair(nexttile(layout), summary.T_oil_C, summary.Ball_Cyy_Ns_m, summary.Roller_Cyy_Ns_m, colors, 'C_{yy} (N·s/m)');
xlabel(layout, '入口油温 (℃)'); title(layout, '轴承工作点刚度与 EHL 阻尼');
end

function fig = figure_modal(summary, colors, labels, font)
fig = base_figure(font, [7.2 3.4]); layout = tiledlayout(fig, 1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
nexttile(layout); hold on; for index = 1:4, plot(1:6, summary{index, {'Tracked_f1_Hz','Tracked_f2_Hz','Tracked_f3_Hz','Tracked_f4_Hz','Tracked_f5_Hz','Tracked_f6_Hz'}}, '-o', 'Color', colors(index,:), 'LineWidth', 1.2, 'DisplayName', char(labels(index))); end; xlabel('追踪模态序号'); ylabel('频率 (Hz)'); xticks(1:6); grid on; legend('Location', 'northwest');
nexttile(layout); plot(summary.T_oil_C, summary.Minimum_MAC, '-o', 'Color', colors(1,:), 'LineWidth', 1.4, 'MarkerFaceColor', colors(1,:)); xlabel('入口油温 (℃)'); ylabel('最小 MAC'); ylim([min(0.8, min(summary.Minimum_MAC) - 0.02) 1.001]); grid on; title(layout, '前六阶追踪频率与模态一致性');
end

function fig = figure_free_decay(records, colors, labels, font)
fig = base_figure(font, [7.2 7.4]); layout = tiledlayout(fig, 3, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
titles = ["R16-x 位移"; "R2 径向位移"; "R10 径向位移"; "C2 径向位移"; "C8 径向位移"];
for panel = 1:5
    ax = nexttile(layout); hold(ax, 'on');
    for index = 1:4
        response = records(index).response; x = response.displacement_m;
        if panel == 1, value = x(:, 5); else, node = [1 2 4 5]; node = node(panel - 1); value = hypot(x(:, 2 * node - 1), x(:, 2 * node)); end
        plot(ax, response.time_s, value * 1e6, 'Color', colors(index,:), 'LineWidth', 0.9, 'DisplayName', char(labels(index)));
    end
    xlabel(ax, '时间 (s)'); ylabel(ax, '位移 (μm)'); title(ax, titles(panel)); grid(ax, 'on'); if panel == 1, legend(ax, 'Location', 'northeast'); end
end
title(layout, '代表性自由衰减响应');
end

function fig = figure_spectrum_ratio(derived, summary, colors, labels, font)
fig = base_figure(font, [7.2 3.6]); layout = tiledlayout(fig, 1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
nexttile(layout); hold on; for index = 1:4, plot(derived(index).frequency_Hz, derived(index).displacement_spectrum(:, 5) * 1e6, 'Color', colors(index,:), 'LineWidth', 1.0, 'DisplayName', char(labels(index))); end; xlabel('频率 (Hz)'); ylabel('R16-x 单边幅值 (μm)'); xlim([0 max([derived.analysis_frequency_ceiling_Hz])]); grid on; legend('Location', 'northeast');
nexttile(layout); values = [summary.Front_peak_ratio summary.Rear_peak_ratio summary.Front_RMS_ratio summary.Rear_RMS_ratio]; bar(values, 'grouped'); xlabel('自由衰减响应比'); ylabel('机匣/转子响应比'); xticklabels({'前端峰值','后端峰值','前端 RMS','后端 RMS'}); legend(labels, 'Location', 'northwest'); grid on; title(layout, '频谱与转子—机匣自由衰减响应比');
end

function plot_pair(ax, temperature_C, ball_value, roller_value, colors, ylabel_text)
hold(ax, 'on'); plot(ax, temperature_C, ball_value, '-o', 'Color', colors(1,:), 'LineWidth', 1.3, 'DisplayName', '前球轴承'); plot(ax, temperature_C, roller_value, '-s', 'Color', colors(2,:), 'LineWidth', 1.3, 'DisplayName', '后滚子轴承'); ylabel(ax, ylabel_text); grid(ax, 'on'); legend(ax, 'Location', 'best');
end

function fig = base_figure(font, size_inches)
fig = figure('Visible', 'off', 'Color', 'w', 'Units', 'inches', 'Position', [0.5 0.5 size_inches], 'DefaultAxesFontName', font, 'DefaultTextFontName', font, 'DefaultLegendFontName', font, 'DefaultAxesFontSize', 8, 'DefaultTextFontSize', 9, 'DefaultLineLineWidth', 1.0);
end

function files = export_pair(fig, output_root, basename)
png_file = fullfile(output_root, basename + ".png"); pdf_file = fullfile(output_root, basename + ".pdf");
cleanup = onCleanup(@() close(fig)); exportgraphics(fig, png_file, 'Resolution', 600); exportgraphics(fig, pdf_file, 'ContentType', 'vector'); files = [string(png_file) string(pdf_file)];
end

function font = chinese_font_name()
candidates = ["PingFang SC" "Noto Sans CJK SC" "Source Han Sans SC" "Microsoft YaHei" "Heiti SC"];
available = string(listfonts); match = candidates(ismember(candidates, available));
if isempty(match), error('Stage9C1:ChineseFont', 'A supported Chinese font is required for Chinese journal figures.'); end
font = match(1);
end
