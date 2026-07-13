function report_generator(params, bearingQD, sim, post)
%REPORT_GENERATOR Write engineering report and analysis CSV files.

if ~isfolder(params.output_dir)
    mkdir(params.output_dir);
end

fid = fopen(params.report_file, 'wt', 'n', 'UTF-8');
if fid < 0
    error('Cannot open report file: %s', params.report_file);
end
cleanupObj = onCleanup(@() fclose(fid));

fprintf(fid, '============================================================\n');
fprintf(fid, '高速滚动轴承-转子-机匣强耦合动力学仿真主报告\n');
fprintf(fid, '============================================================\n\n');

write_condition(fid, params);
write_bearing_layout_check(fid, params);
for ib = 1:numel(params.bearing)
    write_bearing_section(fid, ib, params.bearing(ib), bearingQD{ib}, params);
    write_bearing_detail_csv(params, ib, params.bearing(ib), bearingQD{ib});
    write_bearing_tangent_csv(params, ib, params.bearing(ib), bearingQD{ib});
end
write_tangent_stiffness_section(fid, params, bearingQD, sim, post);
write_response_section(fid, params, sim, post);
write_peak_csv(params, sim, post);
write_solver_diagnostics(fid, params, sim, post);
write_judgement(fid, params, post);
write_figure_description(fid, params);
write_output_list(fid, params, post);
delete(cleanupObj);
normalize_report_labels(params.report_file);
end

function write_bearing_layout_check(fid, params)
layout = bearing_layout_config();
fprintf(fid, 'Bearing layout consistency check\n');
fprintf(fid, '  Unified layout source: bearing_layout_config.m\n');
fprintf(fid, '  Official case source: official_simulation_config.m + apply_official_case_config.m\n');
fprintf(fid, '  %s; %s\n', layout.bearing1.full_label, layout.bearing1.name_cn);
fprintf(fid, '  %s; %s\n', layout.bearing2.full_label, layout.bearing2.name_cn);
for ib = 1:numel(params.bearing)
    b = params.bearing(ib);
    fprintf(fid, '  bearing%d actual type/name: %s / %s\n', ib, b.type, b.name);
    fprintf(fid, '    rotor node / casing node: %d / %d\n', b.rotor_node, b.case_node);
    fprintf(fid, '    preload for quasi-dynamic calculation local y/z: %.6e / %.6e N\n', b.preload_y, b.preload_z);
    fprintf(fid, '    support series stiffness used in coupled equation kx/ky: %.6e / %.6e N/m\n', ...
        get_field_default(b,'support_series_kx',NaN), get_field_default(b,'support_series_ky',NaN));
end
if isfield(params, 'static_load') && ~isempty(params.static_load)
    fprintf(fid, '  rotor-node static_load Fx/Fy: %s / %s N\n', ...
        mat2str(params.static_load.Fx, 6), mat2str(params.static_load.Fy, 6));
end
fprintf(fid, '  Load application rule: quasi-dynamic bearing preload only; no duplicate rotor-node static_load for the official baseline.\n');
fprintf(fid, '  Modification logic: all reports, legends and thermal analyses treat bearing1 as the front angular-contact ball bearing and bearing2 as the rear cylindrical roller bearing. This removes mixed front-roller/rear-ball wording while preserving each bearing model, rolling-element count, oil-film calculation and contact law.\n\n');
end

function write_condition(fid, params)
fprintf(fid, '一、工况信息\n');
fprintf(fid, '  转速 rpm: %.3f r/min\n', params.rpm);
fprintf(fid, '  角速度 omega: %.6f rad/s\n', params.omega);
fprintf(fid, '  Newmark: Fen=%d, n_Fen=%d, gamma=%.4f, beta=%.6f\n', params.Fen, params.n_Fen, params.gamma, params.beta);
if isfield(params, 'modelInfo') && isfield(params.modelInfo, 'source')
    fprintf(fid, '  Rotor/case model source: %s\n', params.modelInfo.source);
elseif isfield(params, 'uploaded_model_dir')
    fprintf(fid, '  Rotor/case model directory: %s\n', params.uploaded_model_dir);
end
if get_field_default(params, 'high_frequency_results_require_timestep_convergence', false)
    fprintf(fid, '  高频响应说明: 高频加速度/接触调制解释需结合 Fen=%s 时间步验证。\n', ...
        mat2str(get_field_default(params, 'timestep_convergence_Fen', [1024 4096 8192])));
end
fprintf(fid, '  求解器表述: Newmark-beta + relaxed nonlinear bearing-force iteration；未显式组装非线性接触切线刚度矩阵。\n');
fprintf(fid, '  工作游隙工况: %s\n', get_field_default(params, 'clearance_case', 'baseline'));
fprintf(fid, '  不平衡相位模式: %s\n', get_field_default(params.unbalance, 'phase_mode', 'legacy/default'));
fprintf(fid, '  不平衡节点/质量g/偏心mm/相位rad/力幅N:\n');
for iu = 1:numel(params.unbalance.nodes)
    me = params.unbalance.mass_g(iu)*1e-3;
    ecc = params.unbalance.ecc_mm(iu)*1e-3;
    F0 = me*ecc*params.omega^2;
    fprintf(fid, '    node %d: %.6g g, %.6g mm, phase %.6f, F0 %.6e N\n', ...
        params.unbalance.nodes(iu), params.unbalance.mass_g(iu), params.unbalance.ecc_mm(iu), params.unbalance.phase(iu), F0);
end
fprintf(fid, '  合成不平衡力幅值估计: %.6e N\n', resultant_unbalance_force(params));
for ib = 1:numel(params.bearing)
    b = params.bearing(ib);
    fprintf(fid, '  轴承%d: %s\n', ib, b.name);
    fprintf(fid, '    类型: %s, 转子节点: %d, 机匣节点: %d\n', b.type, b.rotor_node, b.case_node);
    fprintf(fid, '    拟动力学预载 local y/z: %.3f / %.3f N\n', b.preload_y, b.preload_z);
    fprintf(fid, '    等效支承串联刚度 kx/ky: %.6e / %.6e N/m\n', get_field_default(b,'support_series_kx',inf), get_field_default(b,'support_series_ky',inf));
end
fprintf(fid, '  外部径向扰动 F_ext_y/F_ext_z: %.3f / %.3f N\n', params.F_ext_y, params.F_ext_z);
if isfield(params, 'static_load') && ~isempty(params.static_load)
    fprintf(fid, '  rotor-node static_load Fx/Fy: %s / %s N\n', ...
        mat2str(params.static_load.Fx, 6), mat2str(params.static_load.Fy, 6));
end
fprintf(fid, '  说明: 轴承预载只用于拟动力学和等效支承参数，不在动力学方程中重复施加。\n');
fprintf(fid, '  不平衡节点: %s\n', mat2str(params.unbalance.nodes));
Famp = params.unbalance.mass_g(:)'*1e-3 .* params.unbalance.ecc_mm(:)'*1e-3 * params.omega^2;
fprintf(fid, '  不平衡质量 g: %s\n', mat2str(params.unbalance.mass_g, 6));
fprintf(fid, '  偏心量 mm: %s\n', mat2str(params.unbalance.ecc_mm, 6));
fprintf(fid, '  不平衡力幅值 N: %s\n\n', mat2str(Famp, 6));
end

function write_bearing_section(fid, ib, b, r, params)
[pressure, Fc, slip_i] = bearing_detail_arrays(b, r);
loaded = r.Q > 0;
loaded_index = find(loaded);
nonloaded_index = find(~loaded);

fprintf(fid, '二.%d 轴承%d拟动力学结果\n', ib, ib);
fprintf(fid, '  类型: %s\n', r.type);
fprintf(fid, '  工作游隙 clearance_work: %.6e m\n', r.clearance_work);
fprintf(fid, '    clearance0: %.6e m\n', r.clearInfo.c0);
fprintf(fid, '    clearance_fit: %.6e m\n', r.clearInfo.delta_fit);
fprintf(fid, '    clearance_temp: %.6e m\n', r.clearInfo.delta_thermal);
fprintf(fid, '    clearance_centrifugal: %.6e m\n', r.clearInfo.delta_centrifugal);
fprintf(fid, '    clearance_other: %.6e m\n', r.clearInfo.delta_other);
if r.clearance_work <= 0
    fprintf(fid, '  WARNING: working clearance <= 0, bearing is in preload/interference contact; check fit and thermal inputs.\n');
elseif abs(r.clearance_work - r.clearInfo.c0) > 0.5*max(abs(r.clearInfo.c0), eps)
    fprintf(fid, '  WARNING: working clearance changed by more than 50%% from initial clearance; thermal-fit sensitivity study is recommended.\n');
end
fprintf(fid, '  预载平衡闭合量 x/y: %.6e / %.6e m\n', r.operating_offset_x, r.operating_offset_y);
fprintf(fid, '  承载滚动体数: %d / %d\n', r.loaded_count, b.n);
fprintf(fid, '  最大接触负荷: %.6e N\n', r.max_contact_load);
fprintf(fid, '  最大接触压力诊断值/估计平均接触压强: %.6e Pa\n', max(pressure));
fprintf(fid, '  最小油膜厚度: %.6e m\n', r.min_oil_film);
fprintf(fid, '  最大 PV 值: %.6e Pa*m/s\n', max(r.PV));
fprintf(fid, '  保持架相对打滑指标估计: %.4f\n', r.slip_ratio);
fprintf(fid, '  保持架理论转速: %.6e rad/s, %.3f r/min\n', r.cage_speed, r.cage_speed*30/pi);
if isfield(r, 'spin_speed')
    fprintf(fid, '  滚动体自转角速度估计: %.6e rad/s, %.3f r/min\n', r.spin_speed, r.spin_speed*30/pi);
end
fprintf(fid, '  单个滚动体离心力估计: %.6e N\n', Fc);
fprintf(fid, '  刚度矩阵 local y/z -> force y/z, N/m:\n');
fprintf(fid, '    [%.6e  %.6e]\n', r.stiffness_matrix(1,1), r.stiffness_matrix(1,2));
fprintf(fid, '    [%.6e  %.6e]\n', r.stiffness_matrix(2,1), r.stiffness_matrix(2,2));
fprintf(fid, '  等效支承参数: kx=%.6e N/m, ky=%.6e N/m, cx=%.6e N*s/m, cy=%.6e N*s/m\n', r.kx, r.ky, r.cx, r.cy);
fprintf(fid, '  油膜力开关: include_oil_damping_force=%s, include_oil_stiffness_force=%s\n', ...
    yesno(get_field_default(b, 'include_oil_damping_force', true)), yesno(get_field_default(b, 'include_oil_stiffness_force', false)));
fprintf(fid, '  EHL载荷修正: enable=%s, exponent=%.4f, Q_ref=%.6e N\n', ...
    yesno(get_field_default(b, 'ehl_load_correction_enable', true)), get_field_default(b, 'ehl_load_exponent', -0.067), ehl_load_reference_report(b));
fprintf(fid, '  接触设置: smooth=%s, smooth_delta=%.3e m, damping=%s, cn=%.3e N*s/m\n', ...
    yesno(params.contact.smooth_enable), params.contact.smooth_delta, yesno(params.contact.damping_enable), b.contact_cn);
fprintf(fid, '  映射说明: 程序 x = 轴承局部 y, 程序 y = 轴承局部 z。\n\n');

fprintf(fid, '  受载滚动体明细\n');
fprintf(fid, '  序号  方位角deg   接触负荷N      接触变形m      油膜厚度m      接触压力诊断值Pa      PV(Pa*m/s)    摩擦系数     lambda     打滑指标\n');
if isempty(loaded_index)
    fprintf(fid, '  无受载滚动体。\n');
else
    for k = 1:numel(loaded_index)
        i = loaded_index(k);
        fprintf(fid, '  %3d  %9.3f  %12.6e  %12.6e  %12.6e  %12.6e  %12.6e  %9.5f  %8.3f  %8.4f\n', ...
            i, wrap_deg(r.theta(i)*180/pi), r.Q(i), r.delta(i), r.oil_film(i), pressure(i), r.PV(i), r.mu_eff(i), r.lambda(i), slip_i(i));
    end
end
fprintf(fid, '  非承载滚动体序号: %s\n\n', mat2str(nonloaded_index));
end

function write_response_section(fid, params, sim, post)
fprintf(fid, '三、转子-机匣耦合振动响应统计\n');
fprintf(fid, '  统计区间: 末5个转频周期，时间 %.6e 到 %.6e s\n', sim.time(post.idx_plot(1)), sim.time(post.idx_plot(end)));
for ib = 1:numel(params.bearing)
    s = post.bearing(ib).summary;
    fprintf(fid, '  轴承%d响应统计\n', ib);
    fprintf(fid, '    xmax/ymax: %.6e / %.6e m\n', s.xmax, s.ymax);
    fprintf(fid, '    x峰峰值/y峰峰值: %.6e / %.6e m\n', s.xpp, s.ypp);
    fprintf(fid, '    vxmax/vymax: %.6e / %.6e m/s  = %.6e / %.6e mm/s\n', s.vxmax, s.vymax, s.vxmax*1e3, s.vymax*1e3);
    fprintf(fid, '    axmax/aymax: %.6e / %.6e m/s^2\n', s.axmax, s.aymax);
    fprintf(fid, '    Fxmax/Fymax: %.6e / %.6e N\n', s.Fxmax, s.Fymax);
    fprintf(fid, '    合力最大值: %.6e N\n', s.Fmax);
    fprintf(fid, '    平均承载滚动体数: %.4f\n', s.loaded_mean);
    fprintf(fid, '    最大保持架相对打滑指标: %.4f\n', s.slip_max);
    if isfield(post.bearing(ib), 'delta_max')
        fprintf(fid, '    最大接触压缩量: %.6e m\n', max(post.bearing(ib).delta_max));
        fprintf(fid, '    工作游隙范围: %.6e ~ %.6e m\n', min(post.bearing(ib).clearance_work), max(post.bearing(ib).clearance_work));
    end
end
fprintf(fid, '\n四、动态响应峰值明细\n');
fprintf(fid, '  轴承  参数        峰值时间s        峰值             单位\n');
for ib = 1:numel(params.bearing)
    peaks = response_peak_table(params, sim, post, ib);
    for k = 1:numel(peaks)
        fprintf(fid, '  %3d   %-12s  %.9e  %.9e  %s\n', ib, peaks(k).name, peaks(k).time, peaks(k).value, peaks(k).unit);
    end
end
fprintf(fid, '\n');
end

function write_solver_diagnostics(fid, params, sim, post)
fprintf(fid, '五、Newmark-beta + 松弛非线性力迭代求解器诊断\n');
fprintf(fid, '  gamma/beta: %.6f / %.6f\n', params.gamma, params.beta);
fprintf(fid, '  说明: gamma > 0.5 可增加数值阻尼，有助于抑制高频非物理振荡。\n');
fprintf(fid, '  Fen/n_Fen: %d / %d\n', params.Fen, params.n_Fen);
fprintf(fid, '  dt: %.9e s\n', sim.solver.dt);
fprintf(fid, '  max_iter: %d\n', sim.solver.max_iter);
if isfield(sim.solver, 'avg_iter')
    avg_iter = sim.solver.avg_iter;
else
    iter_used = sim.solver.iter_hist(2:end);
    avg_iter = sum(iter_used)/max(numel(iter_used), 1);
end
if isfield(sim.solver, 'max_used_iter')
    max_used_iter = sim.solver.max_used_iter;
else
    max_used_iter = max(sim.solver.iter_hist(2:end));
end
fprintf(fid, '  平均迭代次数: %.4f\n', avg_iter);
fprintf(fid, '  最大实际迭代次数: %d\n', max_used_iter);
fprintf(fid, '  tol_x/tol_R: %.3e / %.3e\n', sim.solver.tol_x, sim.solver.tol_R);
fprintf(fid, '  最大位移迭代误差: %.6e\n', sim.solver.max_err_x);
fprintf(fid, '  最大残差误差: %.6e\n', sim.solver.max_err_R);
fprintf(fid, '  未收敛时间步数: %d\n', sim.solver.unconverged_steps);
fprintf(fid, '  平滑接触启用: %s, smooth_delta=%.6e m\n', yesno(params.contact.smooth_enable), params.contact.smooth_delta);
fprintf(fid, '  接触阻尼启用: %s, cn_ratio=%.4f\n', yesno(params.contact.damping_enable), params.contact.cn_ratio);
for ib = 1:numel(params.bearing)
    fprintf(fid, '  轴承%d接触阻尼 cn: %.6e N*s/m\n', ib, params.bearing(ib).contact_cn);
end
fprintf(fid, '  工作游隙动态更新: %s\n', yesno(params.contact.dynamic_clearance_enable));
fprintf(fid, '  接触刚度诊断平滑: %s, alpha_k=%.3f\n', yesno(params.contact.stiffness_smooth_enable), params.contact.alpha_k);
for ib = 1:numel(params.bearing)
    b = params.bearing(ib);
    fprintf(fid, '  轴承%d热/界面接口: thermal=%s, texture=%s, roughness=%s, debris=%s, waviness=%s\n', ib, ...
        yesno(get_nested_bool(b,'thermal','enable')), yesno(get_nested_bool(b,'surface','texture_enable')), ...
        yesno(get_nested_bool(b,'roughness','enable')), yesno(get_nested_bool(b,'debris','enable')), ...
        yesno(get_nested_bool(b,'waviness','enable')));
end
fprintf(fid, '  时间步解析检查:\n');
for ib = 1:numel(params.bearing)
    [k_contact_eff, f_contact, dt_limit, ok] = contact_time_step_check(params, sim, post, ib);
    fprintf(fid, '    轴承%d: k_contact_eff=%.6e N/m, f_contact≈%.6e Hz, 建议 dt<%.6e s, 当前%s\n', ...
        ib, k_contact_eff, f_contact, dt_limit, ternary(ok, '满足', '可能不足'));
end
if sim.solver.unconverged_steps > 0
    fprintf(fid, '  提示: 存在未收敛时间步，建议增大 Fen、提高平滑区间、检查接触阻尼或降低过高接触刚度。\n');
    fprintf(fid, '  说明: 该时间步未达到设定收敛阈值，结果仅作为趋势参考，程序未对位移/速度/加速度进行人为修平。\n');
end
write_convergence_residual_diagnostics(fid, sim);
fprintf(fid, '  说明: 等效线性支承刚度已加入全局矩阵；油膜线性刚度默认不在非线性力中重复施加，油膜阻尼可作为附加阻尼项参与计算。\n');
fprintf(fid, '  说明: 局部接触刚度为诊断值，不等同于已进入全局切线刚度矩阵。\n');
fprintf(fid, '  说明: 打滑量为基于接触载荷、离心效应和切向速度构造的保持架打滑指标，并非完整保持架动力学方程直接积分解。\n');
fprintf(fid, '  说明: 油膜厚度采用等效EHL载荷修正模型，载荷指数默认 -0.067，用于反映接触载荷升高导致油膜厚度轻微降低的趋势。\n');
write_contact_pressure_note(fid);
write_sanity_checks(fid, params, sim, post);
fprintf(fid, '\n');
end

function write_convergence_residual_diagnostics(fid, sim)
fprintf(fid, '\nConvergence criterion and residual diagnostics\n');
fprintf(fid, '  当前主判据: 原始严格残差判据 err_R_original <= tol_R 且 err_x <= tol_x。\n');
fprintf(fid, '  原始严格判据是否保留: 是。\n');
fprintf(fid, '  归一化残差是否替代原始判据: 否。\n');
assisted = get_field_default(sim.solver, 'use_normalized_residual_assisted_convergence', false);
allow_batch = get_field_default(sim.solver, 'allow_normalized_assisted_for_batch_screening', false);
fprintf(fid, '  归一化残差辅助判据是否启用: %s。\n', yesno(assisted));
fprintf(fid, '  归一化残差辅助判据默认状态: 关闭。\n');
fprintf(fid, '  批量筛选授权 allow_normalized_assisted_for_batch_screening: %s。\n', yesno(allow_batch));
if assisted && ~allow_batch
    fprintf(fid, '  警示: 归一化残差辅助判据已启用，但当前未授权用于批量筛选；请先完成副本对比验证，确认位移峰值、1X/2X/3X、接触力峰值和加速度响应几乎不变。\n');
end
if ~isfield(sim, 'convergence') || isempty(sim.convergence)
    fprintf(fid, '  未找到 convergence 统计结构；该报告由旧结果生成。\n');
    return;
end
c = sim.convergence;
fprintf(fid, '  工程收敛分类:\n');
fprintf(fid, '    strict_converged ratio: %.4f\n', get_field_default(c, 'strict_ratio', NaN));
fprintf(fid, '    engineering_converged ratio: %.4f\n', get_field_default(c, 'engineering_ratio', NaN));
fprintf(fid, '    mild_exceedance ratio: %.4f\n', get_field_default(c, 'mild_ratio', NaN));
fprintf(fid, '    serious_unconverged ratio: %.4f\n', get_field_default(c, 'serious_ratio', NaN));
fprintf(fid, '    strict + engineering ratio: %.4f\n', get_field_default(c, 'strict_plus_engineering_ratio', NaN));
fprintf(fid, '    original reported unconverged ratio: %.4f\n', get_field_default(c, 'reported_unconverged_ratio', NaN));
fprintf(fid, '  原始残差 err_R_original max/median/p95/p99: %.6e / %.6e / %.6e / %.6e\n', ...
    get_field_default(c, 'err_R_original_max', NaN), get_field_default(c, 'err_R_original_median', NaN), ...
    get_field_default(c, 'err_R_original_p95', NaN), get_field_default(c, 'err_R_original_p99', NaN));
fprintf(fid, '  归一化残差 err_R_norm_ref max/median/p95/p99: %.6e / %.6e / %.6e / %.6e\n', ...
    get_field_default(c, 'err_R_norm_ref_max', NaN), get_field_default(c, 'err_R_norm_ref_median', NaN), ...
    get_field_default(c, 'err_R_norm_ref_p95', NaN), get_field_default(c, 'err_R_norm_ref_p99', NaN));
fprintf(fid, '  位移误差 err_x max/median: %.6e / %.6e\n', ...
    get_field_default(c, 'err_x_max', NaN), get_field_default(c, 'err_x_median', NaN));
fprintf(fid, '  归一化残差分类 ratio strict/engineering/mild/serious: %.4f / %.4f / %.4f / %.4f\n', ...
    get_field_default(c, 'normalized_strict_ratio', NaN), get_field_default(c, 'normalized_engineering_ratio', NaN), ...
    get_field_default(c, 'normalized_mild_ratio', NaN), get_field_default(c, 'normalized_serious_ratio', NaN));
fprintf(fid, '  assisted engineering convergence step count: %d\n', round(get_field_default(c, 'assisted_count', 0)));
fprintf(fid, '  解释: 工程收敛不等同于严格收敛。归一化残差用于判断原始严格残差超限是否属于参考力尺度导致的轻微超限。除非通过副本对比验证响应几乎不变，否则不建议将归一化辅助判据用于批量筛选或最终定量结果。\n');
end

function write_judgement(fid, params, post)
xmax_all = 0;
amax_all = 0;
fmax_all = 0;
loaded_max = 0;
slip_max = 0;
vmax_all = 0;
for ib = 1:numel(params.bearing)
    s = post.bearing(ib).summary;
    xmax_all = max(xmax_all, max(s.xmax, s.ymax));
    amax_all = max(amax_all, max(s.axmax, s.aymax));
    fmax_all = max(fmax_all, s.Fmax);
    loaded_max = max(loaded_max, s.loaded_mean);
    slip_max = max(slip_max, s.slip_max);
    vmax_all = max(vmax_all, max(s.vxmax, s.vymax));
end

small_vibration = xmax_all < 10e-6;
acceptable_vibration = xmax_all >= 10e-6 && xmax_all <= 30e-6;
nonlinear_contact = loaded_max > 0;
impact_obvious = amax_all > 10 || fmax_all > 0.8*max([params.bearing.max_contact_force]);
healthy_base = small_vibration && nonlinear_contact && ~impact_obvious && slip_max < 0.6;

fprintf(fid, '六、工况判断\n');
fprintf(fid, '  是否为小振动: %s\n', yesno(small_vibration));
fprintf(fid, '  位移等级: %s\n', displacement_level(xmax_all));
fprintf(fid, '  速度等级: %s\n', velocity_level(vmax_all));
fprintf(fid, '  加速度等级: %s\n', acceleration_level(amax_all));
fprintf(fid, '  是否进入非线性接触: %s\n', yesno(nonlinear_contact));
fprintf(fid, '  是否存在明显冲击: %s\n', yesno(impact_obvious));
fprintf(fid, '  是否适合作为健康基准/强耦合承载工况: %s\n', yesno(healthy_base || acceptable_vibration));
fprintf(fid, '  判断依据: 位移量级、接触力连续性、承载滚动体数、打滑指标和加速度峰值综合判定。\n');
fprintf(fid, '  分析提示: 可通过对比 bearing*_quasi_detail.csv 中的油膜、PV、lambda、摩擦系数和 dynamic_peak_summary.csv 中的响应峰值，论证微观界面因素如何影响接触状态、支承刚度和转子振动。\n\n');
write_baseline_usage_note(fid);
end

function write_figure_description(fid, params)
fprintf(fid, '\nFigure output description\n');
fprintf(fid, '  Figure folder: %s\n', params.figure_dir);
fprintf(fid, '  Bearing dynamic figures: incremental contact force, local contact-stiffness diagnostic, equivalent working clearance, effective minimum oil film, cage slip indicator.\n');
fprintf(fid, '  Rotor response figures: displacement, velocity, acceleration, demeaned orbit, x/y spectrum.\n');
fprintf(fid, '  Bearing baseline figures: bearing_baseline/bearing*_load_distribution.png, bearing*_oil_film_distribution.png, loaded_count_history.png, slip_ratio_history.png, bearing_force_history.png.\n');
fprintf(fid, '  The remaining figures support: interface factors -> bearing dynamics -> rotor vibration response.\n\n');
return;
fprintf(fid, '七、图片输出说明\n');
fprintf(fid, '  图片目录: %s\n', params.figure_dir);
fprintf(fid, '  轴承动态性能图片用于说明微观界面因素对接触状态、油膜状态、接触刚度和打滑率的影响。\n');
fprintf(fid, '    bearing1_load_distribution.png / bearing2_load_distribution.png: 滚动体承载分布，显示承载区和载荷分配。\n');
fprintf(fid, '    bearing1_contact_force.png / bearing2_contact_force.png: x方向、y方向和合接触力时域响应。\n');
fprintf(fid, '    bearing1_contact_stiffness.png / bearing2_contact_stiffness.png: 接触刚度随时间变化。\n');
fprintf(fid, '    bearing1_clearance_work.png / bearing2_clearance_work.png: 工作游隙变化；当前热效应关闭时为常值曲线。\n');
fprintf(fid, '    bearing1_oil_film.png / bearing2_oil_film.png: 最小油膜厚度变化。\n');
fprintf(fid, '    bearing1_slip_ratio.png / bearing2_slip_ratio.png: 保持架打滑率变化。\n');
fprintf(fid, '    bearing1_loaded_roller_count.png / bearing2_loaded_roller_count.png: 承载滚动体数量变化。\n');
fprintf(fid, '  转子系统响应图片用于说明轴承动态性能变化传递到转子振动响应后的结果。\n');
fprintf(fid, '    rotor_bearing1_displacement.png / rotor_bearing2_displacement.png: 转子轴承处位移响应。\n');
fprintf(fid, '    rotor_bearing1_velocity.png / rotor_bearing2_velocity.png: 转子轴承处速度响应。\n');
fprintf(fid, '    rotor_bearing1_acceleration.png / rotor_bearing2_acceleration.png: 转子轴承处加速度响应。\n');
fprintf(fid, '    rotor_bearing1_orbit.png / rotor_bearing2_orbit.png: 转子轴心轨迹。\n');
fprintf(fid, '    rotor_bearing1_orbit_demean.png / rotor_bearing2_orbit_demean.png: 去均值转子轴心轨迹。\n');
fprintf(fid, '    rotor_bearing*_spectrum_x/y.png: 转子轴承处x/y方向频谱，并标注1X、2X和保持架频率。\n');
fprintf(fid, '  微观界面因素启用状态:\n');
for ib = 1:numel(params.bearing)
    b = params.bearing(ib);
    fprintf(fid, '    轴承%d: thermal=%s, texture=%s, roughness=%s, debris=%s, waviness=%s\n', ib, ...
        yesno(get_nested_bool(b,'thermal','enable')), yesno(get_nested_bool(b,'surface','texture_enable')), ...
        yesno(get_nested_bool(b,'roughness','enable')), yesno(get_nested_bool(b,'debris','enable')), ...
        yesno(get_nested_bool(b,'waviness','enable')));
end
fprintf(fid, '  当前未启用的微观界面因素不会强制生成对比图；启用后会自动输出对应 effect_compare 图片。\n\n');
end

function write_output_list(fid, params, post)
fprintf(fid, '八、输出文件\n');
fprintf(fid, '  主报告: %s\n', params.report_file);
fprintf(fid, '  转子响应 CSV: %s\n', get_field_default(post, 'csv_file', fullfile(params.output_dir, 'coupled_response_last5cycles.csv')));
for ib = 1:numel(params.bearing)
    fprintf(fid, '  轴承%d拟动力学明细 CSV: %s\n', ib, fullfile(params.output_dir, sprintf('bearing%d_quasi_detail.csv', ib)));
end
fprintf(fid, '  动态响应峰值 CSV: %s\n', fullfile(params.output_dir, 'dynamic_peak_summary.csv'));
fprintf(fid, '  图片目录: %s\n', params.figure_dir);
fprintf(fid, '  MAT 文件: %s\n', params.result_mat_file);
end

function write_bearing_detail_csv(params, ib, b, r)
file_name = fullfile(params.output_dir, sprintf('bearing%d_quasi_detail.csv', ib));
fid = fopen(file_name, 'wt');
if fid < 0
    warning('Cannot write %s', file_name);
    return;
end
cleanupObj = onCleanup(@() fclose(fid));
[pressure, Fc, slip_i] = bearing_detail_arrays(b, r);
fprintf(fid, 'bearing_id,element_id,loaded,theta_deg,Q_N,delta_m,oil_film_m,contact_pressure_diagnostic_Pa,PV_Pa_mps,mu,lambda,slip_ratio,centrifugal_force_N\n');
for i = 1:numel(r.Q)
    fprintf(fid, '%d,%d,%d,%.9e,%.9e,%.9e,%.9e,%.9e,%.9e,%.9e,%.9e,%.9e,%.9e\n', ...
        ib, i, r.Q(i) > 0, wrap_deg(r.theta(i)*180/pi), r.Q(i), r.delta(i), r.oil_film(i), pressure(i), r.PV(i), r.mu_eff(i), r.lambda(i), slip_i(i), Fc);
end
end

function write_bearing_tangent_csv(params, ib, b, r)
file_name = fullfile(params.output_dir, sprintf('bearing%d_explicit_tangent_detail.csv', ib));
fid = fopen(file_name, 'wt');
if fid < 0
    warning('Cannot write %s', file_name);
    return;
end
cleanupObj = onCleanup(@() fclose(fid));
diag = bearing_tangent_diagnostics(b, r);
fprintf(fid, 'bearing_id,element_id,loaded,theta_deg,contact_deformation_m,contact_force_N,analytical_tangent_stiffness_Npm,finite_difference_tangent_stiffness_Npm,relative_difference,oil_film_m,lambda,slip_indicator\n');
for i = 1:numel(diag.element_id)
    fprintf(fid, '%d,%d,%d,%.9e,%.9e,%.9e,%.9e,%.9e,%.9e,%.9e,%.9e,%.9e\n', ...
        ib, diag.element_id(i), diag.loaded(i), diag.theta_deg(i), diag.delta(i), diag.Q(i), ...
        diag.kt_analytical(i), diag.kt_finite_difference(i), diag.kt_relative_difference(i), ...
        diag.h(i), diag.lambda(i), diag.slip(i));
end
end

function write_tangent_stiffness_section(fid, params, bearingQD, sim, post)
fprintf(fid, '三、显式非线性切线刚度输出\n');
fprintf(fid, '  修改逻辑: 原先最大接触力和刚度主要用于工况对比诊断，不能充分说明非线性接触在当前平衡状态下的局部线性化特征。本次修改通过对每个受载滚动体的接触力-接触变形关系求导，得到显式非线性切线刚度，并投影汇总为轴承局部 2x2 切线刚度矩阵。\n');
fprintf(fid, '  用途说明: 当前实现将显式非线性切线刚度作为严格后处理量输出，未装配进入全局 Newmark/Newton 有效刚度矩阵，因此不改变已有动力学结果输出流程。\n');
fprintf(fid, '  区分说明: diagnostic equivalent stiffness 为原有工况对比诊断量；explicit nonlinear tangent stiffness 为本节输出的局部线性化量；support series stiffness 是耦合动力学方程中实际使用的串联支承刚度；oil film stiffness force 是否进入动力学由 include_oil_stiffness_force 控制。\n');
for ib = 1:numel(params.bearing)
    b = params.bearing(ib);
    qdiag = bearing_tangent_diagnostics(b, bearingQD{ib});
    fprintf(fid, '  bearing%d (%s) quasi-dynamic equilibrium explicit tangent matrix local y/z, N/m:\n', ib, b.type);
    fprintf(fid, '    [%.6e  %.6e]\n', qdiag.K_local(1,1), qdiag.K_local(1,2));
    fprintf(fid, '    [%.6e  %.6e]\n', qdiag.K_local(2,1), qdiag.K_local(2,2));
    fprintf(fid, '    trace=%.6e, eigenvalues=[%.6e %.6e], max Q=%.6e N, Q RMS=%.6e N\n', ...
        qdiag.trace, qdiag.eigenvalues(1), qdiag.eigenvalues(2), qdiag.max_contact_force, qdiag.contact_force_rms);
    [meanK, maxK, minK, meanTrace, meanEig, qmax, qrms] = dynamic_tangent_summary(params, sim, post, ib);
    fprintf(fid, '  bearing%d末5周期显式切线刚度统计 local y/z, N/m:\n', ib);
    fprintf(fid, '    mean K = [%.6e %.6e; %.6e %.6e]\n', meanK(1,1), meanK(1,2), meanK(2,1), meanK(2,2));
    fprintf(fid, '    max  K = [%.6e %.6e; %.6e %.6e]\n', maxK(1,1), maxK(1,2), maxK(2,1), maxK(2,2));
    fprintf(fid, '    min  K = [%.6e %.6e; %.6e %.6e]\n', minK(1,1), minK(1,2), minK(2,1), minK(2,2));
    fprintf(fid, '    mean trace=%.6e, mean eigenvalues=[%.6e %.6e], max Q=%.6e N, Q RMS=%.6e N\n', ...
        meanTrace, meanEig(1), meanEig(2), qmax, qrms);
    fprintf(fid, '    detail CSV: %s\n', fullfile(params.output_dir, sprintf('bearing%d_explicit_tangent_detail.csv', ib)));
end
fprintf(fid, '\n');
end

function [meanK, maxK, minK, meanTrace, meanEig, qmax, qrms] = dynamic_tangent_summary(params, sim, post, ib)
idx = post.idx_plot;
N = size(sim.yn, 1);
force_params = params;
if ~isfield(force_params, 'modelInfo') && isfield(sim, 'params')
    force_params = sim.params;
end
Ks = zeros(2,2,numel(idx));
traces = zeros(1,numel(idx));
eigs2 = zeros(numel(idx),2);
qmax = 0;
q2sum = 0;
qcount = 0;
for k = 1:numel(idx)
    [~, st] = F_bearing(sim.yn(:,idx(k)), sim.dyn(:,idx(k)), force_params, N);
    d = bearing_tangent_diagnostics(force_params.bearing(ib), st.bearings(ib));
    Ks(:,:,k) = d.K_local;
    traces(k) = d.trace;
    eigs2(k,:) = d.eigenvalues;
    qmax = max(qmax, d.max_contact_force);
    q2sum = q2sum + sum(d.Q(:).^2);
    qcount = qcount + numel(d.Q);
end
meanK = mean(Ks, 3);
maxK = max(Ks, [], 3);
minK = min(Ks, [], 3);
meanTrace = mean(traces);
meanEig = mean(eigs2, 1);
qrms = sqrt(q2sum/max(qcount, 1));
end

function write_peak_csv(params, sim, post)
file_name = fullfile(params.output_dir, 'dynamic_peak_summary.csv');
fid = fopen(file_name, 'wt');
if fid < 0
    warning('Cannot write %s', file_name);
    return;
end
cleanupObj = onCleanup(@() fclose(fid));
fprintf(fid, 'bearing_id,item,time_s,value,unit\n');
for ib = 1:numel(params.bearing)
    peaks = response_peak_table(params, sim, post, ib);
    for k = 1:numel(peaks)
        fprintf(fid, '%d,%s,%.12e,%.12e,%s\n', ib, peaks(k).name, peaks(k).time, peaks(k).value, peaks(k).unit);
    end
end
end

function peaks = response_peak_table(params, sim, post, ib)
b = params.bearing(ib);
idx = post.idx_plot;
t = sim.time(idx);
rn = b.rotor_node;
x = sim.yn(4*rn-3,idx);
y = sim.yn(4*rn-2,idx);
vx = sim.dyn(4*rn-3,idx);
vy = sim.dyn(4*rn-2,idx);
ax = sim.ddyn(4*rn-3,idx);
ay = sim.ddyn(4*rn-2,idx);
Fx = sim.F_b_hist(2*ib-1,idx);
Fy = sim.F_b_hist(2*ib,idx);
Fmag = sqrt(Fx.^2 + Fy.^2);
loaded = sim.loaded_count_hist(ib,idx);
slip = sim.slip_hist(ib,idx);

items = {'x','y','vx','vy','ax','ay','Fx','Fy','Fmag','loaded_count','slip_ratio'};
values = {x,y,vx,vy,ax,ay,Fx,Fy,Fmag,loaded,slip};
units = {'m','m','m/s','m/s','m/s^2','m/s^2','N','N','N','count','1'};
for k = 1:numel(items)
    arr = values{k};
    if strcmp(items{k}, 'loaded_count') || strcmp(items{k}, 'slip_ratio') || strcmp(items{k}, 'Fmag')
        [val, loc] = max(arr);
    else
        [~, loc] = max(abs(arr));
        val = arr(loc);
    end
    peaks(k).name = items{k}; %#ok<AGROW>
    peaks(k).time = t(loc);
    peaks(k).value = val;
    peaks(k).unit = units{k};
end
end

function [pressure, Fc, slip_i] = bearing_detail_arrays(b, r)
if strcmpi(b.type, 'ball')
    area = max(pi*(0.20*b.Db)^2, 1e-12);
    rho_b = get_field_default(b, 'ball_density', 7850);
    m_elem = rho_b*pi*b.Db^3/6;
    Fc = m_elem*r.cage_speed^2*(b.Dm/2);
    slip_i = min(0.55, max(0, 0.015 + 0.25*Fc./(r.Q + Fc + 1)));
    slip_i(r.Q <= 0) = 0.40;
else
    area = max(b.L*b.Dw, 1e-12);
    rho_r = get_field_default(b, 'roller_density', 7850);
    m_elem = rho_r*pi*b.Dw^2/4*b.L;
    Fc = m_elem*r.cage_speed^2*(b.Dm/2);
    slip_i = min(0.60, max(0, 0.02 + 0.30*Fc./(r.Q + Fc + 1)));
    slip_i(r.Q <= 0) = 0.45;
end
pressure = r.Q/area;
end

function [k_contact_eff, f_contact, dt_limit, ok] = contact_time_step_check(params, sim, post, ib)
k_contact_eff = 0;
if isfield(sim, 'bearingStateHist') && ~isempty(sim.bearingStateHist)
    for k = post.idx_plot
        st = sim.bearingStateHist{k};
        if ~isempty(st) && isfield(st, 'bearings') && numel(st.bearings) >= ib && isfield(st.bearings(ib), 'k_contact_eff')
            k_contact_eff = max(k_contact_eff, st.bearings(ib).k_contact_eff);
        end
    end
elseif isfield(post.bearing(ib), 'contact_stiffness')
    k_contact_eff = max(post.bearing(ib).contact_stiffness);
end
b = params.bearing(ib);
if strcmpi(b.type, 'ball')
    rho = get_field_default(b, 'ball_density', 7850);
    m_eff = rho*pi*b.Db^3/6;
else
    rho = get_field_default(b, 'roller_density', 7850);
    m_eff = rho*pi*b.Dw^2/4*b.L;
end
if k_contact_eff <= 0 || m_eff <= 0
    f_contact = 0;
    dt_limit = inf;
    ok = true;
else
    f_contact = 1/(2*pi)*sqrt(k_contact_eff/m_eff);
    dt_limit = 1/(50*f_contact);
    ok = sim.solver.dt < dt_limit;
end
end

function v = get_nested_bool(s, parent_field, child_field)
v = false;
if isfield(s, parent_field) && isfield(s.(parent_field), child_field) && ~isempty(s.(parent_field).(child_field))
    v = s.(parent_field).(child_field);
end
end

function deg = wrap_deg(deg)
deg = mod(deg, 360);
end

function out = ternary(cond, a, b)
if cond
    out = a;
else
    out = b;
end
end

function s = contact_pressure_label_cn()
s = '最大接触压力诊断值/估计平均接触压强';
end

function s = contact_pressure_column_cn()
s = '接触压力诊断值Pa';
end

function write_contact_pressure_note(fid)
fprintf(fid, '  说明：本文输出的接触压力为基于当前等效接触载荷和简化接触面积估算得到的接触压力诊断值，主要用于不同工况之间的相对比较和趋势分析。该值不等同于完整三维 Hertz 接触理论或 TEHL 模型求解得到的严格最大接触应力。\n');
fprintf(fid, '  Note: The reported contact pressure is a diagnostic value estimated from the equivalent contact load and simplified contact area. It is mainly used for relative comparison and trend analysis under different operating conditions, and should not be interpreted as the exact maximum Hertzian contact stress obtained from a full three-dimensional contact or TEHL solution.\n');
end

function write_baseline_usage_note(fid)
fprintf(fid, '  基准结果使用说明：当前结果为微观界面因素未开启条件下的健康基准工况。该结果可用于验证轴承拟动力学模型对承载滚动体分布、油膜厚度、PV 值、保持架相对打滑指标和等效支承刚度的计算能力，并作为后续热效应、表面纹理、粗糙度、杂质和波纹度等界面因素分析的对照组。由于当前 thermal、texture、roughness、debris 和 waviness 接口均处于关闭状态，因此本报告不直接代表微观界面因素影响规律，只作为 baseline 结果。\n');
fprintf(fid, '  The present result corresponds to a healthy baseline condition without activating the micro-interface factor switches. It is used to verify the capability of the bearing quasi-dynamic model in predicting loaded rolling-element distribution, oil-film thickness, PV value, cage relative slip index, and equivalent support stiffness. It also serves as a reference case for subsequent studies involving thermal effects, surface texture, roughness, debris disturbance, and waviness. Since the thermal, texture, roughness, debris, and waviness interfaces are disabled in the current case, this report should be interpreted as a baseline result rather than a direct demonstration of micro-interface effects.\n\n');
end

function normalize_report_labels(report_file)
if ~exist(report_file, 'file')
    return;
end
txt = fileread(report_file);
txt = strrep(txt, '最大接触应力/平均接触压强估计', contact_pressure_label_cn());
txt = strrep(txt, '最大接触应力', '最大接触压力诊断值');
txt = strrep(txt, '最大平均接触压强估计', '最大接触压力诊断值/估计平均接触压强');
txt = strrep(txt, '接触压强Pa', contact_pressure_column_cn());
txt = strrep(txt, '接触压强 Pa', '接触压力诊断值 Pa');
txt = strrep(txt, '平均接触压强(Pa)', '接触压力诊断值(Pa)');
txt = strrep(txt, '严格最大接触压力诊断值', '严格最大接触应力');
txt = strrep(txt, 'pressure_Pa', 'contact_pressure_diagnostic_Pa');
txt = strrep(txt, 'pressure Pa', 'contact pressure diagnostic value Pa');
txt = strrep(txt, 'Removed by current setting: bearing load distribution, loaded roller count history, non-demeaned orbit.', ...
    'Bearing baseline figures restored: bearing_baseline/bearing*_load_distribution.png, bearing*_oil_film_distribution.png, loaded_count_history.png, slip_ratio_history.png, bearing_force_history.png.');
fid = fopen(report_file, 'wt', 'n', 'UTF-8');
if fid < 0
    warning('Cannot normalize report labels: %s', report_file);
    return;
end
cleanupObj = onCleanup(@() fclose(fid));
fprintf(fid, '%s', txt);
end

function s = yesno(v)
if v
    s = '是';
else
    s = '否';
end
end

function s = displacement_level(xmax)
if xmax < 10e-6
    s = '小振动（<10 μm）';
elseif xmax <= 30e-6
    s = '明显振动但工程可接受（10–30 μm）';
else
    s = '可能接近异常（>30 μm）';
end
end

function s = velocity_level(vmax)
vmm = vmax*1e3;
if vmm < 1
    s = '优秀（<1 mm/s）';
elseif vmm < 3
    s = '良好/可接受（1–3 mm/s）';
elseif vmm < 7
    s = '需要关注（3–7 mm/s）';
else
    s = '异常风险（>7 mm/s）';
end
end

function s = acceleration_level(amax)
if amax < 5
    s = '正常（<5 m/s^2）';
elseif amax < 20
    s = '需要关注（5–20 m/s^2）';
else
    s = '可能存在冲击或数值问题（>20 m/s^2）';
end
end

function write_sanity_checks(fid, params, sim, post)
fprintf(fid, '\n  Engineering sanity checks\n');
fr = params.rpm/60;
unb_amp = resultant_unbalance_force(params);
total_steps = max(1, numel(sim.time)-1);
unconv_ratio = get_field_default(sim.solver, 'unconverged_steps', 0)/total_steps;
fprintf(fid, '    unconverged step ratio: %.4f\n', unconv_ratio);
if unconv_ratio > 0.05
    fprintf(fid, '    WARNING: more than 5%% steps did not meet convergence tolerances; numerical credibility is limited.\n');
end
for ib = 1:numel(params.bearing)
    b = params.bearing(ib);
    s = post.bearing(ib).summary;
    cw_vec = post.bearing(ib).clearance_work(:);
    cwork = max(abs(sum(cw_vec)/max(numel(cw_vec), 1)), eps);
    disp_peak = max(s.xmax, s.ymax);
    if disp_peak < 0.3*cwork
        fprintf(fid, '    bearing %d: displacement < 0.3*clearance, small perturbation response.\n', ib);
    elseif disp_peak > cwork
        fprintf(fid, '    bearing %d WARNING: displacement exceeds working clearance; strong nonlinear contact or parameter issue may exist.\n', ib);
    end

    x = sim.yn(4*b.rotor_node-3, post.idx_plot);
    y = sim.yn(4*b.rotor_node-2, post.idx_plot);
    t = sim.time(post.idx_plot);
    X1 = max(first_harmonic_amplitude(t, x, fr), first_harmonic_amplitude(t, y, fr));
    v_est = 2*pi*fr*X1;
    a_est = (2*pi*fr)^2*X1;
    v_peak = max(s.vxmax, s.vymax);
    a_peak = max(s.axmax, s.aymax);
    fprintf(fid, '    bearing %d: 1X displacement amplitude estimate %.6e m, v_est %.6e m/s, a_est %.6e m/s2.\n', ib, X1, v_est, a_est);
    if v_peak > 5*max(v_est, eps) || v_est > 5*max(v_peak, eps)
        fprintf(fid, '    bearing %d WARNING: time-domain velocity differs from 1X estimate by >5x; high-frequency contact modulation/noise may be present.\n', ib);
    end
    if a_peak > 5*max(a_est, eps) || a_est > 5*max(a_peak, eps)
        fprintf(fid, '    bearing %d NOTE: acceleration differs from 1X estimate by >5x; high-frequency contact modulation is significant.\n', ib);
    end
    if unb_amp > 0 && s.Fmax/unb_amp > 100
        fprintf(fid, '    bearing %d WARNING: dynamic bearing force is >100x resultant unbalance; check stiffness, clearance, preload and duplicate stiffness.\n', ib);
    end

    rough = roughness_rms_report(b);
    lambda_min = min(post.bearing(ib).oil_film_min)/max(rough, 1e-12);
    fprintf(fid, '    bearing %d: minimum lambda estimate %.3f.\n', ib, lambda_min);
    if lambda_min < 1
        fprintf(fid, '    bearing %d WARNING: boundary lubrication risk.\n', ib);
    elseif lambda_min < 3
        fprintf(fid, '    bearing %d NOTE: mixed lubrication regime.\n', ib);
    end
    if s.slip_max > 0.5
        fprintf(fid, '    bearing %d WARNING: slip indicator >0.5; empirical model applicability may be exceeded.\n', ib);
    elseif s.slip_max > 0.3
        fprintf(fid, '    bearing %d NOTE: high cage slip risk indicated.\n', ib);
    end
end
end

function A = first_harmonic_amplitude(t, x, fr)
x = x(:);
x = x - sum(x)/max(numel(x), 1);
t = t(:);
if numel(t) < 3 || fr <= 0
    A = 0;
    return;
end
c = cos(2*pi*fr*t);
s = sin(2*pi*fr*t);
ac = 2/numel(t)*sum(x.*c);
as = 2/numel(t)*sum(x.*s);
A = hypot(ac, as);
end

function F = resultant_unbalance_force(params)
if ~isfield(params, 'unbalance') || ~isfield(params.unbalance, 'nodes')
    F = 0;
    return;
end
ub = params.unbalance;
if ~isfield(ub, 'phase') || isempty(ub.phase)
    ub.phase = zeros(size(ub.nodes));
end
Fx = 0;
Fy = 0;
for k = 1:numel(ub.nodes)
    F0 = ub.mass_g(k)*1e-3*ub.ecc_mm(k)*1e-3*params.omega^2;
    Fx = Fx + F0*cos(ub.phase(k));
    Fy = Fy + F0*sin(ub.phase(k));
end
F = hypot(Fx, Fy);
end

function Q_ref = ehl_load_reference_report(bearing)
if isfield(bearing, 'Q_ref') && ~isempty(bearing.Q_ref)
    Q_ref = bearing.Q_ref;
else
    preload = max([abs(get_field_default(bearing, 'preload_z', 0)), abs(get_field_default(bearing, 'preload_y', 0)), 1]);
    Q_ref = preload/max(get_field_default(bearing, 'n', 1)/2, 1);
end
Q_ref = max(Q_ref, 1e-6);
end

function rough = roughness_rms_report(b)
if isfield(b, 'roughness') && get_field_default(b.roughness, 'enable', false)
    rough = sqrt(get_field_default(b.roughness, 'Rq_inner', 0.1e-6)^2 + get_field_default(b.roughness, 'Rq_outer', 0.1e-6)^2);
else
    rough = get_field_default(b, 'roughness_rms', 0.1e-6);
end
rough = max(rough, 1e-12);
end

function v = get_field_default(s, field, default_value)
if isfield(s, field) && ~isempty(s.(field))
    v = s.(field);
else
    v = default_value;
end
end
