function files = write_bearing_quasi_txt_reports(params, bearingQD)
%WRITE_BEARING_QUASI_TXT_REPORTS Write separate quasi-dynamic TXT reports.
% Inputs:
%   params    - global simulation parameters.
%   bearingQD - cell array from bearing_quasi_dynamic_ball/roller.
% Output:
%   files     - generated TXT file paths.
%
% The two reports are intended for the paper section before the coupled
% rotor response: micro-interface factors -> bearing dynamic performance.

out_dir = get_field_default(params, 'bearing_txt_dir', 'D:\mixed\gun_qiu');
if ~isfolder(out_dir)
    mkdir(out_dir);
end

files = cell(1, numel(params.bearing));
for ib = 1:numel(params.bearing)
    b = params.bearing(ib);
    r = bearingQD{ib};
    if strcmpi(b.type, 'ball')
        file_name = fullfile(out_dir, 'qiu.txt');
    elseif strcmpi(b.type, 'roller')
        file_name = fullfile(out_dir, 'gunzi.txt');
    else
        file_name = fullfile(out_dir, sprintf('bearing%d.txt', ib));
    end
    write_one_bearing_report(file_name, ib, b, r, params);
    files{ib} = file_name;
end
end

function write_one_bearing_report(file_name, ib, b, r, params)
fid = fopen(file_name, 'wt', 'n', 'UTF-8');
if fid < 0
    error('Cannot open bearing TXT report: %s', file_name);
end
cleanupObj = onCleanup(@() fclose(fid));
% Write UTF-8 BOM so Windows Notepad/Word can recognize Chinese text.
fwrite(fid, uint8([239 187 191]), 'uint8');

[pressure, Fc, slip_i, contact_area] = detail_arrays(b, r);
loaded = r.Q > 0;

fprintf(fid, '\t\t\t\t\t\t************************\n');
if strcmpi(b.type, 'ball')
    fprintf(fid, '\t\t\t\t\t\t考虑微观界面因素的高速球轴承拟动力学分析\n');
else
    fprintf(fid, '\t\t\t\t\t\t考虑微观界面因素的高速滚子轴承拟动力学分析\n');
end
fprintf(fid, '\t\t\t\t\t\t************************\n\n');

fprintf(fid, '说明：本文件用于论文中“高速滚动轴承动态性能分析”部分，位于轴承-转子-机匣耦合响应分析之前。\n');
fprintf(fid, '      预载仅用于计算轴承工作游隙、接触状态、润滑状态和等效支承参数，不在转子动力学方程中重复施加。\n\n');

fprintf(fid, '*******************************输入参数**********************************\n\n');
fprintf(fid, '轴承编号: %d\n', ib);
fprintf(fid, '轴承名称: %s\n', get_field_default(b, 'name', 'bearing'));
fprintf(fid, '轴承类型: %s\n', b.type);
fprintf(fid, '转子节点: %d    机匣轴承座节点: %d\n', b.rotor_node, b.case_node);
fprintf(fid, '转速: %.6f r/min    角速度: %.6f rad/s\n', params.rpm, params.omega);
fprintf(fid, '局部径向预载: y方向 %.6e N, z方向 %.6e N\n', get_field_default(b,'preload_y',0), get_field_default(b,'preload_z',0));
fprintf(fid, '程序坐标映射: 程序 x方向 = 轴承局部 y向径向；程序 y方向 = 轴承局部 z向径向。\n\n');

if strcmpi(b.type, 'ball')
    fprintf(fid, '  轴承中径(mm)  球直径(mm)   球数目   接触角(deg)   初始游隙(um)\n');
    fprintf(fid, '  %12.6e %12.6e %8d %12.6e %12.6e\n\n', ...
        b.Dm*1000, b.Db*1000, b.n, get_field_default(b,'contact_angle',0)*180/pi, get_field_default(b,'clearance0',0)*1e6);
else
    fprintf(fid, '  轴承中径(mm)  滚子直径(mm)  滚子数目  滚子长度(mm)  初始游隙(um)\n');
    fprintf(fid, '  %12.6e %12.6e %8d %12.6e %12.6e\n\n', ...
        b.Dm*1000, b.Dw*1000, b.n, get_field_default(b,'L',0)*1000, get_field_default(b,'clearance0',0)*1e6);
end

fprintf(fid, '************************************************************************\n');
fprintf(fid, '          弹性与润滑参数\n');
fprintf(fid, '  弹性模量E(Pa): %.6e    泊松比: %.6f\n', get_field_default(params,'E',2.06e11), get_field_default(params,'nu',0.3));
if isfield(params, 'lubricant')
    oilT = lubricant_properties(get_field_default(params.lubricant, 'current_temperature', get_field_default(b, 'temperature', 20)), params);
    fprintf(fid, '  润滑油: %s\n', oilT.name);
    fprintf(fid, '  运动黏度(cSt): %.6e    动力黏度(Pa*s): %.6e    压黏系数(Pa^-1): %.6e\n', oilT.nu_cSt, oilT.eta, oilT.alpha_p);
end
fprintf(fid, '  润滑油密度(kg/m3): %.6e    入口油温(℃): %.6f\n', get_field_default(b,'oil_density',NaN), get_field_default(b,'temperature',NaN));
fprintf(fid, '  动力黏度(Pa*s): %.6e    压黏系数(Pa^-1): %.6e    摩擦系数: %.6f\n', ...
    get_field_default(b,'oil_viscosity',NaN), get_field_default(b,'pressure_viscosity',NaN), get_field_default(b,'mu',NaN));
fprintf(fid, '  基础油膜厚度参数(m): %.6e    粗糙度RMS(m): %.6e\n', get_field_default(b,'oil_film0',NaN), get_field_default(b,'roughness_rms',NaN));
fprintf(fid, '  等效EHL载荷修正: enable=%s, exponent=%.6f, factor range=[%.3f, %.3f]\n\n', ...
    yesno(get_field_default(b,'ehl_load_correction_enable',true)), get_field_default(b,'ehl_load_exponent',-0.067), ...
    get_field_default(b,'ehl_load_factor_min',0.4), get_field_default(b,'ehl_load_factor_max',2.5));

fprintf(fid, '************************************************************************\n');
fprintf(fid, '          微观界面因素开关\n');
fprintf(fid, '  热效应: %s\n', yesno(nested_bool(b,'thermal','enable')));
fprintf(fid, '  表面纹理: %s, texture_gamma=%.6e, Cr=%.6e\n', yesno(nested_bool(b,'surface','texture_enable')), ...
    nested_value(b,'surface','texture_gamma',0), nested_value(b,'surface','Cr',1));
fprintf(fid, '  粗糙度: %s, Rq_inner=%.6e m, Rq_outer=%.6e m\n', yesno(nested_bool(b,'roughness','enable')), ...
    nested_value(b,'roughness','Rq_inner',get_field_default(b,'roughness_rms',0)), nested_value(b,'roughness','Rq_outer',get_field_default(b,'roughness_rms',0)));
fprintf(fid, '  杂质: %s, ud=%.6e m\n', yesno(nested_bool(b,'debris','enable')), nested_value(b,'debris','ud',0));
fprintf(fid, '  波纹度: %s, P2_amp=%.6e m, order=%.6f\n\n', yesno(nested_bool(b,'waviness','enable')), ...
    nested_value(b,'waviness','P2_amp',0), nested_value(b,'waviness','order',get_field_default(b,'waviness_order',0)));

fprintf(fid, '\n\n*****************************计算结果***********************************\n\n');
fprintf(fid, '一、总体动态性能\n');
fprintf(fid, '  承载滚动体数量: %d / %d\n', r.loaded_count, b.n);
fprintf(fid, '  最大单滚动体接触负荷: %.6e N\n', r.max_contact_load);
fprintf(fid, '  最大接触压力诊断值/估计平均接触压强: %.6e Pa\n', max(pressure));
fprintf(fid, '  最小等效油膜厚度: %.6e m = %.6f um\n', r.min_oil_film, r.min_oil_film*1e6);
fprintf(fid, '  最小膜厚比lambda: %.6f\n', min(r.lambda));
fprintf(fid, '  最大PV值: %.6e Pa*m/s\n', max(r.PV));
fprintf(fid, '  保持架相对打滑指标: %.6f\n', r.slip_ratio);
fprintf(fid, '  保持架理论公转角速度: %.6e rad/s = %.6f r/min\n', r.cage_speed, r.cage_speed*30/pi);
if isfield(r, 'spin_speed')
    fprintf(fid, '  滚动体自转角速度估计: %.6e rad/s = %.6f r/min\n', r.spin_speed, r.spin_speed*30/pi);
end
fprintf(fid, '  单个滚动体离心力估计: %.6e N\n', Fc);
fprintf(fid, '  等效接触面积估计: %.6e m^2\n\n', contact_area);

fprintf(fid, '二、各滚动体接触、润滑和摩擦状态\n');
fprintf(fid, '  号  方位角(deg)  是否承载  接触负荷(N)  接触变形(m)  接触压力诊断值(Pa)  油膜厚度(m)  lambda  PV(Pa*m/s)  摩擦系数  打滑指标\n');
for i = 1:numel(r.Q)
    fprintf(fid, ' %3d %12.3f %8d %13.6e %13.6e %13.6e %13.6e %8.3f %13.6e %9.5f %9.5f\n', ...
        i, wrap_deg(r.theta(i)*180/pi), loaded(i), r.Q(i), r.delta(i), pressure(i), r.oil_film(i), r.lambda(i), r.PV(i), r.mu_eff(i), slip_i(i));
end
fprintf(fid, '\n');

fprintf(fid, '三、工作游隙分解\n');
ci = r.clearInfo;
fprintf(fid, '  初始游隙 clearance0:              %.6e m\n', ci.c0);
fprintf(fid, '  初始轴向间隙 axial_clearance0:    %.6e m\n', get_field_default(b, 'axial_clearance0', NaN));
fprintf(fid, '  装配引起的游隙变化 clearance_fit: %.6e m\n', ci.delta_fit);
fprintf(fid, '  温差引起的游隙变化 clearance_temp: %.6e m\n', ci.delta_thermal);
fprintf(fid, '  离心力引起的游隙变化 clearance_centrifugal: %.6e m\n', ci.delta_centrifugal);
fprintf(fid, '  其他界面修正 clearance_other:     %.6e m\n', ci.delta_other);
fprintf(fid, '  最终工作游隙 clearance_work:      %.6e m\n', r.clearance_work);
if isfield(ci, 'T_inner')
    fprintf(fid, '  热模型温度: T_inner=%.3f ℃, T_outer=%.3f ℃, deltaT_io=%.3f ℃\n', ci.T_inner, ci.T_outer, ci.deltaT_io);
end
if r.clearance_work <= 0
    fprintf(fid, '  提示: 工作游隙为负，轴承处于预紧或过盈接触状态，需要检查配合和温升参数。\n');
elseif abs(r.clearance_work - ci.c0) > 0.5*max(abs(ci.c0), eps)
    fprintf(fid, '  提示: 工作游隙相对初始游隙变化超过50%%，建议进行热-配合参数敏感性分析。\n');
end
fprintf(fid, '\n');

fprintf(fid, '四、接触变形、应力与运动状态\n');
fprintf(fid, '  号  接触变形(m)  接触压力诊断值(Pa)  油膜厚度(um)  PV(Pa*m/s)  公转角速度(r/min)  离心力(N)\n');
for i = 1:numel(r.Q)
    fprintf(fid, ' %3d %13.6e %16.6e %13.6f %13.6e %16.6f %13.6e\n', ...
        i, r.delta(i), pressure(i), r.oil_film(i)*1e6, r.PV(i), r.cage_speed*30/pi, Fc);
end
fprintf(fid, '\n');

fprintf(fid, '五、刚度矩阵与最终等效支承参数\n');
fprintf(fid, '  局部刚度矩阵 K_local = d[Fy,Fz]/d[y,z]，单位 N/m\n');
fprintf(fid, '          y方向            z方向\n');
fprintf(fid, '  Fy  %14.6e  %14.6e\n', r.stiffness_matrix(1,1), r.stiffness_matrix(1,2));
fprintf(fid, '  Fz  %14.6e  %14.6e\n', r.stiffness_matrix(2,1), r.stiffness_matrix(2,2));
fprintf(fid, '  最终进入转子-机匣耦合模型的等效参数:\n');
fprintf(fid, '    kx = %.6e N/m, ky = %.6e N/m\n', r.kx, r.ky);
fprintf(fid, '    cx = %.6e N*s/m, cy = %.6e N*s/m\n', r.cx, r.cy);
fprintf(fid, '  注意: 程序 x方向刚度 = 轴承局部 y向径向刚度；程序 y方向刚度 = 轴承局部 z向径向刚度。\n\n');

fprintf(fid, '六、预载平衡位移与非线性耦合入口\n');
fprintf(fid, '  预载平衡闭合量 radial_closure: %.6e m\n', r.radial_closure);
fprintf(fid, '  内圈相对外圈平衡位移 x/y: %.6e / %.6e m\n', r.operating_offset_x, r.operating_offset_y);
fprintf(fid, '  在强耦合转子计算中，上述工作点作为轴承非线性接触力的基准偏置；Newmark-beta每个时间步再根据转子节点与机匣节点的瞬时相对位移和速度更新轴承力。\n\n');

fprintf(fid, '七、工程解释\n');
fprintf(fid, '  本轴承动态性能结果用于说明微观界面因素如何改变轴承内部接触和润滑状态。\n');
fprintf(fid, '  传递链为: 工作游隙/油膜/纹理/粗糙度/杂质/波纹度 -> 接触压缩量 -> 滚动体接触载荷 -> 局部接触刚度和摩擦牵引 -> 等效支承参数 -> 转子-机匣耦合振动响应。\n');
fprintf(fid, '  本报告中的油膜厚度为等效EHL载荷修正模型结果，不是完整TEHL压力-温度-弹性变形场在线求解结果。\n');
fprintf(fid, '  说明：本文输出的接触压力为基于当前等效接触载荷和简化接触面积估算得到的接触压力诊断值，主要用于不同工况之间的相对比较和趋势分析。该值不等同于完整三维 Hertz 接触理论或 TEHL 模型求解得到的严格最大接触应力。\n');
fprintf(fid, '  Note: The reported contact pressure is a diagnostic value estimated from the equivalent contact load and simplified contact area. It is mainly used for relative comparison and trend analysis under different operating conditions, and should not be interpreted as the exact maximum Hertzian contact stress obtained from a full three-dimensional contact or TEHL solution.\n');
fprintf(fid, '  保持架相对打滑指标为经验诊断量，不是完整保持架动力学方程直接积分得到的真实打滑率。\n');
fprintf(fid, '  局部接触刚度用于诊断轴承动态性能和等效支承参数，不等同于每一时间步显式组装的全局非线性切线刚度。\n');
end

function [pressure, Fc, slip_i, area] = detail_arrays(b, r)
if strcmpi(b.type, 'ball')
    area = max(pi*(0.20*b.Db)^2, 1e-12);
    rho = get_field_default(b, 'ball_density', 7850);
    m = rho*pi*b.Db^3/6;
else
    area = max(get_field_default(b,'L',0)*get_field_default(b,'Dw',0), 1e-12);
    rho = get_field_default(b, 'roller_density', 7850);
    m = rho*pi*b.Dw^2/4*b.L;
end
pressure = r.Q/area;
Fc = m*r.cage_speed^2*b.Dm/2;
if any(r.Q > 0)
    load_factor = max(r.Q, 0)/(max(r.Q) + Fc + 1);
else
    load_factor = zeros(size(r.Q));
end
slip_i = min(0.60, max(0, r.slip_ratio*(1 + 0.25*(1 - load_factor))));
end

function v = ehl_load_reference_report(b)
if isfield(b, 'Q_ref') && ~isempty(b.Q_ref)
    v = b.Q_ref;
else
    preload = max([abs(get_field_default(b, 'preload_z', 0)), abs(get_field_default(b, 'preload_y', 0)), 1]);
    v = preload/max(get_field_default(b, 'n', 1)/2, 1);
end
end

function tf = nested_bool(s, group, field)
tf = false;
if isfield(s, group) && isstruct(s.(group)) && isfield(s.(group), field) && ~isempty(s.(group).(field))
    tf = logical(s.(group).(field));
end
end

function v = nested_value(s, group, field, default_value)
if isfield(s, group) && isstruct(s.(group)) && isfield(s.(group), field) && ~isempty(s.(group).(field))
    v = s.(group).(field);
else
    v = default_value;
end
end

function s = yesno(tf)
if tf
    s = '是';
else
    s = '否';
end
end

function deg = wrap_deg(deg)
deg = mod(deg, 360);
if deg <= 1e-12
    deg = 360;
end
end

function v = get_field_default(s, field, default_value)
if isfield(s, field) && ~isempty(s.(field))
    v = s.(field);
else
    v = default_value;
end
end
