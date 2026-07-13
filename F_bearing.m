function [Fb_global, bearingState] = F_bearing(q, qd, params, N_dof)
%F_BEARING 轴承非线性接触力统一入口。
% 输入：
%   q, qd  - 当前全局位移、速度
%   params - 工况与轴承参数
%   N_dof  - 总自由度数
% 输出：
%   Fb_global    - 装配到全局坐标的轴承力
%   bearingState - 每个轴承的接触、油膜、PV、滑移等状态

[Fb_global, bearingState] = nonlinear_bearing_force(q, qd, params, params.bearing, N_dof);
end

