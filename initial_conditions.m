%% 该程序主要进行仿真条件设置
function [rub_sign,loca,loc_rub,wi,Famp1,fai,data]=initial_conditions%需要设置的初始条件有：
% rub_sign：碰摩标志，若rub_sign=0，说明系统无碰摩故障；否则rub_sign=1
% loca: 不平衡质量的位置
% 1oc_rub： 滚动轴承位置
% Famp: 不平衡质量的大小单位为：[g]
% wi： 转速 单位为：[rad]
% r： 偏心半径 单位为：[mm]
% Famp1: 离心力的大小 单位为：[kg，m]
% fai： 不平衡量的初始相位[rad]
rub_sign=0;
loca = [4, 6, 8, 14];  %不平衡量所在位置
loc_rub(1)=2;%轴承位置
loc_rub(2)=10;
%% ===== 不平衡量参数 =====
Mass = [126.3 159.4 149.7 145.9];   % 四个轮盘质量 kg
G = 2.5;            % ISO 平衡等级
rpm = 9900;         % 转速 rpm
wi = rpm*2*pi/60;   % 角速度 rad/s
e_um = 9549*G/rpm;      % ISO公式 %% ===== 允许偏心距（μm）=====
e_um = 0.05 * e_um;%% ===== 取允许值的5%作为仿真值 =====
U = Mass * e_um;%% ===== 残余不平衡量（g·mm）=====
U = U * 1e-6;%% ===== 转换为 kg·m =====
U_base = U(:).';%% ===== 保存四个轮盘基准不平衡量 kg·m =====
if evalin('caller', 'exist(''unbalance_scale'', ''var'')')
    unbalance_scale = evalin('caller', 'unbalance_scale');
else
    unbalance_scale = 1.0;
end
U = unbalance_scale * U_base;
assert(numel(U)==4, 'U 必须包含四个轮盘的不平衡量');
assert(numel(loca)==4, 'loca 必须包含四个轮盘所在节点');
assert(numel(U)==numel(loca), 'U 和 loca 的长度必须一致');
Famp1 = U * wi^2;%% ===== 不平衡力（N）=====
assignin('caller', 'U_base', U_base);
assignin('caller', 'U_current', U);
assignin('caller', 'unbalance_scale_used', unbalance_scale);
assignin('caller', 'rpm_used', rpm);
%不平衡量  
% Famp1=Famp(1)/1000*wi^2*r/1000;
%Famp1(1)=5e-4*wi^2;
%Famp1(2)=5e-4*wi^2;
% 2. 计算产生的不平衡力 (Unbalance Force, F)
% 公式：F = U * wi^2 = M * (Grade / wi) * wi^2 = M * Grade * wi
% 必须使用标准单位 (kg, m/s, rad/s)，Grade要除以1000变为m/s
% 将不平衡力赋给你的力矢
fai=[0 0 0 0]/180*pi;%初始相位
%% %%%%%%  基本参数  %%%%%%
data = zeros(20,1);
%1：滚动体个数；       2：滚动体直径；           3：外圈直径；        4：内圈直径；         5：是否故障     
data(1) = 22;          data(2) = 23.812;        data(3) = 220;       data(4) = 160;         data(5) = 0; 
%6：轴承接触刚度       7：转子转速；             8：1是转子；2是机匣; 9：初始间隙           10:径向力
data(6) = 2.443e9;    data(7) = wi*60/(2*pi);  data(8) = 2;         data(9) = 5*10^(-6);  data(10) = 0;
%11：接触角            12：内圈曲率系数          13外圈曲率系数：     14：粘度              15：粘压系数
data(11) = 14*pi/180;   data(12) = 0.515;        data(13) = 0.515;   data(14) = 0.0318;    data(15) = 1.28e-008;
%16：外套圈弹性模量    17：滚动体弹模            18：套圈泊松比       19：滚动体泊松比      16：摩擦力
data(16) = 2.145e+11;   data(17) = 2.145e+11;   data(18) = 0.2808;   data(19) = 0.2808;   data(20) =0;


