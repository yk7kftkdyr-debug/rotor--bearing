%% 滚动轴承—转子—机匣系统计算仿真程序
% 该程序主要完成Jeffcott转子圆周碰摩故障仿真
% xrub_sign:    第一步：设置初始条件
% rub_sign:     碰摩标志，若rub_sign=0，说明系统无碰摩故障；否则rub_sign=1
% loca:         不平衡质量的位置
% loc_rub:      滚动轴承位置
% Famp：        不平衡质量的大小 单位为：[g]
% wi:           转速 单位为：[rad/s]
% r：           偏心半径 单位为：[mm]
% Famp1:        离心力的大小 单位为：[kg,m]
% fai:          不平衡量的初始相位[rad]
clc
clearvars -except result K_level case_id rpm_fixed f_rot harmonics unbalance_amplification unbalance_scale external_load_N bearing_stiffness_level kb1_case kb2_case suppress_main_plots sweep_output_dir sweep_case_id
[rub_sign,loca,loc_rub,wi,Famp1,fai,data]=initial_conditions;  %初始条件
if exist('external_load_N','var')
    rpm = 9900;
    wi = rpm*2*pi/60;
    data(7) = rpm;
    data(10) = external_load_N;
    data(5) = 0;
    assignin('caller', 'rpm_used', rpm);
end
%% ======== 第二步：设置转子&机匣系统的参数值 ========
% N：           划分的轴段数
% density：     轴的密度 单位为：[kg/m~3]
% Ef：          轴的弹性模量 单位为：[Pa]
% L：           每个轴段的长度 单位为：[m]
% R：           每个轴段的外半径单位为：[m]
% RO：          每个轴段的内半径 单位为：[m]
% miu：         每个轴段的单元质量 [kg/m]
[N,density,Ef,L,R,RO,miu]=rotor_parameters;                           %转子参数    
[N_C,density_C,Ef_C,L_C,R_C,RO_C,miu_C]=case_parameters;              %示例参数
%% ======== 第三步：设置移动单元质量矩阵、转动单元质量矩阵、刚度单元质量矩阵和陀螺力矩矩阵 ========
% Mst：         移动单元质量矩阵
% Msr：         转动单元质量矩阵
% Ks：          刚度单元矩阵，代表梁弯曲程度
% Ge：          陀螺力矩单元矩阵，转子旋转时，横向弯曲会耦合出陀螺效应
[Mst,Msr,Ks,Ge]=Mst_Msr_Ks_Ge(N,density,R,RO,L,Ef,miu);%转子各单元矩阵
[Mst_C,Msr_C,Ks_C,Ge_C]=Mst_Msr_Ks_Ge(N_C,density_C,R_C,RO_C,L_C,Ef_C,miu_C);%机匣单元矩阵
%% ======== 第四步：矩阵组集 ========
%M:             总的质量矩阵
%K:             总的刚度矩阵
%G:             总的陀螺力矩矩阵
[M,G,K]=M_G_K(N,Ef,R,RO,Mst,Msr,Ge,Ks,miu,L);
[M_C,G_C,K_C]=M_G_K(N_C,Ef_C,R_C,RO_C,Mst_C,Msr_C,Ge_C,Ks_C,miu_C,L_C);
%% ======== 第五步：加入支撑刚度和阻尼 ========
if data(8)==1
    is_coupled = 0;
else
    is_coupled = 1;
end
[K,C,D,Ax,WW] = K_D(N,K,M,G,wi,is_coupled);
[K_C,C_C,D_C,Ax_C,WW_C] = K_D_case(N_C,K_C,M_C,G_C,is_coupled);
%% ======== 第六步：机匣—转子矩阵的组合 ========
if data(8) == 2
    num_rotor = length(M); num_case = length(M_C); MM = zeros(num_rotor+num_case); KK = zeros(num_rotor+num_case); CC = zeros(num_rotor+num_case);%（转子自由度+机匣自由度）×（转子自由度×机匣自由度）
    MM(1:4*(N+1),1:4*(N+1)) = M; MM(4*(N+2)-4+1:num_rotor+4*(N_C+1),4*(N+2)-4+1:num_rotor+4*(N_C+1)) = M_C; 
    KK(1:4*(N+1),1:4*(N+1)) = K; KK(4*(N+2)-4+1:num_rotor+4*(N_C+1),4*(N+2)-4+1:num_rotor+4*(N_C+1)) = K_C; 
    CC(1:4*(N+1),1:4*(N+1)) = C; CC(4*(N+2)-4+1:num_rotor+4*(N_C+1),4*(N+2)-4+1:num_rotor+4*(N_C+1)) = C_C; 
    %核心作用：分块赋值，将转子 / 机匣的局部质量矩阵嵌入全局质量矩阵MM的指定位置；
    % 索引逻辑：1:4*(N+1)定位转子矩阵，4N+5:num_rotor+4*(N_C+1)定位机匣矩阵；
    r1 = 2; r2 = 10; %轴承位置
    c1 = 3; c2 = 10;%机匣上与转子连接节点,轴承座
    % Bearing support stiffness entry for the coupled rotor-casing model.
    % kb1 and kb2 are assembled directly into KK as rotor-casing coupling
    % stiffness in both x and y directions at bearing locations r1/c1 and r2/c2.
    if exist('bearing_stiffness_level', 'var')
        kb1 = bearing_stiffness_level;
        kb2 = bearing_stiffness_level;
    else
        kb1 = 2e8;  kb2 = 2.6e8;     % 轴承耦合刚度 N/m
    end
    if exist('kb1_case', 'var')
        kb1 = kb1_case;
    end
    if exist('kb2_case', 'var')
        kb2 = kb2_case;
    end
    cb = 1.5e3;                    % 轴承耦合阻尼 N*s/m
    %% ===== 轴承1：转子节点 r1 与机匣节点 c1 耦合 =====
    irx = 4*r1 - 3;        % 转子轴承1 x自由度
    iry = 4*r1 - 2;        % 转子轴承1 y自由度
    icx = num_rotor + 4*c1 - 3;   % 机匣轴承座1 x自由度
    icy = num_rotor + 4*c1 - 2;   % 机匣轴承座1 y自由度
    % x方向刚度：+kb -kb; -kb +kb
    KK(irx,irx) = KK(irx,irx) + kb1;   % 对角线刚度，转子自身项
    KK(icx,icx) = KK(icx,icx) + kb1;   % 对角线刚度，机匣自身项
    KK(irx,icx) = KK(irx,icx) - kb1;   % 非对角耦合刚度
    KK(icx,irx) = KK(icx,irx) - kb1;   % 非对角耦合刚度
    % y方向刚度
    KK(iry,iry) = KK(iry,iry) + kb1;
    KK(icy,icy) = KK(icy,icy) + kb1;
    KK(iry,icy) = KK(iry,icy) - kb1;
    KK(icy,iry) = KK(icy,iry) - kb1;
    % x方向阻尼：+cb -cb; -cb +cb
    CC(irx,irx) = CC(irx,irx) + cb;
    CC(icx,icx) = CC(icx,icx) + cb;
    CC(irx,icx) = CC(irx,icx) - cb;
    CC(icx,irx) = CC(icx,irx) - cb;
    % y方向阻尼
    CC(iry,iry) = CC(iry,iry) + cb;
    CC(icy,icy) = CC(icy,icy) + cb;
    CC(iry,icy) = CC(iry,icy) - cb;
    CC(icy,iry) = CC(icy,iry) - cb;
    %% ===== 轴承2：转子节点 r2 与机匣节点 c2 耦合 =====
    irx = 4*r2 - 3;
    iry = 4*r2 - 2;
    icx = num_rotor + 4*c2 - 3;
    icy = num_rotor + 4*c2 - 2;
    % x方向刚度
    KK(irx,irx) = KK(irx,irx) + kb2;
    KK(icx,icx) = KK(icx,icx) + kb2;
    KK(irx,icx) = KK(irx,icx) - kb2;
    KK(icx,irx) = KK(icx,irx) - kb2;
    % y方向刚度
    KK(iry,iry) = KK(iry,iry) + kb2;
    KK(icy,icy) = KK(icy,icy) + kb2;
    KK(iry,icy) = KK(iry,icy) - kb2;
    KK(icy,iry) = KK(icy,iry) - kb2;
    % x方向阻尼
    CC(irx,irx) = CC(irx,irx) + cb;
    CC(icx,icx) = CC(icx,icx) + cb;
    CC(irx,icx) = CC(irx,icx) - cb;
    CC(icx,irx) = CC(icx,irx) - cb;
    % y方向阻尼
    CC(iry,iry) = CC(iry,iry) + cb;
    CC(icy,icy) = CC(icy,icy) + cb;
    CC(iry,icy) = CC(iry,icy) - cb;
    CC(icy,iry) = CC(icy,iry) - cb;
elseif data(8) == 1
    MM = M; CC = C; KK = K;   
end
%% ======== 第六步：用Newmark方法进行计算 ========
% Fen:          一个转周期被离散成多少个时间步
% ht:           每步的长度
ut1=[];
xt1=[];
yt1=[];
N3=length(MM);%总自由度
T = 2*pi/wi;          % 一个周期
Fen = 1024;          % 现在就明确：一个周期 10240 步
n_Fen = 20;           % 共算 10 个周期
ht = T / Fen;         % 步长
total_steps = n_Fen * Fen;
gama=0.6; beita=0.3025;
a0=1.0/(beita*ht*ht);  a1=gama/(beita*ht);           a2=1.0/(beita*ht);    a3=0.5/beita-1.0;
a4=gama/beita-1.0;     a5=ht/2.0*(gama/beita-2.0);   a6=ht*(1.0-gama);     a7=gama*ht;
% ======== 状态变量预分配（必须加） ========
yn   = zeros(N3, total_steps+1);
dyn  = zeros(N3, total_steps+1);
ddyn = zeros(N3, total_steps+1);
Fr   = zeros(N3, total_steps+1);
% ======== 轴承力历史（用于后处理） ========
F_b_hist = zeros(4, total_steps+1);
miu_hist = zeros(1, total_steps+1);
for i=1:1:N3
    F(i,1)=0;   %初始条件
end
Fr = zeros(N3,Fen*n_Fen);%非线性力先赋值
%for i=1:1:N3
    %yn(i,1)=0;%第i个自由度的初始位移
    %dyn(i,1)=0;%第i个自由度的初始速度
    %ddyn(i,1)=0;%第i个自由度的初始加速度
%end
t=0;
    load qiuffspeed.mat %摩擦系数
    miuI_cui = 3/0.7*miuI  ;  miuO_cui = 3/0.7*miuO;
   % response_monitor('init', n_Fen*Fen);
   %F_b_hist = zeros(4, total_steps+1);
   %miu_hist = zeros(1, total_steps+1);
for n=1:1:n_Fen*Fen  %%% 运行n_Fen个周期
    t=t+ht;%时间进一步推进
    for i=1:10
    newmark_newton_multi
    F_b_hist(:,n+1) = F_b_now;
    miu_hist(n+1) = miu_now;
    end

    if ~(exist('suppress_main_plots','var') && suppress_main_plots) && mod(n,100)==1%当 n = 1, 101, 201, ... 时输出，只关注每 100 个工况里的第一个
            disp(n)
    end
    % ====== 响应监测 ======
   % stop_flag = response_monitor('update', n, yn(:,n), dyn(:,n), ddyn(:,n));

   % if stop_flag
   %     break
  % end

   %判断收敛
    if max(abs(yn(:,n))) > 0.5
        break
    end
end

%response_monitor('plot', ht, n);
% ===== 提取最后五个周期 =====
n_end = min(n, size(yn,2));
idx_plot = max(1, n_end - 5*Fen + 1):n_end;
time_plot = (idx_plot - idx_plot(1)) * ht;
%% ======== 双轴承：x/y位移合理性快速判断========
clearance = data(9);   % 径向游隙（m）
fprintf('\n===== 系统x、y方向振动位移合理性判断 =====\n');
for k = 1:2
    idx_x = 4*loc_rub(k)-4+1;
    idx_y = 4*loc_rub(k)-4+2;
    ux = yn(idx_x, idx_plot);
    uy = yn(idx_y, idx_plot);
    ux_max = max(abs(ux));
    uy_max = max(abs(uy));
    ratio_x = ux_max / clearance;
    ratio_y = uy_max / clearance;
    % ===== 简洁判断逻辑 =====
    if ratio_x < 0.2 && ratio_y < 0.2
        state = '合理（远小于游隙，未接触小振动）';
    elseif ratio_x < 1 && ratio_y < 1
        state = '基本合理（接近游隙，可能接触前状态）';
    elseif ratio_x >= 1 || ratio_y >= 1
        state = '可能不合理（位移达到/超过游隙，需检查接触或参数）';
    else
        state = '需进一步分析';
    end
    % ===== 只输出关键结果 =====
    fprintf('\n轴承 %d:\n', k);
    fprintf('x峰值 = %.3f um | 比值 = %.3f\n', ux_max*1e6, ratio_x);
    fprintf('y峰值 = %.3f um | 比值 = %.3f\n', uy_max*1e6, ratio_y);
    fprintf('判断  = %s\n', state);
end
fprintf('=====================================\n\n');
if exist('suppress_main_plots','var') && suppress_main_plots
    do_main_plots = false;
else
    do_main_plots = true;
end
if do_main_plots
%% ======== 绘制轴心轨迹图 ========
n_end = min(n, size(yn,2));              % 实际计算到的最后列
idx_plot = max(1, n_end - 5*Fen + 1):n_end;%实际计算结束位置 n_end”之前的最后 5 个周期
%idx_plot = round(0.5*n_Fen*Fen):n_Fen*Fen;%idx_plot索引区间，只取后 5 个周期的数据来画图，这是很合理的，因为前面几个周期往往是过渡过程，后面更接近稳态。
%figure%第一个轴承 
%plot(x1,y1,'Color','black'); %4*loc_rub(1)-4+1该节点 x 位移，4*loc_rub(1)-4+2该节点y位移
%xlabel(['\fontname{Times new roman}(x/m)'],'FontSize',20);  ylabel(['\fontname{Times new roman}(y/m)'],'FontSize',20)
%title(['轴心轨迹'],'FontSize',20)
x1 = yn(4*loc_rub(1)-4+1, idx_plot);
y1 = yn(4*loc_rub(1)-4+2, idx_plot);
mx1 = sum(x1) / numel(x1);
my1 = sum(y1) / numel(y1);
x1 = x1 - mx1;
y1 = y1 - my1;
figure
plot(x1, y1, 'k', 'LineWidth', 1.2)
axis equal
grid on
xlabel('x / m','FontSize',20)
ylabel('y / m','FontSize',20)
title('第一个轴承轴心轨迹','FontSize',20,'FontName','Microsoft YaHei')
x2 = yn(4*loc_rub(2)-4+1, idx_plot);
y2 = yn(4*loc_rub(2)-4+2, idx_plot);
mx2 = sum(x2) / numel(x2);
my2 = sum(y2) / numel(y2); % 去掉静态偏置（关键！）
x2 = x2 - mx2;
y2 = y2 - my2;
%figure%第二个轴承
%plot(x2,y2,'Color','black');
%xlabel(['\fontname{Times new roman}(x/m)'],'FontSize',20) ; ylabel(['\fontname{Times new roman}(y/m)'],'FontSize',20)
%title(['轴心轨迹'],'FontSize',20)
figure
plot(x2, y2, 'k', 'LineWidth', 1.2)
axis equal
grid on
xlabel('x / m','FontSize',20)
ylabel('y / m','FontSize',20)
title('第二个轴承轴心轨迹','FontSize',20,'FontName','Microsoft YaHei')
axis equal
%% ======== 五个周期的速度图 ========
figure
plot(time_plot,1e3*dyn(4*loc_rub(1)-4+1,idx_plot),'Color','black');
xlabel('t/s','FontSize',20) ; ylabel('v_x /（mm/s）','FontSize',20); 
title(['速度'],'FontSize',20,'FontName','Microsoft YaHei')
figure
plot(time_plot,1e3*dyn(4*loc_rub(1)-4+2,idx_plot),'Color','black') ;
xlabel('t/s','FontSize',20) ; ylabel('v_y /（mm/s）','FontSize',20); 
title(['速度'],'FontSize',20,'FontName','Microsoft YaHei')
%% ======== 五个周期的加速度图 ========
figure
plot(time_plot,ddyn(4*loc_rub(1)-4+1,idx_plot),'Color','black');
xlabel('t/s','FontSize',20) ; ylabel('a_x /（m/s^2）','FontSize',20); 
title(['加速度'],'FontSize',20,'FontName','Microsoft YaHei')
figure
plot(time_plot,ddyn(4*loc_rub(1)-4+2,idx_plot),'Color','black') ; 
xlabel('t/s','FontSize',20) ; ylabel('a_y /（m/s^2）','FontSize',20); 
title(['加速度'],'FontSize',20,'FontName','Microsoft YaHei')
%% ======== 五个周期的位移图 ========
figure
plot(time_plot,yn(4*loc_rub(1)-4+1,idx_plot),'Color','black');
xlabel('t/s','FontSize',20) ; ylabel('x_x /m','FontSize',20); 
title(['位移'],'FontSize',20,'FontName','Microsoft YaHei')
figure
plot(time_plot,yn(4*loc_rub(1)-4+2,idx_plot),'Color','black') ; 
xlabel('t/s','FontSize',20) ; ylabel('x_y /m','FontSize',20); 
title(['位移'],'FontSize',20,'FontName','Microsoft YaHei')
xlim([0 time_plot(end)])
ylim([min(yn(4*loc_rub(1)-4+2,idx_plot))*1.1, max(yn(4*loc_rub(1)-4+2,idx_plot))*1.1])
%% ======== 五个周期的轴承力 ========
 figure
 plot(time_plot,F_b_hist(1,idx_plot),'Color','black');
 xlabel('t/s','FontSize',20) ; ylabel('F_x /N','FontSize',20); 
 title(['轴承接触力'],'FontSize',20,'FontName','Microsoft YaHei')
 figure
 plot(time_plot,F_b_hist(2,idx_plot),'Color','black');
 xlabel('t/s','FontSize',20) ; ylabel('F_y /N','FontSize',20); 
 title(['轴承接触力'],'FontSize',20,'FontName','Microsoft YaHei')
 xlim([0 time_plot(end)])
axis tight
end
%% 保存信号处理的参数
guzhang = data(5);
caiyang = 1/ht;   %采样频率
n_end = min(n, size(yn,2));                 % 实际有效列数
fanwei = max(1, n_end - 5*Fen + 1):n_end;  % 例如最后五个周期
%fanwei = 0.8*n_Fen*Fen+1:n_Fen*Fen;   %数据保存范围
tt = (fanwei)*ht;
tt = tt';%把采样点索引转成时间向量，再转置成列向量
yn1   = zeros(2*(N+1),length(fanwei));
dyn1  = zeros(2*(N+1),length(fanwei));
ddyn1 = zeros(2*(N+1),length(fanwei));
for i = 1:N+1
    yn1(2*i-1,:)    = yn(4*i-4+1,fanwei);
    yn1(2*i,:)      = yn(4*i-4+2,fanwei);
    dyn1(2*i-1,:)  = dyn(4*i-4+1,fanwei);
    dyn1(2*i,:)    = dyn(4*i-4+2,fanwei);
    ddyn1(2*i-1,:)= ddyn(4*i-4+1,fanwei);
    ddyn1(2*i,:)  = ddyn(4*i-4+2,fanwei);
end
F_b1 = F_b_hist(:,fanwei);
yn1 = yn1';dyn1 = dyn1';ddyn1 = ddyn1';F_b1 = F_b1';
% %if guzhang == 0
%     save D:\资料\研究生学习资料\程序\毕设程序\Newmar解故障轴承\计算结果\健康\考虑摩擦力(0.3).mat caiyang dyn1 ddyn1 yn1 tt F_b1
% %elseif guzhang == 1
%     save D:\资料\研究生学习资料\程序\毕设程序\Newmar解故障轴承\计算结果\外圈故障\2500N—0μm—2e8—磨损直径1.mat caiyang dyn1 ddyn1 yn1 tt
% %elseif guzhang == 2
%     save D:\资料\研究生学习资料\程序\毕设程序\Newmar解故障轴承\计算结果\内圈故障\2500N—5μm—2e8—磨损直径10.mat caiyang dyn1 ddyn1 yn1 tt
% %elseif guzhang == 3
%     save D:\资料\研究生学习资料\程序\毕设程序\Newmar解故障轴承\计算结果\滚动体故障\2500N—5μm—2e8—磨损直径10.mat caiyang dyn1 ddyn1 yn1 tt
% %elseif guzhang == 12
%     save D:\资料\研究生学习资料\程序\毕设程序\Newmar解故障轴承\计算结果\内外耦合故障\2500N—0μm—2e8—磨损直径1.mat caiyang dyn1 ddyn1 yn1 tt
% %elseif guzhang == 13
%     save D:\资料\研究生学习资料\程序\毕设程序\Newmar解故障轴承\计算结果\外圈滚动体耦合故障\3270rpm_2500N_5μm_2e8_磨损直径10_波纹度.mat caiyang dyn1 ddyn1 yn1 tt
% %elseif guzhang == 23
%     save D:\资料\研究生学习资料\程序\毕设程序\Newmar解故障轴承\计算结果\内圈滚动体耦合故障\2500N—0μm—2e8—磨损直径1.mat caiyang dyn1 ddyn1 yn1 tt
% %end
uy = yn(4*loc_rub(1)-4+2, idx_plot);
fprintf('max uy = %.6e m\n', max(uy));
fprintf('min uy = %.6e m\n', min(uy));
fprintf('max abs uy = %.6e m\n', max(abs(uy)));



