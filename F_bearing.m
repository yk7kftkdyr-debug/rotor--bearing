%% 轴承参数
Z = data(1);                      %球个数
d = data(2);                      %球的直径
R = data(3); r = data(4);         %外圈、内圈直径（内外沟槽曲率半径）
DP = (R+r)/2;                     %球间距直径（节圆直径）
omega_rpm = data(7);              %转子转速
omega = omega_rpm*2*pi/60;
jiechujiao = data(11);            %滚动轴承接触角
Wc= omega*((DP-d*cos(jiechujiao))/(2*DP));              %轴承滚珠的通过频率（保持架转速）
Wb = DP/(2*d)*(1-(((d*cos(jiechujiao))/DP)^2))*omega;%滚动体自转角速度
k = 1;                            %第k个球故障
sita = @(t,loc)(2*pi*(loc-1)/Z+Wc*t);%球方位角函数
sita_k = Wb*t+2*pi/Z*(k-1);       %损伤滚动体角位置
sita_in = omega*t;                %内圈角位移
guzhang = data(5);                %是否故障（1：故障、2：不故障）
C_b = data(6);                    %轴承的接触刚度
f1 = data(12);  f2 = data(13);    %外圈内圈曲率系数
niandu0 = data(14);               %润滑油粘度
nianya0 = data(15);               %粘压系数
ee1 = data(16);   ee2 = data(16);   %外内圈弹模
o1 = data(18);   o2 = data(18);   %外内圈泊松比
ee3 =  data(17)  ;                %滚动体弹模                          
o3 = data(19);                    %滚动体泊松比
beitajiao=atan(sin(jiechujiao)/(cos(jiechujiao)+d/DP));
%%
F_r = [data(10);0;data(10);0];
miu_0 = data(9);                  %初始间隙
L1 = 1;                           %外圈故障点损伤直径直径（mm）
L2 = 5;                           %内圈故障点损伤直径直径（mm）
L3 = 30;                           %滚动体故障点损伤直径直径（mm）
beita1 = asin(L1/(R));            %外圈故障角
beita2 = asin(L2/(2*r));          %内圈故障角
beita3 = asin(L3/d);              %滚动体故障角
for loc=1:Z
    e1(n,loc) = abs(mod(sita(t,loc),2*pi));%外圈故障
    jiaodu = sita(t,loc)-sita_in;
    e2 = abs(mod(jiaodu,2*pi));     %内圈故障
    e3_in = abs(mod(sita_k-1/2*pi,2*pi));
    e3_out = abs(mod(sita_k-3/2*pi,2*pi));
    %% 判断故障形式
    if guzhang == 1              %1:外圈，2：内圈，3：滚动体，12：内外圈耦合，13：外圈滚动体耦合，23：内圈滚动体耦合
        if e1(n,loc) <beita1
            nd=3;    miu=miu_0+((d/2-sqrt((d/2)^2-((L1)/2)^2))*10^(-3));
        else
            nd=1.5;  miu=miu_0;
        end
    elseif guzhang == 2  
        if e2<beita2
            nd=3;    miu=miu_0+((d/2-sqrt((d/2)^2-((L2)/2)^2))*10^(-3));%(没写好)
        else
            nd=1.5;  miu=miu_0;
        end
    elseif guzhang == 3  
        if e3_in<beita3 || e3_out<beita3
            nd=3;    miu=miu_0+((d/2-sqrt((d/2)^2-((L3)/2)^2))*10^(-3));
        else
            nd=1.5;  miu=miu_0;
        end
    elseif guzhang == 12  
        if e1<beita1
            nd=3;    miu=miu_0+((d/2-sqrt((d/2)^2-((L1)/2)^2))*10^(-3));
        elseif e2<beita2
            nd=3;    miu=miu_0+((d/2-sqrt((d/2)^2-((L2)/2)^2))*10^(-3));
        else
            nd=1.5;  miu=miu_0;
        end
    elseif guzhang == 13  
        if e1<beita1
            nd=3;    miu=miu_0+((d/2-sqrt((d/2)^2-((L1)/2)^2))*10^(-3));
        elseif e3_in<beita3 || e3_out<beita3
            nd=3;    miu=miu_0+((d/2-sqrt((d/2)^2-((L3)/2)^2))*10^(-3));
        else
            nd=1.5;  miu=miu_0;
        end
    elseif guzhang == 23  
        if e2<beita2
            nd=3;    miu=miu_0+((d/2-sqrt((d/2)^2-((L2)/2)^2))*10^(-3));
        elseif e3_in<beita3 || e3_out<beita3
            nd=3;    miu=miu_0+((d/2-sqrt((d/2)^2-((L3)/2)^2))*10^(-3));
        else
            nd=1.5;  miu=miu_0;
        end
    else
        nd=1.5;  miu=miu_0;
    end
    %% 计算油膜厚度
%     Rx1=d*(0.5*DP/cos(jiechujiao)+0.5*d)/(DP/cos(jiechujiao) );   %Dw：滚动体直径 Dm：节圆直径  球和外圈在接触椭圆的有效半径
%     Ry1=f1*d/(2*f1-1);                             %f1 外圈曲率半径     0.5232
%     Rx2=d*(0.5*DP/cos(jiechujiao)-0.5*d)/(DP/cos(jiechujiao) ); 
%     Ry2=f2*d/(2*f2-1);
%     E1=2/(((1-o1^2)/ee1)+((1-o3^2)/ee3));             %球外圈的有效弹模  e1，e3为外圈和球的弹性模量，o1，o3为外圈，球的泊松比
%     E2=2/(((1-o2^2)/ee2)+((1-o3^2)/ee3));             %e1，e3为内圈和球的弹性模量，o1，o3为内圈，球的泊松比
%     K1=1.0339*(Ry1/Rx1)^0.636;                      %球和外圈接触椭圆的椭圆率
%     K2=1.0339*(Ry2/Rx2)^0.636;
%     Wo=omega*(1-d/DP*cos(jiechujiao))*(cos(jiechujiao)+tan(beitajiao)*sin(jiechujiao))...
%         /(( 1-d/DP*cos(jiechujiao))*(cos(jiechujiao)+tan(beitajiao)*sin(jiechujiao))...
%         +(1+d/DP*cos(jiechujiao))*(cos(jiechujiao)+tan(beitajiao)*sin(jiechujiao)));
%     Wx=-omega*(1-d/DP*cos(jiechujiao))*(1+d/DP*cos(jiechujiao))/((1-d/DP*cos(jiechujiao))...
%         *(cos(jiechujiao)+tan(beitajiao)*sin(jiechujiao))+(1+d/DP*cos(jiechujiao))...
%         *(cos(jiechujiao)+tan(beitajiao)*sin(jiechujiao)) )/(d/DP);
%     %W2为转子转速
%     U1(loc)=abs(-Wo*DP/4-0.5*(abs(Wx)*cos(jiechujiao)+ Wo*cos(jiechujiao))*(d/2)); %Wo：公转角速度；Wx：绕轴向自转速度；
%     U2(loc)=abs(-Wo*DP/4-0.5*(abs(Wx)*cos(jiechujiao)+ Wo*cos(jiechujiao))*(d/2));
%     if n == 1
%         oilh1(1)=0.114*10^(-6);oilh1(2)=0.114*10^(-6);oilh1(3)=0.114*10^(-6);oilh1(4)=0.114*10^(-6);oilh1(5)=0.115*10^(-6);
%         oilh1(6)=0.116*10^(-6);oilh1(7)=0.117*10^(-6);oilh1(8)=0.118*10^(-6);oilh1(9)=0.118*10^(-6);oilh1(10)=0.119*10^(-6);
%         oilh1(11)=0.119*10^(-6);oilh1(12)=0.119*10^(-6);oilh1(13)=0.118*10^(-6);oilh1(14)=0.118*10^(-6);oilh1(15)=0.117*10^(-6);
%                 
%         oilh2(1)=0.113*10^(-6);oilh2(2)=0.114*10^(-6);oilh2(3)=0.113*10^(-6);oilh2(4)=0.114*10^(-6);oilh2(5)=0.115*10^(-6);
%         oilh2(6)=0.116*10^(-6);oilh2(7)=0.117*10^(-6);oilh2(8)=0.118*10^(-6);oilh2(9)=0.118*10^(-6);oilh2(10)=0.119*10^(-6);
%         oilh2(11)=0.119*10^(-6);oilh2(12)=0.119*10^(-6);oilh2(13)=0.118*10^(-6);oilh2(14)=0.118*10^(-6);oilh2(15)=0.117*10^(-6);
%     else 
% %         oilh1(loc)=Rx1*3.63*(niandu0*U1(loc)/(E1*Rx1))^(0.68)*...
% %                  (nianya0*E1)^(0.49)*(Q_b(loc)/Rx1^2/E1)^(-0.073)*(1-exp(-0.68*K1));
% %         oilh2(loc)=Rx2*3.63*(niandu0*U2(loc)/(E2*Rx2))^(0.68)*...
% %                  (nianya0*E2)^(0.49)*(Q_b(loc)/Rx2^2/E2)^(-0.073)*(1-exp(-0.68*K2));
%         oilh1(1)=0.114*10^(-6);oilh1(2)=0.114*10^(-6);oilh1(3)=0.114*10^(-6);oilh1(4)=0.114*10^(-6);oilh1(5)=0.115*10^(-6);
%         oilh1(6)=0.116*10^(-6);oilh1(7)=0.117*10^(-6);oilh1(8)=0.118*10^(-6);oilh1(9)=0.118*10^(-6);oilh1(10)=0.119*10^(-6);
%         oilh1(11)=0.119*10^(-6);oilh1(12)=0.119*10^(-6);oilh1(13)=0.118*10^(-6);oilh1(14)=0.118*10^(-6);oilh1(15)=0.117*10^(-6);
%                 
%         oilh2(1)=0.113*10^(-6);oilh2(2)=0.114*10^(-6);oilh2(3)=0.113*10^(-6);oilh2(4)=0.114*10^(-6);oilh2(5)=0.115*10^(-6);
%         oilh2(6)=0.116*10^(-6);oilh2(7)=0.117*10^(-6);oilh2(8)=0.118*10^(-6);oilh2(9)=0.118*10^(-6);oilh2(10)=0.119*10^(-6);
%         oilh2(11)=0.119*10^(-6);oilh2(12)=0.119*10^(-6);oilh2(13)=0.118*10^(-6);oilh2(14)=0.118*10^(-6);oilh2(15)=0.117*10^(-6);
%     end
%     oilh(n,loc) = oilh1(loc)+oilh2(loc);
    %% 计算轴承接触力
   %% ===== 计算轴承相对位移 =====
% r1、r2：转子轴承节点
% c1、c2：机匣轴承座节点
% 注意：这里必须和 main 中的 r1、r2、c1、c2 完全一致
if data(8) == 2
    % 轴承1：转子 r1 相对机匣 c1
    X(1) = x_it(4*r1-3) - x_it(num_rotor + 4*c1-3);
    Y(1) = x_it(4*r1-2) - x_it(num_rotor + 4*c1-2);
    % 轴承2：转子 r2 相对机匣 c2
    X(2) = x_it(4*r2-3) - x_it(num_rotor + 4*c2-3);
    Y(2) = x_it(4*r2-2) - x_it(num_rotor + 4*c2-2);
elseif data(8) == 1
    % 固定支承模型：机匣/基础位移为0
    X(1) = x_it(4*r1-3);
    Y(1) = x_it(4*r1-2);
    X(2) = x_it(4*r2-3);
    Y(2) = x_it(4*r2-2);
end
    %P2=5e-6*cos(30*sita(t,loc)+pi/2);  %波纹度
    P2 = 0;
    %ud=(1.11345-1.05186*0.98276^(data(10)))*1e-6;              %杂质产生的特征位移
    ud=0;    
    wenli = 6;                                                 %表面纹理参数，横向条纹表面γ<1;纵向条纹表面γ>1，各向同性表面γ=1。
    Cr = 1.28*exp(-0.057*6-0.0763*wenli);                      %表面纹理对初始油膜厚度的影响
    %Cr = 1;
    %% ===== 工作游隙修正：由预载计算有效游隙 =====
miu0 = miu;     % 初始径向游隙
F_pre = [2000 20000];   % 两个轴承预载，单位 N
dmu_pre = zeros(1,2);
miu_eff = zeros(1,2);
for ib = 1:2
    dmu_pre(ib) = (F_pre(ib)/C_b)^(1/nd);
    miu_eff(ib) = miu0 - dmu_pre(ib);
    % 避免出现非物理负游隙
    miu_eff(ib) = max(miu_eff(ib), 0);
end
delta(1) = X(1).*cos(sita(t,loc))+ Y(1).*sin(sita(t,loc))- miu_eff(1) + ud - P2;
delta(2) = X(2).*cos(sita(t,loc))+ Y(2).*sin(sita(t,loc))- miu_eff(2) + ud - P2;
   % miu = Cr*miu;
   % delta(1) = X(1).*cos(sita(t,loc))+Y(1).*sin(sita(t,loc))-miu+ud-P2;
   % delta(2) = X(2).*cos(sita(t,loc))+Y(2).*sin(sita(t,loc))-miu+ud-P2;
    %球轴承接触力
    Q_b(loc) = C_b*(delta(1)).^(nd).*heaviside(delta(1));%赫兹接触力，F=kδ3/2次幂
    Q_r(loc) = C_b*(delta(2)).^(nd).*heaviside(delta(2));
    F_xbRi(loc) = C_b*(delta(1)).^(nd).*heaviside(delta(1)).*cos(sita(t,loc));
    F_ybRi(loc) = C_b*(delta(1)).^(nd).*heaviside(delta(1)).*sin(sita(t,loc));
    %滚子轴承接触力
    F_xrRi(loc) = C_b*(delta(2)).^(nd).*heaviside(delta(2)).*cos(sita(t,loc));
    F_yrRi(loc) = C_b*(delta(2)).^(nd).*heaviside(delta(2)).*sin(sita(t,loc));
end
F_xbR = sum(F_xbRi(:)); %所有滚珠力叠加
F_ybR = sum(F_ybRi(:));
F_xrR = sum(F_xrRi(:)); 
F_yrR = sum(F_yrRi(:));

%F_b(:,n) = [F_xbR;F_ybR;F_xrR;F_yrR];
%F_xt(:,n) = F_r-F_b(:,n);
%mosun(n) = miu;
F_b_now = [F_xbR;F_ybR;F_xrR;F_yrR];%由变形产生的反接触力，轴承反抗变形
F_xt = F_r - F_b_now;%F_r外载，实际作用在转子上的净轴承力
miu_now = miu;
if loc == 1
    fprintf('miu_eff1 = %.3e, miu_eff2 = %.3e\n',miu_eff(1),miu_eff(2));
end