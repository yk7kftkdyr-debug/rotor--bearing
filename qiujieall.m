function [loadj,returndata]=qiujieall(datafromvb)            % 总的求解函数，求解时只需输入qiujieall即可！
if nargin < 1 || isempty(datafromvb)
    datafromvb = defaultBearingInput();
end
% 数据输入部分！！
n=datafromvb(1);Dw=datafromvb(2);Dm=datafromvb(3);lenroller=datafromvb(4);lenrollerline=datafromvb(5);arcroller=datafromvb(6);
deltar0=datafromvb(7); Rp=datafromvb(8); %兜孔半径
tuoyuandu=datafromvb(9); %外圈的椭圆度
ballden=datafromvb(59);e1=datafromvb(11);e2=datafromvb(12);e3=datafromvb(13);o1=datafromvb(14);o2=datafromvb(15);o3=datafromvb(16); 
W1=datafromvb(17)*pi/30;W2=datafromvb(18)*pi/30; Fyy=datafromvb(19); Fzz=datafromvb(20)+0.1; Myy=datafromvb(21)+0.1; Mzz=datafromvb(22)+0.1; 
cucao1=datafromvb(23);cucao2=datafromvb(24);cucao3=datafromvb(25); 
%润滑油基本参数
oilden=datafromvb(26); sita0=datafromvb(27);K=datafromvb(28); %常温下的导热系数
niandu0=datafromvb(29); nianya0=datafromvb(30); beita0=datafromvb(31);   %粘温系数（近似认为不变，wys论文中）   所有参数均为常温下的参数！！！
yindao=datafromvb(32); yindaojianxi=datafromvb(33);   % 引导方式：引导间隙
%装配的相关参数
Di=datafromvb(35);Do=datafromvb(34);Ds=datafromvb(36);Dh=datafromvb(37);es=datafromvb(38);eh=datafromvb(39);os=datafromvb(40);oh=datafromvb(41);u1=datafromvb(42);u2=datafromvb(43);
taos=datafromvb(52);taoh=datafromvb(53);tao1=datafromvb(54);tao2=datafromvb(55);taor=datafromvb(56);Tr=datafromvb(44);To=datafromvb(45);Ti=datafromvb(46);Ta=datafromvb(47);Ts=datafromvb(57);Th=datafromvb(58);
ruo1=datafromvb(48);ruo2=datafromvb(49);ruos=datafromvb(50);ruoh=datafromvb(51);%ruor=datafromvb(59);
Dr1=Dm+Dw+deltar0;Dr2=Dm-Dw-deltar0;  % 滚道直径
gama=Dw/Dm;
leixing=datafromvb(74);
% ===== 保存原始输入参数 =====
data_input = datafromvb;
% ===== 计算一次工作游隙 =====
[deltaw, clearInfo] = calcWorkingClearance(data_input);
% ===== 用工作游隙参与载荷求解 =====
data_work = data_input;
data_work(7) = deltaw;
% ================================================================
if Mzz>0.1            % 考虑滚子歪斜的计算！
loadState = qiujieLOAD3(data_work);
loadj = loadState.loadj;
loadi = loadState.loadi;
www12 = loadState.www12;
Q2_load_threshold = 1e-6;
weakIndex = find(loadState.contact.Q2 <= Q2_load_threshold);
if ~isempty(weakIndex)
    oldLoadj = loadj;
    keepIndex = setdiff(1:oldLoadj, weakIndex);
    fprintf('\n[刚度/速度求解前处理] 以下滚子 Q2≈0，转为非承载滚子：\n');
    disp(loadi(weakIndex));
    loadi = loadi(keepIndex);
    loadj = length(loadi);
    www12 = [
        www12(keepIndex);
        www12(oldLoadj + keepIndex);
        www12(2*oldLoadj + keepIndex);
        www12(3*oldLoadj+1 : 3*oldLoadj+4)
    ];
    contact = ff3(data_work, loadi, www12);
    loadState.loadi = loadi;
    loadState.loadj = loadj;
    loadState.www12 = www12;
    loadState.www1 = www12;
    loadState.contact = contact;
end
% 刚度计算！！   计算4阶刚度！！
kk=[]; 
if www12(3*loadj+2)<1e-8
   www12(3*loadj+2)=1e-8;
else
end
baseContact = ff3(data_work,loadi,www12); zz11=baseContact.zz1;
neiquan1=www12; neiquan1(3*loadj+2)=www12(3*loadj+2)+1e-10;
pertContact = ff3(data_work,loadi,neiquan1); zz1=pertContact.zz1;
for j=1:4
      kk(1,j)=(zz1(3*loadj+j)-zz11(3*loadj+j))/1e-10;
end
neiquan1=www12; neiquan1(3*loadj+1)=www12(3*loadj+1)+1e-10;
pertContact = ff3(data_work,loadi,neiquan1); zz1=pertContact.zz1;
for j=1:4
      kk(2,j)=(zz1(3*loadj+j)-zz11(3*loadj+j))/1e-10;
end
neiquan1=www12; neiquan1(3*loadj+3)=www12(3*loadj+3)+1e-10;
pertContact = ff3(data_work,loadi,neiquan1); zz1=pertContact.zz1;
for j=1:4
      kk(3,j)=(zz1(3*loadj+j)-zz11(3*loadj+j))/1e-10;
end
neiquan1=www12; neiquan1(3*loadj+4)=www12(3*loadj+4)+1e-10;
pertContact = ff3(data_work,loadi,neiquan1); zz1=pertContact.zz1;
for j=1:4
      kk(4,j)=(zz1(3*loadj+j)-zz11(3*loadj+j))/1e-10;
end
disp('刚度矩阵为：')
kk=-kk'
deltar_load = data_work(7);
Z2=www12(3*loadj+1);Y2=www12(3*loadj+2);sitay=www12(3*loadj+3);sitaz=www12(3*loadj+4);
for i=1:loadj
    sita(i)=2*pi*(loadi(i))/n;
    beita(i)=www12(loadj+i);
    afa(i)=www12(2*loadj+i);
    delta1(i)=www12(i);
    delta2(i)=Y2*sin(sita(i))+Z2*cos(sita(i))-deltar_load/2-delta1(i);
end
contact = ff3(data_work,loadi,www12);
Q1=contact.Q1;Q2=contact.Q2;QQ1=contact.QQ1;QQ2=contact.QQ2;
lengthcontact1=contact.lengthcontact1;widthcontact1=contact.widthcontact1;Ph1=contact.Ph1;
lengthcontact2=contact.lengthcontact2;widthcontact2=contact.widthcontact2;Ph2=contact.Ph2;
Fyindao=contact.Fyindao;fyindao=contact.fyindao;zuoyongjiao=contact.zuoyongjiao;
oilh1=contact.oilh1;oilh2=contact.oilh2;delta1k=contact.delta1k;delta2k=contact.delta2k;
www1=www12;
loadState.contact = contact;
loadState.www1 = www1;
else                  % 不 考虑滚子歪斜的计算！
loadState = qiujieLOAD(data_work);
loadj = loadState.loadj;
loadi = loadState.loadi;
www1 = loadState.www1;
Q2_load_threshold = 1e-6;
weakIndex = find(loadState.contact.Q2 <= Q2_load_threshold);
if ~isempty(weakIndex)
    oldLoadj = loadj;
    keepIndex = setdiff(1:oldLoadj, weakIndex);
    fprintf('\n[刚度/速度求解前处理] 以下滚子 Q2≈0，转为非承载滚子：\n');
    disp(loadi(weakIndex));
    loadi = loadi(keepIndex);
    loadj = length(loadi);
    www1 = [
        www1(keepIndex);
        www1(oldLoadj + keepIndex);
        www1(2*oldLoadj+1 : 2*oldLoadj+4)
    ];
    contact = ff2(data_work, loadi, www1);
    loadState.loadi = loadi;
    loadState.loadj = loadj;
    loadState.www1 = www1;
    loadState.contact = contact;
end
% 刚度计算！！   计算4阶刚度！！
kk=[]; www1(2*loadj+4)=1e-6;  
if www1(2*loadj+2)<1e-8
    www1(2*loadj+2)=1e-8;
else
end
baseContact = ff2(data_work,loadi,www1); zz11=baseContact.zz2;
neiquan1=www1; neiquan1(2*loadj+2)=www1(2*loadj+2)+1e-10;
pertContact = ff2(data_work,loadi,neiquan1); zz2=pertContact.zz2; 
for j=1:4
      kk(1,j)=(zz2(2*loadj+j)-zz11(2*loadj+j))/1e-10;
end
neiquan1=www1; neiquan1(2*loadj+1)=www1(2*loadj+1)+1e-10;
pertContact = ff2(data_work,loadi,neiquan1); zz2=pertContact.zz2;  
for j=1:4
      kk(2,j)=(zz2(2*loadj+j)-zz11(2*loadj+j))/1e-10;
end
neiquan1=www1; neiquan1(2*loadj+3)=www1(2*loadj+3)+1e-10;
pertContact = ff2(data_work,loadi,neiquan1); zz2=pertContact.zz2;  
for j=1:4
      kk(3,j)=(zz2(2*loadj+j)-zz11(2*loadj+j))/1e-10;
end
neiquan1=www1; neiquan1(2*loadj+4)=www1(2*loadj+4)+1e-10;
pertContact = ff2(data_work,loadi,neiquan1); zz2=pertContact.zz2;  
for j=1:4
      kk(4,j)=(zz2(2*loadj+j)-zz11(2*loadj+j))/1e-10;
end
disp('刚度矩阵为：')
kk=-kk';
Z2=www1(2*loadj+1);Y2=www1(2*loadj+2);sitay=www1(2*loadj+3);sitaz=www1(2*loadj+4);
for i=1:loadj
    sita(i)=2*pi*(loadi(i))/n;
    beita(i)=www1(loadj+i);
    afa(i)=0;
    delta1(i)=www1(i);
   deltar_load = data_work(7);
   delta2(i)=Y2*sin(sita(i))+Z2*cos(sita(i))-deltar_load/2-delta1(i);
end
contact = ff2(data_work,loadi,www1);
Q1=contact.Q1;Q2=contact.Q2;QQ1=contact.QQ1;QQ2=contact.QQ2;
lengthcontact1=contact.lengthcontact1;widthcontact1=contact.widthcontact1;Ph1=contact.Ph1;
lengthcontact2=contact.lengthcontact2;widthcontact2=contact.widthcontact2;Ph2=contact.Ph2;
Fyindao=contact.Fyindao;fyindao=contact.fyindao;zuoyongjiao=contact.zuoyongjiao;
oilh1=contact.oilh1;oilh2=contact.oilh2;delta1k=contact.delta1k;delta2k=contact.delta2k;
loadState.contact = contact;
loadState.www1 = www1;
end
% 载荷调试时可以不在 qiujieLOAD 中途删滚子；
% 但进入速度求解前，Q2<=阈值的滚子必须从承载区剔除。
Q2_speed_threshold = 1e-6;
weakIndex = find(Q2 <= Q2_speed_threshold);

if ~isempty(weakIndex)
    oldLoadj = loadj;
    keepIndex = setdiff(1:oldLoadj, weakIndex);

    fprintf('\n[速度求解前处理] 以下滚子 Q2≈0，转为非承载滚子：\n');
    disp(loadi(weakIndex));

    loadi = loadi(keepIndex);
    loadj = length(loadi);

    if Mzz > 0.1
        www12 = [
            www12(keepIndex);
            www12(oldLoadj + keepIndex);
            www12(2*oldLoadj + keepIndex);
            www12(3*oldLoadj+1 : 3*oldLoadj+4)
        ];
        contact = ff3(data_work, loadi, www12);
        www1 = www12;
    else
        www1 = [
            www1(keepIndex);
            www1(oldLoadj + keepIndex);
            www1(2*oldLoadj+1 : 2*oldLoadj+4)
        ];
        contact = ff2(data_work, loadi, www1);
    end

    Q1 = contact.Q1;
    Q2 = contact.Q2;
    QQ1 = contact.QQ1;
    QQ2 = contact.QQ2;
    Ph1 = contact.Ph1;
    Ph2 = contact.Ph2;
    lengthcontact1 = contact.lengthcontact1;
    lengthcontact2 = contact.lengthcontact2;
    widthcontact1 = contact.widthcontact1;
    widthcontact2 = contact.widthcontact2;
    oilh1 = contact.oilh1;
    oilh2 = contact.oilh2;
    delta1k = contact.delta1k;
    delta2k = contact.delta2k;

    loadState.loadi = loadi;
    loadState.loadj = loadj;
    loadState.www1 = www1;
    loadState.contact = contact;
end
speedState = qiujieSPEED(data_work, loadState);
wwmin2 = speedState.wwmin2;
if length(wwmin2(:,1))==1
   wwmin2=wwmin2';
end
www2=wwmin2;
loadii = speedState.loadii;
Wo = www2(1:loadj)';
Wx = www2(loadj+1:2*loadj)';
Wononload = www2(2*loadj+1);
Wxnonload = www2(2*loadj+2);
Wctheory=(W2*(1-gama)-W1*(1+gama))/2;    zhuansuxishu=1;
if Wctheory<max(Wo)
    zhuansuxishu=Wctheory/max(Wo);
end
Wo=Wo*zhuansuxishu;     Wononload=Wononload*zhuansuxishu;
Wc=(sum(Wo)+Wononload*(n-loadj))/n;             % 保持架转速取所有滚子转速的平均值！！
disp('打滑率百分比为：')
dahualv=(Wctheory-Wc)/Wctheory 

marksorti=1;               % 再恢复到原来的顺序！ 即 序号的升序排列！！！
for i=1:loadj
    if loadii(i)==n
        marksorti=i+1;
    end
end
oilh1=oilh1'; oilh2=oilh2';
oilh11=[oilh1(marksorti:loadj);   oilh1(1:marksorti-1);   ];
oilh22=[oilh2(marksorti:loadj);   oilh2(1:marksorti-1);   ];
allWo=[www2(marksorti:loadj)*zhuansuxishu;   ones(n-loadj,1)*www2(2*loadj+1)*zhuansuxishu;  www2(1:marksorti-1)*zhuansuxishu;   ];   %所有公转转速
allWx=[www2(loadj+marksorti:2*loadj); ones(n-loadj,1)*www2(2*loadj+2);  www2(loadj+1:loadj+marksorti-1);];   %所有自转转速
allsitajiao=360/n:360/n:360;
%deltaU1=abs(Dm/2*((1+gama)*(W1+allWo)-gama*allWx));
%deltaU2=abs(Dm/2*((1-gama)*(W2-allWo)-gama*allWx));
speedFinal = ffSPEED(data_work, wwmin2, speedState.speedInput);
T1 = speedFinal.T1;
T2 = speedFinal.T2;
T1nonload = speedFinal.T1nonload;
Q1nonload = speedFinal.Q1nonload;
pvzhi1 = speedFinal.pvzhi1;
pvzhi2 = speedFinal.pvzhi2;
pvzhinonload = speedFinal.pvzhinonload;
oilh1 = speedFinal.oilh1;
oilh2 = speedFinal.oilh2;
widthcontactnonload = speedFinal.widthcontactnonload;
deltaU1 = speedFinal.deltaU1;
deltaU2 = speedFinal.deltaU2;
Wo = speedFinal.Wo;
Wx = speedFinal.Wx;
Fm = speedFinal.Fm;  % 滚子/保持架作用力
Fmnonload = speedFinal.Fmnonload;
deltaU1nonload = speedFinal.deltaU1nonload;
Wononload = speedFinal.Wononload;
Wxnonload = speedFinal.Wxnonload;
Ph1nonload = speedFinal.Ph1nonload;
Wc = (sum(Wo)+Wononload*(n-loadj))/n;
dahualv = max(0,(Wctheory-Wc)/Wctheory);
m=ballden*Dw*Dw*lenroller*pi/4;  %滚子的质量计算
for i=1:loadj
    sita(i)=2*pi*(loadi(i))/n;
end

for i=1:n
    for j=1:loadj
      if i==loadi(j)
          mark1=i;break
      else 
          mark1=0;
      end
    end
    if mark1==i
        allQ1(i)=Q1(j);
        allQ2(i)=Q2(j);
        allpvzhi1(i)=pvzhi1(j);
        allpvzhi2(i)=pvzhi2(j);
    else
        allQ1(i)=m*allWo(i)^2*Dm/2;
        allQ2(i)=0;
        allpvzhi1(i)=pvzhinonload;
        allpvzhi2(i)=0;
    end
end

lifeResult = life_roller(datafromvb,QQ2*lenroller/150,QQ1*lenroller/150,leixing,allWx);
Lxia = lifeResult.Lxia;
L = lifeResult.L;

%考虑配合以及热、离心力后的径向游隙
if Ds==0
    deltai=2*u1*(Dr2/Di)/(((Dr2/Di)^2-1)*((((Dr2/Di)^2+1)/((Dr2/Di)^2-1)+o2)+e2/es*(1-os)));
else
    deltai=2*u1*(Dr2/Di)/(((Dr2/Di)^2-1)*((((Dr2/Di)^2+1)/((Dr2/Di)^2-1)+o2)+e2/es*(((Di/Ds)^2+1)/((Di/Ds)^2-1)-os)));  %Di,Do分别为轴承的内外圈直径;e2,es,o2,os分别是轴承内圈和轴的弹性模量以及泊松比;Ds表示空心轴的内径；u1表示轴与轴承内圈的过盈量
end
deltao=2*u2*(Do/Dr1)/(((Do/Dr1)^2-1)*((((Do/Dr1)^2+1)/((Do/Dr1)^2-1)-o1)+e1/eh*(((Dh/Do)^2+1)/((Dh/Do)^2-1)+oh)));  %eh,e1,oh,o1分别是轴承座和轴承外圈的弹性模量以及泊松比;Dh表示轴承座的外径；u2表示轴承座与轴承外圈的过盈量
if deltai<=0
    deltai=0
end
if deltao<=0
    deltao=0
end
deltapd=-deltai-deltao;  %装配中的过盈引起的游隙变化
%温度引起的膨胀
deltat1=tao1*Do*(To-Ta); %tao1,tao2,taob,taos,taoh表示轴承外圈、内圈、滚动体、轴以及轴承座材料的线性膨胀系数；To,Ti,Tb,Ts,Th,Ta分别表示轴承外圈、内圈、滚动体、轴、轴承座和环境的温度
deltat2=tao2*Di*(Ti-Ta);
deltatb=taor*Dw*(Tr-Ta);
deltats=taos*Ds*(Ts-Ta);
deltath=taoh*Dh*(Th-Ta);
deltapt1=deltat1-2*deltatb-deltat2;%温度差引起的膨胀量
u1t=deltats-deltat2; u2t=deltat1-deltath;
if deltai<=0 && (deltai+u1t)<=0
    u1t=0
elseif deltai<=0 && (deltai+u1t)>0
    u1t=deltai+u1t
elseif deltai>0 && (deltai+u1t)<=0
    u1t=-deltai
end
if deltao<=0 && (deltao+u2t)<=0
    u2t=0
elseif deltao<=0 && (deltao+u2t)>0
    u2t=deltao+u2t
elseif deltao>0 && (deltao+u2t)<=0
    u2t=-deltao
end
if Ds==0
    deltait=2*u1t*(Dr2/Di)/(((Dr2/Di)^2-1)*((((Dr2/Di)^2+1)/((Dr2/Di)^2-1)+o2)+e2/es*(1-os)));
else
    deltait=2*u1t*(Dr2/Di)/(((Dr2/Di)^2-1)*((((Dr2/Di)^2+1)/((Dr2/Di)^2-1)+o2)+e2/es*(((Di/Ds)^2+1)/((Di/Ds)^2-1)-os)));  %Di,Do分别为轴承的内外圈直径;e2,es,o2,os分别是轴承内圈和轴的弹性模量以及泊松比;Ds表示空心轴的内径；u1表示轴与轴承内圈的过盈量
end
deltaot=2*u2t*(Do/Dr1)/(((Do/Dr1)^2-1)*((((Do/Dr1)^2+1)/((Do/Dr1)^2-1)-o1)+e1/eh*(((Dh/Do)^2+1)/((Dh/Do)^2-1)+oh)));  %eh,e1,oh,o1分别是轴承座和轴承外圈的弹性模量以及泊松比;Dh表示轴承座的外径；u2表示轴承座与轴承外圈的过盈量
deltapt2=-deltait-deltaot;  %温度差对装配量补偿引起的游隙变化
deltat=deltapt1+deltapt2;%温度引起的游隙变化
%离心力引起的变形
Ri=(Dr2+Di)/4; %Dr代表沟道直径
Ro=(Dr1+Do)/4;
Rs=(Ds+Di)/4;  Rh=(Do+Dh)/4;
deltaf1=2*ruo2*Ri^3*W2^2/e2/9.8;  %ruo1，ruo2代表外内圈材料密度
deltaf2=2*ruo1*Ro^3*W1^2/e1/9.8;
deltafs=2*ruos*Rs^3*W2^2/es/9.8;
deltafh=2*ruoh*Rh^3*W1^2/eh/9.8;
deltapf1=deltaf1-deltaf2;  %离心力引起的间隙
u1f=deltafs-deltaf2; u2f=deltaf1-deltafh;
if (deltai+deltats-deltat2)<=0 && (deltai+deltats-deltat2+u1f)<=0
    u1f=0
elseif (deltai+deltats-deltat2)<=0 && (deltai+deltats-deltat2+u1f)>0
    u1f=deltai+deltats-deltat2+u1f
elseif (deltai+deltats-deltat2)>0 && (deltai+deltats-deltat2+u1f)<=0
    u1f=-(deltai+deltats-deltat2)
end
if (deltao+deltat1-deltath)<=0 && (deltao+deltat1-deltath+u2f)<=0
    u2f=0
elseif (deltao+deltat1-deltath)<=0 && (deltao+deltat1-deltath+u2f)>0
    u2f=deltao+deltat1-deltath+u2f
elseif (deltao+deltat1-deltath)>0 && (deltao+deltat1-deltath+u2f)<=0
    u2f=-(deltao+deltat1-deltath)
end
if Ds==0
    deltaif=2*u1f*(Dr2/Di)/(((Dr2/Di)^2-1)*((((Dr2/Di)^2+1)/((Dr2/Di)^2-1)+o2)+e2/es*(1-os)));
else
    deltaif=2*u1f*(Dr2/Di)/(((Dr2/Di)^2-1)*((((Dr2/Di)^2+1)/((Dr2/Di)^2-1)+o2)+e2/es*(((Di/Ds)^2+1)/((Di/Ds)^2-1)-os)));  %Di,Do分别为轴承的内外圈直径;e2,es,o2,os分别是轴承内圈和轴的弹性模量以及泊松比;Ds表示空心轴的内径；u1表示轴与轴承内圈的过盈量
end
deltaof=2*u2f*(Do/Dr1)/(((Do/Dr1)^2-1)*((((Do/Dr1)^2+1)/((Do/Dr1)^2-1)-o1)+e1/eh*(((Dh/Do)^2+1)/((Dh/Do)^2-1)+oh)));  %eh,e1,oh,o1分别是轴承座和轴承外圈的弹性模量以及泊松比;Dh表示轴承座的外径；u2表示轴承座与轴承外圈的过盈量
deltapf2=-deltaif-deltaof;  %离心力对装配量补偿引起的游隙变化
deltaf=deltapf1+deltapf2;%离心力引起的游隙变化
deltaw=deltar0+deltapd+deltat+deltaf;%工作游隙
% 报告输出统一使用载荷求解前 calcWorkingClearance 的结果，避免旧公式重复计算导致游隙显示错误。
deltapd = clearInfo.deltapd;
deltapt1 = clearInfo.deltapt1;
deltapt2 = clearInfo.deltapt2;
deltapf1 = clearInfo.deltapf1;
deltapf2 = clearInfo.deltapf2;
deltat = clearInfo.deltat;
deltaf = clearInfo.deltaf;
deltaw = clearInfo.deltaw;
hminmin=min(oilh22);
fprintf('\n===== 游隙检查 =====\n');
fprintf('真实输入初始游隙 = %.6e m = %.6f mm\n', data_input(7), data_input(7)*1000);
fprintf('参与求解工作游隙 = %.6e m = %.6f mm\n', data_work(7), data_work(7)*1000);
fprintf('====================\n');
% 写入文本用来保存
% 页眉
reportPath = 'D:\bearing\gunzi\2.txt';
reportDir = fileparts(reportPath);
if ~isempty(reportDir) && ~isfolder(reportDir)
    mkdir(reportDir);
end
fid=fopen(reportPath,'wt','n','UTF-8');
if fid < 0
    error('无法打开结果文件: %s', reportPath);
end
fprintf(fid,'\t\t\t\t\t\t************************\n');
fprintf(fid,'\t\t\t\t\t\t高速滚子轴承拟动力学分析 \n');
fprintf(fid,'\t\t\t\t\t\t************************\n');
% 输入部分
fprintf(fid,'\n*******************************输 入 参 数**********************************\n');
fprintf(fid,'\n 轴承中径（mm）  滚子直径（mm）     滚子数目     滚子长度（mm）      滚子直线长度（mm）    轴承初始间隙（mm）  \n');
fprintf(fid,'%9d   %15.3d  %10d  %17.3d  %15.3d  %15.3d  \n', [ Dm*1000    Dw*1000   n   lenroller*1000 lenrollerline*1000 deltar0*1000 ]);
fprintf(fid,'\n************************************************************************');
fprintf(fid,'\n            弹 性 模 量 (Pa)                     泊松比   \n');
fprintf(fid,'   外圈         内圈          滚子          外圈         内圈          滚子 \n');
fprintf(fid,'%11.3d   %11.3d  %11.3d  %11.3d  %11.3d %11.3d  %11.3d  \n', [ e1  e2   e3   o1  o2  o3 ]);
fprintf(fid,'\n\n************************************************************************');
fprintf(fid,'\n 滑油密度（kg/m3）  入口油温（度）   热传导系数     粘度        粘压系数     粘温系数  \n');
fprintf(fid,'%9d   %15.3d  %17.3d  %12.3d  %11.3d  %11.3d  \n', [ oilden    sita0  K  niandu0   nianya0  beita0 ]);
fprintf(fid,'\n************************************************************************');
fprintf(fid,'\n  外圈转速      内圈转速                 轴上的负荷(N)   \n');
fprintf(fid,  '   r/min         r/min         沿 Y      沿 Z        绕 Y       绕 Z   \n');
fprintf(fid,'%7d   %12d  %9d  %9d  %9d  %9d  %9d  \n', [ W1*30/pi  W2*30/pi   Fyy Fzz Myy-0.1 Mzz-0.1 ]);
% 计 算 结 果
fprintf(fid,'\n\n\n*****************************计 算 结 果***********************************\n');
fprintf(fid,'\n 滚子    滚子           接触负荷             接触区域长度            接触区域宽度               最小油膜厚度     离心力\n');
fprintf(fid,  '  号    方位角      外圈        内圈       外圈        内圈        外圈        内圈          外圈        内圈\n');
fprintf(fid,  '         deg        N           N          m           m           m           m            m           m       N ');
for i=1:loadj
    if loadi(i)<10
        fprintf(fid,'\n %2d  %7.1f  %11.3d  %9.3d  %9.3d  %9.3d  %9.3d   %9.3d  %9.3d  %9.3d  %9.3d  %9.3d  %9.3d  %9.3d \n', [loadi(i) sita(i)*180/pi  Q1(i) Q2(i) lengthcontact1(i) lengthcontact2(i) widthcontact1(i) widthcontact2(i) oilh11(i)   oilh22(i)    m*Wo(i)^2*Dm/2    ]);
    else
        fprintf(fid,'\n %1d  %7.1f  %11.3d  %9.3d  %9.3d  %9.3d  %9.3d   %9.3d  %9.3d  %9.3d  %9.3d  %9.3d  %9.3d  %9.3d \n', [loadi(i) sita(i)*180/pi  Q1(i) Q2(i) lengthcontact1(i) lengthcontact2(i) widthcontact1(i) widthcontact2(i) oilh11(i)   oilh22(i)    m*Wo(i)^2*Dm/2    ]);
    end
end
% 非承载滚子结果
fprintf(fid,'\n %1d  %7.1f  %11.3d  %10.3d  %9.3d  %10.3d  %9.3d   %9.3d  %9.3d  %9.3d  %27.9d  %9.3d \n', [floor(n/2) 180  Q1nonload  0  lenrollerline 0 widthcontactnonload  0  m*Wononload^2*Dm/2    ]);

fprintf(fid,'\n\n*************************************************************************');
fprintf(fid,'\n     最大压应力               接触变形             自转           公转              转角\n');
fprintf(fid,  '  外圈         内圈       外圈        内圈        角速度         角速度          绕y    绕z\n');
fprintf(fid,  '   Pa          Pa          m           m         r/min         r/min           min    min \n');
for i=1:loadj
    fprintf(fid,' %1.3d  %9.3d  %9.3d  %9.3d  %9.3d  %9.3d  %9.3d  %9.3d   \n', [Ph1(i) Ph2(i) delta1k(i) delta2k(i) Wx(i)*30/pi  Wo(i)*30/pi  beita(i)*180*60/pi   afa(i)*180*60/pi  ]);
end
fprintf(fid,' %1.3d  %10.3d  %10.3d  %10.3d  %10.3d  %10.3d   %10.3d  %10.3d   \n', [Ph1nonload  0 0  0 Wxnonload*30/pi  Wononload*30/pi  ]);

fprintf(fid,'\n\n*************************************************************************');
fprintf(fid,'\n  接触区最大滑动速度            PV值                    拖动力               球/兜孔     滚子位移    滚子位移\n');
fprintf(fid,  '  外圈         内圈       外圈        内圈        外圈        内圈           冲击力       径向        轴向   \n');
fprintf(fid,  '   m/s         m/s       Pam/s       Pam/s        N          N                N           m          m   \n');
for i=1:loadj
    fprintf(fid,' %1.3d  %9.3d  %9.3d  %9.3d  %9.3d   %9.3d  %9.3d  %10.3d  %10.3d   \n', [deltaU1(i) deltaU2(i) pvzhi1(i) pvzhi2(i) T1(i)  T2(i)  Fm(i) www1(i) www1(loadj+i) ]);
end
fprintf(fid,' %1.3d  %10.3d  %9.3d  %10.3d  %9.3d   %10.3d  %9.3d  %10.3d  %10.3d   \n', [deltaU1nonload 0  pvzhinonload  0  T1nonload  0   Fmnonload  0 0 ]);

fprintf(fid,'\n\n*************************************************************************');
fprintf(fid,'\n                      内圈相对外圈的总位移 \n');
fprintf(fid,'    沿 Y         沿 Z         绕 Y         绕 Z   \n');
fprintf(fid,'     m            m           min          min   \n');
fprintf(fid,'  %1.3d  %9.3d  %9.3d  %9.3d  \n', [ Y2 Z2 sitay*180*60/pi sitaz*180*60/pi ]);
fprintf(fid,'\n\n*************************************************************************');
fprintf(fid,'\n                刚度矩阵(作用力对位移的偏导数)\n');
fprintf(fid,'        y           z          deltay       deltaz\n');
fprintf(fid,'       N/m         N/m         N/rad        N/rad \n');
fprintf(fid,' Fy  %1.3d  %9.3d  %9.3d  %9.3d   \n', [ kk(1,1)  kk(1,2) kk(1,3) kk(1,4)  ]);
fprintf(fid,' Fz  %1.3d  %9.3d  %9.3d  %9.3d   \n', [ kk(2,1)  kk(2,2) kk(2,3) kk(2,4)  ]);
fprintf(fid,' My  %1.3d  %9.3d  %9.3d  %9.3d   \n', [ kk(3,1)  kk(3,2) kk(3,3) kk(3,4)  ]);
fprintf(fid,' Mz  %1.3d  %9.3d  %9.3d  %9.3d   \n', [ kk(4,1)  kk(4,2) kk(4,3) kk(4,4)  ]);
fprintf(fid,'\n\n*************************************************************************');
if yindao==1
    fprintf(fid,'\n   保持架打滑度       \n');
    fprintf(fid,'  %3.1f%%    \n', [ dahualv*100   ]);
else
    fprintf(fid,'\n   保持架打滑度        引导面法向力     引导面摩擦力  \n');
    fprintf(fid,'   %3.1f%%       %10.3f      %10.3f  \n', [ dahualv*100    Fyindao  fyindao ]);
end
fprintf(fid,'\n\n*************************************************************************');
fprintf(fid,'\n   装配引起的游隙变化(m)      温差引起的游隙变化(m)     离心力引起的游隙变化(m)      温差对配合影响引起的游隙变化(m)     离心力对配合影响引起的游隙变化(m)     工作游隙(m)\n');
fprintf(fid,'  %8.1d    %20.3d    %20.3d    %20.3d   %20.3d    %20.3d   \n', [ deltapd   deltapt1  deltapf1   deltapt2  deltapf2   deltaw]);
fclose(fid);

% ===== 强耦合转子系统响应：追加到同一个 2.txt，并输出图片和数值表 =====
% 原轴承拟动力学结果保留在报告前半部分；下面运行 Newmark 转子-机匣
% 强耦合模型，把位移、速度、加速度和轴承力输出到报告后半部分。
rotorCoupledResult = run_strong_coupled_rotor_response(reportPath, true);


%返回值
returndata=[kk(1,1)  kk(2,2)  kk(3,3)  kk(4,4)  dahualv   L*1e6/(W2*30/pi*60) hminmin  ]; 


