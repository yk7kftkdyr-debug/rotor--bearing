function [deltaw, clearInfo] = calcWorkingClearance(datafromvb)
% 计算装配、温差、离心力修正后的工作游隙
% 输入 datafromvb(7) 为初始径向游�?
% 输出 deltaw 为实际参与承载计算的工作游隙

Dw = datafromvb(2);
Dm = datafromvb(3);
deltar0 = datafromvb(7);

W1 = datafromvb(17)*pi/30;
W2 = datafromvb(18)*pi/30;

e1 = datafromvb(11);
e2 = datafromvb(12);
o1 = datafromvb(14);
o2 = datafromvb(15);

Di = datafromvb(35);
Do = datafromvb(34);
Ds = datafromvb(36);
Dh = datafromvb(37);

es = datafromvb(38);
eh = datafromvb(39);
os = datafromvb(40);
oh = datafromvb(41);

uOuter = datafromvb(42);   % 外圈/轴承座过盈量
uInner = datafromvb(43);   % 轴/内圈过盈量

To = datafromvb(44);
Ti = datafromvb(45);
Tr = datafromvb(46);
Ta = datafromvb(47);

ruo1 = datafromvb(48);
ruo2 = datafromvb(49);
ruos = datafromvb(50);
ruoh = datafromvb(51);

taos = datafromvb(52);
taoh = datafromvb(53);
taoInner = datafromvb(54);
taoOuter = datafromvb(55);
taor = datafromvb(56);

Ts = datafromvb(57);
Th = datafromvb(58);

Dr1 = Dm + Dw + deltar0;
Dr2 = Dm - Dw - deltar0;

% ===================== 1. 装配过盈引起的游隙变�?=====================
if Ds == 0
    deltai = 2*uInner*(Dr2/Di) / ...
        (((Dr2/Di)^2-1) * ((((Dr2/Di)^2+1)/((Dr2/Di)^2-1)+o2) + e2/es*(1-os)));
else
    deltai = 2*uInner*(Dr2/Di) / ...
        (((Dr2/Di)^2-1) * ((((Dr2/Di)^2+1)/((Dr2/Di)^2-1)+o2) + e2/es*(((Di/Ds)^2+1)/((Di/Ds)^2-1)-os)));
end

deltao = 2*uOuter*(Do/Dr1) / ...
    (((Do/Dr1)^2-1) * ((((Do/Dr1)^2+1)/((Do/Dr1)^2-1)-o1) + e1/eh*(((Dh/Do)^2+1)/((Dh/Do)^2-1)+oh)));

if deltai <= 0
    deltai = 0;
end

if deltao <= 0
    deltao = 0;
end
deltapd = -deltai - deltao;

% ===================== 2. 温度膨胀引起的游隙变�?=====================
% 温度引起的滚道径向游隙变�?
outerRaceExpand = taoOuter*Dr1*(To-Ta);
innerRaceExpand = taoInner*Dr2*(Ti-Ta);
rollerExpand = taor*Dw*(Tr-Ta);
deltapt1 = outerRaceExpand - innerRaceExpand - 2*rollerExpand;
shaftFitExpand = taos*Di*(Ts-Ta);
innerFitExpand = taoInner*Di*(Ti-Ta);
outerFitExpand = taoOuter*Do*(To-Ta);
housingFitExpand = taoh*Do*(Th-Ta);
u1t = shaftFitExpand - innerFitExpand;
u2t = outerFitExpand - housingFitExpand;
if deltai <= 0 && (deltai+u1t) <= 0
    u1t = 0;
elseif deltai <= 0 && (deltai+u1t) > 0
    u1t = deltai + u1t;
elseif deltai > 0 && (deltai+u1t) <= 0
    u1t = -deltai;
end

if deltao <= 0 && (deltao+u2t) <= 0
    u2t = 0;
elseif deltao <= 0 && (deltao+u2t) > 0
    u2t = deltao + u2t;
elseif deltao > 0 && (deltao+u2t) <= 0
    u2t = -deltao;
end

if Ds == 0
    deltait = 2*u1t*(Dr2/Di) / ...
        (((Dr2/Di)^2-1) * ((((Dr2/Di)^2+1)/((Dr2/Di)^2-1)+o2) + e2/es*(1-os)));
else
    deltait = 2*u1t*(Dr2/Di) / ...
        (((Dr2/Di)^2-1) * ((((Dr2/Di)^2+1)/((Dr2/Di)^2-1)+o2) + e2/es*(((Di/Ds)^2+1)/((Di/Ds)^2-1)-os)));
end
deltaot = 2*u2t*(Do/Dr1) / ...
    (((Do/Dr1)^2-1) * ((((Do/Dr1)^2+1)/((Do/Dr1)^2-1)-o1) + e1/eh*(((Dh/Do)^2+1)/((Dh/Do)^2-1)+oh)));
deltapt2 = -deltait - deltaot;
deltat = deltapt1 + deltapt2;
% ===================== 3. 离心力引起的游隙变化 =====================
Ri = (Dr2 + Di)/4;
Ro = (Dr1 + Do)/4;
Rs = (Ds + Di)/4;
Rh = (Do + Dh)/4;
deltaf1 = 2*ruo2*Ri^3*W2^2/e2/9.8;
deltaf2 = 2*ruo1*Ro^3*W1^2/e1/9.8;
deltafs = 2*ruos*Rs^3*W2^2/es/9.8;
deltafh = 2*ruoh*Rh^3*W1^2/eh/9.8;
deltapf1 = deltaf1 - deltaf2;
u1f = deltafs - deltaf2;
u2f = deltaf1 - deltafh;
innerFitAfterThermal = deltai + u1t;
outerFitAfterThermal = deltao + u2t;
if innerFitAfterThermal <= 0 && (innerFitAfterThermal+u1f) <= 0
    u1f = 0;
elseif innerFitAfterThermal <= 0 && (innerFitAfterThermal+u1f) > 0
    u1f = innerFitAfterThermal + u1f;
elseif innerFitAfterThermal > 0 && (innerFitAfterThermal+u1f) <= 0
    u1f = -innerFitAfterThermal;
end
if outerFitAfterThermal <= 0 && (outerFitAfterThermal+u2f) <= 0
    u2f = 0;
elseif outerFitAfterThermal <= 0 && (outerFitAfterThermal+u2f) > 0
    u2f = outerFitAfterThermal + u2f;
elseif outerFitAfterThermal > 0 && (outerFitAfterThermal+u2f) <= 0
    u2f = -outerFitAfterThermal;
end


if Ds == 0
    deltaif = 2*u1f*(Dr2/Di) / ...
        (((Dr2/Di)^2-1) * ((((Dr2/Di)^2+1)/((Dr2/Di)^2-1)+o2) + e2/es*(1-os)));
else
    deltaif = 2*u1f*(Dr2/Di) / ...
        (((Dr2/Di)^2-1) * ((((Dr2/Di)^2+1)/((Dr2/Di)^2-1)+o2) + e2/es*(((Di/Ds)^2+1)/((Di/Ds)^2-1)-os)));
end

deltaof = 2*u2f*(Do/Dr1) / ...
    (((Do/Dr1)^2-1) * ((((Do/Dr1)^2+1)/((Do/Dr1)^2-1)-o1) + e1/eh*(((Dh/Do)^2+1)/((Dh/Do)^2-1)+oh)));

deltapf2 = -deltaif - deltaof;
deltaf = deltapf1 + deltapf2;

% ===================== 4. 最终工作游�?=====================
deltaw = deltar0 + deltapd + deltat + deltaf;

clearInfo.deltar0 = deltar0;
clearInfo.deltapd = deltapd;
clearInfo.deltapt1 = deltapt1;
clearInfo.outerRaceExpand = outerRaceExpand;
clearInfo.innerRaceExpand = innerRaceExpand;
clearInfo.rollerExpand = rollerExpand;
clearInfo.deltapt2 = deltapt2;
clearInfo.deltat = deltat;
clearInfo.deltapf1 = deltapf1;
clearInfo.deltapf2 = deltapf2;
clearInfo.deltaf = deltaf;
clearInfo.deltaw = deltaw;

end
