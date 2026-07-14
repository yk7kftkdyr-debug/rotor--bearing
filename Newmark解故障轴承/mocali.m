   Rx1=Dw*(0.5*Dm/cos(a0)+0.5*Dw)/(Dm/cos(a0) );   %Dw：滚动体直径 Dm：节圆直径  球和外圈在接触椭圆的有效半径
Ry1=f1*Dw/(2*f1-1);                             %f1 外圈曲率半径     0.5232
Rx2=Dw*(0.5*Dm/cos(a0)-0.5*Dw)/(Dm/cos(a0) ); 
Ry2=f2*Dw/(2*f2-1);
niandu0                                         %润滑油粘度
E1=2/(((1-o1^2)/e1)+((1-o3^2)/e3));             %球外圈的有效弹模  e1，e3为外圈和球的弹性模量，o1，o3为外圈，球的泊松比
K1=1.0339*(Ry1/Rx1)^0.636;                      %球和外圈接触椭圆的椭圆率
K2=1.0339*(Ry2/Rx2)^0.636;
Wo=W2*(1-Dw/Dm*cos(a0))*(cos(a0)+tan(beitajiao)*sin(a0))/(( 1-Dw/Dm*cos(a0))*(cos(a0)+tan(beitajiao)*sin(a0))...
    +(1+Dw/Dm*cos(a0))*(cos(a0)+tan(beitajiao)*sin(a0)));
Wx=-W2*(1-Dw/Dm*cos(a0))*(1+Dw/Dm*cos(a0))/((1-Dw/Dm*cos(a0))*(cos(a0)+tan(beitajiao)*sin(a0))+(1+Dw/Dm*cos(a0))*(cos(a0)+tan(beitajiao)*sin(a0)) )/(Dw/Dm);
%W2为转子转速
U1(i)=abs(-Wo*Dm/4-0.5*(abs(Wx)*cos(a0)+ Wo*cos(a0))*(Dw/2)); %Wo：公转角速度；Wx：绕轴向自转速度；
U2(i)=abs(-Wo*Dm/4-0.5*(abs(Wx)*cos(a0)+ Wo*cos(a0))*(Dw/2));

oilh1(i)=Rx1*3.63*(niandu0*U1(i)/(E1*Rx1))^(0.68)*...
         (nianya0*E1)^(0.49)*(Q1(i)/Rx1^2/E1)^(-0.073)*(1-exp(-0.68*K1));
oilh2(i)=Rx2*3.63*(niandu0*U2(i)/(E2*Rx2))^(0.68)*...
         (nianya0*E2)^(0.49)*(Q2(i)/Rx2^2/E2)^(-0.073)*(1-exp(-0.68*K2));
%% 摩擦力
% W2：内圈转速
% 
fs1(i)=miuI(i)*Q1(i);fs2(i)=miuO(i)*Q2(i);
miuI(i)=0.0127*(50/(50-S12(i)))*log(0.584*Q1(i)/niandu0/deltaU1(i)/(U1(i))^2);
miuO(i)=0.0127*(50/(50-S22(i)))*log(0.584*Q2(i)/niandu0/deltaU2(i)/(U2(i))^2);
S12(i)=abs(deltaU1(i)/U1(i));  
S22(i)=abs(deltaU2(i)/U2(i));
deltaU1(i)=abs(-Wo*Dm/2+abs(Wx)*cos(a0)-Wo*cos(a0)*(Dw/2));
deltaU2(i)=abs(+(omega-Wo)*Dm/2- abs(Wx)*cos(a0)+(omega-Wo)*cos(a0)*(Dw/2));
