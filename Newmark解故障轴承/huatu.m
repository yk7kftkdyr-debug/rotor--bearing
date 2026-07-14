ball = 0:2*pi/15:16*pi-2*pi/15;
P= polyfit(ball, Ct2, 8) ;  %三阶多项式拟合
ball_i = 0:0.1*2*pi/15:16*pi;
yi= polyval(P,ball_i);  %求对应y值
plot(yi)
%% 拟合
      x = 0 : 2*pi/15 : 16*pi-2*pi/15;
      y = Ct2;
      p5 = polyfit(x,y,100);				 % 5 阶多项式拟合 
      y5 = polyval(p5,x);
      p5 = vpa(poly2sym(p5),100)			 %显示 5 阶多项式
      p9 = polyfit(x,y,100);				 % 9 阶多项式
      y9 = polyval(p9,x);
      figure;								%画图
      plot(x,y,'bo');
      hold on;
      plot(x,y5,'r:');
      plot(x,y9,'g--');
      legend('原始数据','5 阶多项式拟合','9 阶多项式拟合');
      xlabel('x');
      xlabel('y');
XX_bowen = yn(4*loc_rub(1)-4+1,t);  YY_bowen = yn(4*loc_rub(1)-4+2,t);
dXX_bowen = dyn(4*loc_rub(1)-4+1,t);  dYY_bowen = dyn(4*loc_rub(1)-4+2,t);
ddXX_bowen = ddyn(4*loc_rub(1)-4+1,t);  ddYY_bowen = ddyn(4*loc_rub(1)-4+2,t);
save bowen XX_bowen YY_bowen dXX_bowen dYY_bowen ddXX_bowen ddYY_bowen t

XX = yn(4*loc_rub(1)-4+1,t);  YY = yn(4*loc_rub(1)-4+2,t);
dXX = dyn(4*loc_rub(1)-4+1,t);  dYY = dyn(4*loc_rub(1)-4+2,t);
ddXX = ddyn(4*loc_rub(1)-4+1,t);  ddYY = ddyn(4*loc_rub(1)-4+2,t);
save zhengchang XX YY dXX dYY ddXX ddYY t

load cucao.mat 
load bowen.mat 
load zhengchang.mat 
%% 画图
figure;								%画图
plot(XX,YY,'bo');
hold on;
plot(XX_bowen,YY_bowen,'r-');
legend('原始数据','考虑波纹度');
xlabel('x（m）');
ylabel('y（m）');

figure;								%画图
plot(t,dXX,'bo');
hold on;
plot(t,dXX_bowen,'r-');
legend('原始数据','考虑波纹度');
xlabel('t/s','FontSize',20) ; ylabel('v/（mm/s）','FontSize',20); 

figure;								%画图
plot(t,ddYY,'bo');
hold on;
plot(t,ddYY_bowen,'r-');
legend('原始数据','考虑波纹度');
xlabel('t/s','FontSize',20) ; ylabel('a/（mm/s^2）','FontSize',20); 