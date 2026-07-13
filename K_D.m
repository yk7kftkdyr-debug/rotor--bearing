%% 该程序将支撑刚度、阻尼加入转子总体矩阵，%作用把轴承支承刚度、支承阻尼加入到转子有限元总体矩阵里，并计算加入支承后的固有频率和振型
function [K,C,D,Ax,fn123]=K_D(N,K,M,G,wi,is_coupled)
Kzx = zeros(N+1,1);
Kzy = zeros(N+1,1);
if is_coupled == 0
Kzx(2)=2.5e9;
Kzy(2)=2.5e9;
Kzx(10)=2.5e9;%N/m，支承越硬，转子越不容易横向位移，这会直接影响临界转速和模态频率
Kzy(10)=2.5e9;
end
for i=1:1:N+1
K(i*4+1-4,i*4+1-4)=K(i*4+1-4,i*4+1-4)+Kzx(i);%第 i 个节点的第 1 个自由度，加上 x 向支承刚度
K(i*4+2-4,i*4+2-4)=K(i*4+2-4,i*4+2-4)+Kzy(i);%轴承支承刚度只加在平移方向，不加在转角自由度上，这很合理，所以没有4*i-3
end
format long g
[Ax,WW]=eig(K,M);
f=sqrt(WW)/(2*pi);
f0=diag(f);
f00=abs(sort(f0))
fn123=[f00(1) f00(3) f00(5)]
wi1=54*2*pi;
wi2=284*2*pi;
yita1=0.1;
yita2=0.2;
alf=2*(yita2/wi2-yita1/wi1)*(1/wi2^2-1/wi1^2);%alf = 2*wi1*wi2*(yita1*wi2 - yita2*wi1)/(wi2^2 - wi1^2);
beita=2*(yita2*wi2-yita1*wi1)/(wi2^2-wi1^2);%beita = 2*(yita2*wi2 - yita1*wi1)/(wi2^2 - wi1^2);
C=alf*M+beita*K;%瑞丽阻尼
D=C+wi*G;
Dzx = zeros(N+1,1);
Dzy = zeros(N+1,1);
if is_coupled == 0
Dzx(2)=5e3;%这里要选轴承位置
Dzy(2)=5e3;
Dzx(10)=5e3;
Dzy(10)=5e3;
end
for i=1:1:N+1
    D(i*4+1-4,i*4+1-4)=D(i*4+1-4,i*4+1-4)+Dzx(i);
    D(i*4+2-4,i*4+2-4)=D(i*4+2-4,i*4+2-4)+Dzy(i);
end
C=D*1;%D 复制给 C