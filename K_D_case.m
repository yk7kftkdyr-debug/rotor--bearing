%% 该程序将支撑刚度、阻尼加入转子总体矩阵
function [K_C,C_C,D_C,Ax_C,fn123_C]=K_D_case(N_C,K_C,M_C,G_C,is_coupled)
%% ===== 机匣基础支撑节点 =====
case_base_nodes = [1 17];

k_case_base = 7e9;     % 机匣-基础刚度
c_case_base = 1.5e3;   % 机匣-基础阻尼

Kzx = zeros(N_C+1,1);
Kzy = zeros(N_C+1,1);
Dzx = zeros(N_C+1,1);
Dzy = zeros(N_C+1,1);
%% ===== 把基础刚度/阻尼加到指定机匣节点 =====
for jj = 1:length(case_base_nodes)
    node = case_base_nodes(jj);
    Kzx(node) = k_case_base;
    Kzy(node) = k_case_base;
    Dzx(node) = c_case_base;
    Dzy(node) = c_case_base;
end
%% 开始迭代
for i=1:1:N_C+1
K_C(i*4+1-4,i*4+1-4)=K_C(i*4+1-4,i*4+1-4)+Kzx(i);
K_C(i*4+2-4,i*4+2-4)=K_C(i*4+2-4,i*4+2-4)+Kzy(i);
end
format long g
[Ax_C,WW]=eig(inv(M_C)*K_C);
f=sqrt(WW)/(2*pi);
f0=diag(f);
f00_C=abs(sort(f0))
fn123_C=[f00_C(1) f00_C(3) f00_C(5)]
wi1=54*2*pi;
wi2=284*2*pi;
yita1=0.1;
yita2=0.2;
alf=2*(yita2/wi2-yita1/wi1)*(1/wi2^2-1/wi1^2);
beita=2*(yita2*wi2-yita1*wi1)/(wi2^2-wi1^2);
C_C=alf*M_C+beita*K_C;
D_C=C_C+G_C;
%% 开始迭代
for i=1:1:N_C+1
    D_C(i*4+1-4,i*4+1-4)=D_C(i*4+1-4,i*4+1-4)+Dzx(i);
    D_C(i*4+2-4,i*4+2-4)=D_C(i*4+2-4,i*4+2-4)+Dzy(i);
end
C_C=D_C*1;