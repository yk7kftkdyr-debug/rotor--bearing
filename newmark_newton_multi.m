%% 该程序为newmark_newton算法
% 输入参数：① i: 每一个时间步长内newmark循环的次数
%           ② t: 程序整体的计算时间
xn=yn(:,n);
dxn=dyn(:,n);
ddxn=ddyn(:,n);
%if i == 1
%    x_it = xn;
%else
%    x_it = yn(:,n+1);
%end
K0=KK;
KT=K0;%k0原刚度矩阵
max_iter = 10;
tol = 1e-8;%位移收敛精度
tol_res = 1e-6; % 残差判据
% ===== 当前步初始试探位移 =====
x_it = xn;
for i = 1:max_iter%迭代次数
    t_ht = t;        %每一个时间步对newmark迭代多次
%t_ht = t-ht;     %每一个时间步对newmark迭代一次
% ====== 外力向量初始化 ======
F = zeros(N3,1);
assert(numel(Famp1)==numel(loca), 'Famp1 和 loca 的长度必须一致');
n_force = length(loca);
for j=1:n_force
    node = loca(j);
    Fx = Famp1(j)*cos(wi*t_ht + fai(j));
    Fy = Famp1(j)*sin(wi*t_ht + fai(j));
    F(4*node-3,1) = F(4*node-3,1)+Fx;
    F(4*node-2,1) = F(4*node-2,1)+Fy;
end
%F(loca(1)*4-4+1,1)=Famp1(1)*cos(wi*(t_ht)+fai(1));% 不平衡力
%F(loca(1)*4-4+2,1)=Famp1(1)*sin(wi*(t_ht)+fai(1));
%F(loca(2)*4-4+1,1)=Famp1(2)*cos(wi*(t_ht)+fai(1));
%F(loca(2)*4-4+2,1)=Famp1(2)*sin(wi*(t_ht)+fai(1));

Pn1=F;                                 %外力
F_bearing;
Fr(:,n+1) = 0;   % 先清零，避免历史残留
Fr(loc_rub(1)*4-4+1,n+1)=F_xt(1);         %转子处非线性接触力
Fr(loc_rub(1)*4-4+2,n+1)=F_xt(2);
Fr(loc_rub(2)*4-4+1,n+1)=F_xt(3);
Fr(loc_rub(2)*4-4+2,n+1)=F_xt(4);
if data(8) == 2
Fr(num_rotor+4*c1-3,n+1) = -F_xt(1);
Fr(num_rotor+4*c1-2,n+1) = -F_xt(2);
Fr(num_rotor+4*c2-3,n+1) = -F_xt(3);
Fr(num_rotor+4*c2-2,n+1) = -F_xt(4);
end
Fn1 = Fr(:,n+1);
    Kj=KT+a0*MM+a1*CC;%有效刚度矩阵
    Pj=Pn1+MM*(a0*xn+a2*dxn+a3*ddxn)+CC*(a1*xn+a4*dxn+a5*ddxn);%等效载荷
    %if i==1
%     Pj=Pj-Fn1+KT*xn;
    Pj=Pj-Fn1;

%if i>1
%     Pj=Pj-Fn1+KT*xn1;
   % Pj=Pj-Fn1;
    x_new = Kj \ Pj;
    err = norm(x_new - x_it) / max(1, norm(x_new));%当前这次迭代，新旧两次位移解的差值大小
    x_it = x_new;
    if err < tol %err是当前步内，两次连续迭代位移之间的相对变化量（误差）
        break
    end
end

% if rr<dert
%     i=100;
% end
%[QQ,RR]=qr(Kj);
%xn1=RR\QQ'*Pj;
%ddxn1=a0*(xn1-xn)-a2*dxn-a3*ddxn;
%dxn1=dxn+a6*ddxn+a7*ddxn1 ;
xn1 = x_it;
ddxn1 = a0*(xn1-xn)-a2*dxn-a3*ddxn;
dxn1 = dxn + a6*ddxn + a7*ddxn1;
yn(:,n+1)=xn1;
dyn(:,n+1)=dxn1;
ddyn(:,n+1)=ddxn1;


