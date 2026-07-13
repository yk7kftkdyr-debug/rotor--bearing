% Mst_Msr_Ks_Ge.m;
%该程序设置单元矩阵
function [Mst,Msr,Ks,Ge]=Mst_Msr_Ks_Ge(N,density,R,RO,L,Ef,miu)
% Mst： 移动单元质量矩阵
% Msr： 转动单元质量矩阵
% Ks: 刚度单元矩阵
% Ge: 陀螺力矩单元矩阵
NN0=N;
NN1=NN0+1;
NN2=NN1+1;
for i=1:1:NN0
    Mst(1,1,i)=156;
    Mst(2,1,i)=0;        Mst(2,2,i)=156;
    Mst(3,1,i)=0;        Mst(3,2,i)=-22*L(i); Mst(3,3,i)=4*L(i)^2;
    Mst(4,1,i)=22*L(i);  Mst(4,2,i)=0;        Mst(4,3,i)=0;
    Mst(4,4,i)=4*L(i)^2; Mst(5,1,i)=54;       Mst(5,2,i)=0;
    Mst(5,3,i)=0;        Mst(5,4,i)=13*L(i);  Mst(6,1,i)=0;
    Mst(6,2,i)=54;       Mst(6,3,i)=-13*L(i); Mst(6,4,i)=0;
    Mst(7,1,i)=0;        Mst(7,2,i)=13*L(i);  Mst(7,3,i)=-3*L(i)^2;
    Mst(7,4,i)=0;        Mst(8,1,i)=-13*L(i); Mst(8,2,i)=0;
    Mst(8,3,i)=0;        Mst(8,4,i)=-3*L(i)^2;Mst(5,5,i)=156;
    Mst(6,5,i)=0;        Mst(6,6,i)=156;
    Mst(7,5,i)=0;        Mst(7,6,i)=22*L(i);  Mst(7,7,i)=4*L(i)^2;
    Mst(8,5,i)=-22*L(i); Mst(8,6,i)=0;        Mst(8,7,i)=0;
    Mst(8,8,i)=4*L(i)^2;
end
for i=1:1:NN0
    Msr(1,1,i)=36;
    Msr(2,1,i)=0;        Msr(2,2,i)=36;
    Msr(3,1,i)=0;        Msr(3,2,i)=-3*L(i);  Msr(3,3,i)=4*L(i)^2;
    Msr(4,1,i)=3*L(i);   Msr(4,2,i)=0;        Msr(4,3,i)=0;
    Msr(4,4,i)=4*L(i)^2; Msr(5,1,i)=-36;      Msr(5,2,i)=0;
    Msr(5,3,i)=0;        Msr(5,4,i)=-3*L(i);  Msr(6,1,i)=0;
    Msr(6,2,i)=-36;      Msr(6,3,1)=3*L(i);   Msr(6,4,i)=0;
    Msr(7,1,i)=0;        Msr(7,2,i)=-3*L(i);  Msr(7,3,i)=-L(i)^2;
    Msr(7,4,i)=0;        Msr(8,1,i)=3*L(i);   Msr(8,2,i)=0;
    Msr(8,3,i)=0;        Msr(8,4,i)=-L(i)^2;  Msr(5,5,i)=36;
    Msr(6,5,i)=0;        Msr(6,6,i)=36;       Msr(7,5,i)=0;
    Msr(7,6,i)=3*L(i);   Msr(7,7,i)=4*L(i)^2; Msr(8,5,i)=-3*L(i);
    Msr(8,6,i)=0;        Msr(8,7,i)=0;        Msr(8,8,i)=4*L(i)^2;
end
for i=1:1:NN0
    Ge(1,1,i)=0;
    Ge(2,1,i)=36;       Ge(2,2,i)=0;
    Ge(3,1,i)=-3*L(i);  Ge(3,2,1)=0;          Ge(3,3,i)=0;
    Ge(4,1,i)=0;        Ge(4,2,i)=-3*L(i);    Ge(4,3,i)=4*L(i)^2;
    Ge(4,4,i)=0;        Ge(5,1,i)=0;          Ge(5,2,i)=36;
    Ge(5,3,i)=-3*L(i);  Ge(5,4,i)=0;          Ge(6,1,i)=-36;
    Ge(6,2,i)=0;        Ge(6,3,i)=0;          Ge(6,4,i)=-3*L(i);
    Ge(7,1,i)=-3*L(i);  Ge(7,2,i)=0;          Ge(7,3,i)=0;
    Ge(7,4,i)=L(i)^2;   Ge(8,1,i)=0;          Ge(8,2,i)=-3*L(i);
    Ge(8,3,i)=-L(i)^2;  Ge(8,4,i)=0;          Ge(5,5,i)=0;
    Ge(6,5,i)=36;       Ge(6,6,i)=0;          Ge(7,5,i)=3*L(i);
    Ge(7,6,i)=0;        Ge(7,7,i)=0;          Ge(8,5,i)=0;
    Ge(8,6,i)=3*(i);    Ge(8,7,i)=4*L(i)^2;   Ge(8,8,i)=0;
end
for i=1:1:NN0
    Ks(1,1,i)=12;
    Ks(2,1,i)=0;        Ks(2,2,i)=12;
    Ks(3,1,i)=0;        Ks(3,2,i)=-6*L(i);    Ks(3,3,i)=4*L(i)^2;
    Ks(4,1,i)=6*L(i);   Ks(4,2,i)=0;          Ks(4,3,i)=0;
    Ks(4,4,i)=4*L(i)^2; Ks(5,1,i)=-12;        Ks(5,2,i)=0;
    Ks(5,3,i)=0;        Ks(5,4,i)=-6*L(i);    Ks(6,1,i)=0;    
    Ks(6,2,i)=-12;      Ks(6,3,i)=6*L(i);     Ks(6,4,i)=0;
    Ks(7,1,i)=0;        Ks(7,2,i)=-6*L(i);    Ks(7,3,i)=2*L(i)^2;
    Ks(7,4,i)=0;        Ks(8,1,i)=6*L(i);     Ks(8,2,i)=0;
    Ks(8,3,i)=0;        Ks(8,4,i)=2*L(i)^2;   Ks(5,5,i)=12;
    Ks(6,5,i)=0;        Ks(6,6,i)=12;         Ks(7,5,i)=0;
    Ks(7,6,i)=6*L(i);   Ks(7,7,i)=4*L(i)^2;   Ks(8,5,i)=-6*L(i);
    Ks(8,6,i)=0;        Ks(8,7,i)=0;          Ks(8,8,i)=4*L(i)^2;
end
for i=1:1:NN0
    for j=1:1:8
        for k=1:1:8
            EI=Ef(i)*pi*(R(i)^4-RO(i)^4)/4;
            Mst(j,k,i)=Mst(j,k,i)*miu(i)*L(i)/420;
            Msr(j,k,i)=Msr(j,k,i)*miu(i)*R(i)^2/120/L(i);
            Ge(j,k,i)=-Ge(j,k,i)*2*miu(i)*R(i)^2/120/L(i);
            Ks(j,k,i)=Ks(j,k,i)*EI/L(i)^3;
        end
    end
end
for i=1:1:NN0
    for j=1:1:8
        for k=1:1:8
        Mst(j,k,i)=Mst(k,j,i);
        Msr(j,k,i)=Msr(k,j,i);
        Ks(j,k,i)=Ks(k,j,i);
        Ge(j,k,i)=-Ge(k,j,i);
        end
    end
end