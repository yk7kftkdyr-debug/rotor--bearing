%% 该程序进行矩阵组集;
function [M,G,K]=M_G_K(N,Ef,R,RO,Mst,Msr,Ge,Ks,miu,L)
% M：总的质量矩阵
% K: 总的刚度矩阵
% G：总的陀螺力矩矩阵
NNO=N
for i=1:1:NNO
    for j=1:1:8
        for k=1:1:8
        Ms(j,k,i)=Mst(j,k,i)+Msr(j,k,i);
        Ks(j,k,i)=Ks(j,k,i);
        Ge(j,k,i)=-Ge(j,k,i);
        end
    end
end
for i=1:1:N
    for j=1:1:8
        for k=1:1:8
        M(i*4+j-4,i*4+k-4)=Ms(j,k,i);
        G(i*4+j-4,i*4+k-4)=Ge(j,k,i);
        K(i*4+j-4,i*4+k-4)=Ks(j,k,i);
        end
    end
end
for i=2:1:N
    for j=1:1:4
        for k=1:1:4
        M(i*4+j-4,i*4+k-4)=M(i*4+j-4,i*4+k-4)+Ms(j+4,k+4,i-1);
        G(i*4+j-4,i*4+k-4)=G(i*4+j-4,i*4+k-4)+Ge(j+4,k+4,i-1);
        K(i*4+j-4,i*4+k-4)=K(i*4+j-4,i*4+k-4)+Ks(j+4,k+4,i-1);
        end
    end
end




