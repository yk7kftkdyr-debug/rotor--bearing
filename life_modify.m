function [returndata]=life_modify(datafromvb,S,choice,gangcai,rechuli,jiagongmoshi,h,L10)
%FR  额定过滤值（μm）
%h   最小油膜厚度（μm）
%leixing  轴承类型
%gangcai  钢材类型
%S 可靠度S%
%SFdao 滚道粗糙度（算数平均表明粗糙度）
%SFgun 滚动体粗糙度（算数平均表明粗糙度）
%L10   基本额定寿命
lubricationtype=datafromvb(70);leixing=datafromvb(74);SFdao=datafromvb(23);SFgun=datafromvb(25); Dm=datafromvb(3)*1000; W2=datafromvb(18);niandu0=datafromvb(29); oilden=datafromvb(26); 
%--------------------------可靠度--------------------------------
if(S<=90)
    cr=0;
    e=1.5;
else
    cr=0.05;
    e=1.5;
    end
A1=(1-cr)*(log(100/S)/log(100/90))^(1/e)+cr

%--------------------------污染--------------------------------
if lubricationtype==0
    v=niandu0/oilden*10^6;
    if(W2<1000)
        v1=45000*W2^(-0.83)*Dm^(-0.5);
    else
        v1=4500*W2^(-0.5)*Dm^(-0.5);
    end
    k=v/v1;
    if(choice==1)
        if(0.0864*k^0.68*Dm^0.55<=1)
            ec=0.0864*k^0.68*Dm^0.55*(1-0.5663/Dm^(1/3))
        else
            ec=1-0.5663/Dm^(1/3)
        end
    elseif(choice==2)
        if(0.0432*k^0.68*Dm^0.55<=1)
            ec=0.0432*k^0.68*Dm^0.55*(1-0.9987/Dm^(1/3))
        else
            ec=1-0.9987/Dm^(1/3)
        end
    elseif(choice==3)
        if(0.0288*k^0.68*Dm^0.55<=1)
            ec=0.0288*k^0.68*Dm^0.55*(1-1.6329/Dm^(1/3))
        else
            ec=1-1.6329/Dm^(1/3)
        end
    elseif(choice==4)
        if(0.0216*k^0.68*Dm^0.55<=1)
            ec=0.0216*k^0.68*Dm^0.55*(1-2.3362/Dm^(1/3))
        else
            ec=1-2.3362/Dm^(1/3)
        end
    end    
    if(choice==5)
        if(0.0864*k^0.68*Dm^0.55<=1)
            ec=0.0864*k^0.68*Dm^0.55*(1-0.6796/Dm^(1/3))
        else
            ec=1-0.6796/Dm^(1/3)
        end
    elseif(choice==6)
        if(0.0288*k^0.68*Dm^0.55<=1)
            ec=0.0288*k^0.68*Dm^0.55*(1-1.141/Dm^(1/3))
        else
            ec=1-1.141/Dm^(1/3)
        end
    elseif(choice==7)
        if(0.0133*k^0.68*Dm^0.55<=1)
            ec=0.0133*k^0.68*Dm^0.55*(1-1.67/Dm^(1/3))
        else
            ec=1-1.67/Dm^(1/3)
        end
    elseif(choice==8)
        if(0.00864*k^0.68*Dm^0.55<=1)
            ec=0.00864*k^0.68*Dm^0.55*(1-2.5164/Dm^(1/3))
        else
            ec=1-2.5164/Dm^(1/3)
        end
    elseif(choice==9)
        if(0.00411*k^0.68*Dm^0.55<=1)
            ec=0.00411*k^0.68*Dm^0.55*(1-3.8974/Dm^(1/3))
        else
            ec=1-3.8974/Dm^(1/3)
        end
    end
    if(choice==10)
        if(0.0864*k^0.68*Dm^0.55<=1)
            ec=0.0864*k^0.68*Dm^0.55*(1-0.6796/Dm^(1/3))
        else
            ec=1-0.6796/Dm^(1/3)
        end
    elseif(choice==11)
        if(0.0432*k^0.68*Dm^0.55<=1)
            ec=0.0432*k^0.68*Dm^0.55*(1-1.141/Dm^(1/3))
        else
            ec=1-1.141/Dm^(1/3)
        end
    elseif(choice==12)
        if(0.0177*k^0.68*Dm^0.55<=1)
            ec=0.0177*k^0.68*Dm^0.55*(1-1.677/Dm^(1/3))
        else
            ec=1-1.677/Dm^(1/3)
        end
    elseif(choice==13)
        if(0.0115*k^0.68*Dm^0.55<=1)
            ec=0.0115*k^0.68*Dm^0.55*(1-2.662/Dm^(1/3))
        else
            ec=1-2.662/Dm^(1/3)
        end
    elseif(choice==14)
        if(0.00617*k^0.68*Dm^0.55<=1)
            ec=0.00617*k^0.68*Dm^0.55*(1-4.06/Dm^(1/3))
        else
            ec=1-4.06/Dm^(1/3)
        end
    end    
else
    ec=1;
end
A42=ec;

%----------------------材料及加工工艺----------------------------
if (leixing==1)%深沟球轴承
bm=1.3 ;
elseif(leixing==2)%角接触球轴承
bm=1.3; 
elseif(leixing==3)%推力球轴承
bm=1.1 ;
elseif (leixing==4)%深沟球面滚子轴承
bm=1.15  ;
elseif(leixing==5)%向心圆柱滚子轴承
bm=1.1;
elseif(leixing==6)%向心圆锥滚子轴承
bm=1.1 ;
elseif(leixing==7)%实心套圈向心滚针轴承
bm=1.1 
elseif(leixing==8)%冲压滚针轴承
bm=1
elseif(leixing==9)%推力圆锥滚子轴承
bm=1.1 
elseif(leixing==10)%推力调心滚子轴承
bm=1.15
elseif(leixing==11)%推力圆柱滚子轴承
bm=1
elseif(leixing==12)%推力滚针轴承
bm=1 
end

%-----钢材型号
if(gangcai==1)%AISI52100(GCr15)
Achem=3
elseif(gangcai==2)% M50
Achem=2
elseif(gangcai==3)% M50NiL
Achem=4 
end

%-----热处理
if(rechuli==1)%空气中冶炼
Aheat=1
elseif(rechuli==2)% 真空脱气（CVD）
Aheat=1.5
elseif(rechuli==3)% 真空电弧重榕（VAR）
Aheat=3
elseif(rechuli==4)% 双真空电弧重榕（VARVAR）
Aheat=4.5   
elseif(rechuli==5)% 真空感应熔炼与真空电弧重熔（VIMVAR）
Aheat=6           
end
%----金属加工模式
if(jiagongmoshi==1)%深沟球滚道
Aprocess=1.2
elseif(jiagongmoshi==2)% 角接触球轴承滚道
Aprocess=1
elseif(jiagongmoshi==3)% 角接触球轴承滚道，锻造套圈
Aprocess=1.2
elseif(jiagongmoshi==4)% 圆柱滚子轴承
Aprocess=1
end
%---------------------最终材料系数
if(jiagongmoshi==1 | jiagongmoshi==2 | jiagongmoshi==3)
    A2=Aprocess*Aheat*Achem/(bm)^3
elseif(jiagongmoshi==4)
    A2=Aprocess*Aheat*Achem/(bm)^4
end
%---------------------------------------------------------润滑系数-------------
if lubricationtype==0
    SF=(SFgun^2+SFdao^2)^(1/2)*1.25;
    x=h/SF;
    A3=2.912*exp(-((x- 9.138)/4.624)^2) + 1.546*exp(-((x-4.169 )/2.717)^2) +1.071 *exp(-((x- 1.916 )/1.695 )^2) +  -0.8955*exp(-((x- 0.8197)/  0.5903)^2)
else
    A3=1;
end
%----------------------------------------------------修正寿命------------------
Lmodify=A1*A2*A3*A42*L10;

%----------------------------------------打开文件继续写入--------------------------
fid=fopen('C:\Documents and Settings\Administrator\桌面\动力学分析\球轴承\ballbearing1.txt','a+t');
fprintf(fid,'\n\n*************************************************************************');
fprintf(fid,'\n   轴承基本额定寿命    可靠度    可靠度修正系数    材料修正系数    润滑修正系数    污染修正系数    轴承修正寿命\n');
fprintf(fid,'            h             %                                                                 h      \n');
fprintf(fid,'  %8.1f    %14.3f    %14.3f   %14.3f   %14.3f     %14.3f   %14.3f\n', [ L10  S  A1  A2  A3  A42  Lmodify]);
fclose(fid);


returndata=[A1  A2  A3  A42  Lmodify]; 
