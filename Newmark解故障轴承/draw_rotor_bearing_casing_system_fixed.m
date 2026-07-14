function draw_rotor_bearing_casing_system_v5()
%% 动力涡轮轴承-转子-机匣系统示意图 V5
% 修改内容：
% 1) 轴按照 16 个单元、17 个节点绘制；
% 2) 单元长度严格按照给定数据比例；
% 3) 内径、外径按“相对中心线的半径”处理；
% 4) 轴关于中心线上下对称；
% 5) 去掉原来的所有轮盘；
% 6) 新增四个轮盘：
%    节点4-5、节点6-7、节点8-9、节点14-15；
%    每个轮盘左边界在左节点，右边界在右节点；
% 7) 节点2和节点10增加弹簧+阻尼器并联等效支承；
%    且上下关于中心线对称；
% 8) 等效支承符号采用更紧凑的论文示意图风格。

clc; close all;

fontCN = chooseFont();

%% =========================
% 1. 输入数据
%% =========================
Le = [30, 107, 20.5, 40.5, 99, 115.25, 115.25, 82, ...
      70, 102.02, 66.98, 58, 23, 98.5, 58.5, 42.5];

Rin = [126, 126, 126, 126, ...
       152.4, 152.4, 152.4, 152.4, ...
       152.4, 152.4, 152.4, 152.4, ...
       152.4, 152.4, 152.4, 152.4];

Rout = [160, 160, 180, 210, ...
        178.4, 162.9, 162.9, 174.4, ...
        194, 168, 168, 199.5, ...
        346.5, 168, 168, 162.9];

nElem = numel(Le);
nNode = nElem + 1;

assert(numel(Rin) == nElem,  'Rin 长度必须等于单元数');
assert(numel(Rout) == nElem, 'Rout 长度必须等于单元数');

if any(Rout < Rin)
    error('存在 Rout < Rin 的单元，请检查内径和外径数据。');
end

%% =========================
% 2. 节点坐标
%% =========================
xCum = [0, cumsum(Le)];
Ltot = xCum(end);

%% =========================
% 3. 画布与颜色
%% =========================
fig = figure('Color','w', ...
    'Units','centimeters', ...
    'Position',[2 1.5 28 16]);

ax = axes(fig);
hold(ax,'on');
axis(ax,[0 100 0 100]);
axis(ax,'off');
axis(ax,'equal');
set(ax,'YDir','normal');

col.black   = [0.05 0.05 0.05];
col.red     = [0.93 0.45 0.47];
col.redEdge = [0.55 0.12 0.12];
col.blue    = [0.63 0.63 0.92];
col.cyan    = [0.75 0.93 0.95];
col.gray    = [0.85 0.85 0.85];
col.orange  = [0.87 0.42 0.10];

%% =========================
% 4. 布局参数
%% =========================
xLeft  = 12;
xRight = 88;
yAxis  = 56;

Rmax    = max(Rout);
RvisMax = 8.0;
scaleR  = RvisMax / Rmax;

xNode = xLeft + (xRight - xLeft) * xCum / Ltot;

yCaseTop = 83;
yCaseBot = 24;

%% =========================
% 5. 顶部标题和机匣线
%% =========================
rectangle(ax,'Position',[0 98.2 100 1.0], ...
    'FaceColor',col.blue, ...
    'EdgeColor',col.black, ...
    'LineWidth',1.0);

rectangle(ax,'Position',[36 92.7 38 5.5], ...
    'FaceColor','w', ...
    'EdgeColor',col.black, ...
    'LineWidth',1.1);

text(ax,55,95.5,'动力涡轮轴承-转子-机匣系统', ...
    'HorizontalAlignment','center', ...
    'VerticalAlignment','middle', ...
    'FontName',fontCN, ...
    'FontSize',16, ...
    'FontWeight','bold');

plot(ax,[xLeft-7, xRight+7],[yCaseTop yCaseTop], ...
    'Color',col.blue, ...
    'LineWidth',3.0);

plot(ax,[xLeft-7, xRight+7],[yCaseBot yCaseBot], ...
    'Color',col.blue, ...
    'LineWidth',3.0);

rectangle(ax,'Position',[47.5 18.8 8.6 4.0], ...
    'LineStyle','--', ...
    'EdgeColor',col.black, ...
    'FaceColor','w', ...
    'LineWidth',1.0);

text(ax,51.8,20.8,'机 匣', ...
    'HorizontalAlignment','center', ...
    'FontName',fontCN, ...
    'FontSize',14);

%% =========================
% 6. 绘制分段轴
%% =========================
drawSteppedHollowShaft(ax, xNode, yAxis, Rin, Rout, scaleR, col.red, col.redEdge);

plot(ax,[xLeft-2, xRight+2],[yAxis yAxis], ...
    'k-.', ...
    'LineWidth',1.0);

%% =========================
% 7. 节点编号
%% =========================
yNodeText = yAxis + RvisMax + 2.8;

for i = 1:nNode
    text(ax,xNode(i),yNodeText,num2str(i), ...
        'HorizontalAlignment','center', ...
        'FontName','Times New Roman', ...
        'FontSize',10, ...
        'FontWeight','bold');
end

%% =========================
% 8. 节点辅助虚线
%% =========================
for i = 1:nNode
    plot(ax,[xNode(i) xNode(i)], ...
        [yAxis-RvisMax-2, yAxis+RvisMax+2], ...
        'Color',[0.75 0.75 0.75], ...
        'LineStyle',':', ...
        'LineWidth',0.8);
end

%% =========================
% 9. 轮盘：严格位于指定节点之间
%% =========================
% 节点4-5
drawDiskBetweenNodes(ax, xNode, 4, 5, yAxis, ...
    localOuterForSpan(4,5,Rout)*scaleR, ...
    13.0, col.cyan, col.black, '轮盘1', fontCN);

% 节点6-7
drawDiskBetweenNodes(ax, xNode, 6, 7, yAxis, ...
    localOuterForSpan(6,7,Rout)*scaleR, ...
    15.0, col.cyan, col.black, '轮盘2', fontCN);

% 节点8-9
drawDiskBetweenNodes(ax, xNode, 8, 9, yAxis, ...
    localOuterForSpan(8,9,Rout)*scaleR, ...
    12.5, col.cyan, col.black, '轮盘3', fontCN);

% 节点14-15
drawDiskBetweenNodes(ax, xNode, 14, 15, yAxis, ...
    localOuterForSpan(14,15,Rout)*scaleR, ...
    14.0, col.cyan, col.black, '轮盘4', fontCN);

%% =========================
% 10. 节点2、节点10：弹簧 + 阻尼器并联等效支承
%% =========================
drawSymmetricEquivalentSpringDamper(ax, xNode(2), yAxis, ...
    localOuterForNode(2,Rout)*scaleR, yCaseTop, yCaseBot, ...
    col.orange, col.black, 'k_{eq1}', 'c_{eq1}');

drawSymmetricEquivalentSpringDamper(ax, xNode(10), yAxis, ...
    localOuterForNode(10,Rout)*scaleR, yCaseTop, yCaseBot, ...
    col.orange, col.black, 'k_{eq2}', 'c_{eq2}');

%% =========================
% 11. 轴承支承示意
%% =========================
xB1 = xNode(5);
xB2 = xNode(14);

drawBearingSupportShort(ax,xB1,yAxis-RvisMax-1.5,yCaseBot+1.0, ...
    col,'前支撑滚子轴承',fontCN,'left');

drawBearingSupportShort(ax,xB2,yAxis-RvisMax-1.5,yCaseBot+1.0, ...
    col,'后支撑球轴承',fontCN,'right');

%% =========================
% 12. 基础支承
%% =========================
xBase1 = xLeft - 4.5;
xBase2 = xRight + 4.5;

drawBaseSupportShort(ax,xBase1,yCaseBot,8.5, ...
    '基础支承 1','k_{s1}','c_{s1}',col,fontCN,'left');

drawBaseSupportShort(ax,xBase2,yCaseBot,8.5, ...
    '基础支承 2','k_{s2}','c_{s2}',col,fontCN,'right');

%% =========================
% 13. 导出
%% =========================
exportgraphics(fig,'转子轴承机匣系统_v5.png','Resolution',600);
exportgraphics(fig,'转子轴承机匣系统_v5.pdf','ContentType','vector');

disp('V5 完成：已生成 PNG 和 PDF 文件。');

end

%% =========================================================
%% 局部函数
%% =========================================================

function fontCN = chooseFont()
fonts = listfonts;
candidate = {'Microsoft YaHei','SimHei','SimSun','Arial Unicode MS'};
fontCN = 'Helvetica';

for i = 1:numel(candidate)
    if any(strcmpi(fonts,candidate{i}))
        fontCN = candidate{i};
        return;
    end
end
end

%% ========== 分段空心轴 ==========
function drawSteppedHollowShaft(ax, xNode, yAxis, Rin, Rout, scaleR, faceColor, edgeColor)
nElem = numel(Rin);

for i = 1:nElem
    x1 = xNode(i);
    x2 = xNode(i+1);
    w  = x2 - x1;

    rin  = Rin(i)  * scaleR;
    rout = Rout(i) * scaleR;

    % 上半部分
    rectangle(ax,'Position',[x1, yAxis + rin, w, rout-rin], ...
        'FaceColor',faceColor, ...
        'EdgeColor',edgeColor, ...
        'LineWidth',1.0);

    % 下半部分
    rectangle(ax,'Position',[x1, yAxis - rout, w, rout-rin], ...
        'FaceColor',faceColor, ...
        'EdgeColor',edgeColor, ...
        'LineWidth',1.0);
end
end

%% ========== 轮盘：严格落在两个节点之间 ==========
function drawDiskBetweenNodes(ax, xNode, nL, nR, yAxis, shaftOuter, extraH, faceColor, edgeColor, labelText, fontCN)
x1 = xNode(nL);
x2 = xNode(nR);
w  = x2 - x1;
xc = 0.5*(x1 + x2);

% 上轮盘
rectangle(ax,'Position',[x1, yAxis + shaftOuter, w, extraH], ...
    'FaceColor',faceColor, ...
    'EdgeColor',edgeColor, ...
    'LineWidth',1.0);

% 下轮盘
rectangle(ax,'Position',[x1, yAxis - shaftOuter - extraH, w, extraH], ...
    'FaceColor',faceColor, ...
    'EdgeColor',edgeColor, ...
    'LineWidth',1.0);

% 左右竖边强调：矩形边界落在节点上
plot(ax,[x1 x1],[yAxis-shaftOuter-extraH, yAxis+shaftOuter+extraH], ...
    'Color',edgeColor, ...
    'LineWidth',0.8);

plot(ax,[x2 x2],[yAxis-shaftOuter-extraH, yAxis+shaftOuter+extraH], ...
    'Color',edgeColor, ...
    'LineWidth',0.8);

text(ax,xc,yAxis + shaftOuter + extraH + 1.6,labelText, ...
    'HorizontalAlignment','center', ...
    'FontName',fontCN, ...
    'FontSize',11);
end

%% ========== 节点处弹簧+阻尼器并联支承 ==========
function drawSymmetricEquivalentSpringDamper(ax, x, yAxis, shaftOuter, yCaseTop, yCaseBot, springColor, lineColor, kText, cText)
% 更接近论文示意图风格的“弹簧+阻尼器并联”支承
% 左边弹簧，右边阻尼器
% 上下关于中心线对称

dx = 0.78;
barHalf = 1.35;
springAmp = 0.32;
nSpring = 4;

%% ---------- 上半部分 ----------
y0u = yAxis + shaftOuter;
y1u = y0u + 0.9;
y2u = y0u + 5.0;

% 轴到支承符号的短连杆
plot(ax,[x x],[y0u y1u], ...
    'Color',lineColor, ...
    'LineWidth',1.0);

% 上下横杆
plot(ax,[x-barHalf x+barHalf],[y1u y1u], ...
    'Color',lineColor, ...
    'LineWidth',0.95);

plot(ax,[x-barHalf x+barHalf],[y2u y2u], ...
    'Color',lineColor, ...
    'LineWidth',0.95);

% 左侧弹簧
drawSpringVerticalCompact(ax, x-dx, y1u, y2u, springAmp, nSpring, lineColor);

% 右侧阻尼器
drawDamperVerticalCompact(ax, x+dx, y1u, y2u, 0.48, lineColor, springColor);

% 连到上机匣
plot(ax,[x x],[y2u yCaseTop], ...
    'Color',lineColor, ...
    'LineWidth',1.0);

% 参数标注
text(ax, x-2.35, y1u + 2.0, kText, ...
    'FontName','Times New Roman', ...
    'FontSize',11, ...
    'Interpreter','tex', ...
    'Color',lineColor, ...
    'HorizontalAlignment','center');

text(ax, x+2.10, y1u + 2.0, cText, ...
    'FontName','Times New Roman', ...
    'FontSize',11, ...
    'Interpreter','tex', ...
    'Color',lineColor, ...
    'HorizontalAlignment','center');

%% ---------- 下半部分 ----------
y0d = yAxis - shaftOuter;
y1d = y0d - 0.9;
y2d = y0d - 5.0;

% 轴到支承符号的短连杆
plot(ax,[x x],[y0d y1d], ...
    'Color',lineColor, ...
    'LineWidth',1.0);

% 上下横杆
plot(ax,[x-barHalf x+barHalf],[y1d y1d], ...
    'Color',lineColor, ...
    'LineWidth',0.95);

plot(ax,[x-barHalf x+barHalf],[y2d y2d], ...
    'Color',lineColor, ...
    'LineWidth',0.95);

% 左侧弹簧
drawSpringVerticalCompact(ax, x-dx, y2d, y1d, springAmp, nSpring, lineColor);

% 右侧阻尼器
drawDamperVerticalCompact(ax, x+dx, y2d, y1d, 0.48, lineColor, springColor);

% 连到下机匣
plot(ax,[x x],[yCaseBot y2d], ...
    'Color',lineColor, ...
    'LineWidth',1.0);
end

%% ========== 紧凑弹簧 ==========
function drawSpringVerticalCompact(ax, x, y1, y2, amp, n, color)
yy = linspace(y1, y2, 2*n+1);
xx = ones(size(yy)) * x;

for i = 2:numel(xx)-1
    if mod(i,2)==0
        xx(i) = x - amp;
    else
        xx(i) = x + amp;
    end
end

xx(1) = x;
xx(end) = x;

plot(ax,xx,yy, ...
    'Color',color, ...
    'LineWidth',1.0);
end

%% ========== 紧凑阻尼器 ==========
function drawDamperVerticalCompact(ax, x, y1, y2, w, lineColor, damperColor)
ym = 0.5*(y1 + y2);
boxH = 0.95;

% 上下连杆
plot(ax,[x x],[y1 ym-boxH/2], ...
    'Color',lineColor, ...
    'LineWidth',0.95);

plot(ax,[x x],[ym+boxH/2 y2], ...
    'Color',lineColor, ...
    'LineWidth',0.95);

% 阻尼器主体
rectangle(ax,'Position',[x-w/2, ym-boxH/2, w, boxH], ...
    'FaceColor','w', ...
    'EdgeColor',damperColor, ...
    'LineWidth',1.0);

% 活塞横线
plot(ax,[x-w*0.72 x+w*0.72],[ym+boxH/2 ym+boxH/2], ...
    'Color',damperColor, ...
    'LineWidth',1.0);
end

%% ========== 节点处局部外半径 ==========
function r = localOuterForNode(nodeID, Rout)
nElem = numel(Rout);

if nodeID == 1
    r = Rout(1);
elseif nodeID == nElem + 1
    r = Rout(end);
else
    r = max(Rout(nodeID-1), Rout(nodeID));
end
end

%% ========== 两节点之间局部外半径 ==========
function r = localOuterForSpan(nL, nR, Rout)
elemL = nL;
elemR = nR - 1;

elemL = max(1, min(elemL, numel(Rout)));
elemR = max(1, min(elemR, numel(Rout)));

r = max(Rout(elemL:elemR));
end

%% ========== 轴承支承简化示意 ==========
function drawBearingSupportShort(ax,x,yTop,yBot,col,labelText,fontCN,side)
plot(ax,[x x],[yTop yBot+8], ...
    'Color',col.black, ...
    'LineWidth',1.0);

drawSmallSpringDamperShort(ax,x,yBot+3.5,yBot+8,col.black,col.orange);

if strcmp(side,'left')
    tx = x - 8.5;
else
    tx = x - 4.5;
end

text(ax,tx,yBot+11,labelText, ...
    'FontName',fontCN, ...
    'FontSize',11, ...
    'Color',col.black);
end

%% ========== 小型弹簧阻尼支承 ==========
function drawSmallSpringDamperShort(ax,x,y1,y2,color,damperColor)
drawSpringVertical(ax,x-0.45,y1+0.4,y2-0.4,0.45,4,color);
drawDamperOnly(ax,x+0.45,y1+0.4,y2-0.4,0.50,color,damperColor);

plot(ax,[x-0.85 x+0.85],[y1 y1], ...
    'Color',color, ...
    'LineWidth',0.9);

plot(ax,[x-0.85 x+0.85],[y2 y2], ...
    'Color',color, ...
    'LineWidth',0.9);
end

%% ========== 基础支承 ==========
function drawBaseSupportShort(ax,x,yTop,yGround,titleText,kText,cText,col,fontCN,side)
plot(ax,[x x],[yTop yTop-2.0], ...
    'Color',col.black, ...
    'LineWidth',1.0);

drawSmallSpringDamperShort(ax,x,yGround+2.5,yTop-2.0,col.black,col.orange);

drawGround(ax,x,yGround,5.5,col.black);

if strcmp(side,'left')
    tx = x - 6.0;
else
    tx = x - 1.2;
end

text(ax,tx,yGround+0.8,titleText, ...
    'FontName',fontCN, ...
    'FontSize',12);

text(ax,tx+0.2,yGround-2.8,['$' kText '$'], ...
    'Interpreter','latex', ...
    'FontSize',12);

text(ax,tx+6.0,yGround-2.8,['$' cText '$'], ...
    'Interpreter','latex', ...
    'FontSize',12);
end

%% ========== 普通竖向弹簧 ==========
function drawSpringVertical(ax,x,y1,y2,amp,n,color)
yy = linspace(y1,y2,2*n+1);
xx = ones(size(yy))*x;

for i = 2:numel(xx)-1
    if mod(i,2)==0
        xx(i) = x - amp;
    else
        xx(i) = x + amp;
    end
end

xx(1)=x;
xx(end)=x;

plot(ax,xx,yy, ...
    'Color',color, ...
    'LineWidth',1.0);
end

%% ========== 普通阻尼器 ==========
function drawDamperOnly(ax,x,y1,y2,w,color,damperColor)
ym = 0.5*(y1 + y2);

plot(ax,[x x],[y1 ym-0.6], ...
    'Color',color, ...
    'LineWidth',0.9);

plot(ax,[x x],[ym+0.6 y2], ...
    'Color',color, ...
    'LineWidth',0.9);

rectangle(ax,'Position',[x-w/2, ym-0.6, w, 1.2], ...
    'FaceColor','w', ...
    'EdgeColor',damperColor, ...
    'LineWidth',1.0);

plot(ax,[x-w*0.7 x+w*0.7],[ym+0.6 ym+0.6], ...
    'Color',damperColor, ...
    'LineWidth',1.0);
end

%% ========== 地面符号 ==========
function drawGround(ax,x,y,w,color)
plot(ax,[x-w/2 x+w/2],[y y], ...
    'Color',color, ...
    'LineWidth',1.0);

n = 6;

for i = 1:n
    xi = x - w/2 + i*w/(n+1);
    plot(ax,[xi-0.7 xi-0.15],[y-0.8 y], ...
        'Color',color, ...
        'LineWidth',0.9);
end
end