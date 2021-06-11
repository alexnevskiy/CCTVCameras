function plot3orig(x,y,z,s,xlims,ylims,zlims)
% % plot3orig(x,y,z,s,xlims,ylims,zlims): plot lines and points in 3D space with axes through the origin
% % (see PLOT3)

% adapted from PLOT3AXISATORIGIN
% Michael Robbins

% PLOT
if nargin<4 
    plot3(x,y,z);
else
    plot3(x,y,z,s);
end;

hold on;

if((exist('xlims','var')) && (exist('ylims','var')) && (exist('zlims','var')))
    set(gca,'XLim', xlims, 'YLim', ylims, 'ZLim', zlims);
end

% DRAW AXIS LINEs
plot3(get(gca,'XLim'),[0 0],[0 0],'k');
plot3([0 0],[0 0],get(gca,'ZLim'),'k');
plot3([0 0],get(gca,'YLim'),[0 0],'k');

% % GET TICKS
% X=get(gca,'Xtick');
% Y=get(gca,'Ytick');
% Z=get(gca,'Ztick');
% 
% % GET LABELS
% XtL=get(gca,'XtickLabel');
% YtL=get(gca,'YtickLabel');
% ZtL=get(gca,'ZtickLabel');

% REMOVE TICKS
set(gca,'Xtick',[]);
set(gca,'Ytick',[]);
set(gca,'Ztick',[]);

% % % GET OFFSETS
% Xoff=diff(get(gca,'XLim'))./30;
% Yoff=diff(get(gca,'YLim'))./30;
% Zoff=diff(get(gca,'ZLim'))./30;
% 
% % % DRAW TICKS
% % %%%%%% THIS COULD BE VECTORIZED %%%%%%%
% for i=1:length(X)
%    plot3([X(i) X(i)],[0 0],[-Zoff Zoff],'k');
% end;
% for i=1:length(Y)
%    plot3([-Yoff Yoff],[Y(i) Y(i)],[0 0],'k');
% end;
% for i=1:length(Z)
%    plot3([-Xoff Xoff],[0 0],[Z(i) Z(i)],'k');
% end;
% % 
% % % DRAW tick LABELS
% text(X,zeros(size(X)),zeros(size(X))-3.*Zoff,XtL,'Color','k');
% text(zeros(size(Y))-3.*Xoff,Y,zeros(size(Y)),YtL,'Color','k');
% text(zeros(size(Z))-3.*Xoff,zeros(size(Z)),Z,ZtL,'Color','k');

% hold off;
