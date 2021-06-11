% [xlim,ylim,zlim]=cardinalCoordSys(xlims,ylims,zlims): 
% set the axes centered at the origin; can specify the axis limits or calculate them automatically
%   * Ox axis is WE
%   * Oy axis is SN
%   * Oz axis is height

function [xlim,ylim,zlim]=cardinalCoordSys(xlims,ylims,zlims)
%     set(gcf,'renderer', 'zbuffer'); 
    if((exist('xlims','var')) && (exist('ylims','var')) && (exist('zlims','var')))
%         set(gca,'XLim', xlims, 'YLim', ylims, 'ZLim', zlims);
    else
        % the limits for the axes
        xlims=get(gca,'XLim'); ylims=get(gca,'YLim'); zlims=get(gca,'ZLim');

        xlims=[min(min(xlims),-0.2*max(xlims)) max(xlims)*1.2];
        ylims=[min(min(ylims),-0.2*max(ylims)) max(ylims)*1.2];
        zlims=[min(min(zlims),-0.2*max(zlims)) max(zlims)*1.2];
    end
    
    
    plot3orig(0,0,0,'.k',xlims,ylims,zlims); % change to a plot3orig view
    xlim=xlims; ylim=ylims;zlim=zlims;

    set(get(gca,'XLabel'),'String','X Axis')
    set(get(gca,'YLabel'),'String','Y Axis')
    set(get(gca,'ZLabel'),'String','Z Axis')

    % DRAW LABELS
    pos = get(gca,'XLim');
    text(pos(1),0,0,'West (x-)','Color','k');
    text(pos(2),0,0,'East (x+)','Color','k');

    pos = get(gca,'YLim');
    text(0,pos(1),0,'South (y-)','Color','k');
    text(0,pos(2),0,'North (y+)','Color','k');

    pos = get(gca,'ZLim');
    text(0,0,pos(1),'Down (z-)','Color','k');
    text(0,0,pos(2),'Up (z+)','Color','k');

    axis equal vis3d off;
end