function rocket_orbit_comet()
% Dynamic-scale comet animation of the saved optimized two-burn ascent.
% Left: expanding view of all elapsed trajectory. Right: moving close-up.
% Both panels preserve equal physical x/z scales, with a curved Earth.
saved=load(rocket_file('rocket_two_burn_result.mat'),'result');
r=saved.result; o=r.out; p=r.p;
frameDt=2; playbackRate=20; tailSeconds=25; closeupSeconds=15;
times=unique([0:frameDt:o.t(end),o.t(end)]);
states=interp1(o.t,o.y,times,'pchip');
theta=states(:,2)-p.theta0;
x=states(:,1).*sin(theta)/1000;
z=(states(:,1).*cos(theta)-p.Re)/1000;
h=(states(:,1)-p.Re)/1000; speed=hypot(states(:,3),states(:,4));
flow=interp1(r.control.t,r.control.q,times,'linear');
t1=r.z(8); t2=t1+r.z(9);
fig=figure('Name','Adaptive orbit comet','Color','w','Position',[70 60 1100 700]);
annotation(fig,'textbox',[.06 .925 .9 .06],'String',sprintf('%g km orbit insertion | adaptive-scale comet',r.opt.targetAltitude/1000), ...
    'FontSize',19,'FontWeight','bold','EdgeColor','none','Color',[.12 .18 .27]);
status=annotation(fig,'textbox',[.06 .855 .92 .06],'String','', ...
    'FontSize',12,'EdgeColor','none','Color',[.16 .24 .34]);
ax1=axes(fig,'Position',[.08 .20 .56 .62]);
ax2=axes(fig,'Position',[.73 .34 .24 .40]);
axs=[ax1 ax2];
earth=gobjects(1,2); target=gobjects(1,2); hist=gobjects(1,2);
tail=gobjects(1,2); head=gobjects(1,2); launch=gobjects(1,2);
for k=1:2
    ax=axs(k); hold(ax,'on');grid(ax,'on');axis(ax,'equal');
    set(ax,'Color','w','XColor',[.2 .2 .2],'YColor',[.2 .2 .2],'FontSize',10);
    ax.Toolbar.Visible='off';
    earth(k)=patch(ax,NaN,NaN,[.90 .94 .90],'EdgeColor',[.3 .5 .35]);
    target(k)=plot(ax,NaN,NaN,'--','Color',[.65 .45 .65],'LineWidth',1);
    hist(k)=plot(ax,NaN,NaN,'Color',[.70 .77 .84],'LineWidth',1.7);
    tail(k)=plot(ax,NaN,NaN,'Color',[.05 .4 .85],'LineWidth',3);
    launch(k)=plot(ax,0,p.h0/1000,'^','MarkerSize',6,'MarkerFaceColor',[.2 .3 .4],'MarkerEdgeColor','none');
    head(k)=plot(ax,NaN,NaN,'o','MarkerSize',9,'MarkerEdgeColor','w','LineWidth',1.2);
    xlabel(ax,'Horizontal x (km)');ylabel(ax,'Vertical z (km)');
end
title(ax1,'Full trajectory so far | expanding view','Color',[.15 .2 .3]);
title(ax2,'Moving close-up','Color',[.15 .2 .3]);
annotation(fig,'textbox',[.69 .19 .29 .11],'String', ...
    {'Blue: last 25 s of motion','Gray: earlier trajectory','Dashed: target circle'}, ...
    'EdgeColor','none','FontSize',10,'Color',[.25 .3 .35]);
progress=annotation(fig,'textbox',[.08 .075 .9 .065],'String','', ...
    'EdgeColor','none','FontSize',11,'Color',[.2 .25 .3]);
annotation(fig,'textbox',[.08 .02 .9 .05],'String', ...
    'Equal x/z scales in each panel. Local vertical z differs from altitude h because Earth is curved.', ...
    'EdgeColor','none','FontSize',10,'Color',[.4 .4 .4]);
gifPath=rocket_file('rocket_orbit_comet.gif');
for j=1:numel(times)
    time=times(j);
    if time<t1
        phase='SUBORBITAL BURN'; color=[.95 .36 .08];
    elseif time<t2
        phase='UNPOWERED COAST';color=[.16 .55 .44];
    else
        phase='CIRCULARIZATION BURN';color=[.62 .25 .75];
    end
    if j==numel(times),phase='ENGINE OFF | ORBIT INSERTED';end
    set(status,'String',sprintf('t = %6.1f s     h = %7.2f km     v = %7.1f m/s     q = %6.1f kg/s     %s', ...
        time,h(j),speed(j),flow(j),phase));
    recent=find(times(1:j)>=time-tailSeconds);
    closeup=find(times(1:j)>=time-closeupSeconds);
    for k=1:2
        set(hist(k),'XData',x(1:j),'YData',z(1:j));
        set(tail(k),'XData',x(recent),'YData',z(recent));
        set(head(k),'XData',x(j),'YData',z(j),'MarkerFaceColor',color);
        if k==1, indices=1:j; minimumSpan=1; else, indices=closeup; minimumSpan=2; end
        pos=axs(k).Position;
        aspect=(1100*pos(3))/(700*pos(4));
        spanY=max([minimumSpan,1.35*(max(z(indices))-min(z(indices))), ...
            1.35*(max(x(indices))-min(x(indices)))/aspect]);
        spanX=spanY*aspect;
        cx=(max(x(indices))+min(x(indices)))/2;
        cz=(max(z(indices))+min(z(indices)))/2;
        xl=cx+[-.5 .5]*spanX; yl=cz+[-.5 .5]*spanY;
        xlim(axs(k),xl);ylim(axs(k),yl);
        xx=linspace(xl(1),xl(2),300);
        ground=sqrt((p.Re/1000)^2-xx.^2)-p.Re/1000;
        set(earth(k),'XData',[xx fliplr(xx)],'YData',[ground min(yl(1),min(ground)-1)*ones(size(xx))]);
        orbit=sqrt(((p.Re+r.opt.targetAltitude)/1000)^2-xx.^2)-p.Re/1000;
        set(target(k),'XData',xx,'YData',orbit);
    end
    set(progress,'String',sprintf('Burn 1: 0 - %.2f s     |     Coast: %.2f - %.2f s     |     Burn 2: %.2f - %.2f s\nPlayback: %gx   |   %.0f%% complete', ...
        t1,t1,t2,t2,o.t(end),playbackRate,100*time/o.t(end)));
    drawnow;
    frame=getframe(fig); [indexed,map]=rgb2ind(frame.cdata,256);
    delay=frameDt/playbackRate;
    if j==numel(times),delay=2;end
    if j==1
        imwrite(indexed,map,gifPath,'gif','LoopCount',Inf,'DelayTime',delay);
    else
        imwrite(indexed,map,gifPath,'gif','WriteMode','append','DelayTime',delay);
    end
    if time==80 || time==180
        imwrite(frame.cdata,rocket_file(sprintf('rocket_orbit_comet_%03ds.png',time)));
    end
end
imwrite(frame.cdata,rocket_file('rocket_orbit_comet_final.png'));
fprintf('Saved %d frames, %.2f simulated seconds: %s\n',numel(times),times(end),gifPath);
end
