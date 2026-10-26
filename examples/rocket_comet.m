%ROCKET_COMET Time-resolved comet animation using current rocket_config.
% Run: rocket_comet. Output: rocket_comet.gif and rocket_comet_final.png.
% Geometry uses equal scales and an Earth-centered polar-to-Cartesian map.
clear; clc;
[p,control]=rocket_config();
out=rocket_simulate(p,control);
frameDt=2.5;                    % Simulated seconds per frame
playbackRate=50;                % 50 simulated seconds per playback second
tailSeconds=40;
times=unique([0:frameDt:out.t(end),out.t(end)]);
states=interp1(out.t,out.y,times,'pchip');
angle=states(:,2)-p.theta0;
x=states(:,1).*sin(angle)/1000;
z=(states(:,1).*cos(angle)-p.Re)/1000;
height=(states(:,1)-p.Re)/1000;
speed=hypot(states(:,3),states(:,4));
fig=figure('Name','Rocket comet trajectory','Color','w','Position',[100 60 820 820]);
ax=axes(fig,'Position',[0.12 0.1 0.82 0.8],'Color','w','XColor','k','YColor','k');
hold(ax,'on'); grid(ax,'on'); axis(ax,'equal');
xlim(ax,[-290 410]); ylim(ax,[-35 715]);
xlabel(ax,'Local horizontal coordinate x (km)');
ylabel(ax,'Local vertical coordinate z (km)');
title(ax,{'Rocket trajectory: physical geometry, equal axis scales', ...
    'Moving head + 40 s trail | playback 50x | current parameters'},'Color','k');
earthX=linspace(-290,410,400);
earthZ=sqrt((p.Re/1000)^2-earthX.^2)-p.Re/1000;
fill(ax,[earthX fliplr(earthX)],[earthZ -35*ones(size(earthX))], ...
    [0.90 0.94 0.90],'EdgeColor','none');
plot(ax,earthX,earthZ,'Color',[0.25 0.45 0.30],'LineWidth',1.3);
plot(ax,0,p.h0/1000,'^','MarkerFaceColor',[0.2 0.3 0.4],'MarkerEdgeColor','none');
history=plot(ax,NaN,NaN,'Color',[0.72 0.79 0.86],'LineWidth',1.5);
tail=plot(ax,NaN,NaN,'Color',[0.05 0.38 0.80],'LineWidth',3);
head=plot(ax,NaN,NaN,'o','MarkerSize',9,'MarkerFaceColor',[0.95 0.35 0.10], ...
    'MarkerEdgeColor','w','LineWidth',1.2);
status=text(ax,-260,650,'','FontName','Consolas','FontSize',11, ...
    'Color',[0.12 0.18 0.25],'VerticalAlignment','top');
burnMarker=plot(ax,NaN,NaN,'s','MarkerSize',7,'MarkerEdgeColor',[0.65 0.25 0.05]);
burnLabel=text(ax,0,0,'','FontSize',10,'Color',[0.6 0.25 0.05]);
gifPath=rocket_file('rocket_comet.gif');
for j=1:numel(times)
    recent=find(times(1:j)>=times(j)-tailSeconds);
    set(history,'XData',x(1:j),'YData',z(1:j));
    set(tail,'XData',x(recent),'YData',z(recent));
    set(head,'XData',x(j),'YData',z(j));
    phase='POWERED';
    if ~isnan(out.burnoutTime) && times(j)>=out.burnoutTime
        phase='COAST';
        bs=out.burnoutState;
        bx=bs(1)*sin(bs(2)-p.theta0)/1000;
        bz=(bs(1)*cos(bs(2)-p.theta0)-p.Re)/1000;
        set(burnMarker,'XData',bx,'YData',bz);
        set(burnLabel,'Position',[bx+15 bz 0], ...
            'String',sprintf('Burnout: %.2f s',out.burnoutTime));
    end
    set(status,'String',sprintf('t = %6.1f s\nh = %6.1f km\nv = %6.0f m/s\nm = %6.0f kg\n%s', ...
        times(j),height(j),speed(j),states(j,5),phase));
    drawnow;
    frame=getframe(fig);
    [indexed,map]=rgb2ind(frame.cdata,256);
    delay=frameDt/playbackRate;
    if j==numel(times), delay=1.5; end
    if j==1
        imwrite(indexed,map,gifPath,'gif','LoopCount',Inf,'DelayTime',delay);
    else
        imwrite(indexed,map,gifPath,'gif','WriteMode','append','DelayTime',delay);
    end
end
exportgraphics(fig,rocket_file('rocket_comet_final.png'),'Resolution',150);
fprintf('Saved %d frames: %s\n',numel(times),gifPath);
fprintf('Final x=%.3f km, z=%.3f km, altitude=%.3f km\n',x(end),z(end),height(end));
