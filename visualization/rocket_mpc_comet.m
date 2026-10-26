function rocket_mpc_comet(v)
% Plain white geocentric polar-grid comets: MPC and open-loop launch together.
% Existing paired closed-loop data are used; optional post-cutoff flight coasts.
if nargin<1,v=rocket_comparison_visual_config();end
assert(v.frameRate>0 && v.ascentStepSeconds>0 && v.coastStepSeconds>0 && v.orbitPeriods>=0);
s=load(rocket_file('rocket_mpc_result.mat'),'result');r=s.result;p=r.plant;
assert(isequal(r.closedState(1,:),r.openState(1,:)), ...
    'Comparison requires identical launch states.');
cutoff=r.time(end);R=p.Re/1000;
targetRadius=r.referenceState(end,1);
planning=load(rocket_file(r.referenceFile),'result');
referenceMatches=norm(planning.result.out.y(end,1:5)-r.referenceState(end,:))<1e-4;
if isfield(r,'targetAltitude')
    targetRadius=p.Re+r.targetAltitude;
elseif referenceMatches
    targetRadius=p.Re+planning.result.opt.targetAltitude;
else
    warning('rocket:ReferenceChanged','Planning file changed; using saved MPC terminal radius for target display.');
end
if isfield(r,'burnSchedule')
    firstCutoff=r.burnSchedule.firstCutoff;restart=r.burnSchedule.restart;
    referenceControl=r.referenceControl;
else
    assert(referenceMatches,'PSO reference changed; rerun MPC to save its original burn schedule.');
    firstCutoff=planning.result.z(8);restart=planning.result.z(8)+planning.result.z(9);
    referenceControl=planning.result.control;
end
assert(cutoff>restart,'MPC data end before circularization; run a full MPC simulation.');
targetAltitude=targetRadius-p.Re;period=2*pi*sqrt(targetRadius^3/p.mu);
finish=cutoff+v.orbitPeriods*period;
noise=rocket_gaussian_disturbance(p,finish+1);
timeClosed=r.time;stateClosed=r.closedState;timeOpen=r.time;stateOpen=r.openState;
impactClosed=NaN;impactOpen=NaN;
if v.orbitPeriods>0
    [tc,yc,impactClosed]=extend(r.closedState(end,:));
    [to,yo,impactOpen]=extend(r.openState(end,:));
    timeClosed=[timeClosed;tc(2:end)];stateClosed=[stateClosed;yc(2:end,:)];
    timeOpen=[timeOpen;to(2:end)];stateOpen=[stateOpen;yo(2:end,:)];
end
[times,phaseId]=rocket_comet_timeline(firstCutoff,restart,cutoff,finish,v);
closed=interp1(timeClosed,stateClosed,min(times,timeClosed(end)),'pchip');
openSamples=interp1(timeOpen,stateOpen,min(times,timeOpen(end)),'pchip');
X=[closed(:,1).*cos(closed(:,2)),openSamples(:,1).*cos(openSamples(:,2))]/1000;
Y=[closed(:,1).*sin(closed(:,2)),openSamples(:,1).*sin(openSamples(:,2))]/1000;
altitude=[closed(:,1),openSamples(:,1)]/1000-R;
speed=[hypot(closed(:,3),closed(:,4)),hypot(openSamples(:,3),openSamples(:,4))];
nominalFlow=interp1(referenceControl.t,referenceControl.q,times,'linear',0);
nominalFlow=nominalFlow(:);
flowCorrection=interp1(r.time(1:end-1),r.corrections(:,2), ...
    min(times,r.time(end-1)),'previous',0);
flowCorrection=flowCorrection(:);
flow=[max(0,min(p.qMax,nominalFlow+flowCorrection)),nominalFlow];
flow(phaseId==2 | phaseId==4,:)=0;
flow(closed(:,5)<=p.mdry+1e-6,1)=0;
flow(openSamples(:,5)<=p.mdry+1e-6,2)=0;
assert(isequal(size(flow),[numel(times),2]),'Engine flow display must have one row per frame.');
assert(any(flow(phaseId==3,2)>0),'Circularization frames must show the second engine burn.');
animationData=struct('time',times,'closedState',closed,'openState',openSamples, ...
    'cutoff',cutoff,'closedImpactTime',impactClosed,'openImpactTime',impactOpen, ...
    'plant',p,'settings',v,'targetRadius',targetRadius,'targetAltitude',targetAltitude, ...
    'phaseId',phaseId,'flow',flow,'firstCutoff',firstCutoff,'restart',restart);
save(rocket_file([v.outputPrefix '_data.mat']),'animationData');
fig=figure('Name','MPC and open-loop comets','Color','w','Position',[60 60 1100 700]);
annotation(fig,'textbox',[.055 .925 .9 .055],'String','Simultaneous launch: MPC and open loop', ...
    'FontSize',17,'FontWeight','bold','EdgeColor','none','Color','k');
status=annotation(fig,'textbox',[.055 .84 .9 .075],'String','', ...
    'FontName','Consolas','FontSize',11,'EdgeColor','none','Color','k');
ax1=axes(fig,'Position',[.05 .20 .52 .61]);
ax2=axes(fig,'Position',[.66 .29 .30 .46]);
axs=[ax1 ax2];angles=linspace(0,2*pi,1000);
colors=[0 .35 .8;.85 .15 .1];markers={'o','^'};
history=gobjects(2,2);tails=gobjects(2,2);heads=gobjects(2,2);grids=gobjects(1,2);
conics=gobjects(2,2);target=gobjects(1,2);targetColor=[.15 .45 .15];
for k=1:2
    ax=axs(k);hold(ax,'on');axis(ax,'equal');axis(ax,'off');
    ax.Color='w';ax.Toolbar.Visible='off';
    grids(k)=plot(ax,NaN,NaN,'Color',[.8 .8 .8],'LineWidth',.5);
    plot(ax,R*cos(angles),R*sin(angles),'k-','LineWidth',1);
    target(k)=plot(ax,targetRadius/1000*cos(angles),targetRadius/1000*sin(angles), ...
        '-.','Color',targetColor,'LineWidth',1.1);
    for vehicle=1:2
        conics(k,vehicle)=plot(ax,NaN,NaN,'--','Color', ...
            v.conicLightness+(1-v.conicLightness)*colors(vehicle,:),'LineWidth',v.conicLineWidth);
        if ~v.showOsculatingOrbits,conics(k,vehicle).Visible='off';end
        history(k,vehicle)=plot(ax,NaN,NaN,'Color',.6+.4*colors(vehicle,:),'LineWidth',1);
        tails(k,vehicle)=plot(ax,NaN,NaN,'Color',colors(vehicle,:),'LineWidth',2.2);
        heads(k,vehicle)=plot(ax,NaN,NaN,markers{vehicle},'MarkerSize',8, ...
            'MarkerEdgeColor',colors(vehicle,:),'MarkerFaceColor','w','LineWidth',1.5);
    end
end
legend(ax1,[heads(1,:) conics(1,:) target(1)], ...
    {'MPC rocket','Open-loop rocket','MPC: thrust lost now','Open loop: thrust lost now', ...
    sprintf('Target: %.0f km circular',targetAltitude/1000)},'Location','southoutside', ...
    'NumColumns',2,'TextColor','k','Color','w','AutoUpdate','off','FontSize',9);
plot(ax1,0,0,'k+');text(ax1,200,-300,'O','Color','k');
outer=max([R+800;max(hypot(X,Y),[],'all')+300]);
grid1Limit=outer*1.05;
drawGrid(1,[-grid1Limit grid1Limit],[-grid1Limit grid1Limit],true);
for deg=0:45:315
    text(ax1,outer*cosd(deg),outer*sind(deg),sprintf('%d%c',deg,176), ...
        'HorizontalAlignment','center','FontSize',9,'Color',[.3 .3 .3]);
end
for radius=[2000 4000 R]
    text(ax1,radius*cosd(135),radius*sind(135),sprintf('r=%g km',radius), ...
        'HorizontalAlignment','center','FontSize',9,'Color',[.35 .35 .35]);
end
text(ax1,targetRadius/1000*cosd(120),targetRadius/1000*sind(120), ...
    sprintf('Target %.0f km',targetAltitude/1000),'Color',targetColor,'FontSize',9);
title(ax1,'Earth-centered polar view','Color','k');
title(ax2,'Shared tracking close-up','Color','k');
detail=annotation(fig,'textbox',[.64 .16 .34 .10],'String','', ...
    'FontSize',10,'EdgeColor','none','Color','k');
footer=annotation(fig,'textbox',[.055 .045 .9 .08],'String','', ...
    'FontSize',10,'EdgeColor','none','Color','k');
gifPath=rocket_file([v.outputPrefix '.gif']);
if v.writeMp4
    writer=VideoWriter(rocket_file([v.outputPrefix '.mp4']),'MPEG-4');
    writer.FrameRate=v.frameRate;writer.Quality=92;open(writer);
    cleanup=onCleanup(@()close(writer));
end
for j=1:numel(times)
    t=times(j);
    if t<=cutoff,tailSeconds=v.ascentTailSeconds;rate=v.ascentStepSeconds*v.frameRate;
    else,tailSeconds=v.coastTailSeconds;rate=v.coastStepSeconds*v.frameRate;end
    if phaseId(j)==2,rate=v.transferStepSeconds*v.frameRate;end
    if phaseId(j)==3,rate=v.circularizationStepSeconds*v.frameRate;end
    recent=find(times(1:j)>=t-tailSeconds);
    orbitClosed=rocket_osculating_conic(closed(j,:),p,2*outer*1000);
    orbitOpen=rocket_osculating_conic(openSamples(j,:),p,2*outer*1000);
    for k=1:2
        set(conics(k,1),'XData',orbitClosed.x,'YData',orbitClosed.y);
        set(conics(k,2),'XData',orbitOpen.x,'YData',orbitOpen.y);
        for vehicle=1:2
            set(history(k,vehicle),'XData',X(1:j,vehicle),'YData',Y(1:j,vehicle));
            set(tails(k,vehicle),'XData',X(recent,vehicle),'YData',Y(recent,vehicle));
            set(heads(k,vehicle),'XData',X(j,vehicle),'YData',Y(j,vehicle));
            if flow(j,vehicle)>1e-6,set(heads(k,vehicle),'MarkerFaceColor',colors(vehicle,:));
            else,set(heads(k,vehicle),'MarkerFaceColor','w');end
        end
    end
    % Both heads and recent tails determine one common, equal-scale viewport.
    span=max([v.closeupMinimumKm, ...
        v.closeupMargin*(max(X(recent,:),[],'all')-min(X(recent,:),[],'all')), ...
        v.closeupMargin*(max(Y(recent,:),[],'all')-min(Y(recent,:),[],'all'))]);
    cx=(max(X(recent,:),[],'all')+min(X(recent,:),[],'all'))/2;
    cy=(max(Y(recent,:),[],'all')+min(Y(recent,:),[],'all'))/2;
    drawGrid(2,cx+[-.5 .5]*span,cy+[-.5 .5]*span,false);
    if phaseId(j)==1,phase='BURN 1: suborbital ascent';
    elseif phaseId(j)==2,phase='TRANSFER COAST: waiting for second ignition';
    elseif phaseId(j)==3,phase=sprintf('BURN 2: CIRCULARIZATION (%.1f / %.1f s)',t-restart,cutoff-restart);
    else,phase='ENGINE OFF: orbital coast';end
    set(status,'String',sprintf(['t = %7.1f s  |  %s\n' ...
        'MPC: h=%7.2f km, v=%7.1f m/s, q=%6.1f kg/s | Open: h=%7.2f km, v=%7.1f m/s, q=%6.1f kg/s'], ...
        t,phase,altitude(j,1),speed(j,1),flow(j,1),altitude(j,2),speed(j,2),flow(j,2)));
    separation=hypot(X(j,1)-X(j,2),Y(j,1)-Y(j,2));
    set(detail,'String',sprintf('Separation: %.2f km | close-up: %.0f km\nTarget h = %.0f km, r = %.0f km\nFaint dashes: instantaneous gravity-only conics', ...
        separation,span,targetAltitude/1000,targetRadius/1000));
    impactNote='';
    if ~isnan(impactClosed) && t>=impactClosed,impactNote='MPC rocket reached the surface.';end
    if ~isnan(impactOpen) && t>=impactOpen,impactNote=[impactNote ' Open-loop rocket reached the surface.'];end
    set(footer,'String',sprintf('Playback %gx | Tail: last %.0f s | Green dash-dot: target orbit\nFaint conics assume zero thrust, drag and random forcing from the current state. %s',rate,tailSeconds,impactNote));
    drawnow;frame=getframe(fig);rgb=imresize(frame.cdata,[700 1100]);
    delay=1/v.frameRate;if abs(t-cutoff)<1e-7 || j==numel(times),delay=2;end
    if abs(t-restart)<1e-7,delay=v.ignitionPauseSeconds;end
    if v.writeGif
        [indexed,map]=rgb2ind(rgb,128);
        if j==1,imwrite(indexed,map,gifPath,'gif','LoopCount',Inf,'DelayTime',delay);
        else,imwrite(indexed,map,gifPath,'gif','WriteMode','append','DelayTime',delay);end
    end
    if v.writeMp4,for repeat=1:round(delay*v.frameRate),writeVideo(writer,rgb);end,end
    if abs(t-cutoff)<1e-7,imwrite(rgb,rocket_file([v.outputPrefix '_cutoff.png']));end
    if abs(t-(restart+cutoff)/2)<1e-7,imwrite(rgb,rocket_file([v.outputPrefix '_burn2.png']));end
    if j==numel(times),imwrite(rgb,rocket_file([v.outputPrefix '_final.png']));end
    if mod(j,80)==0,fprintf('Dual comet %d/%d frames\n',j,numel(times));end
end
fprintf('Saved dual-rocket comet: %d frames, %.2f min simulated.\n',numel(times),finish/60);
    function [t,y,impact]=extend(initial)
        opts=odeset('RelTol',1e-10,'AbsTol',[1e-5 1e-12 1e-7 1e-7 1e-7 1e-3], ...
            'MaxStep',min(2,noise.maxStep),'Events',@ground);
        [t,y,te]=ode45(@coast,[cutoff finish],initial(:),opts);
        impact=NaN;if ~isempty(te),impact=te(end);end
    end
    function dy=coast(t,x)
        a=noise.evaluate(t,x(1)-p.Re);dy=rocket_dynamics(t,x,p,0,0,a);
    end
    function [value,terminal,direction]=ground(~,x)
        value=x(1)-p.Re;terminal=1;direction=-1;
    end
    function drawGrid(k,xl,yl,globalView)
        xlim(axs(k),xl);ylim(axs(k),yl);
        if globalView
            radii=[2000 4000 R outer];rays=0:pi/4:2*pi;arc=angles;
        else
            center=[mean(xl),mean(yl)];radius=hypot(center(1),center(2));angle=atan2(center(2),center(1));
            step=max(5,25*ceil(diff(xl)/150));
            radii=(floor((radius-diff(xl))/step):ceil((radius+diff(xl))/step))*step;
            da=max(.1,ceil(diff(xl)/R*180/pi/4)*.25)*pi/180;
            rays=(floor((angle-diff(xl)/R)/da):ceil((angle+diff(xl)/R)/da))*da;
            arc=linspace(angle-2*diff(xl)/R,angle+2*diff(xl)/R,300);
        end
        gx=[];gy=[];
        for radius=radii,gx=[gx radius*cos(arc) NaN];gy=[gy radius*sin(arc) NaN];end
        for ray=rays,gx=[gx 0 2*outer*cos(ray) NaN];gy=[gy 0 2*outer*sin(ray) NaN];end
        set(grids(k),'XData',gx,'YData',gy);
    end
end
