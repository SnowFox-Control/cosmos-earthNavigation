function rocket_polar_comet(v)
% Geocentric polar grid, radial color gradient, one post-insertion revolution.
% Rendering uses equal Cartesian projections of polar geometry, no x/y grid.
if nargin<1,v=rocket_visual_config();end
assert(v.frameRate>0 && v.ascentStepSeconds>0 && v.orbitStepSeconds>0 && v.orbitPeriods>0);
assert(v.gradientScaleKm>0 && v.closeupInitialKm>0 && v.closeupFinalKm>0);
s=load(rocket_file('rocket_two_burn_result.mat'),'result');r=s.result;p=r.p;
o=r.out; cutoff=o.t(end); period=2*pi*sqrt((p.Re+r.opt.targetAltitude)^3/p.mu);
disturbance=rocket_gaussian_disturbance(p,cutoff+v.orbitPeriods*period);
opts=odeset('RelTol',1e-10,'AbsTol',[1e-5 1e-12 1e-7 1e-7],'MaxStep',min(2,disturbance.maxStep));
[ct,cy]=ode45(@coast,[0 v.orbitPeriods*period],o.y(end,1:4).',opts);
allT=[o.t;cutoff+ct(2:end)]; allY=[o.y(:,1:4);cy(2:end,:)];
times=[0:v.ascentStepSeconds:cutoff cutoff cutoff+(v.orbitStepSeconds:v.orbitStepSeconds:ct(end)) cutoff+ct(end)];
times=unique(times); states=interp1(allT,allY,times,'pchip');
theta=states(:,2); rad=states(:,1)/1000;
xx=rad.*cos(theta); yy=rad.*sin(theta);
% Standard polar convention: theta=0 to the right, counterclockwise positive.
R=p.Re/1000; background=v.deepColor; light=[.82 .89 .95];
fig=figure('Name','Geocentric orbit comet','Color',background,'Position',[60 60 1100 700]);
annotation(fig,'textbox',[.045 .92 .92 .065],'String',sprintf('EARTH-CENTERED | ascent + %g orbital revolution(s)',v.orbitPeriods), ...
    'Color',light,'FontSize',18,'FontWeight','bold','EdgeColor','none');
status=annotation(fig,'textbox',[.045 .855 .92 .06],'String','', ...
    'Color',light,'FontSize',12,'EdgeColor','none');
ax1=axes(fig,'Position',[.045 .19 .53 .65]);
ax2=axes(fig,'Position',[.65 .36 .31 .46]);
axs=[ax1 ax2]; bg=gobjects(1,2); earth=gobjects(1,2); gridline=gobjects(1,2);
hist=gobjects(1,2);tail=gobjects(1,2);head=gobjects(1,2);orbit=gobjects(1,2);
angles=linspace(0,2*pi,1000);
for k=1:2
    ax=axs(k);hold(ax,'on');axis(ax,'equal');axis(ax,'off');ax.Toolbar.Visible='off';
    bg(k)=image(ax,'XData',[-7600 7600],'YData',[-7600 7600], ...
        'CData',zeros(2,2,3));set(ax,'YDir','normal');
    gridline(k)=plot(ax,NaN,NaN,'Color',[.18 .26 .34],'LineWidth',.5);
    earth(k)=plot(ax,R*cos(angles),R*sin(angles),'Color',[.35 .65 .65],'LineWidth',1.1);
    orbit(k)=plot(ax,NaN,NaN,'--','Color',v.orbitColor,'LineWidth',.8);
    if ~v.showOsculatingOrbit,orbit(k).Visible='off';end
    hist(k)=plot(ax,NaN,NaN,'Color',[.35 .58 .69],'LineWidth',1.1);
    tail(k)=plot(ax,NaN,NaN,'Color',[.25 .85 1],'LineWidth',2.7);
    head(k)=plot(ax,NaN,NaN,'o','MarkerSize',7,'MarkerEdgeColor','w','LineWidth',.7);
end
plot(ax1,0,0,'+','Color',light);text(ax1,250,-300,'O','Color',light,'FontSize',10);
for deg=0:45:315
    text(ax1,7270*cosd(deg),7270*sind(deg),sprintf('%d%c',deg,176), ...
        'Color',[.52 .63 .72],'HorizontalAlignment','center','FontSize',9);
end
for radius=[2000 4000 6371]
    text(ax1,radius*cosd(135),radius*sind(135),sprintf('r = %g km',radius), ...
        'Color',[.48 .63 .67],'FontSize',9,'HorizontalAlignment','center');
end
title(ax1,'Geocentric polar overview','Color',light);
title(ax2,'Tracking polar-sector close-up','Color',light);
detail=annotation(fig,'textbox',[.65 .20 .32 .12],'String','', ...
    'Color',light,'FontSize',11,'EdgeColor','none');
annotation(fig,'textbox',[.045 .08 .92 .08],'String', ...
    {'Cyan: recent motion   |   muted dashed curve: instantaneous osculating orbit', ...
     'Dashed orbit assumes gravity only from the current state; actual propagation retains thrust and drag.'}, ...
    'Color',[.57 .69 .78],'FontSize',10,'EdgeColor','none');
progress=annotation(fig,'textbox',[.045 .025 .92 .05],'String','', ...
    'Color',[.69 .79 .85],'FontSize',10,'EdgeColor','none');
setBackground(1,[-7600 7600],[-7600 7600]);
setPolarGrid(1,[-7600 7600],[-7600 7600]);
gifPath=rocket_file([v.outputPrefix '_comet.gif']);
if v.writeMp4
    writer=VideoWriter(rocket_file([v.outputPrefix '_comet.mp4']),'MPEG-4');
    writer.FrameRate=v.frameRate;writer.Quality=92;open(writer);
    cleanup=onCleanup(@()close(writer));
end
for j=1:numel(times)
    t=times(j); h=rad(j)-R; speedNow=hypot(states(j,3),states(j,4));
    if t<r.z(8),phase='SUBORBITAL BURN';col=[1 .49 .2];
    elseif t<r.z(8)+r.z(9),phase='COAST TO SECOND BURN';col=[.3 .9 .7];
    elseif t<cutoff,phase='CIRCULARIZATION BURN';col=[.85 .5 1];
    else,phase='ENGINE OFF | ORBITAL FLIGHT';col=[.4 .95 .75];end
    set(status,'String',sprintf('t = %7.1f s    h = %7.3f km    v = %7.1f m/s    %s',t,h,speedNow,phase));
    if t<=cutoff,tailTime=v.ascentTailSeconds;playback=v.ascentStepSeconds*v.frameRate;
    else,tailTime=v.orbitTailSeconds;playback=v.orbitStepSeconds*v.frameRate;end
    recent=find(times(1:j)>=t-tailTime);
    % Osculating conic in inertial coordinates, evaluated from current r,v.
    position=states(j,1)*[cos(theta(j));sin(theta(j))];
    velocity=states(j,3)*[cos(theta(j));sin(theta(j))]+states(j,4)*[-sin(theta(j));cos(theta(j))];
    ev=((dot(velocity,velocity)-p.mu/states(j,1))*position-dot(position,velocity)*velocity)/p.mu;
    eccentricity=norm(ev); longitude=atan2(ev(2),ev(1));
    semilatus=(states(j,1)*states(j,4))^2/p.mu;
    nu=linspace(-pi,pi,1600); denominator=1+eccentricity*cos(nu);
    rr=semilatus./denominator/1000;
    rr(denominator<=0 | rr<R | rr>18000)=NaN;
    ox=rr.*cos(nu+longitude); oy=rr.*sin(nu+longitude);
    for k=1:2
        set(hist(k),'XData',xx(1:j),'YData',yy(1:j));
        set(tail(k),'XData',xx(recent),'YData',yy(recent));
        set(head(k),'XData',xx(j),'YData',yy(j),'MarkerFaceColor',col);
        set(orbit(k),'XData',ox,'YData',oy);
    end
    % Keep the planet overview fixed; smoothly grow the close-up during ascent.
    if t<=cutoff,span=v.closeupInitialKm+(v.closeupFinalKm-v.closeupInitialKm)*(t/cutoff)^1.3;
    else,span=v.closeupFinalKm;end
    xl=xx(j)+[-.5 .5]*span;yl=yy(j)+[-.5 .5]*span;
    setBackground(2,xl,yl);setPolarGrid(2,xl,yl);
    set(detail,'String',sprintf('r = %.3f km\n%c = %.2f%c    e = %.6f\nClose-up width: %.0f km', ...
        rad(j),952,mod(theta(j)*180/pi,360),176,eccentricity,span));
    set(progress,'String',sprintf('Playback: %gx  |  coast after insertion: %.1f / %.1f min  |  radial gradient is illustrative', ...
        playback,max(0,t-cutoff)/60,ct(end)/60));
    drawnow;
    frame=getframe(fig);rgb=imresize(frame.cdata,[700 1100]);
    [indexed,map]=rgb2ind(rgb,128);
    delay=1/v.frameRate;if abs(t-cutoff)<1e-7,delay=2;elseif j==numel(times),delay=2;end
    if v.writeGif
        if j==1,imwrite(indexed,map,gifPath,'gif','LoopCount',Inf,'DelayTime',delay);
        else,imwrite(indexed,map,gifPath,'gif','WriteMode','append','DelayTime',delay);end
    end
    if v.writeMp4,for repeat=1:round(delay*v.frameRate),writeVideo(writer,rgb);end,end
    if abs(t-cutoff)<1e-7,imwrite(rgb,rocket_file([v.outputPrefix '_insertion.png']));end
    if j==80,imwrite(rgb,rocket_file([v.outputPrefix '_ascent.png']));end
    if j==numel(times),imwrite(rgb,rocket_file([v.outputPrefix '_final.png']));end
    if mod(j,60)==0,fprintf('Rendered %d / %d frames\n',j,numel(times));end
end
fprintf('Completed %.2f min of simulated flight, %d frames.\n',times(end)/60,numel(times));
    function dy=coast(coastTime,state)
        radm=state(1);u=state(3);w=state(4);speed=hypot(u,w);
        k=.5*p.rho0*exp(-(radm-p.Re)/p.H)*p.Cd*p.A;
        mass=o.y(end,5);
        noise=disturbance.evaluate(cutoff+coastTime,radm-p.Re);
        dy=[u;w/radm;w*w/radm-p.mu/radm^2-k*speed*u/mass+noise(1); ...
            -u*w/radm-k*speed*w/mass+noise(2)];
    end
    function setBackground(k,xl,yl)
        [gx,gy]=meshgrid(linspace(xl(1),xl(2),450),linspace(yl(1),yl(2),450));
        altitude=hypot(gx,gy)-R; blend=exp(-max(altitude,0)/v.gradientScaleKm);
        near=v.nearColor;deep=background;groundColor=v.groundColor;data=zeros(450,450,3);
        for channel=1:3
            color=deep(channel)+(near(channel)-deep(channel))*blend;
            ground=altitude<0;color(ground)=groundColor(channel);
            data(:,:,channel)=color;
        end
        set(bg(k),'XData',xl,'YData',yl,'CData',data);xlim(axs(k),xl);ylim(axs(k),yl);
    end
    function setPolarGrid(k,xl,yl)
        if k==1,radii=[2000 4000 R 7000];rays=0:pi/4:2*pi;
        else
            center=[mean(xl),mean(yl)];radius=hypot(center(1),center(2));
            span=diff(xl);step=max(5,25*ceil(span/150));
            radii=(floor((radius-span)/step):ceil((radius+span)/step))*step;
            a=atan2(center(2),center(1));da=max(.1,ceil(span/R*180/pi/4)*.25)*pi/180;
            rays=(floor((a-span/R)/da):ceil((a+span/R)/da))*da;
        end
        gx=[];gy=[];
        for radius=radii
            if k==1,aa=angles;else,aa=linspace(a-2*span/R,a+2*span/R,300);end
            gx=[gx radius*cos(aa) NaN];gy=[gy radius*sin(aa) NaN];
        end
        for aRay=rays
            gx=[gx 0 16000*cos(aRay) NaN];gy=[gy 0 16000*sin(aRay) NaN];
        end
        set(gridline(k),'XData',gx,'YData',gy);
    end
end
