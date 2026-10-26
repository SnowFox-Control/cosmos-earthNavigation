function verify_orbit()
% Independently check insertion residuals and propagate an unpowered orbit.
s=load(rocket_file('rocket_orbit_final.mat'),'result'); result=s.result;
assert(result.success,'The saved trajectory does not meet insertion tolerances.');
p=result.p; state=result.out.y(end,:);
period=2*pi*sqrt((p.Re+result.opt.targetAltitude)^3/p.mu);
disturbance=rocket_gaussian_disturbance(p,result.out.t(end)+period);
opts=odeset('RelTol',1e-10,'AbsTol',[1e-5 1e-12 1e-7 1e-7],'MaxStep',min(2,disturbance.maxStep));
[t,y]=ode45(@coast,[0 period],state(1:4).',opts);
coastResult=struct('t',t,'y',y,'period',period,'altitude',y(:,1)-p.Re);
save(rocket_file('rocket_orbit_coast.mat'),'coastResult');
fig=figure('Color','w','Position',[100 100 1000 440]);
subplot(1,2,1);
angle=linspace(0,2*pi,400);
fill(p.Re*cos(angle)/1000,p.Re*sin(angle)/1000,[.88 .93 .98],'EdgeColor',[.4 .5 .6]); hold on;
plot(y(:,1).*cos(y(:,2))/1000,y(:,1).*sin(y(:,2))/1000,'LineWidth',1.5);
axis equal; grid on; xlabel('Inertial x (km)'); ylabel('Inertial y (km)'); title('Unpowered orbit after insertion');
subplot(1,2,2);plot(t/60,coastResult.altitude/1000,'LineWidth',1.5);
xlabel('Time after cutoff (min)');ylabel('Altitude (km)');grid on;title('Gravity and atmospheric drag retained');
set(findall(fig,'Type','axes'),'Color','w','XColor','k','YColor','k');
exportgraphics(fig,rocket_file('rocket_orbit_coast.png'),'Resolution',150);
fprintf('Unpowered nominal period %.3f s; altitude range %.6f .. %.6f km\n', ...
    period,min(coastResult.altitude)/1000,max(coastResult.altitude)/1000);
assert(min(coastResult.altitude)>result.opt.targetAltitude-10000,'Unexpected orbital decay or incorrect insertion.');
    function dy=coast(vTime,v)
        r=v(1); u=v(3); w=v(4); speed=hypot(u,w);
        k=.5*p.rho0*exp(-(r-p.Re)/p.H)*p.Cd*p.A;
        noise=disturbance.evaluate(result.out.t(end)+vTime,r-p.Re);
        dy=[u;w/r;w*w/r-p.mu/r^2-k*speed*u/state(5)+noise(1); ...
            -u*w/r-k*speed*w/state(5)+noise(2)];
    end
end
