% Run this script from its folder. No additional MATLAB toolbox is needed.
clear; clc;
[p,control] = rocket_config();
out = rocket_simulate(p,control);
fprintf('Total impulse: %.6f MN s\n',out.totalImpulse/1e6);
fprintf('Propellant used: %.3f kg\n',out.propellantUsed);
fprintf('Burnout time: %.6f s (NaN means not reached)\n',out.burnoutTime);
fprintf('Terminal time: %.3f s; altitude: %.3f km; speed: %.3f m/s\n', ...
    out.t(end),out.h(end)/1000,out.speed(end));
if ~isempty(out.burnoutState)
    fprintf('Burnout altitude: %.3f km; speed: %.3f m/s\n', ...
        (out.burnoutState(1)-p.Re)/1000,hypot(out.burnoutState(3),out.burnoutState(4)));
end
figure('Name','Rocket ascent','Color','w');
subplot(3,2,1); plot(out.downrange/1000,out.h/1000,'LineWidth',1.4);
xlabel('Surface downrange (km)'); ylabel('Altitude (km)'); grid on;
subplot(3,2,2); plot(out.t,[out.y(:,3),out.y(:,4),out.speed],'LineWidth',1.2);
xlabel('Time (s)'); ylabel('Velocity (m/s)'); legend('Radial','Tangential','Speed'); grid on;
subplot(3,2,3); plot(out.t,out.y(:,5),'LineWidth',1.4);
xlabel('Time (s)'); ylabel('Mass (kg)'); grid on;
subplot(3,2,4); yyaxis left; plot(out.t,out.thrust/1000); ylabel('Thrust (kN)');
yyaxis right; plot(out.t,out.y(:,6)/1e6); ylabel('Impulse (MN s)'); xlabel('Time (s)'); grid on;
subplot(3,2,5); plot(out.t,[out.gamma,out.delta]*180/pi,'LineWidth',1.2);
xlabel('Time (s)'); ylabel('Angle (deg)'); legend('Flight path angle','Thrust deviation'); grid on;
subplot(3,2,6); yyaxis left; plot(out.t,out.q); ylabel('Propellant rate (kg/s)');
yyaxis right; plot(out.t,out.dynamicPressure/1000); ylabel('Dynamic pressure (kPa)');
xlabel('Time (s)'); grid on;
save(rocket_file('rocket_result.mat'),'out','p','control');
set(findall(gcf,'Type','axes'),'Color','w','XColor','k','YColor','k');
set(findall(gcf,'Type','legend'),'Color','w','TextColor','k','EdgeColor',[0.3 0.3 0.3]);
exportgraphics(gcf,rocket_file('rocket_demo.png'),'Resolution',150);
