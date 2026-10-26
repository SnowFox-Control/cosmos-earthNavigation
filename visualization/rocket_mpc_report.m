function rocket_mpc_report(r)
t=r.time;p=r.p;a=r.closedState;b=r.openState;x=r.referenceState;
fig=figure('Color','w','Position',[80 80 1150 780]);
subplot(2,2,1);plot(p.Re*(x(:,2)-x(1,2))/1000,(x(:,1)-p.Re)/1000,'k--','LineWidth',1.2);hold on;
plot(p.Re*(b(:,2)-x(1,2))/1000,(b(:,1)-p.Re)/1000,'Color',[.85 .4 .2]);
plot(p.Re*(a(:,2)-x(1,2))/1000,(a(:,1)-p.Re)/1000,'LineWidth',1.4);
xlabel('Downrange (km)');ylabel('Altitude (km)');legend('PSO reference','Open loop','MPC','Location','best');grid on;
subplot(2,2,2);plot(t,[a(:,1)-x(:,1),b(:,1)-x(:,1)],'LineWidth',1.2);
xlabel('Time (s)');ylabel('Radius tracking error (m)');legend('MPC','Open loop','Location','best');grid on;
subplot(2,2,3);plot(t,[hypot(a(:,3)-x(:,3),a(:,4)-x(:,4)), ...
    hypot(b(:,3)-x(:,3),b(:,4)-x(:,4))],'LineWidth',1.2);
xlabel('Time (s)');ylabel('Velocity tracking error (m/s)');legend('MPC','Open loop','Location','best');grid on;
subplot(2,2,4);yyaxis left;stairs(t(1:end-1),r.corrections(:,1)*180/pi);ylabel('Angle correction (deg)');
yyaxis right;stairs(t(1:end-1),r.corrections(:,2));ylabel('Flow correction (kg/s)');xlabel('Time (s)');grid on;
set(findall(fig,'Type','axes'),'Color','w','XColor','k','YColor','k');
set(findall(fig,'Type','legend'),'Color','w','TextColor','k');
exportgraphics(fig,rocket_file('rocket_mpc_tracking.png'),'Resolution',150);
end
