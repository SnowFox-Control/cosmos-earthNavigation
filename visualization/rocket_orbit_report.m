function rocket_orbit_report(result)
o=result.out; c=result.control;
fprintf('\nVERIFIED TERMINAL STATE\n');
fprintf('Time %.6f s; altitude %.6f km; radial speed %.6f m/s; tangential speed %.6f m/s\n', ...
    o.t(end),o.h(end)/1000,o.y(end,3),o.y(end,4));
fprintf('Residuals: altitude %.6f m, radial %.6f m/s, tangential %.6f m/s\n',result.residual);
fprintf('Insertion tolerances satisfied: %d\n',result.success);
fprintf('Elapsed %.1f s\n',result.elapsedSeconds);
disp(table(c.t(:),c.delta(:)*180/pi,c.q(:),'VariableNames',{'Time_s','Delta_deg','Flow_kg_s'}));
fig=figure('Color','w','Position',[80 80 1100 760]);
subplot(2,2,1); semilogy(result.history(:,1),max(result.history(:,2),eps),'LineWidth',1.5);
xlabel('Iteration'); ylabel('Best weighted squared error'); grid on;
subplot(2,2,2); plot(o.downrange/1000,o.h/1000,'LineWidth',1.5); hold on;
yline(result.opt.targetAltitude/1000,'--','Color',[.3 .3 .3]); xlabel('Surface downrange (km)'); ylabel('Altitude (km)'); grid on;
subplot(2,2,3); plot(o.t,o.y(:,3:4),'LineWidth',1.4); hold on;
yline(result.targetSpeed,'--','Color',[.3 .3 .3]); xlabel('Time (s)'); ylabel('Velocity (m/s)');
legend('Radial','Tangential','Target circular speed','Location','best'); grid on;
subplot(2,2,4); yyaxis left; plot(c.t,c.delta*180/pi,'-o'); ylabel('Thrust deviation (deg)');
yyaxis right; plot(c.t,c.q,'-s'); ylabel('Flow (kg/s)'); xlabel('Time (s)'); grid on;
set(findall(fig,'Type','axes'),'Color','w','XColor','k','YColor','k');
set(findall(fig,'Type','legend'),'Color','w','TextColor','k');
axs=findall(fig,'Type','axes');
for k=1:numel(axs), axs(k).Toolbar.Visible='off'; end
method='ga';
if isfield(result,'optimizer'),method=lower(result.optimizer);end
exportgraphics(fig,rocket_file(['rocket_orbit_' method '.png']),'Resolution',150);
writetable(table(c.t(:),c.delta(:)*180/pi,c.q(:), ...
    'VariableNames',{'Time_s','Delta_deg','Flow_kg_s'}),rocket_file('rocket_orbit_controls.csv'));
end
