function u=rocket_mpc_command(time,correction,p,ref,cfg)
% Continuous reference feedforward plus held MPC correction, physical bounds.
u=ref.command(time)+correction(:);
u(1)=max(-p.deltaMax,min(p.deltaMax,u(1)));
u(2)=max(0,min(p.qMax,u(2)));
if time>=ref.cutoff || (cfg.preserveBurnSchedule && time>=ref.firstCutoff && time<ref.restart)
    u=[0;0];
end
end
