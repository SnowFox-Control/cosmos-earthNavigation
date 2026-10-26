function x=rocket_mpc_step(time,x,correction,p,ref,cfg,step)
% Nominal zero-mean disturbance prediction using the shared nonlinear model.
h=step/cfg.rkSubsteps;
for j=1:cfg.rkSubsteps
    tj=time+(j-1)*h;
    k1=rhs(tj,x);k2=rhs(tj+h/2,x+h*k1/2);
    k3=rhs(tj+h/2,x+h*k2/2);k4=rhs(tj+h,x+h*k3);
    x=x+h*(k1+2*k2+2*k3+k4)/6;
end
    function dx=rhs(t,state)
        u=rocket_mpc_command(t,correction,p,ref,cfg);
        dx=rocket_dynamics(t,state,p,u(1),u(2));
    end
end
