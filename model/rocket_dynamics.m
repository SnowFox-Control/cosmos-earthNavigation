function dy=rocket_dynamics(time,state,p,delta,q,acceleration)
% Shared nonlinear model for PSO integration, MPC prediction and actual plant.
% State: [r theta u w m] with optional sixth engine impulse state.
if nargin<6,acceleration=[0 0];end
r=state(1);u=state(3);w=state(4);m=state(5);speed=hypot(u,w);
k=.5*p.rho0*exp(-max(r-p.Re,0)/p.H)*p.Cd*p.A;
beta=0;if speed>1e-9,beta=atan2(w,u);end
alpha=beta+delta;if time<p.turnStart,alpha=0;end
thrust=p.c*q;
dy=[u;w/r;w*w/r+thrust*cos(alpha)/m-p.mu/r^2-k*speed*u/m+acceleration(1); ...
    -u*w/r+thrust*sin(alpha)/m-k*speed*w/m+acceleration(2);-q];
if numel(state)==6,dy(6)=thrust;end
end
