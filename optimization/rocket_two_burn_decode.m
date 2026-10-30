function [p,control,z]=rocket_two_burn_decode(unit,p,opt)
% One parameterization shared by every optimizer and independent verification.
z=opt.lb+unit(:).'.*(opt.ub-opt.lb);
t1=z(8);restart=t1+z(9);finish=restart+z(10);ramp=opt.rampSeconds;
control.t=[0 p.turnStart .2*t1 .6*t1 t1-ramp t1 restart ...
    restart+ramp restart+.5*z(10) finish-ramp finish];
control.q=[z(6)*ones(1,5) 0 0 z(7)*ones(1,3) 0];
control.delta=[0 0 z(1:3) z(3) z(4) z(4) (z(4)+z(5))/2 z(5) z(5)]*pi/180;
p.tf=finish;
end
