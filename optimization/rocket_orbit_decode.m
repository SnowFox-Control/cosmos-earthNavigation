function [p,control] = rocket_orbit_decode(z,p,opt)
% Flow shape and consumed propellant fraction determine engine cutoff time.
f=opt.fractions;
control.q=interp1([0 .5 1],z(6:8),f,'linear');
burnTime=z(9)*(p.m0-p.mdry)/trapz(f,control.q);
control.t=f*burnTime;
control.delta=[0 0 z(1:5)]*pi/180;
p.tf=burnTime;
end
