function [p,opt] = rocket_orbit_config()
% Separate, upgraded educational vehicle for a 160 km circular insertion.
[p,~]=rocket_config();
p.m0=60000;                      % Initial mass, kg
p.propellantFraction=.98;        % Propellant / initial mass
p.mdry=p.m0*(1-p.propellantFraction);
p.Isp=4500/9.80665;              % Specific impulse, s (458.872...)
p.c=p.Isp*9.80665;               % Effective exhaust speed, m/s
assert(p.m0>0 && p.propellantFraction>0 && p.propellantFraction<1 && p.Isp>0);
opt.targetAltitude=600000;
opt.fractions=[0 .03 .10 .25 .50 .75 1];
opt.population=96;
opt.generations=200;
opt.seed=160;
opt.altitudeTolerance=500; % m
opt.velocityTolerance=10; % m/s in each component
% Five angles, three rates, and the fraction of available propellant used.
opt.lb=[0 -10 -10 -10 -10 180 180 180 .70];
opt.ub=[10 10 10 10 10 350 350 350 1];
end
