function verify_disturbance()
% Check density scaling, Gaussian statistics, repeatability and ODE convergence.
[p,c]=rocket_config();
p.tf=12;p.u0=20;p.w0=1;
originalRng=rng;
f=rocket_gaussian_disturbance(p,p.tf);
assert(isequal(originalRng,rng),'Noise must not consume the optimizer RNG.');
t=(0:.25:10).';
a=f.evaluate(t,zeros(size(t)));
b=f.evaluate(t,p.H*ones(size(t)));
assert(max(abs(b(:)-exp(-1)*a(:)))<1e-12);
long=rocket_gaussian_disturbance(p,50);
assert(isequal(a,long.evaluate(t,zeros(size(t)))),'Time-horizon prefixes must agree.');
other=p;other.noise.seed=p.noise.seed+1;
g=rocket_gaussian_disturbance(other,p.tf);
assert(any(abs(a-g.evaluate(t,zeros(size(t))))>0,'all'));
% Test pointwise standard deviations at knots AND halfway between knots.
stats=rocket_gaussian_disturbance(p,30000);
for offset=[0 .5]
    samples=stats.evaluate((0:29999).'+offset,zeros(30000,1));
    assert(all(abs(mean(samples))<.03*p.noise.sigma0));
    assert(all(abs(std(samples)-p.noise.sigma0)<.03*p.noise.sigma0));
end
vac=p;vac.rho0=0;f0=rocket_gaussian_disturbance(vac,20);
assert(all(f0.evaluate(t,zeros(size(t)))==0,'all'));
quiet=p;quiet.noise.enabled=false;fq=rocket_gaussian_disturbance(quiet,20);
assert(all(fq.evaluate(t,zeros(size(t)))==0,'all'));
o=rocket_simulate(p,c);same=rocket_simulate(p,c);
assert(isequal(o.t,same.t) && isequal(o.y,same.y));
assert(max(abs(o.y(:,6)-p.c*(p.m0-o.y(:,5))))<1);
fine=p;fine.relTol=1e-10;fine.absTol=p.absTol/100;fine.maxStep=.1;
v=rocket_simulate(fine,c);
assert(abs(v.h(end)-o.h(end))<.1);
assert(abs(v.speed(end)-o.speed(end))<.02);
assert(any(o.disturbanceAcceleration~=0,'all'));
fprintf('Gaussian density scaling, statistics, RNG isolation and ODE convergence passed.\n');
end
