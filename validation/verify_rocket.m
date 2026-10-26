function verify_rocket()
% Physical consistency checks and numerical refinement.
[p,c]=rocket_config();
p.noise.enabled=false; % Ideal identities/symmetry require an unperturbed model
o=rocket_simulate(p,c);
assert(abs(o.burnoutTime-(p.m0-p.mdry)/220)<1e-5);
assert(max(abs(o.y(:,6)-p.c*(p.m0-o.y(:,5))))<1);
assert(min(o.y(:,5))>=p.mdry-1e-6);
assert(all(diff(o.y(:,6))>=-1e-6));
% Radial symmetry: no turn command must leave tangential motion zero.
radial=c; radial.delta(:)=0;
r=rocket_simulate(p,radial);
assert(max(abs(r.y(:,4)))<1e-10);
% No gravity/drag: recover the Tsiolkovsky equation.
ideal=p; ideal.mu=0; ideal.rho0=0; ideal.tf=100;
i=rocket_simulate(ideal,radial);
expected=p.c*log(p.m0/p.mdry);
assert(abs(i.speed(end)-expected)<1e-3);
% Coasting in vacuum: specific energy and angular momentum are conserved.
vac=p; vac.rho0=0; vac.h0=200000; vac.u0=50; vac.w0=7800;
vac.tf=500; zero=c; zero.q(:)=0; zero.delta(:)=0;
v=rocket_simulate(vac,zero);
energy=0.5*v.speed.^2-vac.mu./v.y(:,1);
angmom=v.y(:,1).*v.y(:,4);
assert(max(abs((energy-energy(1))/energy(1)))<1e-7);
assert(max(abs((angmom-angmom(1))/angmom(1)))<1e-7);
% Touchdown stops integration, including when no propellant is consumed.
fall=p; fall.h0=100; fall.tf=20;
f=rocket_simulate(fall,zero);
assert(~isnan(f.impactTime) && abs(f.h(end))<1e-3);
% Refine integration and compare terminal position and velocity.
fine=p; fine.maxStep=p.maxStep/2; fine.relTol=p.relTol/10;
fine.absTol=p.absTol/10;
s=rocket_simulate(fine,c);
assert(abs(o.h(end)-s.h(end))<1);
assert(abs(o.speed(end)-s.speed(end))<0.1);
fprintf('All physical and numerical checks passed.\n');
fprintf('Refinement differences: altitude %.6g m; speed %.6g m/s\n', ...
    abs(o.h(end)-s.h(end)),abs(o.speed(end)-s.speed(end)));
end
