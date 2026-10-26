function orbit=rocket_osculating_conic(state,p,maxRadius)
% Instantaneous gravity-only conic from polar [r theta u w ...] state.
% maxRadius is a display clipping radius (m), not a physical constraint.
if nargin<3,maxRadius=20*p.Re;end
r=state(1);theta=state(2);u=state(3);w=state(4);
energy=.5*(u*u+w*w)-p.mu/r;ell=r*w;
eLocal=[r*w*w/p.mu-1;-r*u*w/p.mu];
% e vector in inertial coordinates: er*e_r + etheta*e_theta.
ev=[cos(theta) -sin(theta);sin(theta) cos(theta)]*eLocal;
ecc=norm(ev);longitude=atan2(ev(2),ev(1));
orbit=struct('eccentricity',ecc,'energy',energy,'angularMomentum',ell, ...
    'x',[],'y',[],'degenerate',false);
if abs(ell)<1e-10*sqrt(p.mu*r)
    orbit.degenerate=true;
    if energy<0,top=min(maxRadius,-p.mu/energy);else,top=maxRadius;end
    radii=sort(unique([linspace(p.Re,max(p.Re,top),200),r]));
    orbit.x=radii*cos(theta)/1000;orbit.y=radii*sin(theta)/1000;return;
end
semilatus=ell^2/p.mu;
currentNu=atan2(sin(theta-longitude),cos(theta-longitude));
% Include current anomaly exactly so the rendered conic passes through the head.
nu=sort(unique([linspace(-pi,pi,3000),currentNu]));
denominator=1+ecc*cos(nu);radii=semilatus./denominator;
mask=denominator<=0 | radii<p.Re | radii>maxRadius;
radii(mask)=NaN;
orbit.x=radii.*cos(nu+longitude)/1000;
orbit.y=radii.*sin(nu+longitude)/1000;
end
