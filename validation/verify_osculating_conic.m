function verify_osculating_conic()
[p,~]=rocket_config();r=p.Re+600000;angle=.3;
circle=rocket_osculating_conic([r angle 0 sqrt(p.mu/r)],p,4*p.Re);
assert(circle.eccentricity<1e-12);
assert(max(abs(hypot(circle.x,circle.y)*1000-r))<1e-5);
for speeds=[400 6000;300 12000].'
    state=[r angle speeds.'];orbit=rocket_osculating_conic(state,p,4*p.Re);
    distance=hypot(orbit.x-r*cos(angle)/1000,orbit.y-r*sin(angle)/1000);
    assert(min(distance,[],'omitnan')<1e-6,'Conic must pass through current position.');
    assert(all(hypot(orbit.x,orbit.y)*1000>=p.Re-1e-5 | isnan(orbit.x)));
end
radial=rocket_osculating_conic([r angle 100 0],p,4*p.Re);
assert(radial.degenerate && all(isfinite(radial.x)));
fprintf('Circular, elliptical, hyperbolic and radial conic display checks passed.\n');
end
