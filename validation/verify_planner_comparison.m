function verify_planner_comparison()
% Algorithm-level invariants using a cheap objective; then actual CPU dynamics.
[p,opt]=rocket_two_burn_config();opt.swarmSize=12;opt.maxIterations=4;opt.display='off';
initial=rocket_two_burn_initial(p,opt);methods={'PSO','GA','SA'};
for j=1:3
    rng(12,'twister');a=rocket_population_search(methods{j},@(x)sum((x-.3).^2,2),initial,opt);
    rng(12,'twister');b=rocket_population_search(methods{j},@(x)sum((x-.3).^2,2),initial,opt);
    assert(isequal(a.x,b.x) && isequal(a.history,b.history));
    assert(a.evaluations==12*5 && all(a.x>=0 & a.x<=1));
    assert(all(diff(a.history(:,3))<=0));
    assert(isequal(a.initial,initial));
end
[pp,control]=rocket_two_burn_decode(initial(1,:),p,opt);
z=opt.lb+initial(1,:).*(opt.ub-opt.lb);ramp=opt.rampSeconds;
assert(abs(trapz(control.t,control.q)-(z(6)*(z(8)-ramp/2)+z(7)*(z(10)-ramp)))<1e-6);
score=rocket_two_burn_objective(initial(1,:),pp,opt);assert(isfinite(score));
fprintf('Pure PSO/GA/SA bounds, budgets, deterministic seeds and shared decoder passed.\n');
end
