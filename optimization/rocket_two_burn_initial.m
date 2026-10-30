function initial=rocket_two_burn_initial(p,opt)
% Shared, seed-reproducible initialization; never imports a historical solution.
rng(opt.seed,'twister');initial=rand(opt.swarmSize,10);
for k=1:floor(opt.swarmSize*opt.heuristicFraction)
    q1=260+70*rand;q2=200+100*rand;budget=p.m0-p.mdry;
    t1=budget*(33000+7000*rand)/54000/q1;
    t2=(budget*50000/54000-q1*t1)/q2;
    z=[1+5*rand -2+4*rand -3+6*rand -2+4*rand -2+4*rand ...
       q1 q2 t1 40+130*rand t2];
    initial(k,:)=min(1,max(0,(z-opt.lb)./(opt.ub-opt.lb)));
end
end
