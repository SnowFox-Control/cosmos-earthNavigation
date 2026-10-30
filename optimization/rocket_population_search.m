function search=rocket_population_search(method,fitness,initial,opt)
% Budget-controlled population algorithms. All consume M*(iterations+1) scores.
% SA is explicitly M independent chains, not a single chain with a larger budget.
method=upper(char(method));assert(any(strcmp(method,{'PSO','GA','SA'})));
M=size(initial,1);dim=size(initial,2);iterations=opt.maxIterations;
assert(M>=4 && mod(M,2)==0 && iterations>=1);
x=initial;timer=tic;f=fitness(x);f=f(:);
assert(numel(f)==M && all(isfinite(f)));
[best,index]=min(f);bestx=x(index,:);personal=x;personalF=f;velocity=zeros(M,dim);
history=zeros(iterations+1,4);history(1,:)=[0 M best mean(f)];
T0=1;Tmin=1e-6;accepted=0;
for k=1:iterations
    progress=(k-1)/max(1,iterations-1);
    switch method
        case 'PSO'
            inertia=opt.inertiaRange(2)+(opt.inertiaRange(1)-opt.inertiaRange(2))*progress;
            velocity=inertia*velocity+opt.selfAdjustmentWeight*rand(M,dim).*(personal-x)+ ...
                opt.socialAdjustmentWeight*rand(M,dim).*(bestx-x);
            velocity=max(-.2,min(.2,velocity));
            candidate=reflect(x+velocity);
        case 'GA'
            % Tournament selection, arithmetic/blend crossover and Gaussian mutation.
            parent1=tournament(f,M);parent2=tournament(f,M);
            weight=-.25+1.5*rand(M,dim);
            candidate=weight.*x(parent1,:)+(1-weight).*x(parent2,:);
            mutation=rand(M,dim)<1/dim;
            candidate=reflect(candidate+mutation.*(.1*randn(M,dim)));
            % Re-evaluate one elite as part of the common accounting convention.
            candidate(1,:)=bestx;
        case 'SA'
            temperature=T0*(Tmin/T0)^progress;
            % Coordinate-wise Gaussian Metropolis proposals, bounded by reflection.
            candidate=x;coordinate=randi(dim,M,1);rows=(1:M).';
            linear=sub2ind([M dim],rows,coordinate);
            width=.2*(1-progress)+.005*progress;
            candidate(linear)=candidate(linear)+width*randn(M,1);
            candidate=reflect(candidate);
    end
    scores=fitness(candidate);scores=scores(:);
    assert(numel(scores)==M && all(isfinite(scores)));
    if strcmp(method,'SA')
        take=scores<=f | log(max(realmin,rand(M,1)))<-(scores-f)/temperature;
        accepted=accepted+sum(take);x(take,:)=candidate(take,:);f(take)=scores(take);
    else
        x=candidate;f=scores;
    end
    if strcmp(method,'PSO')
        improve=f<personalF;personal(improve,:)=x(improve,:);personalF(improve)=f(improve);
    end
    [value,index]=min(f);
    if value<best,best=value;bestx=x(index,:);end
    history(k+1,:)=[k M*(k+1) best mean(f)];
    if ~strcmpi(opt.display,'off') && (mod(k,20)==0 || k==iterations)
        fprintf('%s seed=%d iter=%d eval=%d best=%.9g elapsed=%.1fs\n', ...
            method,opt.seed,k,M*(k+1),best,toc(timer));
    end
end
search=struct('x',bestx,'fitness',best,'history',history,'elapsedSeconds',toc(timer), ...
    'evaluations',M*(iterations+1),'initial',initial,'method',method,'accepted',accepted);
end
function y=reflect(y)
y=mod(y,2);y=min(y,2-y);
end
function index=tournament(f,M)
candidate=randi(M,M,3);[~,column]=min(f(candidate),[],2);
index=candidate(sub2ind(size(candidate),(1:M).',column));
end
