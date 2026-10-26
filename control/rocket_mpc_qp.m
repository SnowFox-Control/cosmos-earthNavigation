function [correction,diagnostic]=rocket_mpc_qp(state,index,models,previous,p,cfg)
% Condensed LTV tracking MPC: inputs constrained, mass hard, altitude soft.
N=cfg.predictionSteps;nu=2*N;S=cfg.stateScale;S(2)=S(2)/p.Re;
current=models{index}.current;
e=(state(1:5)-current)./S;
P=eye(5);G=zeros(5,nu);offset=zeros(5,1);
prediction=zeros(5*N,nu);free=zeros(5*N,1);lower=zeros(nu,1);upper=lower;
ground=zeros(N,1);mass=zeros(N,1);
for j=1:N
    model=models{index+j-1};cols=2*j-1:2*j;rows=5*j-4:5*j;
    P=model.A*P;G=model.A*G;G(:,cols)=G(:,cols)+model.B;
    offset=model.A*offset+model.d;
    prediction(rows,:)=G;free(rows)=P*e+offset;
    lower(cols)=model.lb;upper(cols)=model.ub;
    ground(j)=(model.next(1)-p.Re)/S(1);
    mass(j)=(model.next(5)-p.mdry)/S(5);
end
weights=repmat(cfg.stateWeights,N,1);weights(end-4:end)=weights(end-4:end)*cfg.terminalMultiplier;
Q=diag(weights);R=kron(eye(N),diag(cfg.inputWeights));
D=eye(nu);for j=3:nu,D(j,j-2)=-1;end
prior=zeros(nu,1);prior(1:2)=previous./cfg.correctionScale;
Rd=kron(eye(N),diag(cfg.inputRateWeights));
H=2*(prediction.'*Q*prediction+R+D.'*Rd*D);
f=2*(prediction.'*Q*free-D.'*Rd*prior);
% A soft ground constraint is used to keep the QP solvable near launch.
ir=1:5:5*N;im=5:5:5*N;
Aineq=[-prediction(ir,:) -eye(N);-prediction(im,:) zeros(N)];
bineq=[ground+free(ir);mass+free(im)];
H=blkdiag(H,2*cfg.positionSlackWeight*eye(N))+1e-9*eye(nu+N);
H=(H+H.')/2;
f=[f;zeros(N,1)];
options=optimoptions('quadprog','Display','off','Algorithm','interior-point-convex', ...
    'OptimalityTolerance',1e-7,'ConstraintTolerance',1e-8);
timer=tic;
[solution,~,flag,output]=quadprog(H,f,Aineq,bineq,[],[],[lower;zeros(N,1)], ...
    [upper;Inf(N,1)],[],options);
diagnostic=struct('exitflag',flag,'seconds',toc(timer),'iterations',output.iterations,'slack',NaN);
if flag>0 && all(isfinite(solution))
    correction=solution(1:2).*cfg.correctionScale;
    diagnostic.slack=max(solution(nu+1:end));
else
    correction=[0;0]; % Explicit safe fallback: reference feedforward only
end
end
