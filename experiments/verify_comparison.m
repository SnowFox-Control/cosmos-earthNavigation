function verify_comparison()
% Evaluate every returned control unchanged; validation is not optimization.
root=fileparts(fileparts(mfilename('fullpath')));run(fullfile(root,'startup.m'));
folder=fullfile(root,'results','comparison');
inputs=load(fullfile(folder,'inputs.mat'));candidates=load(fullfile(folder,'pure_candidates.mat'));
p=inputs.p;opt=inputs.opt;p.maxStep=.2;p.relTol=1e-10;
p.absTol=[1e-6 1e-13 1e-8 1e-8 1e-8 1e-4];
entries={};best=Inf;bestPSO=[];
for j=1:size(candidates.unit,1)
    x=candidates.unit(j,:);method=strtrim(candidates.methods{j});seed=candidates.seeds(j);
    [pp,control,z]=rocket_two_burn_decode(x,p,opt);
    [score,out,residual]=rocket_two_burn_objective(x,p,opt);
    assert(~isempty(out));
    fine=pp;fine.maxStep=.1;fine.relTol=1e-11;fine.absTol=p.absTol/10;
    check=rocket_simulate(fine,control);difference=check.y(end,1:5)-out.y(end,1:5);
    success=all(abs(residual)<=[opt.altitudeTolerance opt.velocityTolerance opt.velocityTolerance]) ...
        && isnan(out.impactTime) && out.propellantUsed<=p.m0-p.mdry+1e-4;
    saved=jsondecode(fileread(fullfile(folder,sprintf('%s_seed%d_pure.json',lower(method),seed))));
    result=struct('optimizer',method,'z',z,'fitness',score,'searchFitness',saved.searchFitness, ...
        'history',saved.history(:,[1 3 4]),'residual',residual,'success',success,'out',out,'p',pp, ...
        'opt',opt,'control',control,'targetSpeed',sqrt(p.mu/(p.Re+opt.targetAltitude)), ...
        'elapsedSeconds',saved.seconds,'refinementDifference',difference, ...
        'localExitflag',NaN,'localOutput',struct('message','Disabled'));
    result.opt.useLocalRefinement=false;result.opt.seed=seed;
    r=out.y(end,1);u=out.y(end,3);w=out.y(end,4);
    energy=(u*u+w*w)/2-p.mu/r;a=-p.mu/(2*energy);
    eccentricity=hypot(r*w*w/p.mu-1,r*u*w/p.mu);
    result.osculatingEccentricity=eccentricity;
    result.osculatingPerigee=a*(1-eccentricity)-p.Re;result.osculatingApogee=a*(1+eccentricity)-p.Re;
    save(fullfile(folder,sprintf('%s_seed%d_verified.mat',lower(method),seed)),'result');
    item=struct('method',method,'seed',seed,'verifiedFitness',score,'residual',residual, ...
        'success',success,'difference',difference,'z',z,'fuel',out.propellantUsed, ...
        'eccentricity',eccentricity,'endTime',pp.tf);
    entries{end+1}=item; %#ok<AGROW>
    if strcmp(method,'PSO') && success && score<best,best=score;bestPSO=result;end
    fprintf('VERIFIED %s seed=%d J=%.8g residual=%s pass=%d\n',method,seed,score,mat2str(residual,8),success);
end
fid=fopen(fullfile(folder,'verification.json'),'w','n','UTF-8');assert(fid>=0);
fwrite(fid,jsonencode(struct('entries',{entries}),PrettyPrint=true),'char');fclose(fid);
assert(~isempty(bestPSO),'No pure PSO candidate passed verification; MPC was not run.');
result=bestPSO;save(fullfile(folder,'best_pso_reference.mat'),'result');
% Save under a distinct filename; historical refined results remain untouched.
save(fullfile(root,'results','data','rocket_pure_pso_reference.mat'),'result');
mpc=rocket_mpc_run(struct('referenceFile','rocket_pure_pso_reference.mat', ...
    'saveResults',false,'makeReport',false,'showProgress',false,'useParallel',false));
save(fullfile(folder,'mpc_pure_pso.mat'),'mpc');
fid=fopen(fullfile(folder,'mpc_metrics.json'),'w','n','UTF-8');assert(fid>=0);
item=struct('referenceSeed',candidates.seeds(find(arrayfun(@(k) ...
    strcmp(candidates.methods{k},'PSO') && norm(candidates.unit(k,:)- ...
    (bestPSO.z-opt.lb)./(opt.ub-opt.lb))<1e-8,1:size(candidates.unit,1)),1)), ...
    'metrics',mpc.metrics,'referenceResidual',bestPSO.residual,'time',mpc.time(end));
fwrite(fid,jsonencode(item,PrettyPrint=true),'char');fclose(fid);
fprintf('Pure PSO reference MPC complete.\n');
end
