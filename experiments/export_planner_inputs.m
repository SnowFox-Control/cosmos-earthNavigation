function export_planner_inputs()
% Export exact MATLAB disturbance samples and common initial points for CUDA runs.
[p,opt]=rocket_two_burn_config();seeds=161:165;initial=zeros(96,10,numel(seeds));
for j=1:numel(seeds),opt.seed=seeds(j);initial(:,:,j)=rocket_two_burn_initial(p,opt);end
stream=RandStream('mt19937ar','Seed',p.noise.seed);
samples=randn(stream,2,ceil((sum(opt.ub(8:10))+2)/p.noise.sampleInterval)+3);
folder=fullfile(fileparts(fileparts(mfilename('fullpath'))),'results','comparison');
if ~exist(folder,'dir'),mkdir(folder);end
save(fullfile(folder,'inputs.mat'),'p','opt','initial','samples','seeds');
end
