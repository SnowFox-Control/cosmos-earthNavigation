function [times,phase]=rocket_comet_timeline(firstCutoff,restart,cutoff,finish,v)
% Sample each physical phase separately and include exact switch instants.
assert(0<firstCutoff && firstCutoff<restart && restart<cutoff && cutoff<=finish);
assert(v.ascentStepSeconds>0 && v.transferStepSeconds>0 && ...
    v.circularizationStepSeconds>0 && v.coastStepSeconds>0);
times=unique([0:v.ascentStepSeconds:firstCutoff,firstCutoff, ...
    firstCutoff:v.transferStepSeconds:restart,restart, ...
    restart:v.circularizationStepSeconds:cutoff,(restart+cutoff)/2,cutoff, ...
    cutoff:v.coastStepSeconds:finish,finish]);
phase=ones(size(times));
phase(times>=firstCutoff & times<restart)=2;
phase(times>=restart & times<cutoff)=3;
phase(times>=cutoff)=4;
end
