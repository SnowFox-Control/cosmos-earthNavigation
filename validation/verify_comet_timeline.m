function verify_comet_timeline()
v=rocket_comparison_visual_config();
first=153.2755;restart=403.2755;cutoff=465.258499;
[t,phase]=rocket_comet_timeline(first,restart,cutoff,cutoff+500,v);
assert(all(diff(t)>0));
assert(any(t==first) && any(t==restart) && any(t==cutoff));
assert(any(t==(restart+cutoff)/2));
assert(sum(phase==3)>=ceil((cutoff-restart)/v.circularizationStepSeconds));
burnTimes=t(phase==3);
assert(all(diff(burnTimes)<=v.circularizationStepSeconds+1e-9));
assert(all(t(phase==3)>=restart & t(phase==3)<cutoff));
fprintf('Burn-2 frame coverage passed: %d frames, at least %.1f s screen time.\n', ...
    sum(phase==3),sum(phase==3)/v.frameRate);
end
