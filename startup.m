% Run once after opening this project, then call the entry points in README.
projectRoot=fileparts(mfilename('fullpath'));
addpath(projectRoot);
folders={'config','model','optimization','visualization','validation','examples','control'};
for folderIndex=1:numel(folders)
    addpath(fullfile(projectRoot,folders{folderIndex}));
end
clear projectRoot folders folderIndex;
