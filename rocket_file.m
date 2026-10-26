function path=rocket_file(name)
%ROCKET_FILE Resolve generated artifacts independently of MATLAB's pwd.
root=fileparts(mfilename('fullpath'));
[~,~,ext]=fileparts(name);
if contains(name,'checkpoint'),folder='checkpoints';
elseif any(strcmpi(ext,{'.gif','.mp4'})),folder='animations';
elseif strcmpi(ext,'.png'),folder='figures';
else,folder='data';end
directory=fullfile(root,'results',folder);
if ~isfolder(directory),mkdir(directory);end
path=fullfile(directory,name);
end
