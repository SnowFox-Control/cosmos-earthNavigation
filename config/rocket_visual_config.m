function v=rocket_visual_config()
% Visual-only settings. Changes do not require rerunning optimization.
v.frameRate=10;                  % frames/s for MP4 and GIF
v.ascentStepSeconds=2;           % 2*10 = 20x playback
v.orbitStepSeconds=30;           % 30*10 = 300x playback
v.orbitPeriods=1;                % number of periods after engine cutoff
v.ascentTailSeconds=30;
v.orbitTailSeconds=450;
v.closeupInitialKm=12;
v.closeupFinalKm=242;
v.gradientScaleKm=160;          % illustrative atmosphere/space color transition
v.nearColor=[.18 .43 .57];
v.deepColor=[.025 .04 .09];
v.groundColor=[.055 .17 .20];
v.orbitColor=[.46 .52 .57];      % muted osculating conic
v.showOsculatingOrbit=true;
v.writeGif=true;                % GIF is considerably larger than MP4
v.writeMp4=true;
v.outputPrefix='rocket_polar';
end
