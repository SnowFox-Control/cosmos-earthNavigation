function v=rocket_comparison_visual_config()
% Plain simultaneous MPC/open-loop comet animation.
v.frameRate=10;
v.ascentStepSeconds=2;
v.transferStepSeconds=2;
v.circularizationStepSeconds=.5; % 5x playback, so the second burn remains readable
v.ignitionPauseSeconds=1;
v.coastStepSeconds=30;
v.orbitPeriods=1;             % Unpowered flight after both planned burns end
v.ascentTailSeconds=30;
v.coastTailSeconds=450;
v.closeupMinimumKm=20;
v.closeupMargin=1.4;
v.showOsculatingOrbits=true;
v.conicLightness=.65;         % Blend orbit dashes toward white (0..1)
v.conicLineWidth=.8;
v.writeGif=true;
v.writeMp4=true;
v.outputPrefix='rocket_mpc_comparison';
end
