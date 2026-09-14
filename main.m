%% MAIN Run the complete estimated-state quadrotor GNC demonstration.
% Open this project folder in MATLAB and press Run, or type main.
% The canonical mission resets the workspace, closes figures, and uses rng(12).
% Real figures and run data are exported relative to this project root.

run(fullfile(fileparts(mfilename('fullpath')), ...
    'simulations', 'test_estimated_state_mission.m'));
