%% TEST_LOOKAHEAD_MISSION
% Fly-through waypoint mission using lookahead guidance.

clear;
clc;
close all;

projectRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(projectRoot));

P = vehicle_params();


%% ================================================================
% PATH DEFINITION
% ================================================================

% NED coordinates
%
% First waypoint is the initial vehicle location.

waypoints = [ ...
     0      0      0;
     0      0     -2;
     4      0     -2;
     4      4     -2;
     0      4     -2;
     0      0     -2];


%% Initial state

xCurrent = zeros(12,1);

tCurrent = 0;

segmentIdx = 1;

yawReference = 0;


%% Simulation settings

maxMissionTime = 60;


%% History

tHistory = tCurrent;

xHistory = xCurrent.';

targetHistory = [];

yawCmdHistory = [];

segmentHistory = [];

crossTrackHistory = [];

speedHistory = [];


fprintf('\n--- LOOKAHEAD PATH-FOLLOWING MISSION ---\n');


%% ================================================================
% GUIDANCE LOOP
% ================================================================

while tCurrent < maxMissionTime


    %% Guidance update

    [positionCmd, yawCmd, segmentIdx, info] = ...
        lookahead_guidance( ...
        xCurrent(1:3), ...
        waypoints, ...
        segmentIdx, ...
        yawReference, ...
        P);


    %% Integrate lower-level controlled vehicle for one guidance period

    tNext = min( ...
        tCurrent + P.guidanceDt, ...
        maxMissionTime);

    [~,xStep] = ode45( ...
        @(t,x) position_controlled_dynamics( ...
        t,x,positionCmd,yawCmd,P), ...
        [tCurrent tNext], ...
        xCurrent);


    %% New vehicle state

    xCurrent = xStep(end,:).';

    tCurrent = tNext;


    %% Calculate NED speed

    phi   = xCurrent(7);
    theta = xCurrent(8);
    psi   = xCurrent(9);

    R_BN = rotation_matrix(phi,theta,psi);

    vN = R_BN * xCurrent(4:6);

    vehicleSpeed = norm(vN);


    %% Store history

    tHistory(end+1,1) = tCurrent;

    xHistory(end+1,:) = xCurrent.';

    targetHistory(end+1,:) = positionCmd.';

    yawCmdHistory(end+1,1) = yawCmd;

    segmentHistory(end+1,1) = segmentIdx;

    crossTrackHistory(end+1,1) = ...
        info.crossTrackError;

    speedHistory(end+1,1) = vehicleSpeed;


    %% Update heading reference

    yawReference = yawCmd;


    %% Final waypoint completion

    finalWaypoint = waypoints(end,:).';

    finalError = norm( ...
        xCurrent(1:3) - finalWaypoint);

    if finalError < P.waypointAcceptanceRadius && ...
       vehicleSpeed < P.waypointAcceptanceSpeed

        fprintf('Final waypoint reached at t = %.2f s\n', ...
            tCurrent);

        break;

    end

end


%% ================================================================
% RESULTS
% ================================================================

north    = xHistory(:,1);
east     = xHistory(:,2);
altitude = -xHistory(:,3);


%% Guidance metrics

rmsCrossTrack = ...
    sqrt(mean(crossTrackHistory.^2));

maxCrossTrack = ...
    max(crossTrackHistory);

fprintf('RMS cross-track error: %.3f m\n', ...
    rmsCrossTrack);

fprintf('Max cross-track error: %.3f m\n', ...
    maxCrossTrack);


%% ================================================================
% 3-D TRAJECTORY
% ================================================================

figure;

plot3( ...
    north, ...
    east, ...
    altitude, ...
    'LineWidth',1.8);

hold on;

plot3( ...
    waypoints(:,1), ...
    waypoints(:,2), ...
    -waypoints(:,3), ...
    'o--', ...
    'LineWidth',1.2, ...
    'MarkerSize',7);

xlabel('North [m]');
ylabel('East [m]');
zlabel('Altitude [m]');

title('Lookahead Path-Following Mission');

legend( ...
    'Aircraft Trajectory', ...
    'Desired Path');

grid on;
axis equal;
view(3);


%% ================================================================
% TOP VIEW
% ================================================================

figure;

plot(east,north,'LineWidth',1.8);

hold on;

plot( ...
    waypoints(:,2), ...
    waypoints(:,1), ...
    'o--', ...
    'LineWidth',1.2, ...
    'MarkerSize',7);

xlabel('East [m]');
ylabel('North [m]');

title('Lookahead Guidance — Top View');

legend( ...
    'Aircraft Trajectory', ...
    'Desired Path');

grid on;
axis equal;


%% ================================================================
% VEHICLE SPEED
% ================================================================

figure;

plot( ...
    tHistory(2:end), ...
    speedHistory, ...
    'LineWidth',1.5);

xlabel('Time [s]');
ylabel('Vehicle Speed [m/s]');

title('Mission Speed');

grid on;


%% ================================================================
% CROSS-TRACK ERROR
% ================================================================

figure;

plot( ...
    tHistory(2:end), ...
    crossTrackHistory, ...
    'LineWidth',1.5);

xlabel('Time [s]');
ylabel('Cross-Track Error [m]');

title('Path-Following Error');

grid on;


%% ================================================================
% YAW TRACKING
% ================================================================

actualYaw = unwrap(xHistory(:,9));

commandedYaw = unwrap(yawCmdHistory);

figure;

plot( ...
    tHistory, ...
    rad2deg(actualYaw), ...
    'LineWidth',1.5);

hold on;

plot( ...
    tHistory(2:end), ...
    rad2deg(commandedYaw), ...
    '--', ...
    'LineWidth',1.3);

xlabel('Time [s]');
ylabel('Yaw [deg]');

title('Guidance Heading');

legend('Actual','Command');

grid on;