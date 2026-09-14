%% TEST_WAYPOINT_MISSION
% Autonomous multi-waypoint quadrotor mission.

clear;
clc;
close all;

projectRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(projectRoot));

P = vehicle_params();


%% ================================================================
% MISSION DEFINITION
% ================================================================

% Waypoints expressed in NED:
%
% [North   East   Down]

waypoints = [ ...
     0      0     -2;
     4      0     -2;
     4      4     -2;
     0      4     -2;
     0      0     -2];


%% Initial state

xCurrent = zeros(12,1);

tCurrent = 0;

yawReference = 0;


%% Storage

tMission = [];
xMission = [];

activeWaypointHistory = [];


%% Maximum time allowed per leg

maxLegTime = 10;


%% ================================================================
% GUIDANCE / MISSION LOOP
% ================================================================

fprintf('\n--- AUTONOMOUS WAYPOINT MISSION ---\n');


for i = 1:size(waypoints,1)

    targetWaypoint = waypoints(i,:).';

    %% Guidance

    referencePosition = xCurrent(1:3);

    [positionCmd, yawCmd] = ...
        waypoint_guidance( ...
        targetWaypoint, ...
        referencePosition, ...
        yawReference);

    fprintf('\nWaypoint %d\n',i);

    fprintf('Target: [%.2f %.2f %.2f] m\n', ...
        positionCmd(1), ...
        positionCmd(2), ...
        positionCmd(3));

    fprintf('Yaw command: %.1f deg\n', ...
        rad2deg(yawCmd));


    %% ODE event for waypoint arrival

    options = odeset( ...
        'Events', ...
        @(t,x) waypoint_reached_event( ...
        t,x,targetWaypoint,P));


    %% Simulate current leg

    tspan = [tCurrent, ...
             tCurrent + maxLegTime];

    [tLeg,xLeg,tEvent,~,~] = ode45( ...
        @(t,x) position_controlled_dynamics( ...
        t,x,positionCmd,yawCmd,P), ...
        tspan, ...
        xCurrent, ...
        options);


    %% Avoid duplicate time sample between legs

    if ~isempty(tMission)

        tLeg = tLeg(2:end);
        xLeg = xLeg(2:end,:);

    end


    %% Save results

    tMission = [tMission;
                tLeg];

    xMission = [xMission;
                xLeg];

    activeWaypointHistory = [ ...
        activeWaypointHistory;
        i * ones(length(tLeg),1)];


    %% Check waypoint completion

    if isempty(tEvent)

        warning('Waypoint %d was not reached.',i);

        break;

    else

        fprintf('Reached at t = %.2f s\n', ...
            tEvent(end));

    end


    %% Initial condition for next leg

    xCurrent = xLeg(end,:).';

    tCurrent = tLeg(end);

    yawReference = yawCmd;

end


%% ================================================================
% RESULTS
% ================================================================

north    = xMission(:,1);
east     = xMission(:,2);
altitude = -xMission(:,3);

yaw = rad2deg(xMission(:,9));


%% 3-D flight path

figure;

plot3(north,east,altitude, ...
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

title('Autonomous Waypoint Mission');

legend('Aircraft Trajectory','Waypoints');

grid on;
axis equal;
view(3);


%% Top-down mission view

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

title('Waypoint Mission — Top View');

legend('Aircraft Trajectory','Waypoints');

grid on;
axis equal;


%% Altitude

figure;

plot(tMission,altitude,'LineWidth',1.5);

xlabel('Time [s]');
ylabel('Altitude [m]');

title('Mission Altitude');

grid on;


%% Yaw

figure;

plot(tMission,yaw,'LineWidth',1.5);

xlabel('Time [s]');
ylabel('Yaw \psi [deg]');

title('Mission Heading');

grid on;