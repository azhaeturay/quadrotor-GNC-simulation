%% TEST_POSITION_CONTROLLER
% Full XYZ position tracking test.

clear;
clc;
close all;

projectRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(projectRoot));

P = vehicle_params();


%% Initial state

x0 = zeros(12,1);


%% Desired position
%
% NED coordinates:
%   +x = North
%   +y = East
%   +z = Down

positionCmd = [ 5;
                3;
               -2];

yawCmd = 0;


%% Simulation

tspan = 0:0.02:8;

[t,x] = ode45( ...
    @(t,x) position_controlled_dynamics( ...
        t,x,positionCmd,yawCmd,P), ...
    tspan,x0);


%% Extract positions

north = x(:,1);
east  = x(:,2);

altitude = -x(:,3);


%% Final error

finalPosition = x(end,1:3).';

positionError = positionCmd - finalPosition;

fprintf('\n--- XYZ POSITION TRACKING ---\n');

fprintf('Final North:    %.4f m | Error: %.6f m\n', ...
    finalPosition(1),positionError(1));

fprintf('Final East:     %.4f m | Error: %.6f m\n', ...
    finalPosition(2),positionError(2));

fprintf('Final Altitude: %.4f m | Error: %.6f m\n', ...
    -finalPosition(3), ...
    (-positionCmd(3))-(-finalPosition(3)));


%% North position

figure;

plot(t,north,'LineWidth',1.5);
hold on;

yline(positionCmd(1),'--');

xlabel('Time [s]');
ylabel('North Position [m]');
title('North Position Tracking');
legend('Actual','Command');
grid on;


%% East position

figure;

plot(t,east,'LineWidth',1.5);
hold on;

yline(positionCmd(2),'--');

xlabel('Time [s]');
ylabel('East Position [m]');
title('East Position Tracking');
legend('Actual','Command');
grid on;


%% Altitude

figure;

plot(t,altitude,'LineWidth',1.5);
hold on;

yline(-positionCmd(3),'--');

xlabel('Time [s]');
ylabel('Altitude [m]');
title('Altitude Tracking');
legend('Actual','Command');
grid on;


%% 3-D trajectory

figure;

plot3(north,east,altitude, ...
    'LineWidth',1.8);

hold on;

plot3(positionCmd(1), ...
      positionCmd(2), ...
      -positionCmd(3), ...
      'o', ...
      'MarkerSize',8, ...
      'LineWidth',1.5);

xlabel('North [m]');
ylabel('East [m]');
zlabel('Altitude [m]');

title('3-D Position Tracking');

legend('Trajectory','Commanded Position');

grid on;
axis equal;
view(3);