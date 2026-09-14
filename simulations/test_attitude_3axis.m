%% TEST_ATTITUDE_3AXIS
% Validate simultaneous roll, pitch, and yaw attitude tracking.

clear;
clc;
close all;

projectRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(projectRoot));

P = vehicle_params();

%% Initial state

x0 = zeros(12,1);

%% Desired attitude
% Keep angles moderate for this validation.

attitudeCmd = deg2rad([5;
    -4;
    8]);

%% Simulation

tspan = [0 3];

[t,x] = ode45( ...
    @(t,x) attitude_controlled_dynamics( ...
    t,x,attitudeCmd,P), ...
    tspan,x0);

%% Extract attitude

phi   = rad2deg(x(:,7));
theta = rad2deg(x(:,8));
psi   = rad2deg(x(:,9));

cmdDeg = rad2deg(attitudeCmd);

%% Plot roll

figure;

plot(t,phi,'LineWidth',1.5);
hold on;
yline(cmdDeg(1),'--');

xlabel('Time [s]');
ylabel('\phi [deg]');
title('Roll Attitude Tracking');
legend('Actual','Command');
grid on;

%% Plot pitch

figure;

plot(t,theta,'LineWidth',1.5);
hold on;
yline(cmdDeg(2),'--');

xlabel('Time [s]');
ylabel('\theta [deg]');
title('Pitch Attitude Tracking');
legend('Actual','Command');
grid on;

%% Plot yaw

figure;

plot(t,psi,'LineWidth',1.5);
hold on;
yline(cmdDeg(3),'--');

xlabel('Time [s]');
ylabel('\psi [deg]');
title('Yaw Attitude Tracking');
legend('Actual','Command');
grid on;

%% Final tracking errors

finalAttitude = [phi(end);
    theta(end);
    psi(end)];

errorFinal = cmdDeg - finalAttitude;

fprintf('\n--- 3-AXIS ATTITUDE TRACKING ---\n');

fprintf('Final roll:  %.4f deg | Error: %.6f deg\n', ...
    phi(end),errorFinal(1));

fprintf('Final pitch: %.4f deg | Error: %.6f deg\n', ...
    theta(end),errorFinal(2));

fprintf('Final yaw:   %.4f deg | Error: %.6f deg\n', ...
    psi(end),errorFinal(3));