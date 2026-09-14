%% TEST_ALTITUDE_CONTROLLER

clear;
clc;
close all;

projectRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(projectRoot));

P = vehicle_params();

%% Initial state

x0 = zeros(12,1);

%% Command

% NED: -2 m means 2 m upward
zCmd = -2;

%% Simulation

tspan = [0 5];

[t,x] = ode45( ...
    @(t,x) altitude_controlled_dynamics(t,x,zCmd,P), ...
    tspan,x0);

%% Extract position

zN = x(:,3);

% Convert NED z to intuitive altitude
altitude = -zN;
altitudeCmd = -zCmd;

%% Reconstruct controller signals for plotting

n = length(t);

vZN   = zeros(n,1);
vZcmd = zeros(n,1);
aZcmd = zeros(n,1);
Tcmd  = zeros(n,1);

for k = 1:n

    phi   = x(k,7);
    theta = x(k,8);
    psi   = x(k,9);

    R_BN = rotation_matrix(phi,theta,psi);

    vN = R_BN * x(k,4:6).';

    vZN(k) = vN(3);

    [Tcmd(k),vZcmd(k),aZcmd(k)] = ...
        altitude_controller( ...
        zCmd,zN(k),vZN(k),phi,theta,P);

end

%% Altitude plot

figure;

plot(t,altitude,'LineWidth',1.5);
hold on;

yline(altitudeCmd,'--');

xlabel('Time [s]');
ylabel('Altitude [m]');
title('Altitude Command Tracking');
legend('Actual','Command');
grid on;


%% Vertical velocity plot

figure;

plot(t,-vZN,'LineWidth',1.5);

xlabel('Time [s]');
ylabel('Upward Velocity [m/s]');
title('Vertical Velocity');
grid on;


%% Thrust plot

figure;

plot(t,Tcmd,'LineWidth',1.5);
hold on;

yline(P.hoverThrustTotal,'--');

xlabel('Time [s]');
ylabel('Total Thrust [N]');
title('Altitude Controller Thrust Command');
legend('Commanded Thrust','Hover Thrust');
grid on;