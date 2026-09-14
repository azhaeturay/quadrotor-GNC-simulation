%% TEST_ATTITUDE_CONTROLLER

clear;
clc;
close all;

projectRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(projectRoot));

P = vehicle_params();

%% Initial state

x0 = zeros(12,1);

% Begin with 20 deg roll error
x0(7) = deg2rad(20);

%% Desired attitude

attitudeCmd = [0;
    0;
    0];

%% Simulation

tspan = [0 2];

[t,x] = ode45( ...
    @(t,x) attitude_controlled_dynamics( ...
    t,x,attitudeCmd,P), ...
    tspan,x0);

%% Extract states

phi = x(:,7);
p   = x(:,10);

%% Plots

figure;

plot(t,rad2deg(phi),'LineWidth',1.5);
hold on;

yline(0,'--');

xlabel('Time [s]');
ylabel('Roll Angle \phi [deg]');
title('Attitude Disturbance Recovery');
grid on;


figure;

plot(t,rad2deg(p),'LineWidth',1.5);

xlabel('Time [s]');
ylabel('Roll Rate p [deg/s]');
title('Body Roll Rate During Recovery');
grid on;