%% TEST_RATE_CONTROLLER

clear;
clc;
close all;

projectRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(projectRoot));

P = vehicle_params();

%% Initial state

x0 = zeros(12,1);

%% Command

omegaCmd = [deg2rad(20);
            0;
            0];

%% Simulate

tspan = [0 0.6];

[t, x] = ode45( ...
    @(t,x) rate_controlled_dynamics(t,x,omegaCmd,P), ...
    tspan, x0);

%% Extract roll rate

p = x(:,10);

%% Analytical first-order prediction

tauRate = P.Ixx / P.rateKp(1);

pExpected = omegaCmd(1) * ...
    (1 - exp(-t/tauRate));

%% Plot

figure;

plot(t,rad2deg(p),'LineWidth',1.5);
hold on;

plot(t,rad2deg(pExpected),'--','LineWidth',1.5);

yline(rad2deg(omegaCmd(1)),':');

xlabel('Time [s]');
ylabel('Roll Rate p [deg/s]');
title('Roll Rate Command Tracking');
legend('Nonlinear Simulation', ...
       'First-Order Prediction', ...
       'Command');

grid on;