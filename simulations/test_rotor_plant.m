%% TEST_ROTOR_PLANT
% Integrated validation of rotor forces, control allocation,
% and nonlinear 6-DOF vehicle dynamics.

clear;
clc;
close all;

%% Setup

projectRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(projectRoot));

P = vehicle_params();

x0 = zeros(12,1);


%% ================================================================
% TEST 1: ROTOR-LEVEL HOVER
% ================================================================

fprintf('\n--- TEST 1: ROTOR-LEVEL HOVER ---\n');

rotorHover = P.hoverThrustRotor * ones(4,1);

tspan = [0 5];

[tHover, xHover] = ode45( ...
    @(t,x) quad_dynamics_rotors(t,x,rotorHover,P), ...
    tspan, x0);

maxStateMagnitude = max(abs(xHover),[],'all');

fprintf('Maximum state magnitude during hover: %.6e\n', ...
    maxStateMagnitude);


%% ================================================================
% TEST 2: ROTOR-LEVEL ROLL INPUT
% ================================================================

fprintf('\n--- TEST 2: ROTOR-LEVEL ROLL INPUT ---\n');

deltaT = 0.10;

rotorRoll = [P.hoverThrustRotor + deltaT;
             P.hoverThrustRotor - deltaT;
             P.hoverThrustRotor - deltaT;
             P.hoverThrustRotor + deltaT];

% Determine the actual wrench created by these rotor thrusts
controlRoll = rotor_wrench(rotorRoll,P);

tauRoll = controlRoll(2);

% Expected initial roll acceleration
pDot_expected = tauRoll / P.Ixx;

% Short simulation
tspan = [0 0.25];

[tRoll, xRoll] = ode45( ...
    @(t,x) quad_dynamics_rotors(t,x,rotorRoll,P), ...
    tspan, x0);

% Extract final values
p_final = xRoll(end,10);
phi_final = xRoll(end,7);

% Analytical expectation
tf = tspan(end);

p_expected = pDot_expected * tf;
phi_expected = 0.5 * pDot_expected * tf^2;

fprintf('Roll moment: %.6f N*m\n', tauRoll);

fprintf('\nInitial p_dot expected: %.6f rad/s^2\n', ...
    pDot_expected);

fprintf('\nFinal p actual:   %.6f rad/s\n', p_final);
fprintf('Final p expected: %.6f rad/s\n', p_expected);

fprintf('\nFinal phi actual:   %.6f deg\n', rad2deg(phi_final));
fprintf('Final phi expected: %.6f deg\n', rad2deg(phi_expected));


%% ================================================================
% PLOTS
% ================================================================

figure;

plot(tRoll,rad2deg(xRoll(:,7)),'LineWidth',1.5);

xlabel('Time [s]');
ylabel('\phi [deg]');
title('Rotor-Level Roll Response');
grid on;


figure;

plot(tRoll,xRoll(:,10),'LineWidth',1.5);

xlabel('Time [s]');
ylabel('p [rad/s]');
title('Body Roll Rate');
grid on;