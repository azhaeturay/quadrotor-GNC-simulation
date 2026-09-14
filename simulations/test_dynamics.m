%% TEST_DYNAMICS
% Basic validation tests for the nonlinear quadrotor model.

clear;
clc;
close all;

%% Setup

projectRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(projectRoot));

P = vehicle_params();

% Initial state:
% [xN yN zN u v w phi theta psi p q r]'

x0 = zeros(12,1);


%% ================================================================
% TEST 1: FREE FALL
% ================================================================

fprintf('\n--- TEST 1: FREE FALL ---\n');

inputs = [0; 0; 0; 0];

tspan = [0 2];

[t1, x1] = ode45(@(t,x) quad_dynamics(t,x,inputs,P), ...
                 tspan, x0);

z_final = x1(end,3);
w_final = x1(end,6);

% Analytical free-fall solution
z_expected = 0.5 * P.g * tspan(end)^2;
w_expected = P.g * tspan(end);

fprintf('Final z position: %.4f m\n', z_final);
fprintf('Expected z:       %.4f m\n', z_expected);

fprintf('Final w velocity: %.4f m/s\n', w_final);
fprintf('Expected w:       %.4f m/s\n', w_expected);


%% ================================================================
% TEST 2: EXACT HOVER
% ================================================================

fprintf('\n--- TEST 2: EXACT HOVER ---\n');

inputs = [P.hoverThrustTotal;
          0;
          0;
          0];

tspan = [0 5];

[t2, x2] = ode45(@(t,x) quad_dynamics(t,x,inputs,P), ...
                 tspan, x0);

maxStateMagnitude = max(abs(x2),[],'all');

fprintf('Maximum state magnitude during hover: %.6e\n', ...
        maxStateMagnitude);


%% ================================================================
% TEST 3: PURE ROLL MOMENT
% ================================================================

fprintf('\n--- TEST 3: PURE ROLL MOMENT ---\n');

tauRoll = 0.05;     % [N*m]

inputs = [P.hoverThrustTotal;
          tauRoll;
          0;
          0];

xdot0 = quad_dynamics(0,x0,inputs,P);

pDot_actual = xdot0(10);

pDot_expected = tauRoll / P.Ixx;

fprintf('Calculated p_dot: %.6f rad/s^2\n', pDot_actual);
fprintf('Expected p_dot:   %.6f rad/s^2\n', pDot_expected);


%% ================================================================
% PLOTS
% ================================================================

figure;

plot(t1,x1(:,3),'LineWidth',1.5);

xlabel('Time [s]');
ylabel('z_N [m]');
title('Free-Fall Test');
grid on;


figure;

plot(t2,x2(:,3),'LineWidth',1.5);

xlabel('Time [s]');
ylabel('z_N [m]');
title('Exact Hover Test');
grid on;