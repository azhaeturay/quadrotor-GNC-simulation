%% TEST_ALLOCATION
% Validation tests for quadrotor rotor mixing and control allocation.

clear;
clc;
close all;

%% Setup

projectRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(projectRoot));

P = vehicle_params();

A = allocation_matrix(P);

fprintf('\n--- ALLOCATION MATRIX ---\n');
disp(A);


%% ================================================================
% TEST 1: EQUAL ROTOR THRUST / HOVER
% ================================================================

fprintf('\n--- TEST 1: HOVER ---\n');

rotorHover = P.hoverThrustRotor * ones(4,1);

controlHover = rotor_wrench(rotorHover, P);

fprintf('Total thrust: %.6f N\n', controlHover(1));
fprintf('Roll moment:  %.6f N*m\n', controlHover(2));
fprintf('Pitch moment: %.6f N*m\n', controlHover(3));
fprintf('Yaw moment:   %.6f N*m\n', controlHover(4));


%% ================================================================
% TEST 2: PURE ROLL DIFFERENTIAL
% ================================================================

fprintf('\n--- TEST 2: PURE ROLL ---\n');

deltaT = 0.10;

rotorRoll = [P.hoverThrustRotor + deltaT;
             P.hoverThrustRotor - deltaT;
             P.hoverThrustRotor - deltaT;
             P.hoverThrustRotor + deltaT];

controlRoll = rotor_wrench(rotorRoll, P);

fprintf('Total thrust: %.6f N\n', controlRoll(1));
fprintf('Roll moment:  %.6f N*m\n', controlRoll(2));
fprintf('Pitch moment: %.6f N*m\n', controlRoll(3));
fprintf('Yaw moment:   %.6f N*m\n', controlRoll(4));


%% ================================================================
% TEST 3: PURE YAW DIFFERENTIAL
% ================================================================

fprintf('\n--- TEST 3: PURE YAW ---\n');

rotorYaw = [P.hoverThrustRotor + deltaT;
            P.hoverThrustRotor - deltaT;
            P.hoverThrustRotor + deltaT;
            P.hoverThrustRotor - deltaT];

controlYaw = rotor_wrench(rotorYaw, P);

fprintf('Total thrust: %.6f N\n', controlYaw(1));
fprintf('Roll moment:  %.6f N*m\n', controlYaw(2));
fprintf('Pitch moment: %.6f N*m\n', controlYaw(3));
fprintf('Yaw moment:   %.6f N*m\n', controlYaw(4));


%% ================================================================
% TEST 4: INVERSE CONTROL ALLOCATION
% ================================================================

fprintf('\n--- TEST 4: INVERSE ALLOCATION ---\n');

desiredControl = [P.hoverThrustTotal;
                  0.05;
                  0;
                  0];

[rotorCommand, achievedControl] = ...
    control_allocator(desiredControl, P);

fprintf('\nRotor thrust commands [N]:\n');
disp(rotorCommand);

fprintf('Desired control:\n');
disp(desiredControl);

fprintf('Achieved control:\n');
disp(achievedControl);

fprintf('Allocation error:\n');
disp(achievedControl - desiredControl);