function [rotorThrust, achievedControl] = ...
    control_allocator(desiredControl, P)
% CONTROL_ALLOCATOR
% Maps desired total thrust and body moments into individual rotor thrusts.
%
% desiredControl = [T; tau_x; tau_y; tau_z]

A = allocation_matrix(P);

%% Unconstrained allocation

rotorThrust = A \ desiredControl;


%% Apply actuator limits

rotorThrust = max(rotorThrust, P.minRotorThrust);
rotorThrust = min(rotorThrust, P.maxRotorThrust);


%% Determine what control was actually achieved

achievedControl = A * rotorThrust;

end