function xdot = altitude_controlled_dynamics(t, x, zCmd, P)
% ALTITUDE_CONTROLLED_DYNAMICS
%
% Closed-loop altitude + attitude + body-rate controlled quadrotor.

%% Current states

attitude = x(7:9);
omega    = x(10:12);

phi   = attitude(1);
theta = attitude(2);

%% Convert body velocity to NED velocity

R_BN = rotation_matrix( ...
    attitude(1), attitude(2), attitude(3));

vN = R_BN * x(4:6);

vZN = vN(3);

%% Altitude controller

[Tcmd, ~, ~] = altitude_controller( ...
    zCmd, x(3), vZN, phi, theta, P);

%% Hold vehicle level

attitudeCmd = [0;
    0;
    0];

%% Attitude controller

omegaCmd = attitude_controller( ...
    attitudeCmd, attitude, P);

%% Rate controller

tauCmd = rate_controller( ...
    omegaCmd, omega, P);

%% Desired vehicle wrench

desiredControl = [Tcmd;
    tauCmd];

%% Control allocation

[rotorThrust, ~] = ...
    control_allocator(desiredControl, P);

%% Rotor-level nonlinear dynamics

xdot = quad_dynamics_rotors( ...
    t, x, rotorThrust, P);

end