function xdot = attitude_controlled_dynamics(t, x, attitudeCmd, P)
% ATTITUDE_CONTROLLED_DYNAMICS
% Cascaded attitude + body-rate controlled quadrotor.

%% Current state

attitude = x(7:9);
omega    = x(10:12);

%% Outer attitude loop

omegaCmd = attitude_controller( ...
    attitudeCmd, attitude, P);

%% Inner rate loop

tauCmd = rate_controller( ...
    omegaCmd, omega, P);

%% Maintain hover collective thrust

Tcmd = P.hoverThrustTotal;

desiredControl = [Tcmd;
    tauCmd];

%% Control allocation

[rotorThrust, ~] = ...
    control_allocator(desiredControl, P);

%% Nonlinear rotor-level plant

xdot = quad_dynamics_rotors( ...
    t, x, rotorThrust, P);

end