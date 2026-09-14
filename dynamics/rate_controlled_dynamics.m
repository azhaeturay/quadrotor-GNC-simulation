function xdot = rate_controlled_dynamics(t, x, omegaCmd, P)
% RATE_CONTROLLED_DYNAMICS
% Closed-loop quadrotor dynamics with body-rate feedback control.

%% Current body rates

omega = x(10:12);

%% Rate controller

tauCmd = rate_controller(omegaCmd, omega, P);

%% Maintain hover collective thrust

Tcmd = P.hoverThrustTotal;

desiredControl = [Tcmd;
    tauCmd];

%% Control allocation

[rotorThrust, ~] = control_allocator(desiredControl, P);

%% Rotor-level nonlinear plant

xdot = quad_dynamics_rotors(t, x, rotorThrust, P);

end