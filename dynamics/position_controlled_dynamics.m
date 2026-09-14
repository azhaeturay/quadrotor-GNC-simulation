function xdot = position_controlled_dynamics( ...
    t, x, positionCmd, yawCmd, P)
% POSITION_CONTROLLED_DYNAMICS
%
% Full cascaded XYZ position, attitude, and rate control.

%% Current states

position = x(1:3);

attitude = x(7:9);

omega = x(10:12);

phi   = attitude(1);
theta = attitude(2);
psi   = attitude(3);


%% Convert body-frame velocity into NED velocity

R_BN = rotation_matrix(phi,theta,psi);

vN = R_BN * x(4:6);

vXY = vN(1:2);
vZN = vN(3);


%% Horizontal position controller

[phiCmd, thetaCmd, ~, ~] = ...
    horizontal_position_controller( ...
    positionCmd(1:2), ...
    position(1:2), ...
    vXY, ...
    psi, ...
    P);


%% Altitude controller

[Tcmd, ~, ~] = altitude_controller( ...
    positionCmd(3), ...
    position(3), ...
    vZN, ...
    phi, ...
    theta, ...
    P);


%% Desired attitude

attitudeCmd = [phiCmd;
    thetaCmd;
    yawCmd];


%% Outer attitude loop

omegaCmd = attitude_controller( ...
    attitudeCmd, attitude, P);


%% Inner body-rate loop

tauCmd = rate_controller( ...
    omegaCmd, omega, P);


%% Desired wrench

desiredControl = [Tcmd;
    tauCmd];


%% Control allocation

[rotorThrust, ~] = ...
    control_allocator(desiredControl, P);


%% Nonlinear rotor-level plant

xdot = quad_dynamics_rotors( ...
    t, x, rotorThrust, P);

end