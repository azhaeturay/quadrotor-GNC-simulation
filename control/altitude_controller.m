function [Tcmd, vZcmd, aZcmd] = ...
    altitude_controller(zCmd, zN, vZN, phi, theta, P)
% ALTITUDE_CONTROLLER
%
% Cascaded vertical position and velocity controller.
%
% NED convention:
%   +z = downward
%   -z = upward
%
% Inputs:
%   zCmd        commanded NED vertical position [m]
%   zN          current NED vertical position [m]
%   vZN         current NED vertical velocity [m/s]
%   phi         roll angle [rad]
%   theta       pitch angle [rad]
%
% Outputs:
%   Tcmd        commanded total thrust [N]
%   vZcmd       commanded vertical velocity [m/s]
%   aZcmd       commanded vertical acceleration [m/s^2]

%% Position loop

zError = zCmd - zN;

vZcmd = P.posKpZ * zError;

%% Velocity limit

vZcmd = max(vZcmd, -P.maxVerticalVelocity);
vZcmd = min(vZcmd,  P.maxVerticalVelocity);


%% Velocity loop

vZerror = vZcmd - vZN;

aZcmd = P.velKpZ * vZerror;

%% Acceleration limit

aZcmd = max(aZcmd, -P.maxVerticalAcceleration);
aZcmd = min(aZcmd,  P.maxVerticalAcceleration);


%% Convert desired vertical acceleration to thrust

tiltFactor = cos(phi) * cos(theta);

% Prevent numerical problems for extreme attitudes.
% Normal operation should remain far from this limit.
tiltFactor = max(tiltFactor, 0.2);

Tcmd = P.mass * (P.g - aZcmd) / tiltFactor;


%% Physical thrust limits

Tmax = 4 * P.maxRotorThrust;

Tcmd = max(Tcmd, 0);
Tcmd = min(Tcmd, Tmax);

end