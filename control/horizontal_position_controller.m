function [phiCmd, thetaCmd, vXYcmd, aXYcmd] = ...
    horizontal_position_controller( ...
    posCmdXY, posXY, vXY, psi, P)
% HORIZONTAL_POSITION_CONTROLLER
%
% Cascaded North/East position and velocity controller.
%
% Inputs:
%   posCmdXY = desired [North; East] position [m]
%   posXY    = actual  [North; East] position [m]
%   vXY      = actual  [North; East] velocity [m/s]
%   psi      = current yaw angle [rad]
%
% Outputs:
%   phiCmd   = commanded roll angle [rad]
%   thetaCmd = commanded pitch angle [rad]
%   vXYcmd   = commanded horizontal velocity [m/s]
%   aXYcmd   = commanded horizontal acceleration [m/s^2]


%% Position loop

posError = posCmdXY - posXY;

vXYcmd = P.posKpXY * posError;


%% Limit horizontal velocity magnitude

speedCmd = norm(vXYcmd);

if speedCmd > P.maxHorizontalVelocity
    vXYcmd = vXYcmd / speedCmd ...
        * P.maxHorizontalVelocity;
end


%% Velocity loop

velError = vXYcmd - vXY;

aXYcmd = P.velKpXY * velError;


%% Limit horizontal acceleration magnitude

accelCmd = norm(aXYcmd);

if accelCmd > P.maxHorizontalAcceleration
    aXYcmd = aXYcmd / accelCmd ...
        * P.maxHorizontalAcceleration;
end


%% Convert NED acceleration into heading-aligned acceleration

aNorth = aXYcmd(1);
aEast  = aXYcmd(2);

aForward =  cos(psi)*aNorth + sin(psi)*aEast;
aRight   = -sin(psi)*aNorth + cos(psi)*aEast;


%% Desired attitude using near-hover approximation
%
% a_forward ~= -g*theta
% a_right   ~=  g*phi

thetaCmd = -aForward / P.g;
phiCmd   =  aRight   / P.g;


%% Limit total commanded tilt

tiltCmd = [phiCmd;
    thetaCmd];

tiltMagnitude = norm(tiltCmd);

if tiltMagnitude > P.maxTilt

    tiltCmd = tiltCmd / tiltMagnitude ...
        * P.maxTilt;

    phiCmd   = tiltCmd(1);
    thetaCmd = tiltCmd(2);

end

end