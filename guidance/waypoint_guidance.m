function [positionCmd, yawCmd] = ...
    waypoint_guidance(targetWaypoint, referencePosition, referenceYaw)
% WAYPOINT_GUIDANCE
%
% Generates position and yaw commands for the active waypoint.
%
% Inputs:
%   targetWaypoint    = desired [North; East; Down] position [m]
%   referencePosition = position at beginning of current leg [m]
%   referenceYaw      = fallback yaw command [rad]
%
% Outputs:
%   positionCmd       = commanded NED position [m]
%   yawCmd            = commanded heading [rad]

%% Position command

positionCmd = targetWaypoint(:);

%% Determine horizontal direction of travel

deltaNE = targetWaypoint(1:2) ...
    - referencePosition(1:2);

horizontalDistance = norm(deltaNE);

%% Command heading toward waypoint

if horizontalDistance > 1e-6

    deltaNorth = deltaNE(1);
    deltaEast  = deltaNE(2);

    yawCmd = atan2(deltaEast, deltaNorth);

else

    % Vertical-only movement:
    % retain previous heading
    yawCmd = referenceYaw;

end

end