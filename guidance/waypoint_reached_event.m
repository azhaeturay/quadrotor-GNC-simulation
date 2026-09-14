function [value, isterminal, direction] = ...
    waypoint_reached_event(~, x, targetWaypoint, P)
% WAYPOINT_REACHED_EVENT
%
% Stops the current ODE integration when the vehicle is both:
%   1. Inside the waypoint acceptance radius
%   2. Moving slowly enough to count as settled

%% Position error

position = x(1:3);

positionError = norm( ...
    targetWaypoint(:) - position);

%% Navigation-frame velocity

phi   = x(7);
theta = x(8);
psi   = x(9);

R_BN = rotation_matrix(phi,theta,psi);

vN = R_BN * x(4:6);

speed = norm(vN);

%% Both conditions must be satisfied

positionCondition = ...
    positionError - P.waypointAcceptanceRadius;

speedCondition = ...
    speed - P.waypointAcceptanceSpeed;

value = max(positionCondition, speedCondition);

%% Stop integration when value crosses zero

isterminal = 1;
direction  = -1;

end