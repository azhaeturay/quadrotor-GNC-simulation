function [positionCmd, yawCmd, segmentIdx, info] = ...
    lookahead_guidance( ...
    position, waypoints, segmentIdx, yawReference, P)
% LOOKAHEAD_GUIDANCE
%
% 3-D polyline lookahead / virtual-target guidance.
%
% The aircraft is projected onto the currently active path segment.
% A virtual position target is then placed a fixed distance ahead
% along the waypoint path.
%
% Inputs:
%   position      = current NED position [3x1] [m]
%   waypoints     = waypoint matrix [N x 3], NED [m]
%   segmentIdx    = currently active segment
%   yawReference  = fallback heading [rad]
%
% Outputs:
%   positionCmd   = virtual target position [3x1] [m]
%   yawCmd        = desired heading [rad]
%   segmentIdx    = updated active segment
%   info          = diagnostic information

position = position(:);

nWaypoints = size(waypoints,1);

% Segment i connects waypoint i -> waypoint i+1
segmentIdx = min(max(segmentIdx,1),nWaypoints-1);


%% ================================================================
% ADVANCE ACTIVE SEGMENT IF NEEDED
% ================================================================

while segmentIdx < nWaypoints-1

    A = waypoints(segmentIdx,:).';
    B = waypoints(segmentIdx+1,:).';

    d = B - A;

    segmentLength = norm(d);

    if segmentLength < 1e-9
        segmentIdx = segmentIdx + 1;
        continue;
    end


    %% Progress along current segment

    sRaw = dot(position - A, d) / dot(d,d);


    %% Check whether current segment is vertical

    horizontalLength = norm(d(1:2));


    if horizontalLength < 1e-6

        % For vertical takeoff / landing:
        % require actual proximity to the endpoint.
        shouldAdvance = ...
            norm(position - B) <= P.pathSwitchRadius;

    else

        % For fly-through horizontal / 3-D segments:
        % switch based on along-track progress rather than
        % Euclidean distance to the waypoint.

        alongTrackRemaining = ...
            (1 - sRaw) * segmentLength;

        shouldAdvance = ...
            alongTrackRemaining <= P.pathSwitchRadius;

    end


    if shouldAdvance

        segmentIdx = segmentIdx + 1;

    else

        break;

    end

end

%% ================================================================
% PROJECT AIRCRAFT ONTO CURRENT SEGMENT
% ================================================================

A = waypoints(segmentIdx,:).';
B = waypoints(segmentIdx+1,:).';

d = B - A;

d2 = dot(d,d);

if d2 < 1e-12

    s = 0;
    projection = A;

else

    s = dot(position-A,d) / d2;

    % Keep projection on finite segment
    s = max(0,min(1,s));

    projection = A + s*d;

end


%% Cross-track error

crossTrackError = norm(position-projection);

%% ================================================================
% HANDLE PURELY VERTICAL SEGMENTS
% ================================================================

currentSegment = B - A;

horizontalSegmentLength = norm(currentSegment(1:2));

% For a vertical takeoff / landing segment, do not carry
% lookahead into the next horizontal segment.
if horizontalSegmentLength < 1e-6

    positionCmd = B;

    yawCmd = yawReference;

    info.projection      = projection;
    info.crossTrackError = crossTrackError;
    info.pathProgress    = s;
    info.targetSegment   = segmentIdx;

    return;

end

%% ================================================================
% WALK LOOKAHEAD DISTANCE ALONG POLYLINE
% ================================================================

remaining = P.lookaheadDistance;

target = projection;

j = segmentIdx;

targetSegment = segmentIdx;

firstSegment = true;


while remaining > 0 && j <= nWaypoints-1

    if firstSegment

        segmentStart = projection;
        firstSegment = false;

    else

        segmentStart = waypoints(j,:).';

    end

    segmentEnd = waypoints(j+1,:).';

    segmentVector = segmentEnd - segmentStart;

    segmentLength = norm(segmentVector);


    if segmentLength < 1e-9

        j = j + 1;
        continue;

    end


    if remaining <= segmentLength

        target = segmentStart ...
               + (remaining/segmentLength)*segmentVector;

        targetSegment = j;

        remaining = 0;

    else

        remaining = remaining - segmentLength;

        target = segmentEnd;

        targetSegment = j;

        j = j + 1;

    end

end


%% If lookahead extends beyond final waypoint

if remaining > 0

    target = waypoints(end,:).';

    targetSegment = nWaypoints-1;

end


positionCmd = target;


%% ================================================================
% YAW COMMAND
% ================================================================

deltaNE = positionCmd(1:2) - position(1:2);

horizontalDistance = norm(deltaNE);


if horizontalDistance > 1e-3

    yawCmd = atan2( ...
        deltaNE(2), ...
        deltaNE(1));

else

    yawCmd = yawReference;

end


%% Diagnostics

info.projection       = projection;
info.crossTrackError  = crossTrackError;
info.pathProgress     = s;
info.targetSegment    = targetSegment;

end