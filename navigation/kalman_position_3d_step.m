function [xHat, P] = kalman_position_3d_step( ...
    xHat, P, accelNED, gpsPosition, gpsAvailable, dt, ...
    accelNoiseStd, gpsPositionStd)

% KALMAN_POSITION_3D_STEP
%
% 3-D translational Kalman filter for quadrotor navigation.
%
% State:
%
%   x = [pN
%        pE
%        pD
%        vN
%        vE
%        vD]
%
% Input:
%   accelNED      - inertial acceleration expressed in NED [m/s^2]
%
% Measurement:
%   gpsPosition   - GPS NED position [m]
%
% The filter predicts at the IMU rate and corrects whenever
% a new GPS measurement becomes available.


%% ------------------------------------------------------------
%  STATE TRANSITION MODEL
% -------------------------------------------------------------

I3 = eye(3);
Z3 = zeros(3);

F = [I3, dt*I3;
     Z3, I3];

G = [0.5*dt^2*I3;
     dt*I3];


%% ------------------------------------------------------------
%  PROCESS NOISE
% -------------------------------------------------------------

% Accelerometer uncertainty drives uncertainty in position
% and velocity propagation.

Qa = diag(accelNoiseStd(:).^2);

Q = G * Qa * G.';


%% ------------------------------------------------------------
%  PREDICTION
% -------------------------------------------------------------

xHat = F*xHat + G*accelNED(:);

P = F*P*F.' + Q;


%% ------------------------------------------------------------
%  GPS CORRECTION
% -------------------------------------------------------------

if gpsAvailable

    % GPS measures position only.

    H = [I3, Z3];

    R = diag(gpsPositionStd(:).^2);

    % Innovation / measurement residual

    innovation = gpsPosition(:) - H*xHat;

    % Innovation covariance

    S = H*P*H.' + R;

    % Kalman gain

    K = P*H.'/S;

    % Correct state estimate

    xHat = xHat + K*innovation;

    % Joseph covariance update
    %
    % Slightly more numerically robust than:
    % P = (I - K*H)*P

    I6 = eye(6);

    P = (I6 - K*H)*P*(I6 - K*H).' + K*R*K.';

end

end