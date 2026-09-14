function [xHat, Pcov, info] = kalman_1d_step( ...
    xHat, Pcov, accelInput, gpsPosition, gpsAvailable, ...
    dt, accelNoiseStd, gpsPositionNoiseStd)
%KALMAN_1D_STEP
%
% One prediction/correction step for a simple 1-D Kalman filter.
%
% State:
%
%   xHat = [position
%           velocity]
%
% Input:
%
%   accelInput     measured acceleration [m/s^2]
%
% Measurement:
%
%   gpsPosition    GPS position [m]
%
% The filter predicts at the IMU rate and only performs a
% measurement correction when gpsAvailable == true.

%% ------------------------------------------------------------
%  SYSTEM MODEL
% -------------------------------------------------------------

% State transition:
%
% p(k+1) = p(k) + v(k)dt
% v(k+1) = v(k)

F = [1, dt;
     0, 1];

% Acceleration input matrix:
%
% p(k+1) += 1/2 a dt^2
% v(k+1) += a dt

B = [0.5*dt^2;
     dt];

% GPS measures position only

H = [1, 0];


%% ------------------------------------------------------------
%  NOISE MODELS
% -------------------------------------------------------------

% Accelerometer uncertainty becomes process uncertainty

Q = accelNoiseStd^2 * (B * B.');

% GPS position measurement variance

R = gpsPositionNoiseStd^2;


%% ------------------------------------------------------------
%  PREDICTION
% -------------------------------------------------------------

xPred = F*xHat + B*accelInput;

PPred = F*Pcov*F.' + Q;


%% ------------------------------------------------------------
%  GPS CORRECTION
% -------------------------------------------------------------

innovation = NaN;
S          = NaN;
K          = [0; 0];

if gpsAvailable

    % Difference between what GPS says and what we predicted

    innovation = gpsPosition - H*xPred;

    % Innovation covariance

    S = H*PPred*H.' + R;

    % Kalman gain

    K = PPred*H.' / S;

    % Correct state estimate

    xHat = xPred + K*innovation;

    % Joseph-form covariance update
    % Numerically safer than P = (I-KH)P

    I = eye(2);

    Pcov = (I - K*H)*PPred*(I - K*H).' + K*R*K.';

else

    % No GPS measurement:
    % prediction becomes our new estimate

    xHat = xPred;
    Pcov = PPred;

end


%% ------------------------------------------------------------
%  NUMERICAL CLEANUP
% -------------------------------------------------------------

% Keep covariance exactly symmetric despite roundoff

Pcov = 0.5*(Pcov + Pcov.');


%% ------------------------------------------------------------
%  DEBUG / ANALYSIS OUTPUT
% -------------------------------------------------------------

info.xPred      = xPred;
info.PPred      = PPred;
info.innovation = innovation;
info.S          = S;
info.K          = K;

end