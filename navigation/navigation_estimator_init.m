function navState = navigation_estimator_init(P)
% NAVIGATION_ESTIMATOR_INIT
%
% Initializes the integrated quadrotor navigation estimator.
%
% Estimated navigation state:
%
%   position NED   [m]
%   velocity NED   [m/s]
%   Euler attitude [rad]
%
% The estimator combines:
%
%   - IMU gyroscope
%   - IMU accelerometer
%   - GPS position
%   - GPS velocity
%   - magnetometer
%
% Position / velocity:
%   Linear Kalman filter
%
% Attitude:
%   Complementary filter
%
% ---------------------------------------------------------------

if nargin < 1
    P = struct();
end


%% ================================================================
%  CONSTANTS
% ================================================================

if isfield(P,'g')
    navState.cfg.g = P.g;
else
    navState.cfg.g = 9.80665;
end


%% ================================================================
%  ATTITUDE FILTER SETTINGS
% ================================================================

% These are the same values we used in the successful
% magnetometer-aided attitude estimator test.

navState.cfg.alphaRP  = 0.980;
navState.cfg.alphaYaw = 0.995;


%% ================================================================
%  POSITION KALMAN FILTER SETTINGS
% ================================================================

% GPS position measurement standard deviation [m]
%
% N, E, D

navState.cfg.gpsPositionStd = ...
    [0.25;
     0.25;
     0.40];


% GPS velocity measurement standard deviation [m/s]

navState.cfg.gpsVelocityStd = ...
    [0.05;
     0.05;
     0.08];


% Effective acceleration uncertainty used by the Kalman filter.
%
% This is not simply accelerometer white noise.
%
% It also accounts for:
%
%   attitude error
%   accelerometer bias
%   imperfect vehicle model
%   unmodeled disturbances

navState.cfg.accelProcessStd = ...
    [0.35;
     0.35;
     0.45];


%% ================================================================
%  POSITION / VELOCITY STATE
%
%  xPV =
%
%  [ pN
%    pE
%    pD
%    vN
%    vE
%    vD ]
%
% ================================================================

navState.xPV = zeros(6,1);


%% Initial covariance

positionSigma0 = 1.0;    % [m]
velocitySigma0 = 0.5;    % [m/s]

navState.PPV = diag([ ...
    positionSigma0^2 ...
    positionSigma0^2 ...
    positionSigma0^2 ...
    velocitySigma0^2 ...
    velocitySigma0^2 ...
    velocitySigma0^2]);


%% ================================================================
%  ATTITUDE STATE
%
%  Euler 3-2-1:
%
%  [ roll
%    pitch
%    yaw ]
%
% ================================================================

navState.euler = zeros(3,1);


%% Initialization flags

navState.attitudeInitialized = false;
navState.gpsInitialized      = false;


%% Diagnostics

navState.lastAccelNED = zeros(3,1);
navState.lastGPSUsed  = false;

end