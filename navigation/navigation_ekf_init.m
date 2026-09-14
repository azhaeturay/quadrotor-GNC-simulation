function navState = navigation_ekf_init(P)
% NAVIGATION_EKF_INIT
%
% Initialize a coupled 15-state INS/GPS attitude-navigation EKF.
%
% State:
%
% x =
%
% [ pN
%   pE
%   pD
%
%   vN
%   vE
%   vD
%
%   phi
%   theta
%   psi
%
%   bgx
%   bgy
%   bgz
%
%   bax
%   bay
%   baz ]
%
% where:
%
%   p       = NED position
%   v       = NED velocity
%   Euler   = 3-2-1 attitude
%   bg      = gyro bias estimate
%   ba      = accelerometer bias estimate
%
% ---------------------------------------------------------------


%% ================================================================
% CONSTANTS
% ================================================================

if isfield(P,'g')

    navState.cfg.g = P.g;

else

    navState.cfg.g = 9.80665;

end


%% ================================================================
% GPS MEASUREMENT NOISE
% ================================================================

navState.cfg.gpsPositionStd = ...
    [0.25;
     0.25;
     0.40];


navState.cfg.gpsVelocityStd = ...
    [0.05;
     0.05;
     0.08];


%% ================================================================
% IMU PROCESS UNCERTAINTY
%
% Slightly inflated relative to pure sensor white noise.
%
% This lets the filter account for:
%
%   sensor noise
%   model mismatch
%   discretization
%   unmodeled vehicle effects
% ================================================================

navState.cfg.accelInputStd = ...
    [0.08;
     0.08;
     0.10];


navState.cfg.gyroInputStd = ...
    deg2rad( ...
    [0.15;
     0.15;
     0.15]);


%% ================================================================
% IMU BIAS RANDOM WALK
% ================================================================

navState.cfg.gyroBiasRWStd = ...
    deg2rad( ...
    [0.01;
     0.01;
     0.01]);


navState.cfg.accelBiasRWStd = ...
    [0.005;
     0.005;
     0.005];


%% ================================================================
% MAGNETOMETER HEADING UNCERTAINTY
%
% We deliberately do not trust the magnetometer as aggressively as
% the gyro.
% ================================================================

navState.cfg.magYawStd = ...
    deg2rad(4.0);


%% ================================================================
% EKF STATE
% ================================================================

navState.x = zeros(15,1);


%% ================================================================
% INITIAL COVARIANCE
% ================================================================

positionStd0 = ...
    [0.50;
     0.50;
     0.80];


velocityStd0 = ...
    [0.20;
     0.20;
     0.30];


attitudeStd0 = ...
    deg2rad( ...
    [8;
     8;
     12]);


gyroBiasStd0 = ...
    deg2rad( ...
    [0.50;
     0.50;
     0.50]);


accelBiasStd0 = ...
    [0.25;
     0.25;
     0.25];


navState.P = ...
    diag( ...
    [positionStd0.^2;
     velocityStd0.^2;
     attitudeStd0.^2;
     gyroBiasStd0.^2;
     accelBiasStd0.^2]);


%% ================================================================
% INITIALIZATION FLAGS
% ================================================================

navState.initialized = false;

navState.lastGPSUsed = false;


%% ================================================================
% DIAGNOSTICS
% ================================================================

navState.lastAccelNED = zeros(3,1);

navState.lastGPSInnovation = nan(6,1);

navState.lastMagInnovation = nan;

end