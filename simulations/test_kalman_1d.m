%% TEST_KALMAN_1D
%
% First Kalman-filter state-estimation test.
%
% Estimates North position and North velocity using:
%
%   100 Hz IMU acceleration
%   10 Hz GPS position
%
% GPS velocity is intentionally NOT used.
%
% State estimate:
%
%   xHat = [north position
%           north velocity]

clear;
clc;
close all;

rng(10);


%% ============================================================
%  PROJECT SETUP
% =============================================================

projectRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(projectRoot));

P = vehicle_params();


%% ============================================================
%  SENSOR SETTINGS
% =============================================================

imuRate = P.imuRate;
gpsRate = P.gpsRate;

dt = 1/imuRate;

gpsStride = round(imuRate/gpsRate);

accelNoiseStd = P.accelNoiseStd;

gpsPositionNoiseStd = P.gpsPositionNoiseStd(1);

% We know the simulated accelerometer bias in this introductory
% example, so remove it before passing acceleration to the filter.
%
% Later, the navigation estimator will estimate IMU bias itself.

accelBiasNorth = P.accelBias(1);


%% ============================================================
%  SIMULATION TIME
% =============================================================

simulationTime = 10;

t = (0:dt:simulationTime).';

N = length(t);


%% ============================================================
%  CREATE A SIMPLE 1-D TRUTH TRAJECTORY
%
%  0-2 s   : accelerate North
%  2-6 s   : coast
%  6-8 s   : decelerate
%  8-10 s  : stop
% =============================================================

aTrue = zeros(N,1);

aTrue(t < 2) = 0.40;

aTrue(t >= 6 & t < 8) = -0.40;


northTrue    = zeros(N,1);
northVelTrue = zeros(N,1);

for k = 2:N

    northTrue(k) = ...
        northTrue(k-1) + ...
        northVelTrue(k-1)*dt + ...
        0.5*aTrue(k-1)*dt^2;

    northVelTrue(k) = ...
        northVelTrue(k-1) + ...
        aTrue(k-1)*dt;

end


%% ============================================================
%  GENERATE SENSOR MEASUREMENTS
% =============================================================

imuAccelNorth = zeros(N,1);

gpsNorth = nan(N,1);

gpsAvailable = false(N,1);


for k = 1:N

    %% Build truth state

    xTruth = zeros(12,1);

    % NED position
    xTruth(1) = northTrue(k);

    % Hold aircraft at 2 m altitude
    xTruth(3) = -2;

    % Body velocity
    %
    % Since attitude is zero, body x aligns with North.

    xTruth(4) = northVelTrue(k);


    %% Truth state derivative

    xDotTruth = zeros(12,1);

    xDotTruth(1) = northVelTrue(k);

    xDotTruth(4) = aTrue(k);


    %% IMU measurement

    imu = simulate_imu(xTruth, xDotTruth, P);

    imuAccelNorth(k) = imu.accel(1);


    %% GPS measurement

    if mod(k-1, gpsStride) == 0

        gps = simulate_gps(xTruth, P);

        gpsNorth(k) = gps.position(1);

        gpsAvailable(k) = true;

    end

end


%% ============================================================
%  REMOVE KNOWN ACCELEROMETER BIAS
% =============================================================

accelForFilter = imuAccelNorth - accelBiasNorth;


%% ============================================================
%  INITIALIZE KALMAN FILTER
% =============================================================

firstGPS = find(gpsAvailable,1,'first');

% Initial state estimate
%
% Start position at first GPS measurement.
% Assume zero initial velocity.

xHat = [gpsNorth(firstGPS);
        0];


% Initial uncertainty
%
% Position uncertainty comes from GPS.
% Velocity uncertainty is intentionally fairly large.

Pcov = diag([gpsPositionNoiseStd^2, ...
             0.50^2]);


%% History

xHatHistory = zeros(2,N);

PHistory = zeros(2,2,N);

KHistory = nan(2,N);

innovationHistory = nan(N,1);


xHatHistory(:,1) = xHat;

PHistory(:,:,1) = Pcov;


%% ============================================================
%  KALMAN FILTER LOOP
% =============================================================

for k = 2:N

    % Acceleration measured over previous propagation interval

    accelInput = accelForFilter(k-1);


    [xHat, Pcov, info] = kalman_1d_step( ...
        xHat, ...
        Pcov, ...
        accelInput, ...
        gpsNorth(k), ...
        gpsAvailable(k), ...
        dt, ...
        accelNoiseStd, ...
        gpsPositionNoiseStd);


    %% Store results

    xHatHistory(:,k) = xHat;

    PHistory(:,:,k) = Pcov;

    if gpsAvailable(k)

        KHistory(:,k) = info.K;

        innovationHistory(k) = info.innovation;

    end

end


%% ============================================================
%  EXTRACT ESTIMATES
% =============================================================

northEstimate = xHatHistory(1,:).';

velocityEstimate = xHatHistory(2,:).';


%% Uncertainty bounds

positionSigma = zeros(N,1);

velocitySigma = zeros(N,1);

for k = 1:N

    positionSigma(k) = sqrt(PHistory(1,1,k));

    velocitySigma(k) = sqrt(PHistory(2,2,k));

end


%% ============================================================
%  PERFORMANCE METRICS
% =============================================================

positionError = northEstimate - northTrue;

velocityError = velocityEstimate - northVelTrue;


positionRMSE = sqrt(mean(positionError.^2));

velocityRMSE = sqrt(mean(velocityError.^2));


fprintf('\n--- 1-D KALMAN FILTER TEST ---\n');

fprintf('IMU rate: %.1f Hz\n', imuRate);
fprintf('GPS rate: %.1f Hz\n', gpsRate);

fprintf('\nPosition RMSE: %.4f m\n', positionRMSE);
fprintf('Velocity RMSE: %.4f m/s\n', velocityRMSE);

fprintf('\nFinal true position:      %.4f m\n', northTrue(end));
fprintf('Final estimated position: %.4f m\n', northEstimate(end));

fprintf('\nFinal true velocity:      %.4f m/s\n', northVelTrue(end));
fprintf('Final estimated velocity: %.4f m/s\n', velocityEstimate(end));


%% ============================================================
%  FIGURE 1 — POSITION ESTIMATE
% =============================================================

figure;

hold on;

plot(t, northTrue, ...
    'LineWidth', 2);

scatter( ...
    t(gpsAvailable), ...
    gpsNorth(gpsAvailable), ...
    18, ...
    'filled');

plot(t, northEstimate, ...
    '--', ...
    'LineWidth', 2);

xlabel('Time [s]');
ylabel('North Position [m]');

title('Kalman Filter — North Position Estimate');

legend( ...
    'Truth', ...
    'GPS Measurements', ...
    'Kalman Estimate', ...
    'Location','best');

grid on;


%% ============================================================
%  FIGURE 2 — VELOCITY ESTIMATE
% =============================================================

figure;

plot(t, northVelTrue, ...
    'LineWidth', 2);

hold on;

plot(t, velocityEstimate, ...
    '--', ...
    'LineWidth', 2);

xlabel('Time [s]');
ylabel('North Velocity [m/s]');

title('Kalman Filter — North Velocity Estimate');

legend( ...
    'Truth', ...
    'Kalman Estimate', ...
    'Location','best');

grid on;


%% ============================================================
%  FIGURE 3 — ESTIMATION ERROR / UNCERTAINTY
% =============================================================

figure;

tiledlayout(2,1);


%% Position error

nexttile;

plot(t, positionError, ...
    'LineWidth', 1.5);

hold on;

plot(t, 3*positionSigma, ...
    '--', ...
    'LineWidth', 1.2);

plot(t, -3*positionSigma, ...
    '--', ...
    'LineWidth', 1.2);

yline(0, ':');

xlabel('Time [s]');
ylabel('Position Error [m]');

title('Position Estimation Error');

legend( ...
    'Error', ...
    '+3\sigma', ...
    '-3\sigma', ...
    'Location','best');

grid on;


%% Velocity error

nexttile;

plot(t, velocityError, ...
    'LineWidth', 1.5);

hold on;

plot(t, 3*velocitySigma, ...
    '--', ...
    'LineWidth', 1.2);

plot(t, -3*velocitySigma, ...
    '--', ...
    'LineWidth', 1.2);

yline(0, ':');

xlabel('Time [s]');
ylabel('Velocity Error [m/s]');

title('Velocity Estimation Error');

legend( ...
    'Error', ...
    '+3\sigma', ...
    '-3\sigma', ...
    'Location','best');

grid on;


%% ============================================================
%  FIGURE 4 — KALMAN GAIN
% =============================================================

validGain = ~isnan(KHistory(1,:));

figure;

stairs( ...
    t(validGain), ...
    KHistory(1,validGain), ...
    'LineWidth',1.5);

hold on;

stairs( ...
    t(validGain), ...
    KHistory(2,validGain), ...
    'LineWidth',1.5);

xlabel('Time [s]');
ylabel('Kalman Gain');

title('Kalman Gain at GPS Updates');

legend( ...
    'Position Gain', ...
    'Velocity Gain', ...
    'Location','best');

grid on;