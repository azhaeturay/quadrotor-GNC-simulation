%% test_sensor_stream.m
% Step 7C - Multi-rate IMU/GPS measurement stream
%
% Simulates a moving vehicle and generates:
%   - IMU measurements at 100 Hz
%   - GPS measurements at 10 Hz
%
% The estimator will eventually consume this measurement stream.

clear;
clc;
close all;

%% ============================================================
%  PROJECT SETUP
% =============================================================

projectRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(projectRoot));

P = vehicle_params();

%% ============================================================
%  SENSOR TIMING
% =============================================================

imuRate = P.imuRate;
gpsRate = P.gpsRate;

imuDt = 1 / imuRate;
gpsDt = 1 / gpsRate;

gpsStride = round(imuRate / gpsRate);

%% Simulation duration

tFinal = 8;

t = (0:imuDt:tFinal)';
N = length(t);

%% ============================================================
%  PREALLOCATE TRUTH HISTORY
% =============================================================

xTrueHistory = zeros(N,12);
xDotTrueHistory = zeros(N,12);

%% ============================================================
%  PREALLOCATE SENSOR HISTORY
% =============================================================

imuGyroHistory = zeros(N,3);
imuAccelHistory = zeros(N,3);

gpsPositionHistory = nan(N,3);
gpsVelocityHistory = nan(N,3);

gpsAvailable = false(N,1);
imuAvailable = true(N,1);

%% ============================================================
%  GENERATE SENSOR STREAM
% =============================================================

for k = 1:N

    tk = t(k);

    % ---------------------------------------------------------
    % TRUE VEHICLE MOTION
    % ---------------------------------------------------------

    % Smooth horizontal trajectory.
    %
    % Vehicle attitude is kept level with yaw = 0 here so that
    % body axes align with NED axes.
    %
    % North:
    %   constant forward velocity
    %
    % East:
    %   sinusoidal lateral motion
    %
    % Down:
    %   constant altitude of 2 m

    north = 0.60 * tk;

    east = 0.75 * sin(0.50 * tk);

    down = -2;


    % ---------------------------------------------------------
    % TRUE VELOCITIES
    % ---------------------------------------------------------

    u = 0.60;

    v = 0.375 * cos(0.50 * tk);

    w = 0;


    % ---------------------------------------------------------
    % ATTITUDE
    % ---------------------------------------------------------

    phi   = 0;
    theta = 0;
    psi   = 0;

    p = 0;
    q = 0;
    r = 0;


    % ---------------------------------------------------------
    % BUILD TRUE STATE VECTOR
    %
    % x =
    % [N E D u v w phi theta psi p q r]'
    % ---------------------------------------------------------

    xTrue = [ ...
        north;
        east;
        down;
        u;
        v;
        w;
        phi;
        theta;
        psi;
        p;
        q;
        r];


    % ---------------------------------------------------------
    % TRUE STATE DERIVATIVE
    % ---------------------------------------------------------

    northDot = u;

    eastDot = v;

    downDot = 0;

    uDot = 0;

    vDot = -0.1875 * sin(0.50 * tk);

    wDot = 0;

    phiDot   = 0;
    thetaDot = 0;
    psiDot   = 0;

    pDot = 0;
    qDot = 0;
    rDot = 0;


    xDotTrue = [ ...
        northDot;
        eastDot;
        downDot;
        uDot;
        vDot;
        wDot;
        phiDot;
        thetaDot;
        psiDot;
        pDot;
        qDot;
        rDot];


    % Store truth

    xTrueHistory(k,:) = xTrue.';
    xDotTrueHistory(k,:) = xDotTrue.';


    % =========================================================
    % IMU MEASUREMENT
    % =========================================================

    imu = simulate_imu(xTrue, xDotTrue, P);

    imuGyroHistory(k,:) = imu.gyro.';
    imuAccelHistory(k,:) = imu.accel.';


    % =========================================================
    % GPS MEASUREMENT
    %
    % GPS only updates every gpsStride IMU samples.
    % =========================================================

    if mod(k-1, gpsStride) == 0

        gps = simulate_gps(xTrue, P);

        gpsPositionHistory(k,:) = gps.position.';
        gpsVelocityHistory(k,:) = gps.velocity.';

        gpsAvailable(k) = true;

    end

end


%% ============================================================
%  SENSOR UPDATE COUNTS
% =============================================================

numImuMeasurements = sum(imuAvailable);
numGpsMeasurements = sum(gpsAvailable);

fprintf('\n');
fprintf('--- SENSOR MEASUREMENT STREAM TEST ---\n\n');

fprintf('Simulation time: %.2f s\n', tFinal);

fprintf('IMU rate:       %.1f Hz\n', imuRate);
fprintf('GPS rate:       %.1f Hz\n\n', gpsRate);

fprintf('IMU measurements generated: %d\n', ...
    numImuMeasurements);

fprintf('GPS measurements generated: %d\n\n', ...
    numGpsMeasurements);


%% ============================================================
%  SHOW FIRST SENSOR EVENTS
% =============================================================

fprintf('--- FIRST SENSOR EVENTS ---\n');

fprintf('Time [s]     IMU     GPS\n');
fprintf('-------------------------\n');

numRowsToPrint = min(31,N);

for k = 1:numRowsToPrint

    fprintf('%6.2f        %d       %d\n', ...
        t(k), ...
        imuAvailable(k), ...
        gpsAvailable(k));

end


%% ============================================================
%  FIGURE 1 - TRUE TRAJECTORY VS GPS
% =============================================================

figure;

plot( ...
    xTrueHistory(:,2), ...
    xTrueHistory(:,1), ...
    'LineWidth',1.5);

hold on;

scatter( ...
    gpsPositionHistory(gpsAvailable,2), ...
    gpsPositionHistory(gpsAvailable,1), ...
    15, ...
    'filled');

xlabel('East Position [m]');
ylabel('North Position [m]');

title('Truth Trajectory vs GPS Measurements');

legend( ...
    'True Trajectory', ...
    'GPS Measurements', ...
    'Location','best');

grid on;

axis equal;


%% ============================================================
%  FIGURE 2 - ACCELEROMETER STREAM
% =============================================================

figure;

plot( ...
    t, ...
    imuAccelHistory(:,1), ...
    'LineWidth',1);

hold on;

plot( ...
    t, ...
    imuAccelHistory(:,2), ...
    'LineWidth',1);

plot( ...
    t, ...
    imuAccelHistory(:,3), ...
    'LineWidth',1);

xlabel('Time [s]');
ylabel('Specific Force [m/s^2]');

title('100 Hz IMU Accelerometer Measurement Stream');

legend( ...
    'a_x', ...
    'a_y', ...
    'a_z', ...
    'Location','best');

grid on;


%% ============================================================
%  FIGURE 3 - GPS NORTH POSITION VS TRUTH
% =============================================================

figure;

plot( ...
    t, ...
    xTrueHistory(:,1), ...
    'LineWidth',1.5);

hold on;

scatter( ...
    t(gpsAvailable), ...
    gpsPositionHistory(gpsAvailable,1), ...
    18, ...
    'filled');

xlabel('Time [s]');
ylabel('North Position [m]');

title('10 Hz GPS Position Measurement Stream');

legend( ...
    'True Position', ...
    'GPS Measurements', ...
    'Location','best');

grid on;