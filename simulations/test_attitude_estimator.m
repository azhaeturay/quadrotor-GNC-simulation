%% =============================================================
%  TEST_ATTITUDE_ESTIMATOR
%
%  Tests complementary-filter attitude estimation using simulated
%  gyroscope and accelerometer measurements.
%
%  Navigation convention:
%       NED
%
%  Attitude convention:
%       3-2-1 Euler angles
%       [roll pitch yaw] = [phi theta psi]
%
% =============================================================

clear;
clc;
close all;


%% =============================================================
%  PROJECT PATH
% =============================================================

projectRoot = fileparts(fileparts(mfilename('fullpath')));

addpath(genpath(projectRoot));


%% =============================================================
%  SIMULATION SETTINGS
% =============================================================

dt = 0.01;                  % 100 Hz IMU
T  = 20;

t = (0:dt:T).';

N = length(t);

g = 9.80665;


%% =============================================================
%  TRUE ATTITUDE TRAJECTORY
% =============================================================
%
% Smooth motion chosen so that roll, pitch, and yaw all vary.

phiTrue = ...
    deg2rad(15) .* sin(0.60 .* t);

thetaTrue = ...
    deg2rad(10) .* sin(0.40 .* t + 0.30);

psiTrue = ...
    deg2rad(30) .* sin(0.25 .* t);


%% True Euler angle derivatives

phiDotTrue = ...
    deg2rad(15) .* 0.60 .* cos(0.60 .* t);

thetaDotTrue = ...
    deg2rad(10) .* 0.40 .* cos(0.40 .* t + 0.30);

psiDotTrue = ...
    deg2rad(30) .* 0.25 .* cos(0.25 .* t);


%% =============================================================
%  SENSOR PARAMETERS
% =============================================================

gyroBias = deg2rad([ ...
     0.15;
    -0.10;
     0.08]);

gyroNoiseStd = deg2rad(0.08);

accelBias = [ ...
     0.03;
    -0.02;
     0.04];

accelNoiseStd = 0.04;


%% =============================================================
%  STORAGE
% =============================================================

gyroMeasHistory = zeros(3,N);

accelMeasHistory = zeros(3,N);

gyroOnlyHistory = zeros(3,N);

attitudeEstimateHistory = zeros(3,N);


%% =============================================================
%  INITIAL MEASUREMENT
% =============================================================

phi0   = phiTrue(1);
theta0 = thetaTrue(1);
psi0   = psiTrue(1);

R_BN = euler321_body_to_ned(phi0, theta0, psi0);

specificForceTrue = ...
    R_BN.' * [0; 0; -g];

accelInitial = ...
    specificForceTrue ...
    + accelBias ...
    + accelNoiseStd .* randn(3,1);


%% =============================================================
%  ESTIMATOR INITIALIZATION
% =============================================================

phiAccel0 = atan2( ...
    -accelInitial(2), ...
    -accelInitial(3));

thetaAccel0 = atan2( ...
    accelInitial(1), ...
    sqrt(accelInitial(2)^2 + accelInitial(3)^2));

state.euler = [ ...
    phiAccel0;
    thetaAccel0;
    0];

state.gyroBiasHat = zeros(3,1);

attitudeEstimateHistory(:,1) = state.euler;

gyroOnlyHistory(:,1) = state.euler;


%% Complementary filter time constant

tau = 0.50;


%% =============================================================
%  MAIN SIMULATION LOOP
% =============================================================

for k = 1:N

    phi   = phiTrue(k);
    theta = thetaTrue(k);
    psi   = psiTrue(k);

    phiDot   = phiDotTrue(k);
    thetaDot = thetaDotTrue(k);
    psiDot   = psiDotTrue(k);


    %% ---------------------------------------------------------
    %  TRUE BODY ANGULAR RATES
    %
    %  Convert Euler rates -> body rates p, q, r
    % ----------------------------------------------------------

    p = ...
        phiDot ...
        - psiDot * sin(theta);

    q = ...
        thetaDot*cos(phi) ...
        + psiDot*sin(phi)*cos(theta);

    r = ...
        -thetaDot*sin(phi) ...
        + psiDot*cos(phi)*cos(theta);

    gyroTrue = [p; q; r];


    %% ---------------------------------------------------------
    %  SIMULATED GYROSCOPE
    % ----------------------------------------------------------

    gyroMeas = ...
        gyroTrue ...
        + gyroBias ...
        + gyroNoiseStd .* randn(3,1);


    %% ---------------------------------------------------------
    %  SIMULATED ACCELEROMETER
    %
    %  No translational acceleration in this first test.
    % ----------------------------------------------------------

    R_BN = ...
        euler321_body_to_ned(phi, theta, psi);

    specificForceTrue = ...
        R_BN.' * [0; 0; -g];

    accelMeas = ...
        specificForceTrue ...
        + accelBias ...
        + accelNoiseStd .* randn(3,1);


    gyroMeasHistory(:,k) = gyroMeas;

    accelMeasHistory(:,k) = accelMeas;


    %% ---------------------------------------------------------
    %  ESTIMATOR UPDATE
    % ----------------------------------------------------------

    if k > 1

        [eulerHat, state] = ...
            attitude_estimator_step( ...
                gyroMeas, ...
                accelMeas, ...
                state, ...
                dt, ...
                tau);

        attitudeEstimateHistory(:,k) = ...
            eulerHat;


        %% -----------------------------------------------------
        %  GYRO-ONLY ATTITUDE
        % ------------------------------------------------------

        eulerGyro = gyroOnlyHistory(:,k-1);

        phiG   = eulerGyro(1);
        thetaG = eulerGyro(2);
        psiG   = eulerGyro(3);

        pG = gyroMeas(1);
        qG = gyroMeas(2);
        rG = gyroMeas(3);

        cphi = cos(phiG);
        sphi = sin(phiG);

        ctheta = cos(thetaG);
        stheta = sin(thetaG);

        if abs(ctheta) < 1e-6
            ctheta = sign(ctheta + eps) * 1e-6;
        end

        phiDotG = ...
            pG ...
            + qG*sphi*stheta/ctheta ...
            + rG*cphi*stheta/ctheta;

        thetaDotG = ...
            qG*cphi ...
            - rG*sphi;

        psiDotG = ...
            qG*sphi/ctheta ...
            + rG*cphi/ctheta;

        gyroOnlyHistory(:,k) = [ ...
            wrap_to_pi_local(phiG   + phiDotG*dt);
            wrap_to_pi_local(thetaG + thetaDotG*dt);
            wrap_to_pi_local(psiG   + psiDotG*dt)];

    end

end


%% =============================================================
%  ESTIMATION ERRORS
% =============================================================

truth = [ ...
    phiTrue.';
    thetaTrue.';
    psiTrue.'];

errorComp = ...
    wrap_to_pi_local(attitudeEstimateHistory - truth);

errorGyro = ...
    wrap_to_pi_local(gyroOnlyHistory - truth);


%% =============================================================
%  METRICS
% =============================================================

compRMSE = rad2deg( ...
    sqrt(mean(errorComp.^2,2)));

gyroRMSE = rad2deg( ...
    sqrt(mean(errorGyro.^2,2)));


fprintf('\n');
fprintf('--- ATTITUDE ESTIMATOR TEST ---\n\n');

fprintf('Complementary-filter RMSE [deg]\n');
fprintf('Roll:  %.3f\n', compRMSE(1));
fprintf('Pitch: %.3f\n', compRMSE(2));
fprintf('Yaw:   %.3f\n\n', compRMSE(3));

fprintf('Gyro-only RMSE [deg]\n');
fprintf('Roll:  %.3f\n', gyroRMSE(1));
fprintf('Pitch: %.3f\n', gyroRMSE(2));
fprintf('Yaw:   %.3f\n\n', gyroRMSE(3));

fprintf('Final complementary yaw error: %.3f deg\n', ...
    rad2deg(errorComp(3,end)));


%% =============================================================
%  FIGURE 1 — ATTITUDE ESTIMATION
% =============================================================

figure;

tiledlayout(3,1);

angleNames = {'Roll \phi', 'Pitch \theta', 'Yaw \psi'};

for i = 1:3

    nexttile;

    plot(t, rad2deg(truth(i,:)), ...
        'LineWidth', 1.6);

    hold on;

    plot(t, rad2deg(gyroOnlyHistory(i,:)), ...
        '--', ...
        'LineWidth', 1.2);

    plot(t, rad2deg(attitudeEstimateHistory(i,:)), ...
        '-.', ...
        'LineWidth', 1.6);

    grid on;

    ylabel([angleNames{i}, ' [deg]']);

    if i == 1
        title('Attitude Estimation');
    end

    if i == 3
        xlabel('Time [s]');
    end

    legend( ...
        'Truth', ...
        'Gyro Only', ...
        'Complementary Estimate', ...
        'Location', ...
        'best');

end


%% =============================================================
%  FIGURE 2 — ESTIMATION ERROR
% =============================================================

figure;

tiledlayout(3,1);

for i = 1:3

    nexttile;

    plot(t, rad2deg(errorGyro(i,:)), ...
        '--', ...
        'LineWidth', 1.2);

    hold on;

    plot(t, rad2deg(errorComp(i,:)), ...
        'LineWidth', 1.6);

    yline(0, ':');

    grid on;

    ylabel([angleNames{i}, ' Error [deg]']);

    if i == 1
        title('Attitude Estimation Error');
    end

    if i == 3
        xlabel('Time [s]');
    end

    legend( ...
        'Gyro Only', ...
        'Complementary Filter', ...
        'Location', ...
        'best');

end


%% =============================================================
%  LOCAL FUNCTIONS
% =============================================================

function R_BN = euler321_body_to_ned(phi, theta, psi)

cphi = cos(phi);
sphi = sin(phi);

ctheta = cos(theta);
stheta = sin(theta);

cpsi = cos(psi);
spsi = sin(psi);

R_BN = [ ...
    ctheta*cpsi, ...
    sphi*stheta*cpsi - cphi*spsi, ...
    cphi*stheta*cpsi + sphi*spsi;

    ctheta*spsi, ...
    sphi*stheta*spsi + cphi*cpsi, ...
    cphi*stheta*spsi - sphi*cpsi;

    -stheta, ...
    sphi*ctheta, ...
    cphi*ctheta];

end


function angle = wrap_to_pi_local(angle)

angle = mod(angle + pi, 2*pi) - pi;

end