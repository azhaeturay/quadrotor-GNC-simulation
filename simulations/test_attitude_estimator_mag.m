%% TEST_ATTITUDE_ESTIMATOR_MAG
%
% Demonstrates full 3-axis attitude estimation using:
%
%   - gyroscope
%   - accelerometer
%   - magnetometer
%
% Roll / pitch are corrected using gravity.
% Yaw is corrected using Earth's magnetic field.

clear;
clc;
close all;

rng(10);


%% ========================================================================
% SETTINGS
% =========================================================================

dt = 0.01;

imuRate = 1 / dt;

magRate = 50;

magStride = round(imuRate / magRate);

tEnd = 20;

t = 0:dt:tEnd;

N = length(t);

g = 9.80665;


%% ========================================================================
% SENSOR PARAMETERS
% =========================================================================

% Gyroscope bias [rad/s]
gyroBiasDeg = [ ...
     0.15;
    -0.10;
     0.08 ];

gyroBias = deg2rad(gyroBiasDeg);

gyroNoiseStd = deg2rad(0.08);


% Accelerometer bias [m/s^2]
accelBias = [ ...
     0.03;
    -0.02;
     0.04 ];

accelNoiseStd = 0.04;


% Normalized magnetic-field vector in NED coordinates.
%
% The small East component represents magnetic declination.
fieldNED = [ ...
    0.45;
    0.08;
    0.89 ];

fieldNED = fieldNED / norm(fieldNED);


% Magnetometer bias.
magBias = [ ...
     0.004;
    -0.003;
     0.002 ];

magNoiseStd = 0.008;


%% ========================================================================
% COMPLEMENTARY FILTER GAINS
% =========================================================================

% Close to 1 = trust gyro heavily over short time scales.

alphaRP = 0.98;

% Slightly slower yaw correction.
alphaYaw = 0.995;


%% ========================================================================
% TRUTH ATTITUDE
% =========================================================================

phiTrue = deg2rad( ...
    15 * sin(0.60*t));

thetaTrue = deg2rad( ...
    10 * sin(0.40*t + 0.35));

psiTrue = deg2rad( ...
    35 * sin(0.22*t));


%% Euler-angle derivatives

phiDotTrue = gradient(phiTrue, dt);

thetaDotTrue = gradient(thetaTrue, dt);

psiDotTrue = gradient(psiTrue, dt);


%% ========================================================================
% TRUE BODY ANGULAR RATES
% =========================================================================

bodyRatesTrue = zeros(3,N);

for k = 1:N

    phi   = phiTrue(k);
    theta = thetaTrue(k);

    eulerDot = [ ...
        phiDotTrue(k);
        thetaDotTrue(k);
        psiDotTrue(k)];

    % Euler-rate -> body-rate transformation.
    E = [ ...
        1, 0, -sin(theta);
        0, cos(phi), sin(phi)*cos(theta);
        0, -sin(phi), cos(phi)*cos(theta)];

    bodyRatesTrue(:,k) = E * eulerDot;

end


%% ========================================================================
% SENSOR STORAGE
% =========================================================================

gyroMeas = zeros(3,N);

accelMeas = zeros(3,N);

magMeas = nan(3,N);

magAvailable = false(1,N);


%% ========================================================================
% GENERATE SENSOR DATA
% =========================================================================

for k = 1:N

    phi   = phiTrue(k);
    theta = thetaTrue(k);
    psi   = psiTrue(k);

    %% Gyroscope

    gyroMeas(:,k) = ...
        bodyRatesTrue(:,k) + ...
        gyroBias + ...
        gyroNoiseStd .* randn(3,1);


    %% Accelerometer

    Rbn = euler321_body_to_ned( ...
        phi, theta, psi);

    % Stationary translational acceleration assumption.
    %
    % Accelerometer measures specific force:
    %
    %       f_b = R_nb (a_n - g_n)
    %
    gravityNED = [0; 0; g];

    accelTrue = ...
        Rbn' * (-gravityNED);

    accelMeas(:,k) = ...
        accelTrue + ...
        accelBias + ...
        accelNoiseStd .* randn(3,1);


    %% Magnetometer

    if mod(k-1,magStride) == 0

        xTruth = zeros(12,1);

        xTruth(7:9) = [ ...
            phi;
            theta;
            psi];

        mag = simulate_magnetometer( ...
            xTruth, ...
            fieldNED, ...
            magBias, ...
            magNoiseStd);

        magMeas(:,k) = mag.fieldBody;

        magAvailable(k) = true;

    end

end


%% ========================================================================
% RUN ESTIMATORS
% =========================================================================

gyroOnly = zeros(3,N);

compNoMag = zeros(3,N);

compMag = zeros(3,N);


% Give all estimators the same initial estimate.
initialEstimate = [ ...
    deg2rad(2);
    deg2rad(-2);
    deg2rad(5)];

gyroOnly(:,1) = initialEstimate;

compNoMag(:,1) = initialEstimate;

compMag(:,1) = initialEstimate;


for k = 2:N

    %% ---------------------------------------------------------------
    % Gyro only
    % ----------------------------------------------------------------

    gyroOnly(:,k) = propagate_attitude( ...
        gyroOnly(:,k-1), ...
        gyroMeas(:,k), ...
        dt);


    %% ---------------------------------------------------------------
    % Existing accel + gyro complementary estimator
    %
    % We emulate it here so the comparison remains self-contained.
    % ----------------------------------------------------------------

    pred = propagate_attitude( ...
        compNoMag(:,k-1), ...
        gyroMeas(:,k), ...
        dt);

    ax = accelMeas(1,k);
    ay = accelMeas(2,k);
    az = accelMeas(3,k);

    phiAccel = atan2(-ay,-az);

    thetaAccel = atan2( ...
        ax, sqrt(ay^2 + az^2));

    phiCorrected = pred(1) + ...
        (1-alphaRP) * ...
        wrap_to_pi_local(phiAccel-pred(1));

    thetaCorrected = pred(2) + ...
        (1-alphaRP) * ...
        wrap_to_pi_local(thetaAccel-pred(2));

    compNoMag(:,k) = [ ...
        wrap_to_pi_local(phiCorrected);
        wrap_to_pi_local(thetaCorrected);
        wrap_to_pi_local(pred(3))];


    %% ---------------------------------------------------------------
    % Full gyro + accel + magnetometer estimator
    % ----------------------------------------------------------------

    if magAvailable(k)

        currentMag = magMeas(:,k);

    else

        currentMag = zeros(3,1);

    end

    compMag(:,k) = ...
        attitude_estimator_mag_step( ...
            compMag(:,k-1), ...
            gyroMeas(:,k), ...
            accelMeas(:,k), ...
            currentMag, ...
            magAvailable(k), ...
            dt, ...
            alphaRP, ...
            alphaYaw, ...
            fieldNED);

end


%% ========================================================================
% ERRORS
% =========================================================================

truth = [ ...
    phiTrue;
    thetaTrue;
    psiTrue];

errorGyro = zeros(3,N);

errorNoMag = zeros(3,N);

errorMag = zeros(3,N);

for k = 1:N

    errorGyro(:,k) = wrap_to_pi_local( ...
        gyroOnly(:,k) - truth(:,k));

    errorNoMag(:,k) = wrap_to_pi_local( ...
        compNoMag(:,k) - truth(:,k));

    errorMag(:,k) = wrap_to_pi_local( ...
        compMag(:,k) - truth(:,k));

end


%% RMSE

rmseGyro = sqrt(mean(rad2deg(errorGyro).^2,2));

rmseNoMag = sqrt(mean(rad2deg(errorNoMag).^2,2));

rmseMag = sqrt(mean(rad2deg(errorMag).^2,2));


fprintf('\n--- MAGNETOMETER-AIDED ATTITUDE ESTIMATOR ---\n');

fprintf('\nGyro-only RMSE [deg]\n');
fprintf('Roll:  %.3f\n',rmseGyro(1));
fprintf('Pitch: %.3f\n',rmseGyro(2));
fprintf('Yaw:   %.3f\n',rmseGyro(3));

fprintf('\nAccel + gyro RMSE [deg]\n');
fprintf('Roll:  %.3f\n',rmseNoMag(1));
fprintf('Pitch: %.3f\n',rmseNoMag(2));
fprintf('Yaw:   %.3f\n',rmseNoMag(3));

fprintf('\nAccel + gyro + magnetometer RMSE [deg]\n');
fprintf('Roll:  %.3f\n',rmseMag(1));
fprintf('Pitch: %.3f\n',rmseMag(2));
fprintf('Yaw:   %.3f\n',rmseMag(3));

fprintf('\nFinal yaw errors [deg]\n');

fprintf('Gyro only:           %.3f\n', ...
    rad2deg(errorGyro(3,end)));

fprintf('Accel + gyro:        %.3f\n', ...
    rad2deg(errorNoMag(3,end)));

fprintf('Accel + gyro + mag:  %.3f\n\n', ...
    rad2deg(errorMag(3,end)));


%% ========================================================================
% FIGURE 1 — ATTITUDE ESTIMATION
% =========================================================================

figure;

subplot(3,1,1);

plot(t,rad2deg(phiTrue),'LineWidth',1.7);
hold on;

plot(t,rad2deg(compNoMag(1,:)),'--','LineWidth',1.4);

plot(t,rad2deg(compMag(1,:)),'-.','LineWidth',1.4);

grid on;

ylabel('Roll \phi [deg]');

title('Roll Estimation');

legend( ...
    'Truth', ...
    'Gyro + Accel', ...
    'Gyro + Accel + Mag', ...
    'Location','best');


subplot(3,1,2);

plot(t,rad2deg(thetaTrue),'LineWidth',1.7);
hold on;

plot(t,rad2deg(compNoMag(2,:)),'--','LineWidth',1.4);

plot(t,rad2deg(compMag(2,:)),'-.','LineWidth',1.4);

grid on;

ylabel('Pitch \theta [deg]');

title('Pitch Estimation');


subplot(3,1,3);

plot(t,rad2deg(psiTrue),'LineWidth',1.7);
hold on;

plot(t,rad2deg(compNoMag(3,:)),'--','LineWidth',1.4);

plot(t,rad2deg(compMag(3,:)),'-.','LineWidth',1.4);

grid on;

xlabel('Time [s]');
ylabel('Yaw \psi [deg]');

title('Yaw Estimation');

legend( ...
    'Truth', ...
    'Gyro + Accel', ...
    'Gyro + Accel + Mag', ...
    'Location','best');


%% ========================================================================
% FIGURE 2 — ATTITUDE ERRORS
% =========================================================================

figure;

names = {'Roll','Pitch','Yaw'};

for axisIdx = 1:3

    subplot(3,1,axisIdx);

    plot( ...
        t, ...
        rad2deg(errorNoMag(axisIdx,:)), ...
        '--', ...
        'LineWidth',1.4);

    hold on;

    plot( ...
        t, ...
        rad2deg(errorMag(axisIdx,:)), ...
        'LineWidth',1.5);

    yline(0,':');

    grid on;

    ylabel('Error [deg]');

    title( ...
        sprintf('%s Estimation Error', ...
        names{axisIdx}));

    legend( ...
        'Without Magnetometer', ...
        'With Magnetometer', ...
        'Location','best');

end

xlabel('Time [s]');


%% ========================================================================
% FIGURE 3 — YAW ONLY
% =========================================================================

figure;

plot( ...
    t, ...
    rad2deg(psiTrue), ...
    'LineWidth',1.8);

hold on;

plot( ...
    t, ...
    rad2deg(compNoMag(3,:)), ...
    '--', ...
    'LineWidth',1.5);

plot( ...
    t, ...
    rad2deg(compMag(3,:)), ...
    '-.', ...
    'LineWidth',1.5);

grid on;

xlabel('Time [s]');
ylabel('Yaw \psi [deg]');

title('Effect of Magnetometer Heading Correction');

legend( ...
    'Truth', ...
    'Without Magnetometer', ...
    'With Magnetometer', ...
    'Location','best');


%% ========================================================================
% LOCAL FUNCTIONS
% =========================================================================

function eulerNext = propagate_attitude( ...
    eulerCurrent, gyro, dt)

    phi   = eulerCurrent(1);
    theta = eulerCurrent(2);

    p = gyro(1);
    q = gyro(2);
    r = gyro(3);

    ctheta = cos(theta);

    if abs(ctheta) < 1e-6
        ctheta = sign(ctheta)*1e-6;
    end

    ttheta = sin(theta)/ctheta;

    phiDot = ...
        p + q*sin(phi)*ttheta + ...
        r*cos(phi)*ttheta;

    thetaDot = ...
        q*cos(phi) - r*sin(phi);

    psiDot = ...
        q*sin(phi)/ctheta + ...
        r*cos(phi)/ctheta;

    eulerNext = eulerCurrent + ...
        dt*[phiDot;thetaDot;psiDot];

    eulerNext = ...
        wrap_to_pi_local(eulerNext);
end


function Rbn = euler321_body_to_ned( ...
    phi, theta, psi)

    cphi = cos(phi);
    sphi = sin(phi);

    ctheta = cos(theta);
    stheta = sin(theta);

    cpsi = cos(psi);
    spsi = sin(psi);

    Rbn = [ ...
        ctheta*cpsi, ...
        sphi*stheta*cpsi-cphi*spsi, ...
        cphi*stheta*cpsi+sphi*spsi; ...

        ctheta*spsi, ...
        sphi*stheta*spsi+cphi*cpsi, ...
        cphi*stheta*spsi-sphi*cpsi; ...

        -stheta, ...
        sphi*ctheta, ...
        cphi*ctheta ];
end


function angle = wrap_to_pi_local(angle)

    angle = mod(angle + pi,2*pi) - pi;

end