%% TEST_NAVIGATION_ESTIMATOR
%
% Integrated navigation-estimator validation.
%
% Tests:
%
%   - 100 Hz IMU
%   - 10 Hz GPS
%   - magnetometer
%   - attitude estimation
%   - NED acceleration reconstruction
%   - 3-D Kalman position / velocity estimation
%
% The estimator does NOT receive truth-state feedback.
%
% ---------------------------------------------------------------

clear;
clc;
close all;

rng(12);


%% ================================================================
%  PROJECT PATH
% ================================================================

projectRoot = fileparts( ...
    fileparts(mfilename('fullpath')));

addpath(genpath(projectRoot));


%% ================================================================
%  PARAMETERS
% ================================================================

P = vehicle_params();

if ~isfield(P,'g')
    P.g = 9.80665;
end

g = P.g;


%% ================================================================
%  SIMULATION SETTINGS
% ================================================================

dt = 0.01;             % 100 Hz IMU
T  = 25;

t = 0:dt:T;

N = length(t);

gpsRate   = 10;
gpsStride = round(1/(gpsRate*dt));


fprintf('\n');
fprintf('--- INTEGRATED NAVIGATION ESTIMATOR TEST ---\n\n');

fprintf('Simulation time: %.1f s\n', T);
fprintf('IMU rate: %.1f Hz\n', 1/dt);
fprintf('GPS rate: %.1f Hz\n\n', gpsRate);


%% ================================================================
%  TRUE TRANSLATIONAL TRAJECTORY
% ================================================================

% Smooth 3-D trajectory

pN = ...
    0.22*t + ...
    1.10*sin(0.24*t);

pE = ...
    1.40*(1-cos(0.22*t));

pD = ...
    -2.0 + ...
    0.35*sin(0.18*t);


positionTrue = ...
    [pN;
     pE;
     pD];


%% Velocity

velocityTrue = zeros(3,N);

for axisIdx = 1:3

    velocityTrue(axisIdx,:) = ...
        gradient( ...
        positionTrue(axisIdx,:), ...
        dt);

end


%% Acceleration

accelTrue = zeros(3,N);

for axisIdx = 1:3

    accelTrue(axisIdx,:) = ...
        gradient( ...
        velocityTrue(axisIdx,:), ...
        dt);

end


%% ================================================================
%  TRUE ATTITUDE
% ================================================================

phiTrue = ...
    deg2rad(10) * ...
    sin(0.42*t);

thetaTrue = ...
    deg2rad(7) * ...
    sin(0.31*t + 0.30);

psiTrue = ...
    deg2rad(4)*t + ...
    deg2rad(8)*sin(0.16*t);


attitudeTrue = ...
    [phiTrue;
     thetaTrue;
     psiTrue];


%% Euler derivatives

eulerDot = zeros(3,N);

for axisIdx = 1:3

    eulerDot(axisIdx,:) = ...
        gradient( ...
        attitudeTrue(axisIdx,:), ...
        dt);

end


%% ================================================================
%  SENSOR CHARACTERISTICS
%
%  These match the general sensor levels we've already been using.
% ================================================================

gyroBias = deg2rad( ...
    [ 0.15;
     -0.10;
      0.08]);

gyroNoiseStd = deg2rad( ...
    [0.08;
     0.08;
     0.08]);


accelBias = ...
    [ 0.03;
     -0.02;
      0.04];

accelNoiseStd = ...
    [0.04;
     0.04;
     0.04];


gpsPositionStd = ...
    [0.25;
     0.25;
     0.40];

gpsVelocityStd = ...
    [0.05;
     0.05;
     0.08];


magBias = ...
    [ 0.015;
     -0.010;
      0.008];

magNoiseStd = 0.010;


%% ================================================================
%  EARTH MAGNETIC FIELD
%
%  Arbitrary normalized NED magnetic vector.
%
%  Positive D means field points downward.
% ================================================================

magNED = ...
    [0.45;
     0.00;
     0.89];

magNED = ...
    magNED / norm(magNED);


%% ================================================================
%  INITIALIZE NAVIGATION ESTIMATOR
% ================================================================

navState = navigation_estimator_init(P);


%% ================================================================
%  HISTORY
% ================================================================

positionEstimate = zeros(3,N);
velocityEstimate = zeros(3,N);
attitudeEstimate = zeros(3,N);

gpsPositionHistory = nan(3,N);
gpsVelocityHistory = nan(3,N);

gpsAvailableHistory = false(1,N);

accelNEDHistory = zeros(3,N);


%% ================================================================
%  MAIN SENSOR / NAVIGATION LOOP
% ================================================================

for k = 1:N

    phi   = phiTrue(k);
    theta = thetaTrue(k);
    psi   = psiTrue(k);


    %% ------------------------------------------------------------
    % True rotation matrix
    % -------------------------------------------------------------

    Rbn = Rbn321_test( ...
        phi, ...
        theta, ...
        psi);


    %% ------------------------------------------------------------
    % True body rates
    %
    % Convert Euler-angle rates:
    %
    % [phiDot thetaDot psiDot]
    %
    % into gyro body rates:
    %
    % [p q r]
    % -------------------------------------------------------------

    eDot = eulerDot(:,k);

    bodyRateMap = ...
        [ ...
        1, 0, -sin(theta);

        0, cos(phi), ...
        sin(phi)*cos(theta);

        0, -sin(phi), ...
        cos(phi)*cos(theta) ...
        ];

    omegaBodyTrue = ...
        bodyRateMap * eDot;


    %% ------------------------------------------------------------
    % Ideal accelerometer specific force
    %
    % f_b = R_nb * (a_n - g_n)
    % -------------------------------------------------------------

    gravityNED = ...
        [0;
         0;
         g];

    specificForceBody = ...
        Rbn' * ...
        (accelTrue(:,k) - gravityNED);


    %% ------------------------------------------------------------
    % Simulated IMU measurement
    % -------------------------------------------------------------

    imuMeas.gyro = ...
        omegaBodyTrue + ...
        gyroBias + ...
        gyroNoiseStd .* randn(3,1);


    imuMeas.accel = ...
        specificForceBody + ...
        accelBias + ...
        accelNoiseStd .* randn(3,1);


    %% ------------------------------------------------------------
    % Simulated magnetometer
    % -------------------------------------------------------------

    magBodyTrue = ...
        Rbn' * magNED;


    magMeas = ...
        magBodyTrue + ...
        magBias + ...
        magNoiseStd*randn(3,1);


    %% ------------------------------------------------------------
    % GPS measurement at 10 Hz
    % -------------------------------------------------------------

    gpsAvailable = ...
        mod(k-1,gpsStride) == 0;


    if gpsAvailable

        gpsMeas.position = ...
            positionTrue(:,k) + ...
            gpsPositionStd .* randn(3,1);

        gpsMeas.velocity = ...
            velocityTrue(:,k) + ...
            gpsVelocityStd .* randn(3,1);


        gpsPositionHistory(:,k) = ...
            gpsMeas.position;

        gpsVelocityHistory(:,k) = ...
            gpsMeas.velocity;

    else

        gpsMeas.position = ...
            nan(3,1);

        gpsMeas.velocity = ...
            nan(3,1);

    end


    gpsAvailableHistory(k) = gpsAvailable;


    %% ------------------------------------------------------------
    % Navigation estimator
    %
    % THIS is the entire navigation stack from the perspective
    % of the flight software.
    % -------------------------------------------------------------

    [nav, navState] = ...
        navigation_estimator_step( ...
        navState, ...
        imuMeas, ...
        gpsMeas, ...
        magMeas, ...
        gpsAvailable, ...
        dt);


    %% ------------------------------------------------------------
    % Store estimates
    % -------------------------------------------------------------

    positionEstimate(:,k) = ...
        nav.position;

    velocityEstimate(:,k) = ...
        nav.velocity;

    attitudeEstimate(:,k) = ...
        nav.euler;

    accelNEDHistory(:,k) = ...
        nav.accelNED;

end


%% ================================================================
%  ANGLE ERROR WRAPPING
% ================================================================

attitudeError = ...
    attitudeEstimate - ...
    attitudeTrue;

for k = 1:N

    attitudeError(:,k) = ...
        wrap_pi_test( ...
        attitudeError(:,k));

end


%% ================================================================
%  RMSE
% ================================================================

positionError = ...
    positionEstimate - ...
    positionTrue;

velocityError = ...
    velocityEstimate - ...
    velocityTrue;


positionRMSE = ...
    sqrt(mean(positionError.^2,2));

velocityRMSE = ...
    sqrt(mean(velocityError.^2,2));

attitudeRMSE = ...
    rad2deg( ...
    sqrt(mean(attitudeError.^2,2)));


fprintf('POSITION RMSE\n');
fprintf('North: %.4f m\n', positionRMSE(1));
fprintf('East:  %.4f m\n', positionRMSE(2));
fprintf('Down:  %.4f m\n\n', positionRMSE(3));


fprintf('VELOCITY RMSE\n');
fprintf('North: %.4f m/s\n', velocityRMSE(1));
fprintf('East:  %.4f m/s\n', velocityRMSE(2));
fprintf('Down:  %.4f m/s\n\n', velocityRMSE(3));


fprintf('ATTITUDE RMSE\n');
fprintf('Roll:  %.4f deg\n', attitudeRMSE(1));
fprintf('Pitch: %.4f deg\n', attitudeRMSE(2));
fprintf('Yaw:   %.4f deg\n\n', attitudeRMSE(3));


%% ================================================================
%  FIGURE 1
%  3-D NAVIGATION TRAJECTORY
% ================================================================

figure;

plot3( ...
    positionTrue(2,:), ...
    positionTrue(1,:), ...
    -positionTrue(3,:), ...
    'LineWidth',1.8);

hold on;

plot3( ...
    positionEstimate(2,:), ...
    positionEstimate(1,:), ...
    -positionEstimate(3,:), ...
    '--', ...
    'LineWidth',1.8);


gpsIdx = find(gpsAvailableHistory);

scatter3( ...
    gpsPositionHistory(2,gpsIdx), ...
    gpsPositionHistory(1,gpsIdx), ...
    -gpsPositionHistory(3,gpsIdx), ...
    15, ...
    'filled');


grid on;
axis equal;

xlabel('East [m]');
ylabel('North [m]');
zlabel('Altitude [m]');

title('Integrated Navigation Estimate');

legend( ...
    'Truth', ...
    'Navigation Estimate', ...
    'GPS Measurements', ...
    'Location','best');


%% ================================================================
%  FIGURE 2
%  POSITION
% ================================================================

figure;

axisNames = {'North','East','Down'};

for axisIdx = 1:3

    subplot(3,1,axisIdx);

    plot( ...
        t, ...
        positionTrue(axisIdx,:), ...
        'LineWidth',1.6);

    hold on;

    plot( ...
        t, ...
        positionEstimate(axisIdx,:), ...
        '--', ...
        'LineWidth',1.6);

    grid on;

    ylabel( ...
        sprintf('%s [m]', ...
        axisNames{axisIdx}));

    if axisIdx == 1
        title('Navigation Position Estimate');
    end

    if axisIdx == 3
        xlabel('Time [s]');
    end

end

legend('Truth','Estimate');


%% ================================================================
%  FIGURE 3
%  VELOCITY
% ================================================================

figure;

for axisIdx = 1:3

    subplot(3,1,axisIdx);

    plot( ...
        t, ...
        velocityTrue(axisIdx,:), ...
        'LineWidth',1.6);

    hold on;

    plot( ...
        t, ...
        velocityEstimate(axisIdx,:), ...
        '--', ...
        'LineWidth',1.6);

    grid on;

    ylabel( ...
        sprintf('v_%s [m/s]', ...
        axisNames{axisIdx}(1)));

    if axisIdx == 1
        title('Navigation Velocity Estimate');
    end

    if axisIdx == 3
        xlabel('Time [s]');
    end

end

legend('Truth','Estimate');


%% ================================================================
%  FIGURE 4
%  ATTITUDE
% ================================================================

figure;

attitudeNames = ...
    {'Roll \phi','Pitch \theta','Yaw \psi'};

for axisIdx = 1:3

    subplot(3,1,axisIdx);

    plot( ...
        t, ...
        rad2deg(attitudeTrue(axisIdx,:)), ...
        'LineWidth',1.6);

    hold on;

    plot( ...
        t, ...
        rad2deg(attitudeEstimate(axisIdx,:)), ...
        '--', ...
        'LineWidth',1.6);

    grid on;

    ylabel( ...
        sprintf('%s [deg]', ...
        attitudeNames{axisIdx}));

    if axisIdx == 1
        title('Navigation Attitude Estimate');
    end

    if axisIdx == 3
        xlabel('Time [s]');
    end

end

legend('Truth','Estimate');


%% ================================================================
%  FIGURE 5
%  NAVIGATION ERRORS
% ================================================================

figure;

subplot(3,1,1);

plot(t,positionError');

grid on;

ylabel('Position Error [m]');

title('Integrated Navigation Estimation Error');

legend('N','E','D');


subplot(3,1,2);

plot(t,velocityError');

grid on;

ylabel('Velocity Error [m/s]');

legend('N','E','D');


subplot(3,1,3);

plot( ...
    t, ...
    rad2deg(attitudeError'));

grid on;

ylabel('Attitude Error [deg]');
xlabel('Time [s]');

legend('\phi','\theta','\psi');


%% =================================================================
%  LOCAL ROTATION MATRIX
% =================================================================

function Rbn = Rbn321_test(phi,theta,psi)

cphi = cos(phi);
sphi = sin(phi);

ctheta = cos(theta);
stheta = sin(theta);

cpsi = cos(psi);
spsi = sin(psi);


Rbn = ...
    [ ...
    ctheta*cpsi, ...
    sphi*stheta*cpsi-cphi*spsi, ...
    cphi*stheta*cpsi+sphi*spsi;

    ctheta*spsi, ...
    sphi*stheta*spsi+cphi*cpsi, ...
    cphi*stheta*spsi-sphi*cpsi;

    -stheta, ...
    sphi*ctheta, ...
    cphi*ctheta ...
    ];

end


%% =================================================================
%  ANGLE WRAP
% =================================================================

function angle = wrap_pi_test(angle)

angle = mod(angle + pi,2*pi) - pi;

end