%% test_kalman_3d.m
%
% 3-D translational Kalman filter test
%
% State estimate:
%
%   xHat = [pN pE pD vN vE vD]'
%
% Sensor configuration:
%
%   IMU accelerometer: 100 Hz
%   GPS position:       10 Hz
%
% IMPORTANT:
% For this test, TRUE attitude is used to transform the simulated
% body-frame accelerometer measurement back into NED.
%
% This isolates the translational navigation filter.
%
% Later, true attitude will be replaced with ESTIMATED attitude.

clear;
clc;
close all;

%% ------------------------------------------------------------
%  PROJECT SETUP
% -------------------------------------------------------------

projectRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(projectRoot));

rng(7);     % Repeatable noise realization


%% ------------------------------------------------------------
%  SIMULATION SETTINGS
% -------------------------------------------------------------

imuRate = 100;          % [Hz]
gpsRate = 10;           % [Hz]

dt = 1 / imuRate;

gpsStride = round(imuRate / gpsRate);

Tfinal = 12;            % [s]

t = (0:dt:Tfinal)';
N = length(t);


%% ------------------------------------------------------------
%  SENSOR PARAMETERS
% -------------------------------------------------------------

% Accelerometer noise standard deviation
%
% For now use equal noise in all 3 axes.

accelNoiseStd = [0.04;
                 0.04;
                 0.04];             % [m/s^2]

% GPS position noise
%
% Vertical GPS is intentionally somewhat worse than horizontal.

gpsPositionStd = [0.25;
                  0.25;
                  0.40];             % [m]


%% ------------------------------------------------------------
%  TRUTH TRAJECTORY
% -------------------------------------------------------------
%
% Create a smooth 3-D trajectory.
%
% North:
%       0 -> 5 m
%
% East:
%       curves outward and returns
%
% Down:
%       0 -> -2 m
%
% Since we use NED coordinates:
%
%       altitude = -Down
%

s = t / Tfinal;

% Smoothstep trajectory
h = 3*s.^2 - 2*s.^3;

hDot = (6*s - 6*s.^2) / Tfinal;

hDDot = (6 - 12*s) / Tfinal^2;


%% North motion

northTrue = 5*h;

northVelTrue = 5*hDot;

northAccelTrue = 5*hDDot;


%% East motion
%
% Smooth side-to-side curve with zero initial/final velocity.

eastAmplitude = 2.0;

eastTrue = ...
    0.5*eastAmplitude .* (1 - cos(2*pi*s));

eastVelTrue = ...
    eastAmplitude*pi/Tfinal .* sin(2*pi*s);

eastAccelTrue = ...
    2*eastAmplitude*pi^2/Tfinal^2 .* cos(2*pi*s);


%% Down motion

downTrue = -2*h;

downVelTrue = -2*hDot;

downAccelTrue = -2*hDDot;


%% Combine truth states

positionTrue = [northTrue.';
                eastTrue.';
                downTrue.'];

velocityTrue = [northVelTrue.';
                eastVelTrue.';
                downVelTrue.'];

accelTrue = [northAccelTrue.';
             eastAccelTrue.';
             downAccelTrue.'];


%% ------------------------------------------------------------
%  TRUE ATTITUDE
% -------------------------------------------------------------
%
% For now attitude is KNOWN PERFECTLY.
%
% We deliberately let attitude change slightly so the body-to-NED
% acceleration transformation actually does something.
%
% These attitude angles are NOT being estimated yet.

phiTrue = deg2rad( ...
    5*sin(0.45*t));

thetaTrue = deg2rad( ...
    4*sin(0.35*t));

psiTrue = deg2rad( ...
    15*sin(0.20*t));


%% ------------------------------------------------------------
%  SIMULATED IMU ACCELEROMETER
% -------------------------------------------------------------
%
% Accelerometers measure SPECIFIC FORCE:
%
%       f_B = R_NB' * (a_N - g_N)
%
% where:
%
%       a_N = inertial acceleration in NED
%       g_N = gravity in NED
%
% Then the navigation system reconstructs:
%
%       a_N = R_NB * f_B + g_N
%

g = 9.80665;

gNED = [0;
        0;
        g];

specificForceBody = zeros(3,N);

specificForceMeasured = zeros(3,N);

accelNEDMeasured = zeros(3,N);


for k = 1:N

    % Body -> NED rotation matrix

    R_NB = euler321_body_to_ned( ...
        phiTrue(k), ...
        thetaTrue(k), ...
        psiTrue(k));


    % True accelerometer specific force in body coordinates

    specificForceBody(:,k) = ...
        R_NB.' * (accelTrue(:,k) - gNED);


    % Add accelerometer noise

    specificForceMeasured(:,k) = ...
        specificForceBody(:,k) + ...
        accelNoiseStd .* randn(3,1);


    % Navigation system converts accelerometer reading back to
    % inertial acceleration in NED.
    %
    % TRUE attitude is being used here for now.

    accelNEDMeasured(:,k) = ...
        R_NB * specificForceMeasured(:,k) + gNED;

end


%% ------------------------------------------------------------
%  SIMULATED GPS
% -------------------------------------------------------------

gpsAvailable = false(1,N);

gpsPosition = nan(3,N);

for k = 1:N

    if mod(k-1, gpsStride) == 0

        gpsAvailable(k) = true;

        gpsPosition(:,k) = ...
            positionTrue(:,k) + ...
            gpsPositionStd .* randn(3,1);

    end

end


%% ------------------------------------------------------------
%  KALMAN FILTER INITIALIZATION
% -------------------------------------------------------------

% Initial estimate
%
% Start at the origin with zero velocity.

xHat = zeros(6,1);


% Initial covariance
%
% We initially have moderate uncertainty in both position
% and velocity.

P = diag([ ...
    1.0^2, ...
    1.0^2, ...
    1.0^2, ...
    0.75^2, ...
    0.75^2, ...
    0.75^2]);


%% History arrays

xHatHistory = zeros(6,N);

sigmaHistory = zeros(6,N);

xHatHistory(:,1) = xHat;

sigmaHistory(:,1) = sqrt(diag(P));


%% ------------------------------------------------------------
%  RUN THE FILTER
% -------------------------------------------------------------

for k = 1:N

    % Special case:
    %
    % At t = 0, dt = 0 so we can perform the initial GPS
    % correction without propagating forward in time.

    if k == 1
        dtFilter = 0;
    else
        dtFilter = dt;
    end


    if gpsAvailable(k)

        gpsMeasurement = gpsPosition(:,k);

    else

        % Value ignored when gpsAvailable = false

        gpsMeasurement = zeros(3,1);

    end


    [xHat, P] = kalman_position_3d_step( ...
        xHat, ...
        P, ...
        accelNEDMeasured(:,k), ...
        gpsMeasurement, ...
        gpsAvailable(k), ...
        dtFilter, ...
        accelNoiseStd, ...
        gpsPositionStd);


    xHatHistory(:,k) = xHat;

    sigmaHistory(:,k) = sqrt(diag(P));

end


%% ------------------------------------------------------------
%  EXTRACT ESTIMATES
% -------------------------------------------------------------

positionEstimate = xHatHistory(1:3,:);

velocityEstimate = xHatHistory(4:6,:);


%% ------------------------------------------------------------
%  ESTIMATION ERROR
% -------------------------------------------------------------

positionError = positionEstimate - positionTrue;

velocityError = velocityEstimate - velocityTrue;


%% ------------------------------------------------------------
%  PERFORMANCE METRICS
% -------------------------------------------------------------

positionRMSE = sqrt(mean(positionError.^2,2));

velocityRMSE = sqrt(mean(velocityError.^2,2));


fprintf('\n');
fprintf('--- 3-D KALMAN FILTER TEST ---\n\n');

fprintf('IMU rate: %.1f Hz\n', imuRate);
fprintf('GPS rate: %.1f Hz\n\n', gpsRate);

fprintf('POSITION RMSE\n');
fprintf('North: %.4f m\n', positionRMSE(1));
fprintf('East:  %.4f m\n', positionRMSE(2));
fprintf('Down:  %.4f m\n\n', positionRMSE(3));

fprintf('VELOCITY RMSE\n');
fprintf('North: %.4f m/s\n', velocityRMSE(1));
fprintf('East:  %.4f m/s\n', velocityRMSE(2));
fprintf('Down:  %.4f m/s\n\n', velocityRMSE(3));

fprintf('Final true position [N E D]:\n');
fprintf('[%.4f %.4f %.4f] m\n\n', ...
    positionTrue(:,end));

fprintf('Final estimated position [N E D]:\n');
fprintf('[%.4f %.4f %.4f] m\n\n', ...
    positionEstimate(:,end));

fprintf('Final true velocity [N E D]:\n');
fprintf('[%.4f %.4f %.4f] m/s\n\n', ...
    velocityTrue(:,end));

fprintf('Final estimated velocity [N E D]:\n');
fprintf('[%.4f %.4f %.4f] m/s\n\n', ...
    velocityEstimate(:,end));


%% ============================================================
%  FIGURE 1 — 3-D TRAJECTORY
% =============================================================

figure;

plot3( ...
    positionTrue(2,:), ...
    positionTrue(1,:), ...
    -positionTrue(3,:), ...
    'LineWidth',1.8);

hold on;

gpsIdx = find(gpsAvailable);

scatter3( ...
    gpsPosition(2,gpsIdx), ...
    gpsPosition(1,gpsIdx), ...
    -gpsPosition(3,gpsIdx), ...
    18, ...
    'filled');

plot3( ...
    positionEstimate(2,:), ...
    positionEstimate(1,:), ...
    -positionEstimate(3,:), ...
    '--', ...
    'LineWidth',1.8);

grid on;
axis equal;

xlabel('East [m]');
ylabel('North [m]');
zlabel('Altitude [m]');

title('3-D Kalman Navigation Estimate');

legend( ...
    'Truth', ...
    'GPS Measurements', ...
    'Kalman Estimate', ...
    'Location','best');

view(3);


%% ============================================================
%  FIGURE 2 — POSITION ESTIMATION
% =============================================================

figure;

tiledlayout(3,1);


nexttile;

plot(t,positionTrue(1,:), ...
    'LineWidth',1.5);

hold on;

scatter( ...
    t(gpsIdx), ...
    gpsPosition(1,gpsIdx), ...
    15, ...
    'filled');

plot(t,positionEstimate(1,:), ...
    '--', ...
    'LineWidth',1.5);

grid on;

ylabel('North [m]');

title('North Position');

legend( ...
    'Truth', ...
    'GPS', ...
    'Estimate', ...
    'Location','best');


nexttile;

plot(t,positionTrue(2,:), ...
    'LineWidth',1.5);

hold on;

scatter( ...
    t(gpsIdx), ...
    gpsPosition(2,gpsIdx), ...
    15, ...
    'filled');

plot(t,positionEstimate(2,:), ...
    '--', ...
    'LineWidth',1.5);

grid on;

ylabel('East [m]');

title('East Position');


nexttile;

plot(t,positionTrue(3,:), ...
    'LineWidth',1.5);

hold on;

scatter( ...
    t(gpsIdx), ...
    gpsPosition(3,gpsIdx), ...
    15, ...
    'filled');

plot(t,positionEstimate(3,:), ...
    '--', ...
    'LineWidth',1.5);

grid on;

xlabel('Time [s]');
ylabel('Down [m]');

title('Down Position');


%% ============================================================
%  FIGURE 3 — VELOCITY ESTIMATION
% =============================================================

figure;

tiledlayout(3,1);


nexttile;

plot(t,velocityTrue(1,:), ...
    'LineWidth',1.5);

hold on;

plot(t,velocityEstimate(1,:), ...
    '--', ...
    'LineWidth',1.5);

grid on;

ylabel('v_N [m/s]');

title('North Velocity');

legend( ...
    'Truth', ...
    'Estimate', ...
    'Location','best');


nexttile;

plot(t,velocityTrue(2,:), ...
    'LineWidth',1.5);

hold on;

plot(t,velocityEstimate(2,:), ...
    '--', ...
    'LineWidth',1.5);

grid on;

ylabel('v_E [m/s]');

title('East Velocity');


nexttile;

plot(t,velocityTrue(3,:), ...
    'LineWidth',1.5);

hold on;

plot(t,velocityEstimate(3,:), ...
    '--', ...
    'LineWidth',1.5);

grid on;

xlabel('Time [s]');
ylabel('v_D [m/s]');

title('Down Velocity');


%% ============================================================
%  FIGURE 4 — POSITION ERROR / UNCERTAINTY
% =============================================================

figure;

tiledlayout(3,1);

axisNames = {'North','East','Down'};

for axisIdx = 1:3

    nexttile;

    sigma = sigmaHistory(axisIdx,:);

    plot(t,positionError(axisIdx,:), ...
        'LineWidth',1.3);

    hold on;

    plot(t,3*sigma, ...
        '--', ...
        'LineWidth',1.1);

    plot(t,-3*sigma, ...
        '--', ...
        'LineWidth',1.1);

    yline(0);

    grid on;

    ylabel('Error [m]');

    title([axisNames{axisIdx} ' Position Error']);

    if axisIdx == 1

        legend( ...
            'Error', ...
            '+3\sigma', ...
            '-3\sigma', ...
            'Location','best');

    end

end

xlabel('Time [s]');


%% ============================================================
%  FIGURE 5 — VELOCITY ERROR / UNCERTAINTY
% =============================================================

figure;

tiledlayout(3,1);

for axisIdx = 1:3

    nexttile;

    sigma = sigmaHistory(axisIdx+3,:);

    plot(t,velocityError(axisIdx,:), ...
        'LineWidth',1.3);

    hold on;

    plot(t,3*sigma, ...
        '--', ...
        'LineWidth',1.1);

    plot(t,-3*sigma, ...
        '--', ...
        'LineWidth',1.1);

    yline(0);

    grid on;

    ylabel('Error [m/s]');

    title([axisNames{axisIdx} ' Velocity Error']);

    if axisIdx == 1

        legend( ...
            'Error', ...
            '+3\sigma', ...
            '-3\sigma', ...
            'Location','best');

    end

end

xlabel('Time [s]');


%% ============================================================
%  LOCAL FUNCTION
% =============================================================

function R_NB = euler321_body_to_ned(phi,theta,psi)

% EULER321_BODY_TO_NED
%
% Direction cosine matrix mapping body-frame vectors into
% the NED navigation frame.
%
% 3-2-1 Euler sequence:
%
%       yaw -> pitch -> roll
%
% v_N = R_NB * v_B

cphi = cos(phi);
sphi = sin(phi);

ctheta = cos(theta);
stheta = sin(theta);

cpsi = cos(psi);
spsi = sin(psi);


R_NB = [ ...
    ctheta*cpsi, ...
    sphi*stheta*cpsi - cphi*spsi, ...
    cphi*stheta*cpsi + sphi*spsi;

    ctheta*spsi, ...
    sphi*stheta*spsi + cphi*cpsi, ...
    cphi*stheta*spsi - sphi*cpsi;

    -stheta, ...
    sphi*ctheta, ...
    cphi*ctheta ];

end