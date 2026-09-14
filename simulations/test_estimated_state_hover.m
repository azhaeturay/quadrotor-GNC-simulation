%% TEST_ESTIMATED_STATE_HOVER
%
% Full estimated-state closed-loop hover test.
%
% Architecture:
%
%   Quadrotor truth dynamics
%           ↓
%   IMU / GPS / magnetometer
%           ↓
%   Navigation estimator
%           ↓
%   Estimated position / velocity / attitude
%           ↓
%   Position controller
%           ↓
%   Attitude controller
%           ↓
%   Rate controller
%           ↓
%   Control allocation
%           ↓
%   Individual rotor thrusts
%           ↓
%   Nonlinear 6-DOF quadrotor plant
%
% IMPORTANT:
%
% Guidance/control do NOT receive true position, velocity, or attitude.
% Truth is retained only for:
%
%   - sensor simulation
%   - dynamics propagation
%   - validation / plotting
%
% The rate controller uses bias-corrected gyro measurements,
% analogous to a calibrated onboard IMU.
%
% ---------------------------------------------------------------

clear;
clc;
close all;

rng(12);


%% ================================================================
% PROJECT SETUP
% ================================================================

projectRoot = fileparts( ...
    fileparts(mfilename('fullpath')));

addpath(genpath(projectRoot));

P = vehicle_params();


%% ================================================================
% SIMULATION TIMING
% ================================================================

% Use IMU rate as the base flight-computer loop rate.

if isfield(P,'imu') && isfield(P.imu,'rateHz')

    imuRate = P.imu.rateHz;

elseif isfield(P,'imuRate')

    imuRate = P.imuRate;

else

    imuRate = 100;

end


if isfield(P,'gps') && isfield(P.gps,'rateHz')

    gpsRate = P.gps.rateHz;

elseif isfield(P,'gpsRate')

    gpsRate = P.gpsRate;

else

    gpsRate = 10;

end


dt = 1 / imuRate;

T = 20;

t = 0:dt:T;

N = length(t);


gpsStride = round( ...
    imuRate / gpsRate);


fprintf('\n');
fprintf('--- ESTIMATED-STATE CLOSED-LOOP HOVER ---\n\n');

fprintf('Simulation time: %.1f s\n',T);
fprintf('Flight-computer / IMU rate: %.1f Hz\n',imuRate);
fprintf('GPS rate: %.1f Hz\n\n',gpsRate);


%% ================================================================
% HOVER COMMAND
%
% NED convention:
%
% +N = North
% +E = East
% +D = Down
%
% D = -2 m therefore means:
%
% altitude = +2 m
% ================================================================

positionCmd = ...
    [0;
     0;
    -2];

yawCmd = 0;


%% ================================================================
% TRUE INITIAL VEHICLE STATE
%
% x =
%
% [ N
%   E
%   D
%   u
%   v
%   w
%   phi
%   theta
%   psi
%   p
%   q
%   r ]
%
% u,v,w are BODY-frame velocities.
% ================================================================

xTrue = zeros(12,1);


%% Start near, but not exactly at, the desired hover point

xTrue(1:3) = ...
    [ 0.25;
     -0.20;
     -1.75];


%% Add initial attitude disturbance

xTrue(7:9) = deg2rad( ...
    [ 4;
     -3;
      5]);


%% ================================================================
% MAGNETOMETER MODEL
% ================================================================

% Normalized magnetic field expressed in NED.
%
% East component = 0 so magnetic north aligns with navigation north
% for this first closed-loop test.

magNED = ...
    [0.45;
     0.00;
     0.89];

magNED = ...
    magNED / norm(magNED);


magBias = ...
    [ 0.015;
     -0.010;
      0.008];

magNoiseStd = 0.010;


%% ================================================================
% INITIALIZE NAVIGATION ESTIMATOR
% ================================================================

navState = ...
    navigation_ekf_init(P);


%% ================================================================
% INITIAL ROTOR THRUST
%
% Start from hover thrust before the first controller update.
% ================================================================

if isfield(P,'hoverThrustTotal')

    initialTotalThrust = ...
        P.hoverThrustTotal;

else

    initialTotalThrust = ...
        P.mass * P.g;

end


initialControl = ...
    [initialTotalThrust;
     0;
     0;
     0];


[rotorThrust, ~] = ...
    control_allocator( ...
        initialControl, ...
        P);


%% ================================================================
% FEEDBACK MODE
%
% TRUE  = actual estimated-state closed-loop flight
% FALSE = truth-feedback baseline for debugging
%
% Leave TRUE for this test.
% ================================================================

useEstimatedState = true;


%% ================================================================
% HISTORY STORAGE
% ================================================================

xTrueHistory = zeros(12,N);

positionTrueHistory = zeros(3,N);
velocityTrueHistory = zeros(3,N);
attitudeTrueHistory = zeros(3,N);

positionEstimateHistory = zeros(3,N);
velocityEstimateHistory = zeros(3,N);
attitudeEstimateHistory = zeros(3,N);

gpsPositionHistory = nan(3,N);
gpsAvailableHistory = false(1,N);

attitudeCmdHistory = zeros(3,N);

omegaCmdHistory = zeros(3,N);

TcmdHistory = zeros(1,N);
tauCmdHistory = zeros(3,N);

rotorThrustHistory = zeros( ...
    length(rotorThrust),N);

achievedControlHistory = zeros(4,N);

positionCmdHistory = ...
    repmat(positionCmd,1,N);


%% ================================================================
% MAIN CLOSED-LOOP SIMULATION
% ================================================================

for k = 1:N

    tCurrent = t(k);


    %% ============================================================
    % 1. TRUE VEHICLE STATE
    %
    % xTrue is NEVER passed directly to guidance/control when
    % useEstimatedState == true.
    % ============================================================

    xTrueHistory(:,k) = ...
        xTrue;


    phiTrue   = xTrue(7);
    thetaTrue = xTrue(8);
    psiTrue   = xTrue(9);


    %% Body -> NED rotation

    R_BN_true = ...
        rotation_matrix( ...
            phiTrue, ...
            thetaTrue, ...
            psiTrue);


    %% Convert true body velocity to NED velocity

    velocityNEDTrue = ...
        R_BN_true * ...
        xTrue(4:6);


    positionTrueHistory(:,k) = ...
        xTrue(1:3);

    velocityTrueHistory(:,k) = ...
        velocityNEDTrue;

    attitudeTrueHistory(:,k) = ...
        xTrue(7:9);


    %% ============================================================
    % 2. TRUE STATE DERIVATIVE
    %
    % Current physical acceleration is determined by the rotor
    % thrust command from the previous control cycle.
    % ============================================================

    xDotTrue = ...
        quad_dynamics_rotors( ...
            tCurrent, ...
            xTrue, ...
            rotorThrust, ...
            P);


    %% ============================================================
    % 3. SENSOR SIMULATION
    %
    % These are the ONLY state observations available to the
    % onboard navigation/control software.
    % ============================================================


    %% ------------------------------------------------------------
    % IMU @ 100 Hz
    % -------------------------------------------------------------

    imuMeas = ...
        simulate_imu( ...
            xTrue, ...
            xDotTrue, ...
            P);


    %% ------------------------------------------------------------
    % GPS @ 10 Hz
    % -------------------------------------------------------------

    gpsAvailable = ...
        mod(k-1,gpsStride) == 0;


    if gpsAvailable

        gpsMeas = ...
            simulate_gps( ...
                xTrue, ...
                P);

        gpsPositionHistory(:,k) = ...
            gpsMeas.position;

    else

        gpsMeas.position = ...
            nan(3,1);

        gpsMeas.velocity = ...
            nan(3,1);

    end


    gpsAvailableHistory(k) = ...
        gpsAvailable;


    %% ------------------------------------------------------------
    % Magnetometer
    %
    % We currently update it at the base navigation rate.
    % -------------------------------------------------------------

    mag = ...
        simulate_magnetometer( ...
            xTrue, ...
            magNED, ...
            magBias, ...
            magNoiseStd);

    magMeas = ...
        mag.fieldBody - magBias;


    %% ============================================================
    % 4. NAVIGATION ESTIMATOR
    %
    % Sensor measurements in:
    %
    %       IMU + GPS + magnetometer
    %
    % Estimated navigation state out:
    %
    %       position
    %       velocity
    %       attitude
    % ============================================================

    [nav, navState] = ...
        navigation_ekf_step( ...
            navState, ...
            imuMeas, ...
            gpsMeas, ...
            magMeas, ...
            gpsAvailable, ...
            dt);


    positionEstimateHistory(:,k) = ...
        nav.position;

    velocityEstimateHistory(:,k) = ...
        nav.velocity;

    attitudeEstimateHistory(:,k) = ...
        nav.euler;


    %% ============================================================
    % 5. SELECT FEEDBACK SOURCE
    %
    % For THIS test:
    %
    %       estimated position
    %       estimated velocity
    %       estimated attitude
    %
    % are fed to the controllers.
    %
    % Truth remains available only as an optional debugging baseline.
    % ============================================================

    if useEstimatedState

        positionFB = ...
            nav.position;

        velocityFB = ...
            nav.velocity;

        attitudeFB = ...
            nav.euler;

    else

        positionFB = ...
            xTrue(1:3);

        velocityFB = ...
            velocityNEDTrue;

        attitudeFB = ...
            xTrue(7:9);

    end


    %% ============================================================
    % 6. HORIZONTAL POSITION CONTROL
    %
    % [N,E] position error
    %        ↓
    % desired horizontal velocity
    %        ↓
    % desired horizontal acceleration
    %        ↓
    % desired roll + pitch
    % ============================================================

    [phiCmd, thetaCmd, ~, ~] = ...
        horizontal_position_controller( ...
            positionCmd(1:2), ...
            positionFB(1:2), ...
            velocityFB(1:2), ...
            attitudeFB(3), ...
            P);


    %% ============================================================
    % 7. ALTITUDE CONTROL
    %
    % Down-position error
    %       ↓
    % vertical velocity
    %       ↓
    % vertical acceleration
    %       ↓
    % total thrust
    % ============================================================

    [Tcmd, ~, ~] = ...
        altitude_controller( ...
            positionCmd(3), ...
            positionFB(3), ...
            velocityFB(3), ...
            attitudeFB(1), ...
            attitudeFB(2), ...
            P);


    %% ============================================================
    % 8. ATTITUDE CONTROL
    % ============================================================

    attitudeCmd = ...
        [phiCmd;
         thetaCmd;
         yawCmd];


    omegaCmd = ...
        attitude_controller( ...
            attitudeCmd, ...
            attitudeFB, ...
            P);


    %% ============================================================
    % 9. RATE CONTROL
    %
    % Body angular rates come directly from the IMU gyro.
    %
    % Remove the nominal calibrated gyro bias before control.
    %
    % This is sensor calibration, NOT truth-state feedback.
    % ============================================================

    omegaMeasured = ...
        imuMeas.gyro;


    if isfield(P,'gyroBias')

        omegaMeasured = ...
            omegaMeasured - ...
            P.gyroBias;

    end


    tauCmd = ...
        rate_controller( ...
            omegaCmd, ...
            omegaMeasured, ...
            P);


    %% ============================================================
    % 10. CONTROL ALLOCATION
    %
    % Desired:
    %
    % [ total thrust
    %   roll moment
    %   pitch moment
    %   yaw moment ]
    %
    % becomes individual rotor thrust commands.
    % ============================================================

    desiredControl = ...
        [Tcmd;
         tauCmd];


    [rotorThrustNew, achievedControl] = ...
        control_allocator( ...
            desiredControl, ...
            P);


    %% Save controller histories

    attitudeCmdHistory(:,k) = ...
        attitudeCmd;

    omegaCmdHistory(:,k) = ...
        omegaCmd;

    TcmdHistory(k) = ...
        Tcmd;

    tauCmdHistory(:,k) = ...
        tauCmd;

    rotorThrustHistory(:,k) = ...
        rotorThrustNew;

    achievedControlHistory(:,k) = ...
        achievedControl;


    %% ============================================================
    % 11. PROPAGATE NONLINEAR 6-DOF VEHICLE
    %
    % Keep rotor command constant for one 100-Hz control interval.
    %
    % Use RK4 instead of a simple Euler step.
    % ============================================================

    if k < N

        xTrue = ...
            rk4_quad_step( ...
                tCurrent, ...
                xTrue, ...
                rotorThrustNew, ...
                dt, ...
                P);

    end


    %% Command becomes the current physical rotor input

    rotorThrust = ...
        rotorThrustNew;

end


%% ================================================================
% NAVIGATION ESTIMATION ERRORS
% ================================================================

positionEstimationError = ...
    positionEstimateHistory - ...
    positionTrueHistory;


velocityEstimationError = ...
    velocityEstimateHistory - ...
    velocityTrueHistory;


attitudeEstimationError = ...
    attitudeEstimateHistory - ...
    attitudeTrueHistory;


for k = 1:N

    attitudeEstimationError(:,k) = ...
        wrap_pi_local( ...
            attitudeEstimationError(:,k));

end


%% ================================================================
% TRUE HOVER TRACKING ERROR
% ================================================================

positionTrackingError = ...
    positionTrueHistory - ...
    positionCmdHistory;


%% ================================================================
% PERFORMANCE METRICS
% ================================================================

positionNavRMSE = ...
    sqrt(mean( ...
    positionEstimationError.^2,2));


velocityNavRMSE = ...
    sqrt(mean( ...
    velocityEstimationError.^2,2));


attitudeNavRMSE = ...
    rad2deg( ...
    sqrt(mean( ...
    attitudeEstimationError.^2,2)));


%% Last 5 seconds = steady-state hover performance

steadyMask = ...
    t >= (T - 5);


hoverRMSE = ...
    sqrt(mean( ...
    positionTrackingError(:,steadyMask).^2, ...
    2));


finalPosition = ...
    positionTrueHistory(:,end);

finalVelocity = ...
    velocityTrueHistory(:,end);

finalAttitude = ...
    rad2deg( ...
    attitudeTrueHistory(:,end));


finalPositionError = ...
    finalPosition - ...
    positionCmd;


%% ================================================================
% PRINT RESULTS
% ================================================================

fprintf('\n');

fprintf('--- NAVIGATION PERFORMANCE ---\n\n');

fprintf('Position estimation RMSE:\n');
fprintf('North: %.4f m\n', ...
    positionNavRMSE(1));
fprintf('East:  %.4f m\n', ...
    positionNavRMSE(2));
fprintf('Down:  %.4f m\n\n', ...
    positionNavRMSE(3));


fprintf('Velocity estimation RMSE:\n');
fprintf('North: %.4f m/s\n', ...
    velocityNavRMSE(1));
fprintf('East:  %.4f m/s\n', ...
    velocityNavRMSE(2));
fprintf('Down:  %.4f m/s\n\n', ...
    velocityNavRMSE(3));


fprintf('Attitude estimation RMSE:\n');
fprintf('Roll:  %.4f deg\n', ...
    attitudeNavRMSE(1));
fprintf('Pitch: %.4f deg\n', ...
    attitudeNavRMSE(2));
fprintf('Yaw:   %.4f deg\n\n', ...
    attitudeNavRMSE(3));


fprintf('--- TRUE CLOSED-LOOP HOVER PERFORMANCE ---\n\n');

fprintf('Final position error:\n');
fprintf('North: %.4f m\n', ...
    finalPositionError(1));
fprintf('East:  %.4f m\n', ...
    finalPositionError(2));
fprintf('Down:  %.4f m\n\n', ...
    finalPositionError(3));


fprintf('Final NED velocity:\n');
fprintf('North: %.4f m/s\n', ...
    finalVelocity(1));
fprintf('East:  %.4f m/s\n', ...
    finalVelocity(2));
fprintf('Down:  %.4f m/s\n\n', ...
    finalVelocity(3));


fprintf('Final attitude:\n');
fprintf('Roll:  %.3f deg\n', ...
    finalAttitude(1));
fprintf('Pitch: %.3f deg\n', ...
    finalAttitude(2));
fprintf('Yaw:   %.3f deg\n\n', ...
    finalAttitude(3));


fprintf('Steady-state hover RMSE (last 5 s):\n');
fprintf('North: %.4f m\n', ...
    hoverRMSE(1));
fprintf('East:  %.4f m\n', ...
    hoverRMSE(2));
fprintf('Down:  %.4f m\n\n', ...
    hoverRMSE(3));


%% ================================================================
% FIGURE 1 — 3-D HOVER TRAJECTORY
% ================================================================

figure;

plot3( ...
    positionTrueHistory(2,:), ...
    positionTrueHistory(1,:), ...
    -positionTrueHistory(3,:), ...
    'LineWidth',1.8);

hold on;

plot3( ...
    positionEstimateHistory(2,:), ...
    positionEstimateHistory(1,:), ...
    -positionEstimateHistory(3,:), ...
    '--', ...
    'LineWidth',1.5);

plot3( ...
    positionCmd(2), ...
    positionCmd(1), ...
    -positionCmd(3), ...
    'o', ...
    'MarkerSize',9, ...
    'LineWidth',1.8);

grid on;
axis equal;

xlabel('East [m]');
ylabel('North [m]');
zlabel('Altitude [m]');

title('Estimated-State Closed-Loop Hover');

legend( ...
    'True Vehicle', ...
    'Navigation Estimate', ...
    'Hover Command', ...
    'Location','best');

view(3);


%% ================================================================
% FIGURE 2 — POSITION
% ================================================================

figure;

axisNames = ...
    {'North','East','Down'};


for axisIdx = 1:3

    subplot(3,1,axisIdx);

    plot( ...
        t, ...
        positionTrueHistory(axisIdx,:), ...
        'LineWidth',1.6);

    hold on;

    plot( ...
        t, ...
        positionEstimateHistory(axisIdx,:), ...
        '--', ...
        'LineWidth',1.4);

    plot( ...
        t, ...
        positionCmdHistory(axisIdx,:), ...
        ':', ...
        'LineWidth',1.4);

    grid on;

    ylabel( ...
        sprintf('%s [m]', ...
        axisNames{axisIdx}));

    if axisIdx == 1

        title( ...
            'Estimated-State Hover — Position');

    end

    if axisIdx == 3

        xlabel('Time [s]');

    end

end


legend( ...
    'Truth', ...
    'Estimate', ...
    'Command');


%% ================================================================
% FIGURE 3 — VELOCITY
% ================================================================

figure;


for axisIdx = 1:3

    subplot(3,1,axisIdx);

    plot( ...
        t, ...
        velocityTrueHistory(axisIdx,:), ...
        'LineWidth',1.6);

    hold on;

    plot( ...
        t, ...
        velocityEstimateHistory(axisIdx,:), ...
        '--', ...
        'LineWidth',1.4);

    yline(0,':');

    grid on;

    ylabel( ...
        sprintf('v_%s [m/s]', ...
        axisNames{axisIdx}(1)));

    if axisIdx == 1

        title( ...
            'Estimated-State Hover — Velocity');

    end

    if axisIdx == 3

        xlabel('Time [s]');

    end

end


legend( ...
    'Truth', ...
    'Estimate', ...
    'Command');


%% ================================================================
% FIGURE 4 — ATTITUDE
% ================================================================

figure;

attitudeNames = ...
    {'Roll \phi', ...
     'Pitch \theta', ...
     'Yaw \psi'};


for axisIdx = 1:3

    subplot(3,1,axisIdx);

    plot( ...
        t, ...
        rad2deg( ...
        attitudeTrueHistory(axisIdx,:)), ...
        'LineWidth',1.6);

    hold on;

    plot( ...
        t, ...
        rad2deg( ...
        attitudeEstimateHistory(axisIdx,:)), ...
        '--', ...
        'LineWidth',1.4);

    plot( ...
        t, ...
        rad2deg( ...
        attitudeCmdHistory(axisIdx,:)), ...
        ':', ...
        'LineWidth',1.4);

    grid on;

    ylabel( ...
        sprintf('%s [deg]', ...
        attitudeNames{axisIdx}));

    if axisIdx == 1

        title( ...
            'Estimated-State Hover — Attitude');

    end

    if axisIdx == 3

        xlabel('Time [s]');

    end

end


legend( ...
    'Truth', ...
    'Navigation Estimate', ...
    'Command');


%% ================================================================
% FIGURE 5 — NAVIGATION ESTIMATION ERRORS
% ================================================================

figure;


subplot(3,1,1);

plot( ...
    t, ...
    positionEstimationError');

grid on;

ylabel('Position Error [m]');

title( ...
    'Closed-Loop Navigation Estimation Error');

legend('N','E','D');


subplot(3,1,2);

plot( ...
    t, ...
    velocityEstimationError');

grid on;

ylabel('Velocity Error [m/s]');

legend('N','E','D');


subplot(3,1,3);

plot( ...
    t, ...
    rad2deg( ...
    attitudeEstimationError'));

grid on;

ylabel('Attitude Error [deg]');
xlabel('Time [s]');

legend('\phi','\theta','\psi');


%% ================================================================
% FIGURE 6 — ROTOR THRUSTS
% ================================================================

figure;

plot( ...
    t, ...
    rotorThrustHistory', ...
    'LineWidth',1.2);

grid on;

xlabel('Time [s]');
ylabel('Rotor Thrust [N]');

title('Estimated-State Hover — Rotor Thrust Commands');

legend( ...
    'Rotor 1', ...
    'Rotor 2', ...
    'Rotor 3', ...
    'Rotor 4', ...
    'Location','best');


%% ================================================================
% LOCAL FUNCTION — RK4 QUADROTOR PROPAGATION
% ================================================================

function xNext = ...
    rk4_quad_step( ...
    t, ...
    x, ...
    rotorThrust, ...
    dt, ...
    P)

k1 = ...
    quad_dynamics_rotors( ...
        t, ...
        x, ...
        rotorThrust, ...
        P);


k2 = ...
    quad_dynamics_rotors( ...
        t + dt/2, ...
        x + (dt/2)*k1, ...
        rotorThrust, ...
        P);


k3 = ...
    quad_dynamics_rotors( ...
        t + dt/2, ...
        x + (dt/2)*k2, ...
        rotorThrust, ...
        P);


k4 = ...
    quad_dynamics_rotors( ...
        t + dt, ...
        x + dt*k3, ...
        rotorThrust, ...
        P);


xNext = ...
    x + ...
    (dt/6) * ...
    (k1 + 2*k2 + 2*k3 + k4);

end


%% ================================================================
% LOCAL FUNCTION — ANGLE WRAP
% ================================================================

function angle = ...
    wrap_pi_local(angle)

angle = ...
    mod(angle + pi,2*pi) - pi;

end