%% TEST_ESTIMATED_STATE_MISSION
%
% Full closed-loop GNC mission using estimated-state feedback.
%
% Architecture:
%
%   Lookahead Guidance
%           ↓
%   Position / Altitude Control
%           ↓
%   Attitude Control
%           ↓
%   Rate Control
%           ↓
%   Control Allocation
%           ↓
%   Nonlinear Quadrotor Dynamics
%           ↓
%   IMU / GPS / Magnetometer
%           ↓
%   15-State INS/GPS EKF
%           ↓
%   Estimated State
%           └─────────────── back to Guidance + Control
%
% Truth is used ONLY for:
%
%   - physical vehicle dynamics
%   - simulated sensor generation
%   - validation / plotting
%
% ---------------------------------------------------------------

clear;
clc;
close all;

rng(12);


%% ================================================================
% PROJECT SETUP
% ================================================================

projectRoot = ...
    fileparts( ...
    fileparts( ...
    mfilename('fullpath')));

addpath(genpath(projectRoot));

P = vehicle_params();


%% ================================================================
% PATH DEFINITION
% ================================================================

% NED coordinates.
%
% Same mission used in TEST_LOOKAHEAD_MISSION.

waypoints = [ ...
     0      0      0;
     0      0     -2;
     4      0     -2;
     4      4     -2;
     0      4     -2;
     0      0     -2];


numWaypoints = ...
    size(waypoints,1);


%% ================================================================
% SIMULATION TIMING
% ================================================================

% IMU / flight-computer base rate.

if isfield(P,'imu') && ...
        isfield(P.imu,'rateHz')

    imuRate = P.imu.rateHz;

elseif isfield(P,'imuRate')

    imuRate = P.imuRate;

else

    imuRate = 100;

end


% GPS rate.

if isfield(P,'gps') && ...
        isfield(P.gps,'rateHz')

    gpsRate = P.gps.rateHz;

elseif isfield(P,'gpsRate')

    gpsRate = P.gpsRate;

else

    gpsRate = 10;

end


dt = 1 / imuRate;

maxMissionTime = 60;

maxSteps = ...
    floor(maxMissionTime/dt) + 1;


gpsStride = ...
    max( ...
    1, ...
    round(imuRate/gpsRate));


guidanceStride = ...
    max( ...
    1, ...
    round(P.guidanceDt/dt));


fprintf('\n');
fprintf('--- ESTIMATED-STATE FULL GNC MISSION ---\n\n');

fprintf('Maximum mission time: %.1f s\n', ...
    maxMissionTime);

fprintf('IMU / control rate: %.1f Hz\n', ...
    imuRate);

fprintf('GPS rate: %.1f Hz\n', ...
    gpsRate);

fprintf('Guidance rate: %.1f Hz\n\n', ...
    1/P.guidanceDt);


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

xTrue = ...
    zeros(12,1);


tCurrent = 0;


%% ================================================================
% GUIDANCE INITIALIZATION
% ================================================================

segmentIdx = 1;

yawReference = 0;

positionCmd = ...
    waypoints(1,:).';

yawCmd = 0;


%% ================================================================
% MAGNETOMETER MODEL
% ================================================================

magNED = ...
    [0.45;
     0.00;
     0.89];

magNED = ...
    magNED / norm(magNED);


% Hard-iron bias injected by sensor model.

magBias = ...
    [ 0.015;
     -0.010;
      0.008];

magNoiseStd = ...
    0.010;


%% ================================================================
% NAVIGATION EKF
% ================================================================

navState = ...
    navigation_ekf_init(P);


%% ================================================================
% INITIAL ROTOR THRUST
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


numRotors = ...
    length(rotorThrust);


%% ================================================================
% HISTORY STORAGE
% ================================================================

tHistory = ...
    nan(1,maxSteps);


xTrueHistory = ...
    nan(12,maxSteps);


positionTrueHistory = ...
    nan(3,maxSteps);

velocityTrueHistory = ...
    nan(3,maxSteps);

attitudeTrueHistory = ...
    nan(3,maxSteps);


positionEstimateHistory = ...
    nan(3,maxSteps);

velocityEstimateHistory = ...
    nan(3,maxSteps);

attitudeEstimateHistory = ...
    nan(3,maxSteps);


positionCmdHistory = ...
    nan(3,maxSteps);

yawCmdHistory = ...
    nan(1,maxSteps);


segmentHistory = ...
    nan(1,maxSteps);


crossTrackEstimateHistory = ...
    nan(1,maxSteps);

crossTrackTrueHistory = ...
    nan(1,maxSteps);


speedTrueHistory = ...
    nan(1,maxSteps);

speedEstimateHistory = ...
    nan(1,maxSteps);


gpsPositionHistory = ...
    nan(3,maxSteps);

gpsAvailableHistory = ...
    false(1,maxSteps);


attitudeCmdHistory = ...
    nan(3,maxSteps);

omegaCmdHistory = ...
    nan(3,maxSteps);

TcmdHistory = ...
    nan(1,maxSteps);

tauCmdHistory = ...
    nan(3,maxSteps);

rotorThrustHistory = ...
    nan(numRotors,maxSteps);


gyroBiasEstimateHistory = ...
    nan(3,maxSteps);

accelBiasEstimateHistory = ...
    nan(3,maxSteps);


%% ================================================================
% WAYPOINT / SEGMENT TRANSITION LOG
% ================================================================

transitionTimes = [];

transitionSegments = [];


%% ================================================================
% MAIN CLOSED-LOOP GNC LOOP
% ================================================================

missionComplete = false;

lastGuidanceCrossTrack = 0;


for k = 1:maxSteps

    tCurrent = ...
        (k-1)*dt;


    %% ============================================================
    % 1. TRUE PHYSICAL VEHICLE STATE
    %
    % Truth is available to the simulator, NOT the flight software.
    % ============================================================

    phiTrue = ...
        xTrue(7);

    thetaTrue = ...
        xTrue(8);

    psiTrue = ...
        xTrue(9);


    R_BN_true = ...
        rotation_matrix( ...
            phiTrue, ...
            thetaTrue, ...
            psiTrue);


    %% True NED velocity

    velocityNEDTrue = ...
        R_BN_true * ...
        xTrue(4:6);


    %% True speed

    speedTrue = ...
        norm( ...
        velocityNEDTrue);


    %% ============================================================
    % 2. CURRENT TRUE STATE DERIVATIVE
    %
    % Used only so the simulated IMU can measure the actual vehicle
    % motion produced by the current rotor commands.
    % ============================================================

    xDotTrue = ...
        quad_dynamics_rotors( ...
            tCurrent, ...
            xTrue, ...
            rotorThrust, ...
            P);


    %% ============================================================
    % 3. SIMULATED SENSORS
    % ============================================================


    %% ------------------------------------------------------------
    % IMU
    % -------------------------------------------------------------

    imuMeas = ...
        simulate_imu( ...
            xTrue, ...
            xDotTrue, ...
            P);


    %% ------------------------------------------------------------
    % GPS
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
    % -------------------------------------------------------------

    mag = ...
        simulate_magnetometer( ...
            xTrue, ...
            magNED, ...
            magBias, ...
            magNoiseStd);


    % Apply known hard-iron calibration bias.
    %
    % Same calibrated magnetometer configuration that passed the
    % estimated-state hover test.

    magMeas = ...
        mag.fieldBody - ...
        magBias;


    %% ============================================================
    % 4. NAVIGATION EKF
    %
    % This is the ONLY source of position / velocity / attitude
    % information supplied to Guidance and the outer control loops.
    % ============================================================

    [nav, navState] = ...
        navigation_ekf_step( ...
            navState, ...
            imuMeas, ...
            gpsMeas, ...
            magMeas, ...
            gpsAvailable, ...
            dt);


    %% Estimated speed

    speedEstimate = ...
        norm( ...
        nav.velocity);


    %% ============================================================
    % 5. LOOKAHEAD GUIDANCE
    %
    % Runs at P.guidanceDt rather than every IMU timestep.
    %
    % CRITICAL:
    %
    % Guidance receives nav.position, NOT xTrue.
    % ============================================================

    guidanceUpdate = ...
        mod(k-1,guidanceStride) == 0;


    if guidanceUpdate

        previousSegmentIdx = ...
            segmentIdx;


        [positionCmd, ...
         yawCmd, ...
         segmentIdx, ...
         info] = ...
            lookahead_guidance( ...
                nav.position, ...
                waypoints, ...
                segmentIdx, ...
                yawReference, ...
                P);


        lastGuidanceCrossTrack = ...
            info.crossTrackError;


        %% Log segment transitions

        if segmentIdx ~= previousSegmentIdx

            transitionTimes(end+1,1) = ...
                tCurrent;

            transitionSegments(end+1,1) = ...
                segmentIdx;


            fprintf( ...
                'Guidance advanced to segment %d at t = %.2f s\n', ...
                segmentIdx, ...
                tCurrent);

        end


        %% Preserve heading continuity

        yawReference = ...
            yawCmd;

    end


    %% ============================================================
    % 6. HORIZONTAL POSITION CONTROL
    %
    % Estimated position + velocity only.
    % ============================================================

    [phiCmd, ...
     thetaCmd, ...
     ~, ...
     ~] = ...
        horizontal_position_controller( ...
            positionCmd(1:2), ...
            nav.position(1:2), ...
            nav.velocity(1:2), ...
            nav.euler(3), ...
            P);


    %% ============================================================
    % 7. ALTITUDE CONTROL
    %
    % Estimated Down position / vertical velocity / attitude only.
    % ============================================================

    [Tcmd, ...
     ~, ...
     ~] = ...
        altitude_controller( ...
            positionCmd(3), ...
            nav.position(3), ...
            nav.velocity(3), ...
            nav.euler(1), ...
            nav.euler(2), ...
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
            nav.euler, ...
            P);


    %% ============================================================
    % 9. RATE CONTROL
    %
    % Fast rate loop receives gyro measurement directly.
    % ============================================================

    omegaMeasured = ...
        imuMeas.gyro;


    % Preserve the same bias handling used in the hover test.

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
    % ============================================================

    desiredControl = ...
        [Tcmd;
         tauCmd];


    [rotorThrustNew, ...
     ~] = ...
        control_allocator( ...
            desiredControl, ...
            P);


    %% ============================================================
    % 11. TRUE CROSS-TRACK ERROR
    %
    % Guidance only knows its estimated cross-track error.
    %
    % We separately calculate truth-based error for validation.
    % ============================================================

    trueCrossTrack = ...
        point_to_active_segment_distance( ...
            xTrue(1:3), ...
            waypoints, ...
            segmentIdx);


    %% ============================================================
    % 12. STORE HISTORY
    % ============================================================

    tHistory(k) = ...
        tCurrent;


    xTrueHistory(:,k) = ...
        xTrue;


    positionTrueHistory(:,k) = ...
        xTrue(1:3);


    velocityTrueHistory(:,k) = ...
        velocityNEDTrue;


    attitudeTrueHistory(:,k) = ...
        xTrue(7:9);


    positionEstimateHistory(:,k) = ...
        nav.position;


    velocityEstimateHistory(:,k) = ...
        nav.velocity;


    attitudeEstimateHistory(:,k) = ...
        nav.euler;


    positionCmdHistory(:,k) = ...
        positionCmd;


    yawCmdHistory(k) = ...
        yawCmd;


    segmentHistory(k) = ...
        segmentIdx;


    crossTrackEstimateHistory(k) = ...
        lastGuidanceCrossTrack;


    crossTrackTrueHistory(k) = ...
        trueCrossTrack;


    speedTrueHistory(k) = ...
        speedTrue;


    speedEstimateHistory(k) = ...
        speedEstimate;


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


    gyroBiasEstimateHistory(:,k) = ...
    navState.x(10:12);

    accelBiasEstimateHistory(:,k) = ...
    navState.x(13:15);


    %% ============================================================
    % 13. MISSION COMPLETION CHECK
    %
    % The onboard system uses ESTIMATED state for completion.
    %
    % It is not allowed to inspect truth.
    % ============================================================

    finalWaypoint = ...
        waypoints(end,:).';


    estimatedFinalError = ...
        norm( ...
        nav.position - ...
        finalWaypoint);


    if segmentIdx >= numWaypoints-1 && ...
       estimatedFinalError < ...
           P.waypointAcceptanceRadius && ...
       speedEstimate < ...
           P.waypointAcceptanceSpeed

        fprintf( ...
            '\nEstimated final waypoint reached at t = %.2f s\n', ...
            tCurrent);


        missionComplete = true;

        finalStep = k;

        break;

    end


    %% ============================================================
    % 14. PROPAGATE NONLINEAR VEHICLE
    %
    % Rotor commands held constant for one 100-Hz timestep.
    % ============================================================

    if k < maxSteps

        xTrue = ...
            rk4_quad_step( ...
                tCurrent, ...
                xTrue, ...
                rotorThrustNew, ...
                dt, ...
                P);

    end


    rotorThrust = ...
        rotorThrustNew;

end


%% ================================================================
% HANDLE TIMEOUT
% ================================================================

if ~missionComplete

    finalStep = k;

    fprintf( ...
        '\nMission timed out at %.2f s\n', ...
        tCurrent);

end


%% ================================================================
% TRIM UNUSED HISTORY
% ================================================================

validIdx = ...
    1:finalStep;


tHistory = ...
    tHistory(validIdx);


xTrueHistory = ...
    xTrueHistory(:,validIdx);


positionTrueHistory = ...
    positionTrueHistory(:,validIdx);

velocityTrueHistory = ...
    velocityTrueHistory(:,validIdx);

attitudeTrueHistory = ...
    attitudeTrueHistory(:,validIdx);


positionEstimateHistory = ...
    positionEstimateHistory(:,validIdx);

velocityEstimateHistory = ...
    velocityEstimateHistory(:,validIdx);

attitudeEstimateHistory = ...
    attitudeEstimateHistory(:,validIdx);


positionCmdHistory = ...
    positionCmdHistory(:,validIdx);

yawCmdHistory = ...
    yawCmdHistory(validIdx);


segmentHistory = ...
    segmentHistory(validIdx);


crossTrackEstimateHistory = ...
    crossTrackEstimateHistory(validIdx);

crossTrackTrueHistory = ...
    crossTrackTrueHistory(validIdx);


speedTrueHistory = ...
    speedTrueHistory(validIdx);

speedEstimateHistory = ...
    speedEstimateHistory(validIdx);


attitudeCmdHistory = ...
    attitudeCmdHistory(:,validIdx);


rotorThrustHistory = ...
    rotorThrustHistory(:,validIdx);


gyroBiasEstimateHistory = ...
    gyroBiasEstimateHistory(:,validIdx);

accelBiasEstimateHistory = ...
    accelBiasEstimateHistory(:,validIdx);


gpsPositionHistory = ...
    gpsPositionHistory(:,validIdx);


gpsAvailableHistory = ...
    gpsAvailableHistory(validIdx);


%% ================================================================
% NAVIGATION ERRORS
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


for k = 1:length(tHistory)

    attitudeEstimationError(:,k) = ...
        wrap_pi_local( ...
            attitudeEstimationError(:,k));

end


%% ================================================================
% NAVIGATION RMSE
% ================================================================

positionNavRMSE = ...
    sqrt( ...
    mean( ...
    positionEstimationError.^2, ...
    2));


velocityNavRMSE = ...
    sqrt( ...
    mean( ...
    velocityEstimationError.^2, ...
    2));


attitudeNavRMSE = ...
    rad2deg( ...
    sqrt( ...
    mean( ...
    attitudeEstimationError.^2, ...
    2)));


%% ================================================================
% GUIDANCE METRICS
% ================================================================

rmsTrueCrossTrack = ...
    sqrt( ...
    mean( ...
    crossTrackTrueHistory.^2));


maxTrueCrossTrack = ...
    max( ...
    crossTrackTrueHistory);


rmsEstimatedCrossTrack = ...
    sqrt( ...
    mean( ...
    crossTrackEstimateHistory.^2));


maxEstimatedCrossTrack = ...
    max( ...
    crossTrackEstimateHistory);


%% ================================================================
% FINAL ERRORS
% ================================================================

finalWaypoint = ...
    waypoints(end,:).';


trueFinalPositionError = ...
    norm( ...
    positionTrueHistory(:,end) - ...
    finalWaypoint);


estimatedFinalPositionError = ...
    norm( ...
    positionEstimateHistory(:,end) - ...
    finalWaypoint);


%% ================================================================
% PRINT RESULTS
% ================================================================

fprintf('\n');
fprintf('--- MISSION RESULTS ---\n\n');


if missionComplete

    fprintf('Mission completed successfully.\n');

else

    fprintf('Mission did NOT complete before timeout.\n');

end


fprintf('Mission duration: %.2f s\n\n', ...
    tHistory(end));


fprintf('TRUE GUIDANCE PERFORMANCE\n');

fprintf('RMS cross-track error: %.3f m\n', ...
    rmsTrueCrossTrack);

fprintf('Max cross-track error: %.3f m\n', ...
    maxTrueCrossTrack);

fprintf('True final waypoint error: %.3f m\n\n', ...
    trueFinalPositionError);


fprintf('ONBOARD GUIDANCE ESTIMATE\n');

fprintf('Estimated RMS cross-track error: %.3f m\n', ...
    rmsEstimatedCrossTrack);

fprintf('Estimated max cross-track error: %.3f m\n', ...
    maxEstimatedCrossTrack);

fprintf('Estimated final waypoint error: %.3f m\n\n', ...
    estimatedFinalPositionError);


fprintf('NAVIGATION POSITION RMSE\n');

fprintf('North: %.4f m\n', ...
    positionNavRMSE(1));

fprintf('East:  %.4f m\n', ...
    positionNavRMSE(2));

fprintf('Down:  %.4f m\n\n', ...
    positionNavRMSE(3));


fprintf('NAVIGATION VELOCITY RMSE\n');

fprintf('North: %.4f m/s\n', ...
    velocityNavRMSE(1));

fprintf('East:  %.4f m/s\n', ...
    velocityNavRMSE(2));

fprintf('Down:  %.4f m/s\n\n', ...
    velocityNavRMSE(3));


fprintf('NAVIGATION ATTITUDE RMSE\n');

fprintf('Roll:  %.4f deg\n', ...
    attitudeNavRMSE(1));

fprintf('Pitch: %.4f deg\n', ...
    attitudeNavRMSE(2));

fprintf('Yaw:   %.4f deg\n\n', ...
    attitudeNavRMSE(3));


fprintf('FINAL TRUE ATTITUDE\n');

fprintf('Roll:  %.3f deg\n', ...
    rad2deg(attitudeTrueHistory(1,end)));

fprintf('Pitch: %.3f deg\n', ...
    rad2deg(attitudeTrueHistory(2,end)));

fprintf('Yaw:   %.3f deg\n\n', ...
    rad2deg(attitudeTrueHistory(3,end)));


%% ================================================================
% TRANSITION TIMES
% ================================================================

if ~isempty(transitionTimes)

    fprintf('GUIDANCE SEGMENT TRANSITIONS\n');

    for idx = 1:length(transitionTimes)

        fprintf( ...
            'Segment %d at t = %.2f s\n', ...
            transitionSegments(idx), ...
            transitionTimes(idx));

    end

    fprintf('\n');

end


%% ================================================================
% FIGURE 1 — 3-D MISSION
% ================================================================

figure;

plot3( ...
    positionTrueHistory(1,:), ...
    positionTrueHistory(2,:), ...
    -positionTrueHistory(3,:), ...
    'LineWidth',1.8);

hold on;


plot3( ...
    positionEstimateHistory(1,:), ...
    positionEstimateHistory(2,:), ...
    -positionEstimateHistory(3,:), ...
    '--', ...
    'LineWidth',1.4);


plot3( ...
    waypoints(:,1), ...
    waypoints(:,2), ...
    -waypoints(:,3), ...
    'o--', ...
    'LineWidth',1.2, ...
    'MarkerSize',7);


xlabel('North [m]');
ylabel('East [m]');
zlabel('Altitude [m]');

title( ...
    'Estimated-State Full GNC Mission');

legend( ...
    'True Vehicle', ...
    'Navigation Estimate', ...
    'Desired Path', ...
    'Location','best');

grid on;
axis equal;
view(3);


%% ================================================================
% FIGURE 2 — TOP VIEW
% ================================================================

figure;

plot( ...
    positionTrueHistory(2,:), ...
    positionTrueHistory(1,:), ...
    'LineWidth',1.8);

hold on;


plot( ...
    positionEstimateHistory(2,:), ...
    positionEstimateHistory(1,:), ...
    '--', ...
    'LineWidth',1.4);


plot( ...
    waypoints(:,2), ...
    waypoints(:,1), ...
    'o--', ...
    'LineWidth',1.2, ...
    'MarkerSize',7);


xlabel('East [m]');
ylabel('North [m]');

title( ...
    'Estimated-State Lookahead Guidance — Top View');

legend( ...
    'True Vehicle', ...
    'Navigation Estimate', ...
    'Desired Path', ...
    'Location','best');

grid on;
axis equal;


%% ================================================================
% FIGURE 3 — VEHICLE SPEED
% ================================================================

figure;

plot( ...
    tHistory, ...
    speedTrueHistory, ...
    'LineWidth',1.5);

hold on;


plot( ...
    tHistory, ...
    speedEstimateHistory, ...
    '--', ...
    'LineWidth',1.3);


xlabel('Time [s]');
ylabel('Vehicle Speed [m/s]');

title('Mission Speed');

legend( ...
    'True', ...
    'Estimated');

grid on;


%% ================================================================
% FIGURE 4 — CROSS-TRACK ERROR
% ================================================================

figure;

plot( ...
    tHistory, ...
    crossTrackTrueHistory, ...
    'LineWidth',1.5);

hold on;


plot( ...
    tHistory, ...
    crossTrackEstimateHistory, ...
    '--', ...
    'LineWidth',1.3);


xlabel('Time [s]');
ylabel('Cross-Track Error [m]');

title('Path-Following Error');

legend( ...
    'Truth-Based Error', ...
    'Onboard Estimated Error');

grid on;


%% ================================================================
% FIGURE 5 — YAW TRACKING
% ================================================================

actualYaw = ...
    unwrap( ...
    attitudeTrueHistory(3,:));


estimatedYaw = ...
    unwrap( ...
    attitudeEstimateHistory(3,:));


commandedYaw = ...
    unwrap( ...
    yawCmdHistory);


figure;

plot( ...
    tHistory, ...
    rad2deg(actualYaw), ...
    'LineWidth',1.5);

hold on;


plot( ...
    tHistory, ...
    rad2deg(estimatedYaw), ...
    '--', ...
    'LineWidth',1.3);


plot( ...
    tHistory, ...
    rad2deg(commandedYaw), ...
    ':', ...
    'LineWidth',1.4);


xlabel('Time [s]');
ylabel('Yaw [deg]');

title('Guidance Heading');

legend( ...
    'True Yaw', ...
    'Estimated Yaw', ...
    'Command');

grid on;


%% ================================================================
% FIGURE 6 — NAVIGATION ESTIMATION ERROR
% ================================================================

figure;


subplot(3,1,1);

plot( ...
    tHistory, ...
    positionEstimationError');

grid on;

ylabel('Position Error [m]');

title('Mission Navigation Estimation Error');

legend('N','E','D');


subplot(3,1,2);

plot( ...
    tHistory, ...
    velocityEstimationError');

grid on;

ylabel('Velocity Error [m/s]');

legend('N','E','D');


subplot(3,1,3);

plot( ...
    tHistory, ...
    rad2deg( ...
    attitudeEstimationError'));

grid on;

ylabel('Attitude Error [deg]');
xlabel('Time [s]');

legend('\phi','\theta','\psi');


%% ================================================================
% FIGURE 7 — ATTITUDE TRACKING
% ================================================================

figure;

attitudeNames = ...
    {'Roll \phi', ...
     'Pitch \theta', ...
     'Yaw \psi'};


for axisIdx = 1:3

    subplot(3,1,axisIdx);

    plot( ...
        tHistory, ...
        rad2deg( ...
        attitudeTrueHistory(axisIdx,:)), ...
        'LineWidth',1.5);

    hold on;


    plot( ...
        tHistory, ...
        rad2deg( ...
        attitudeEstimateHistory(axisIdx,:)), ...
        '--', ...
        'LineWidth',1.3);


    plot( ...
        tHistory, ...
        rad2deg( ...
        attitudeCmdHistory(axisIdx,:)), ...
        ':', ...
        'LineWidth',1.3);


    grid on;

    ylabel( ...
        sprintf('%s [deg]', ...
        attitudeNames{axisIdx}));

    if axisIdx == 1

        title('Mission Attitude Tracking');

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
% FIGURE 8 — ROTOR THRUSTS
% ================================================================

figure;

plot( ...
    tHistory, ...
    rotorThrustHistory', ...
    'LineWidth',1.1);

grid on;

xlabel('Time [s]');
ylabel('Rotor Thrust [N]');

title('Mission Rotor Thrust Commands');


rotorLabels = ...
    arrayfun( ...
        @(idx) sprintf('Rotor %d',idx), ...
        1:numRotors, ...
        'UniformOutput',false);


legend( ...
    rotorLabels, ...
    'Location','best');


%% ================================================================
% LOCAL FUNCTION — RK4 VEHICLE PROPAGATION
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
% LOCAL FUNCTION — TRUE CROSS-TRACK DISTANCE
% ================================================================

function distance = ...
    point_to_active_segment_distance( ...
    position, ...
    waypoints, ...
    segmentIdx)


numWaypoints = ...
    size(waypoints,1);


% Clamp index so there is always a valid segment.

idx1 = ...
    max( ...
    1, ...
    min( ...
    segmentIdx, ...
    numWaypoints-1));


idx2 = ...
    idx1 + 1;


segmentStart = ...
    waypoints(idx1,:).';


segmentEnd = ...
    waypoints(idx2,:).';


segmentVector = ...
    segmentEnd - ...
    segmentStart;


segmentLengthSquared = ...
    dot( ...
    segmentVector, ...
    segmentVector);


if segmentLengthSquared < 1e-12

    distance = ...
        norm( ...
        position - ...
        segmentStart);

    return;

end


projection = ...
    dot( ...
        position-segmentStart, ...
        segmentVector) / ...
    segmentLengthSquared;


% Clamp to finite segment.

projection = ...
    max( ...
    0, ...
    min( ...
    1, ...
    projection));


closestPoint = ...
    segmentStart + ...
    projection*segmentVector;


distance = ...
    norm( ...
    position - ...
    closestPoint);

end


%% ================================================================
% LOCAL FUNCTION — ANGLE WRAP
% ================================================================

function angle = ...
    wrap_pi_local(angle)


angle = ...
    mod(angle + pi,2*pi) - pi;

end