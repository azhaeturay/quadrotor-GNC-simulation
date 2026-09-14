function [nav, navState] = navigation_ekf_step( ...
    navState, ...
    imuMeas, ...
    gpsMeas, ...
    magMeas, ...
    gpsAvailable, ...
    dt)
% NAVIGATION_EKF_STEP
%
% One update of a coupled 15-state INS/GPS EKF.
%
%
% State:
%
% x =
%
% [ position_NED       3
%   velocity_NED       3
%   Euler attitude     3
%   gyro bias          3
%   accel bias         3 ]
%
%
% Sensors:
%
%   IMU     -> EKF prediction
%   GPS     -> position / velocity correction
%   Mag     -> yaw correction
%
%
% Key difference from previous estimator:
%
% GPS velocity residuals can now indirectly correct:
%
%   attitude
%   gyro bias
%   accelerometer bias
%
% through EKF cross-covariance.
%
% ---------------------------------------------------------------


g = navState.cfg.g;


gyroMeas = ...
    imuMeas.gyro(:);

accelMeas = ...
    imuMeas.accel(:);

magMeas = ...
    magMeas(:);


%% ================================================================
% 1. INITIALIZATION
% ================================================================

if ~navState.initialized

    %% ------------------------------------------------------------
    % Initial roll / pitch
    %
    % Used only to initialize the filter.
    %
    % The EKF does NOT continuously force accelerometer direction
    % to equal gravity afterward.
    % -------------------------------------------------------------

    ax = accelMeas(1);
    ay = accelMeas(2);
    az = accelMeas(3);


    phi0 = ...
        atan2( ...
        -ay, ...
        -az);


    theta0 = ...
        atan2( ...
        ax, ...
        sqrt(ay^2 + az^2));


    %% Initial yaw

    if norm(magMeas) > 1e-8

        psi0 = ...
            magnetometer_heading( ...
                magMeas, ...
                phi0, ...
                theta0);

    else

        psi0 = 0;

    end


    %% Initial position / velocity

    position0 = zeros(3,1);
    velocity0 = zeros(3,1);


    if gpsAvailable

        if all( ...
            isfinite( ...
            gpsMeas.position))

            position0 = ...
                gpsMeas.position(:);

        end


        if all( ...
            isfinite( ...
            gpsMeas.velocity))

            velocity0 = ...
                gpsMeas.velocity(:);

        end

    end


    %% State initialization

    navState.x = ...
        [position0;
         velocity0;
         phi0;
         theta0;
         psi0;
         zeros(3,1);
         zeros(3,1)];


    navState.initialized = true;


    %% Calculate initial acceleration output

    Rbn = ...
        euler321_body_to_ned( ...
            phi0, ...
            theta0, ...
            psi0);


    accelNED = ...
        Rbn * accelMeas + ...
        [0;0;g];


    navState.lastAccelNED = ...
        accelNED;


    %% Return initialized navigation output

    nav = ...
        make_nav_output( ...
            navState, ...
            accelNED);

    return

end


%% ================================================================
% 2. EKF PREDICTION
% ================================================================

xOld = ...
    navState.x;


Pold = ...
    navState.P;


%% ------------------------------------------------------------
% Nonlinear state prediction
% -------------------------------------------------------------

xPred = ...
    process_model( ...
        xOld, ...
        gyroMeas, ...
        accelMeas, ...
        dt, ...
        g);


%% ------------------------------------------------------------
% Numerical state-transition Jacobian
%
% For this MATLAB prototype, numerical differentiation keeps the
% implementation readable and lets us validate the architecture.
%
% Later, for Simulink / C++, we can replace this with an analytical
% error-state Jacobian.
% -------------------------------------------------------------

F = ...
    numerical_state_jacobian( ...
        xOld, ...
        gyroMeas, ...
        accelMeas, ...
        dt, ...
        g);


%% ------------------------------------------------------------
% Process-noise covariance
% -------------------------------------------------------------

Q = ...
    process_noise_covariance( ...
        xOld, ...
        navState.cfg, ...
        dt);


%% Covariance prediction

Ppred = ...
    F * Pold * F' + Q;


Ppred = ...
    0.5 * ...
    (Ppred + Ppred');


%% Store prediction

navState.x = ...
    xPred;

navState.P = ...
    Ppred;


%% ================================================================
% 3. GPS POSITION / VELOCITY UPDATE
% ================================================================

navState.lastGPSUsed = false;

navState.lastGPSInnovation(:) = nan;


if gpsAvailable

    positionValid = ...
        isfield(gpsMeas,'position') && ...
        numel(gpsMeas.position) == 3 && ...
        all(isfinite(gpsMeas.position));


    velocityValid = ...
        isfield(gpsMeas,'velocity') && ...
        numel(gpsMeas.velocity) == 3 && ...
        all(isfinite(gpsMeas.velocity));


    if positionValid && velocityValid

        %% Measurement

        z = ...
            [gpsMeas.position(:);
             gpsMeas.velocity(:)];


        %% Measurement matrix

        H = zeros(6,15);

        H(1:3,1:3) = eye(3);

        H(4:6,4:6) = eye(3);


        %% Measurement covariance

        Rgps = ...
            diag( ...
            [navState.cfg.gpsPositionStd.^2;
             navState.cfg.gpsVelocityStd.^2]);


        %% Update

        [navState.x, ...
         navState.P, ...
         innovation] = ...
            linear_measurement_update( ...
                navState.x, ...
                navState.P, ...
                z, ...
                H, ...
                Rgps);


        navState.lastGPSInnovation = ...
            innovation;

        navState.lastGPSUsed = true;


    elseif positionValid

        %% Position-only fallback

        z = ...
            gpsMeas.position(:);


        H = zeros(3,15);

        H(:,1:3) = eye(3);


        Rgps = ...
            diag( ...
            navState.cfg.gpsPositionStd.^2);


        [navState.x, ...
         navState.P, ...
         innovation] = ...
            linear_measurement_update( ...
                navState.x, ...
                navState.P, ...
                z, ...
                H, ...
                Rgps);


        navState.lastGPSInnovation(1:3) = ...
            innovation;

        navState.lastGPSUsed = true;

    end

end


%% ================================================================
% 4. MAGNETOMETER YAW UPDATE
% ================================================================

navState.lastMagInnovation = nan;


if norm(magMeas) > 1e-8 && ...
        all(isfinite(magMeas))


    phiHat = ...
        navState.x(7);

    thetaHat = ...
        navState.x(8);


    psiMag = ...
        magnetometer_heading( ...
            magMeas, ...
            phiHat, ...
            thetaHat);


    psiHat = ...
        navState.x(9);


    yawInnovation = ...
        wrap_pi_local( ...
            psiMag - psiHat);


    %% Measurement matrix:
    %
    % z = psi
    %
    % Only yaw is measured directly here.
    %
    % Other states can still be corrected through covariance
    % cross-coupling.

    Hmag = zeros(1,15);

    Hmag(9) = 1;


    Rmag = ...
        navState.cfg.magYawStd^2;


    S = ...
        Hmag * navState.P * Hmag' + ...
        Rmag;


    K = ...
        navState.P * Hmag' / S;


    navState.x = ...
        navState.x + ...
        K * yawInnovation;


    I = eye(15);


    navState.P = ...
        (I-K*Hmag) * ...
        navState.P * ...
        (I-K*Hmag)' + ...
        K*Rmag*K';


    navState.P = ...
        0.5 * ...
        (navState.P + navState.P');


    navState.lastMagInnovation = ...
        yawInnovation;

end


%% ================================================================
% 5. NORMALIZE EULER ANGLES
% ================================================================

navState.x(7:9) = ...
    wrap_pi_local( ...
        navState.x(7:9));


%% ================================================================
% 6. CALCULATE CURRENT NED ACCELERATION
% ================================================================

phiHat = navState.x(7);
thetaHat = navState.x(8);
psiHat = navState.x(9);

accelBiasHat = ...
    navState.x(13:15);


Rbn = ...
    euler321_body_to_ned( ...
        phiHat, ...
        thetaHat, ...
        psiHat);


specificForceCorrected = ...
    accelMeas - ...
    accelBiasHat;


accelNED = ...
    Rbn * ...
    specificForceCorrected + ...
    [0;0;g];


navState.lastAccelNED = ...
    accelNED;


%% ================================================================
% 7. OUTPUT
% ================================================================

nav = ...
    make_nav_output( ...
        navState, ...
        accelNED);


end


%% =================================================================
% PROCESS MODEL
% =================================================================

function xNext = ...
    process_model( ...
    x, ...
    gyroMeas, ...
    accelMeas, ...
    dt, ...
    g)


%% State extraction

position = ...
    x(1:3);

velocity = ...
    x(4:6);

phi   = x(7);
theta = x(8);
psi   = x(9);

gyroBias = ...
    x(10:12);

accelBias = ...
    x(13:15);


%% Correct IMU measurements

omegaBody = ...
    gyroMeas - ...
    gyroBias;


specificForceBody = ...
    accelMeas - ...
    accelBias;


%% Rotation matrix

Rbn = ...
    euler321_body_to_ned( ...
        phi, ...
        theta, ...
        psi);


%% Translational acceleration

gravityNED = ...
    [0;
     0;
     g];


accelNED = ...
    Rbn * ...
    specificForceBody + ...
    gravityNED;


%% Position / velocity propagation

positionNext = ...
    position + ...
    velocity*dt + ...
    0.5*accelNED*dt^2;


velocityNext = ...
    velocity + ...
    accelNED*dt;


%% Attitude propagation

E = ...
    euler_rate_matrix( ...
        phi, ...
        theta);


eulerDot = ...
    E * ...
    omegaBody;


eulerNext = ...
    [phi;
     theta;
     psi] + ...
    eulerDot*dt;


eulerNext = ...
    wrap_pi_local( ...
        eulerNext);


%% Bias random-walk mean:
%
% zero change during deterministic prediction.

gyroBiasNext = ...
    gyroBias;

accelBiasNext = ...
    accelBias;


%% State

xNext = ...
    [positionNext;
     velocityNext;
     eulerNext;
     gyroBiasNext;
     accelBiasNext];

end


%% =================================================================
% NUMERICAL STATE JACOBIAN
% =================================================================

function F = ...
    numerical_state_jacobian( ...
    x, ...
    gyroMeas, ...
    accelMeas, ...
    dt, ...
    g)


fx = ...
    process_model( ...
        x, ...
        gyroMeas, ...
        accelMeas, ...
        dt, ...
        g);


F = zeros(15,15);


perturbation = ...
    [ ...
    1e-5*ones(3,1);     % position
    1e-5*ones(3,1);     % velocity
    1e-6*ones(3,1);     % Euler angle
    1e-6*ones(3,1);     % gyro bias
    1e-5*ones(3,1) ...  % accel bias
    ];


for stateIdx = 1:15

    dx = ...
        perturbation(stateIdx);


    xPerturbed = ...
        x;


    xPerturbed(stateIdx) = ...
        xPerturbed(stateIdx) + dx;


    fPerturbed = ...
        process_model( ...
            xPerturbed, ...
            gyroMeas, ...
            accelMeas, ...
            dt, ...
            g);


    delta = ...
        fPerturbed - fx;


    %% Angle differences must wrap

    delta(7:9) = ...
        wrap_pi_local( ...
            delta(7:9));


    F(:,stateIdx) = ...
        delta / dx;

end

end


%% =================================================================
% PROCESS-NOISE COVARIANCE
% =================================================================

function Q = ...
    process_noise_covariance( ...
    x, ...
    cfg, ...
    dt)


phi   = x(7);
theta = x(8);
psi   = x(9);


Rbn = ...
    euler321_body_to_ned( ...
        phi, ...
        theta, ...
        psi);


E = ...
    euler_rate_matrix( ...
        phi, ...
        theta);


%% ------------------------------------------------------------
% Noise vector:
%
% w =
%
% [ accelerometer white noise
%   gyro white noise
%   gyro bias random walk
%   accel bias random walk ]
%
% 12 components total.
% -------------------------------------------------------------

G = zeros(15,12);


%% Accelerometer noise -> position / velocity

G(1:3,1:3) = ...
    0.5*dt^2 * ...
    Rbn;


G(4:6,1:3) = ...
    dt * ...
    Rbn;


%% Gyro noise -> Euler attitude

G(7:9,4:6) = ...
    dt * ...
    E;


%% Gyro-bias random walk

G(10:12,7:9) = ...
    sqrt(dt) * ...
    eye(3);


%% Accel-bias random walk

G(13:15,10:12) = ...
    sqrt(dt) * ...
    eye(3);


%% Noise covariance

Qc = ...
    diag( ...
    [cfg.accelInputStd.^2;
     cfg.gyroInputStd.^2;
     cfg.gyroBiasRWStd.^2;
     cfg.accelBiasRWStd.^2]);


Q = ...
    G * Qc * G';

end


%% =================================================================
% LINEAR MEASUREMENT UPDATE
% =================================================================

function [x, P, innovation] = ...
    linear_measurement_update( ...
    x, ...
    P, ...
    z, ...
    H, ...
    R)


innovation = ...
    z - H*x;


S = ...
    H*P*H' + R;


K = ...
    P*H' / S;


x = ...
    x + ...
    K*innovation;


I = eye(size(P));


%% Joseph covariance update

P = ...
    (I-K*H) * ...
    P * ...
    (I-K*H)' + ...
    K*R*K';


P = ...
    0.5 * ...
    (P + P');

end


%% =================================================================
% NAVIGATION OUTPUT
% =================================================================

function nav = ...
    make_nav_output( ...
    navState, ...
    accelNED)

x = ...
    navState.x;


%% Position

nav.position = ...
    x(1:3);


%% Velocity

nav.velocity = ...
    x(4:6);


%% Attitude

nav.euler = ...
    x(7:9);

%% NED acceleration

nav.accelNED = ...
    accelNED;


%% State uncertainty

nav.positionStd = ...
    sqrt( ...
    diag( ...
    navState.P(1:3,1:3)));


nav.velocityStd = ...
    sqrt( ...
    diag( ...
    navState.P(4:6,4:6)));


nav.attitudeStd = ...
    sqrt( ...
    diag( ...
    navState.P(7:9,7:9)));


%% Bias estimates: present on initialization and every normal update.

nav.gyroBias = x(10:12);
nav.accelBias = x(13:15);


%% GPS update flag

nav.gpsUsed = ...
    navState.lastGPSUsed;

end


%% =================================================================
% EULER RATE MATRIX
% =================================================================

function E = ...
    euler_rate_matrix( ...
    phi, ...
    theta)


ctheta = ...
    cos(theta);


if abs(ctheta) < 1e-5

    if ctheta >= 0

        ctheta = 1e-5;

    else

        ctheta = -1e-5;

    end

end


ttheta = ...
    sin(theta) / ...
    ctheta;


E = ...
    [ ...
    1, ...
    sin(phi)*ttheta, ...
    cos(phi)*ttheta;

    0, ...
    cos(phi), ...
    -sin(phi);

    0, ...
    sin(phi)/ctheta, ...
    cos(phi)/ctheta ...
    ];

end


%% =================================================================
% MAGNETOMETER HEADING
% =================================================================

function psiMag = ...
    magnetometer_heading( ...
    magBody, ...
    phi, ...
    theta)


mx = magBody(1);
my = magBody(2);
mz = magBody(3);


Rx = ...
    [ ...
    1, 0, 0;

    0, cos(phi), -sin(phi);

    0, sin(phi), cos(phi) ...
    ];


Ry = ...
    [ ...
    cos(theta), 0, sin(theta);

    0, 1, 0;

    -sin(theta), 0, cos(theta) ...
    ];


mLevel = ...
    Ry * Rx * ...
    [mx;my;mz];


psiMag = ...
    atan2( ...
        -mLevel(2), ...
         mLevel(1));


psiMag = ...
    wrap_pi_local( ...
        psiMag);

end


%% =================================================================
% BODY -> NED ROTATION
% =================================================================

function Rbn = ...
    euler321_body_to_ned( ...
    phi, ...
    theta, ...
    psi)


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
% ANGLE WRAP
% =================================================================

function angle = ...
    wrap_pi_local(angle)


angle = ...
    mod(angle + pi,2*pi) - pi;

end