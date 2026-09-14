function [nav, navState] = navigation_estimator_step( ...
    navState, imuMeas, gpsMeas, magMeas, gpsAvailable, dt)
% NAVIGATION_ESTIMATOR_STEP
%
% Performs one update of the integrated navigation estimator.
%
%
% INPUTS
%
% navState
%     Internal estimator state
%
% imuMeas.gyro
%     Body angular rates [rad/s]
%
% imuMeas.accel
%     Body-frame specific force [m/s^2]
%
% gpsMeas.position
%     GPS NED position [m]
%
% gpsMeas.velocity
%     GPS NED velocity [m/s]
%
% magMeas
%     Body-frame magnetic-field vector
%
% gpsAvailable
%     True when a new GPS measurement is available
%
% dt
%     Estimator timestep [s]
%
%
% OUTPUTS
%
% nav.position
% nav.velocity
% nav.euler
% nav.accelNED
%
% navState
%     Updated estimator memory
%
% ---------------------------------------------------------------


%% ================================================================
%  MEASUREMENTS
% ================================================================

gyroBody  = imuMeas.gyro(:);
accelBody = imuMeas.accel(:);
magBody   = magMeas(:);

g = navState.cfg.g;


%% ================================================================
%  ATTITUDE INITIALIZATION
% ================================================================

if ~navState.attitudeInitialized

    ax = accelBody(1);
    ay = accelBody(2);
    az = accelBody(3);

    % Gravity-based roll and pitch estimate
    phi0 = atan2(-ay, -az);

    theta0 = atan2( ...
        ax, ...
        sqrt(ay^2 + az^2));

    % Magnetometer-based yaw estimate
    psi0 = magnetometer_heading( ...
        magBody, ...
        phi0, ...
        theta0);

    navState.euler = ...
        [phi0;
         theta0;
         psi0];

    navState.attitudeInitialized = true;

end


%% ================================================================
%  1. ATTITUDE PREDICTION USING GYROSCOPE
% ================================================================

phi   = navState.euler(1);
theta = navState.euler(2);
psi   = navState.euler(3);

p = gyroBody(1);
q = gyroBody(2);
r = gyroBody(3);


%% Euler angle kinematics

ctheta = cos(theta);

% Protect against singularity near pitch = +/-90 deg
if abs(ctheta) < 1e-4
    ctheta = sign(ctheta) * 1e-4;
end

phiDot = ...
    p + ...
    q*sin(phi)*tan(theta) + ...
    r*cos(phi)*tan(theta);

thetaDot = ...
    q*cos(phi) - ...
    r*sin(phi);

psiDot = ...
    q*sin(phi)/ctheta + ...
    r*cos(phi)/ctheta;


%% Gyro propagated attitude

phiGyro   = phi   + phiDot   * dt;
thetaGyro = theta + thetaDot * dt;
psiGyro   = psi   + psiDot   * dt;

psiGyro = wrap_pi_local(psiGyro);


%% ================================================================
%  2. ACCELEROMETER ROLL / PITCH CORRECTION
% ================================================================

accelMagnitude = norm(accelBody);

phiAccel = atan2( ...
    -accelBody(2), ...
    -accelBody(3));

thetaAccel = atan2( ...
    accelBody(1), ...
    sqrt(accelBody(2)^2 + accelBody(3)^2));


%% Only trust accelerometer as gravity reference when magnitude
%  is reasonably close to g.

accelReliable = ...
    abs(accelMagnitude - g) < 0.30*g;


if accelReliable

    alphaRP = navState.cfg.alphaRP;

    phiNew = ...
        phiGyro + ...
        (1-alphaRP) * ...
        wrap_pi_local(phiAccel - phiGyro);

    thetaNew = ...
        thetaGyro + ...
        (1-alphaRP) * ...
        wrap_pi_local(thetaAccel - thetaGyro);

else

    phiNew   = phiGyro;
    thetaNew = thetaGyro;

end


%% ================================================================
%  3. MAGNETOMETER YAW CORRECTION
% ================================================================

if norm(magBody) > 1e-8

    psiMag = magnetometer_heading( ...
        magBody, ...
        phiNew, ...
        thetaNew);

    alphaYaw = navState.cfg.alphaYaw;

    yawError = ...
        wrap_pi_local(psiMag - psiGyro);

    psiNew = ...
        psiGyro + ...
        (1-alphaYaw)*yawError;

else

    psiNew = psiGyro;

end


psiNew = wrap_pi_local(psiNew);


%% Store attitude estimate

navState.euler = ...
    [phiNew;
     thetaNew;
     psiNew];


%% ================================================================
%  4. BODY ACCELERATION -> NED ACCELERATION
% ================================================================

Rbn = euler321_body_to_ned( ...
    phiNew, ...
    thetaNew, ...
    psiNew);


% Accelerometer measures specific force:
%
%       f = a - g
%
% therefore:
%
%       a_NED = Rbn*f_body + g_NED

gravityNED = ...
    [0;
     0;
     g];

accelNED = ...
    Rbn * accelBody + gravityNED;

navState.lastAccelNED = accelNED;


%% ================================================================
%  5. POSITION / VELOCITY KALMAN PREDICTION
% ================================================================

I3 = eye(3);

F = ...
    [I3       dt*I3;
     zeros(3) I3];

G = ...
    [0.5*dt^2*I3;
          dt*I3];


%% Process noise

sigmaA = navState.cfg.accelProcessStd;

Qa = diag(sigmaA.^2);

Q = G * Qa * G';


%% State prediction

navState.xPV = ...
    F * navState.xPV + ...
    G * accelNED;


%% Covariance prediction

navState.PPV = ...
    F * navState.PPV * F' + Q;


%% ================================================================
%  6. GPS UPDATE
% ================================================================

navState.lastGPSUsed = false;

if gpsAvailable

    gpsPosition = gpsMeas.position(:);


    %% ------------------------------------------------------------
    % First GPS measurement initializes translation state
    % -------------------------------------------------------------

    if ~navState.gpsInitialized

        navState.xPV(1:3) = gpsPosition;

        if isfield(gpsMeas,'velocity') && ...
                all(isfinite(gpsMeas.velocity))

            navState.xPV(4:6) = ...
                gpsMeas.velocity(:);

        end

        navState.gpsInitialized = true;

    end


    %% ------------------------------------------------------------
    % Normal Kalman measurement update
    % -------------------------------------------------------------

    velocityAvailable = ...
        isfield(gpsMeas,'velocity') && ...
        numel(gpsMeas.velocity) == 3 && ...
        all(isfinite(gpsMeas.velocity));


    if velocityAvailable

        %% GPS position + velocity

        z = ...
            [gpsPosition;
             gpsMeas.velocity(:)];

        H = eye(6);

        R = diag([ ...
            navState.cfg.gpsPositionStd.^2;
            navState.cfg.gpsVelocityStd.^2]);

    else

        %% GPS position only

        z = gpsPosition;

        H = ...
            [eye(3) zeros(3)];

        R = ...
            diag( ...
            navState.cfg.gpsPositionStd.^2);

    end


    %% Innovation

    innovation = ...
        z - H*navState.xPV;


    %% Innovation covariance

    S = ...
        H*navState.PPV*H' + R;


    %% Kalman gain

    K = ...
        navState.PPV * H' / S;


    %% Correct state estimate

    navState.xPV = ...
        navState.xPV + K*innovation;


    %% Joseph-form covariance update

    I = eye(6);

    navState.PPV = ...
        (I-K*H) * ...
        navState.PPV * ...
        (I-K*H)' + ...
        K*R*K';


    navState.lastGPSUsed = true;

end


%% ================================================================
%  NAVIGATION OUTPUT
% ================================================================

nav.position = navState.xPV(1:3);
nav.velocity = navState.xPV(4:6);

nav.euler = navState.euler;

nav.accelNED = accelNED;

nav.positionStd = ...
    sqrt(diag(navState.PPV(1:3,1:3)));

nav.velocityStd = ...
    sqrt(diag(navState.PPV(4:6,4:6)));

nav.gpsUsed = navState.lastGPSUsed;

end


%% =================================================================
%  MAGNETOMETER HEADING
% =================================================================

function psiMag = magnetometer_heading(magBody, phi, theta)

% Remove roll and pitch while preserving heading.

Rx = ...
    [1       0          0;
     0 cos(phi) -sin(phi);
     0 sin(phi)  cos(phi)];

Ry = ...
    [ cos(theta) 0 sin(theta);
              0  1          0;
     -sin(theta) 0 cos(theta)];

mLevel = ...
    Ry * Rx * magBody;

psiMag = atan2( ...
    -mLevel(2), ...
     mLevel(1));

psiMag = wrap_pi_local(psiMag);

end


%% =================================================================
%  BODY -> NED ROTATION MATRIX
% =================================================================

function Rbn = euler321_body_to_ned(phi, theta, psi)

cphi = cos(phi);
sphi = sin(phi);

ctheta = cos(theta);
stheta = sin(theta);

cpsi = cos(psi);
spsi = sin(psi);


Rbn = ...
    [ ...
    ctheta*cpsi, ...
    sphi*stheta*cpsi - cphi*spsi, ...
    cphi*stheta*cpsi + sphi*spsi;

    ctheta*spsi, ...
    sphi*stheta*spsi + cphi*cpsi, ...
    cphi*stheta*spsi - sphi*cpsi;

    -stheta, ...
    sphi*ctheta, ...
    cphi*ctheta ...
    ];

end


%% =================================================================
%  ANGLE WRAP
% =================================================================

function angle = wrap_pi_local(angle)

angle = mod(angle + pi, 2*pi) - pi;

end