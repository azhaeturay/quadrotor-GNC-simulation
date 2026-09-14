function imu = simulate_imu(x, xDot, P)
%SIMULATE_IMU Simulate accelerometer and gyroscope measurements.
%
% Inputs:
%   x     - 12-state vehicle state
%   xDot  - derivative of 12-state vehicle state
%   P     - vehicle parameter structure
%
% Outputs:
%   imu.gyro  - measured angular rate [rad/s]
%   imu.accel - measured specific force [m/s^2]

%% Extract states

vBody = x(4:6);

phi   = x(7);
theta = x(8);
psi   = x(9);

omegaBody = x(10:12);

vDotBody = xDot(4:6);


%% Rotation matrix

% Body -> NED
R_BN = rotation_matrix(phi, theta, psi);


%% ------------------------------------------------------------
% GYROSCOPE
% ------------------------------------------------------------

gyroTrue = omegaBody;

gyroNoise = ...
    P.gyroNoiseStd .* randn(3,1);

gyroMeasured = ...
    gyroTrue ...
    + P.gyroBias ...
    + gyroNoise;


%% ------------------------------------------------------------
% ACCELEROMETER
% ------------------------------------------------------------

% Gravitational acceleration expressed in NED frame
gravityNED = [0; 0; P.g];

% Inertial acceleration expressed in body coordinates
%
% Since velocity is stored in the rotating body frame:
%
% a_B = vDot_B + omega x v_B

accelBody = ...
    vDotBody ...
    + cross(omegaBody, vBody);

% Accelerometers measure SPECIFIC FORCE:
%
% f_B = a_B - R_BN' * g_N

specificForceTrue = ...
    accelBody ...
    - R_BN' * gravityNED;

accelNoise = ...
    P.accelNoiseStd .* randn(3,1);

accelMeasured = ...
    specificForceTrue ...
    + P.accelBias ...
    + accelNoise;


%% Output structure

imu.gyro = gyroMeasured;
imu.accel = accelMeasured;

imu.gyroTrue = gyroTrue;
imu.accelTrue = specificForceTrue;

end