function P = vehicle_params()
% VEHICLE_PARAMS
% Physical parameters for the quadrotor GNC simulation.
%
% Coordinate conventions:
%   Navigation frame: North-East-Down (NED)
%   Body frame:      x-forward, y-right, z-down
%   Attitude:        3-2-1 yaw-pitch-roll Euler angles
%
% All quantities use SI units.

%% environment

P.g = 9.80665;             % gravitational acceleration [m/s^2]


%% vehicle mass properties

P.mass = 1.50;             % vehicle mass [kg]

P.Ixx = 0.029;             % roll moment of inertia [kg*m^2]
P.Iyy = 0.029;             % pitch moment of inertia [kg*m^2]
P.Izz = 0.055;             % yaw moment of inertia [kg*m^2]

P.I = diag([P.Ixx P.Iyy P.Izz]);


%% geometry

P.arm = 0.225;             % CG-to-rotor distance [m]

% X-configuration rotor coordinates in the body frame
a = P.arm / sqrt(2);

%                 x       y       z
P.rotorPos = [    a      -a       0;      % rotor 1: front-left
    a       a       0;      % rotor 2: front-right
    -a       a       0;      % rotor 3: rear-right
    -a      -a       0];     % rotor 4: rear-left


%% rotor properties

P.minRotorThrust = 0;      % minimum individual rotor thrust [N]
P.maxRotorThrust = 8;      % maximum individual rotor thrust [N]

% Rotor spin direction viewed from above:
% +1 = CCW
% -1 = CW

P.rotorSpin = [1; -1; 1; -1];

% Ratio between rotor reaction torque and thrust.
% Q = kappa*T

P.kappa = 0.015;           % [m]


%% useful derived quantities

P.weight = P.mass * P.g;

P.hoverThrustTotal = P.weight;

P.hoverThrustRotor = P.weight / 4;

%% Rate controller gains

P.rateKp = [0.29;
    0.29;
    0.55];

%% Attitude controller gains

P.attitudeKp = [4.0;
    4.0;
    2.0];

P.maxBodyRate = deg2rad([120;
    120;
    90]);

%% Altitude controller gains

P.posKpZ = 1.0;            % position -> velocity [1/s]
P.velKpZ = 4.0;            % velocity -> acceleration [1/s]

P.maxVerticalVelocity = 2.0;       % [m/s]
P.maxVerticalAcceleration = 4.0;   % [m/s^2]

%% Horizontal position controller gains

P.posKpXY = 0.75;          % position -> velocity [1/s]
P.velKpXY = 3.0;           % velocity -> acceleration [1/s]

P.maxHorizontalVelocity = 2.5;       % [m/s]
P.maxHorizontalAcceleration = 3.0;   % [m/s^2]

P.maxTilt = deg2rad(20);              % [rad]

%% Waypoint guidance

P.waypointAcceptanceRadius = 0.20;   % [m]
P.waypointAcceptanceSpeed  = 0.15;   % [m/s]

%% Lookahead path guidance

P.lookaheadDistance = 1.0;     % [m]

P.pathSwitchRadius = 0.35;     % [m]

P.guidanceDt = 0.02;           % [s] = 50 Hz guidance update

%% Navigation sensor parameters

% ============================================================
% IMU
% ============================================================

% Measurement update rate
P.imuRate = 100;                 % [Hz]

% Gyroscope
P.gyroBias = deg2rad([0.15; -0.10; 0.08]);   % [rad/s]
P.gyroNoiseStd = deg2rad(0.08);               % [rad/s]

% Accelerometer
P.accelBias = [0.03; -0.02; 0.04];            % [m/s^2]
P.accelNoiseStd = 0.04;                        % [m/s^2]


% ============================================================
% GPS
% ============================================================

P.gpsRate = 10;                  % [Hz]

% Position measurement noise
P.gpsPositionNoiseStd = ...
    [0.25; 0.25; 0.40];          % [m]

% Velocity measurement noise
P.gpsVelocityNoiseStd = ...
    [0.05; 0.05; 0.08];          % [m/s]

%% Sensor update rates

P.imu.rateHz = 100;                 % IMU update rate [Hz]
P.imu.dt     = 1/P.imu.rateHz;      % IMU sample period [s]

P.gps.rateHz = 10;                  % GPS update rate [Hz]
P.gps.dt     = 1/P.gps.rateHz;      % GPS sample period [s]
end