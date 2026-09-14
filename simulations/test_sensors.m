clear;
clc;
close all;

projectRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(projectRoot));

P = vehicle_params();

% Make random noise repeatable
rng(7);


%% ============================================================
% TEST 1: STATIONARY LEVEL HOVER
% ============================================================

fprintf('\n--- TEST 1: STATIONARY HOVER ---\n');

x = zeros(12,1);

% Vehicle is stationary
xDot = zeros(12,1);

N = 1000;

gyroSamples = zeros(N,3);
accelSamples = zeros(N,3);

for k = 1:N

    imu = simulate_imu(x, xDot, P);

    gyroSamples(k,:) = imu.gyro.';
    accelSamples(k,:) = imu.accel.';

end

fprintf('\nMean gyro measurement [deg/s]:\n');
disp(rad2deg(mean(gyroSamples,1)));

fprintf('Expected gyro bias [deg/s]:\n');
disp(rad2deg(P.gyroBias).');

fprintf('\nMean accelerometer measurement [m/s^2]:\n');
disp(mean(accelSamples,1));

fprintf('Expected approximate accelerometer mean [m/s^2]:\n');
disp(([0;0;-P.g] + P.accelBias).');

fprintf('\nMeasured gyro noise std [deg/s]:\n');
disp(rad2deg(std(gyroSamples,0,1)));

fprintf('Specified gyro noise std [deg/s]:\n');
disp(rad2deg(P.gyroNoiseStd) .* ones(1,3));

fprintf('\nMeasured accelerometer noise std [m/s^2]:\n');
disp(std(accelSamples,0,1));

fprintf('Specified accelerometer noise std [m/s^2]:\n');
disp(P.accelNoiseStd .* ones(1,3));

%% ============================================================
% TEST 2: FREE FALL
% ============================================================

fprintf('\n--- TEST 2: FREE FALL ---\n');

x = zeros(12,1);

xDot = zeros(12,1);

% NED z is positive downward.
% A freely falling vehicle accelerates downward at +g.
xDot(6) = P.g;

imu = simulate_imu(x, xDot, P);

fprintf('\nAccelerometer measurement during free fall:\n');
disp(imu.accel.');

fprintf('Ideal specific force during free fall:\n');
disp([0 0 0]);


%% ============================================================
% TEST 3: GPS
% ============================================================

fprintf('\n--- TEST 3: GPS ---\n');

x = zeros(12,1);

% Known NED position
x(1:3) = [5; 3; -2];

% 1 m/s forward body velocity
x(4:6) = [1; 0; 0];

N = 500;

gpsPosition = zeros(N,3);
gpsVelocity = zeros(N,3);

for k = 1:N

    gps = simulate_gps(x,P);

    gpsPosition(k,:) = gps.position.';
    gpsVelocity(k,:) = gps.velocity.';

end

fprintf('\nMean GPS position [m]:\n');
disp(mean(gpsPosition,1));

fprintf('True GPS position [m]:\n');
disp(x(1:3).');

fprintf('\nMean GPS velocity [m/s]:\n');
disp(mean(gpsVelocity,1));

fprintf('True NED velocity [m/s]:\n');
disp([1 0 0]);

fprintf('\nMeasured GPS position std [m]:\n');
disp(std(gpsPosition,0,1));

fprintf('Specified GPS position std [m]:\n');
disp(P.gpsPositionNoiseStd.');

fprintf('\nMeasured GPS velocity std [m/s]:\n');
disp(std(gpsVelocity,0,1));

fprintf('Specified GPS velocity std [m/s]:\n');
disp(P.gpsVelocityNoiseStd.');


%% ============================================================
% FIGURES
% ============================================================

figure;

plot(accelSamples(:,1));
hold on;
plot(accelSamples(:,2));
plot(accelSamples(:,3));

xlabel('Sample');
ylabel('Specific Force [m/s^2]');
title('Simulated Accelerometer Measurements');

legend('x','y','z');
grid on;


figure;

plot(gpsPosition(:,1), gpsPosition(:,2), '.');

hold on;

plot(5,3,'rx','MarkerSize',12,'LineWidth',2);

xlabel('North Position [m]');
ylabel('East Position [m]');

title('Simulated GPS Position Measurements');

legend('GPS Measurements','True Position');

axis equal;
grid on;