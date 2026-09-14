clear;
clc;
close all;

%% ================================================================
%  PROJECT SETUP
%  ================================================================

projectRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(projectRoot));

P = vehicle_params();


%% ================================================================
%  MULTI-RATE SENSOR TIMING TEST
%  ================================================================

fprintf('\n--- MULTI-RATE SENSOR TIMING TEST ---\n');

imuRate = P.imu.rateHz;
gpsRate = P.gps.rateHz;

imuDt = P.imu.dt;
gpsDt = P.gps.dt;

% Number of IMU samples between GPS updates
gpsIntervalTicks = round(imuRate / gpsRate);

fprintf('IMU rate: %.1f Hz\n', imuRate);
fprintf('GPS rate: %.1f Hz\n', gpsRate);

fprintf('IMU sample period: %.4f s\n', imuDt);
fprintf('GPS sample period: %.4f s\n', gpsDt);

fprintf('GPS update every %d IMU samples\n', ...
    gpsIntervalTicks);


%% ================================================================
%  SIMULATION SETTINGS
%  ================================================================

simulationTime = 2.0;   % seconds

numTicks = round(simulationTime / imuDt);

tHistory = zeros(numTicks + 1,1);

imuUpdateHistory = false(numTicks + 1,1);
gpsUpdateHistory = false(numTicks + 1,1);


%% ================================================================
%  SENSOR SCHEDULER
%  ================================================================

for k = 0:numTicks

    % Current simulation time
    t = k * imuDt;

    % ------------------------------------------------------------
    % IMU
    %
    % The IMU is our base-rate sensor, so it updates every tick.
    % ------------------------------------------------------------

    imuUpdate = true;


    % ------------------------------------------------------------
    % GPS
    %
    % GPS updates every N IMU ticks.
    % ------------------------------------------------------------

    gpsUpdate = mod(k,gpsIntervalTicks) == 0;


    % ------------------------------------------------------------
    % Store timing information
    % ------------------------------------------------------------

    idx = k + 1;

    tHistory(idx) = t;

    imuUpdateHistory(idx) = imuUpdate;
    gpsUpdateHistory(idx) = gpsUpdate;

end


%% ================================================================
%  VALIDATE UPDATE COUNTS
%  ================================================================

numImuUpdates = sum(imuUpdateHistory);
numGpsUpdates = sum(gpsUpdateHistory);

expectedImuUpdates = numTicks + 1;

expectedGpsUpdates = ...
    floor(numTicks/gpsIntervalTicks) + 1;


fprintf('\n--- UPDATE COUNTS ---\n');

fprintf('IMU updates: %d\n', numImuUpdates);
fprintf('Expected:    %d\n', expectedImuUpdates);

fprintf('\nGPS updates: %d\n', numGpsUpdates);
fprintf('Expected:    %d\n', expectedGpsUpdates);


%% ================================================================
%  PRINT FIRST FEW EVENTS
%  ================================================================

fprintf('\n--- FIRST SENSOR EVENTS ---\n');

fprintf(' Time [s]      IMU       GPS\n');
fprintf(' ----------------------------\n');

for k = 1:min(31,length(tHistory))

    fprintf(' %7.2f        %d         %d\n', ...
        tHistory(k), ...
        imuUpdateHistory(k), ...
        gpsUpdateHistory(k));

end


%% ================================================================
%  PLOT SENSOR UPDATE TIMES
%  ================================================================

figure;

hold on;

% IMU events
stem( ...
    tHistory(imuUpdateHistory), ...
    ones(sum(imuUpdateHistory),1), ...
    'Marker','none');

% GPS events
stem( ...
    tHistory(gpsUpdateHistory), ...
    2*ones(sum(gpsUpdateHistory),1), ...
    'Marker','o');

xlabel('Time [s]');
ylabel('Sensor');

yticks([1 2]);
yticklabels({'IMU','GPS'});

title('Multi-Rate Sensor Update Timing');

ylim([0.5 2.5]);

grid on;
hold off;