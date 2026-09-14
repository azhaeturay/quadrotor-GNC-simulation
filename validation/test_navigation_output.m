function test_navigation_output()
%TEST_NAVIGATION_OUTPUT Regression for the EKF's initialization interface.
root = fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(root));
P = vehicle_params();
state = navigation_ekf_init(P);
imu.gyro = P.gyroBias;
imu.accel = [0; 0; -P.g] + P.accelBias;
gps.position = [1; 2; -2];
gps.velocity = zeros(3,1);
mag = [0.45; 0; 0.89];

% First call initializes and returns early; both bias fields must exist.
[initial, state] = navigation_ekf_step(state, imu, gps, mag, true, 0.01);
assert(isequal(initial.gyroBias, zeros(3,1)));
assert(isequal(initial.accelBias, zeros(3,1)));
assert(isequal(initial.position, gps.position));

% Exercise prediction-only and GPS-corrected paths with nonzero bias states.
state.x(10:12) = P.gyroBias;
state.x(13:15) = P.accelBias;
for gpsAvailable = [false true]
    [nav, state] = navigation_ekf_step( ...
        state, imu, gps, mag, gpsAvailable, 0.01);
    assert(isequal(sort(fieldnames(initial)), sort(fieldnames(nav))));
    assert(isequal(nav.gyroBias, state.x(10:12)));
    assert(isequal(nav.accelBias, state.x(13:15)));
    assert(all(isfinite(state.x)) && all(isfinite(state.P), 'all'));
end
fprintf('PASS: consistent navigation output on initialization and normal calls.\n');
end
