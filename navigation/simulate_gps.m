function gps = simulate_gps(x, P)
%SIMULATE_GPS Simulate GPS position and velocity measurements.
%
% GPS measurements are expressed in the NED navigation frame.

%% State extraction

positionNED = x(1:3);

vBody = x(4:6);

phi   = x(7);
theta = x(8);
psi   = x(9);


%% Convert body velocity -> NED velocity

R_BN = rotation_matrix(phi, theta, psi);

velocityNED = R_BN * vBody;


%% GPS position measurement

positionNoise = ...
    P.gpsPositionNoiseStd .* randn(3,1);

positionMeasured = ...
    positionNED + positionNoise;


%% GPS velocity measurement

velocityNoise = ...
    P.gpsVelocityNoiseStd .* randn(3,1);

velocityMeasured = ...
    velocityNED + velocityNoise;


%% Output structure

gps.position = positionMeasured;
gps.velocity = velocityMeasured;

gps.positionTrue = positionNED;
gps.velocityTrue = velocityNED;

end