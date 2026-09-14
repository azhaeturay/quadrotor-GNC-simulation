function mag = simulate_magnetometer(x, fieldNED, bias, noiseStd)
%SIMULATE_MAGNETOMETER Simulate a 3-axis magnetometer measurement.
%
% Inputs:
%   x         - 12-state quadrotor state
%               [pn pe pd u v w phi theta psi p q r]'
%
%   fieldNED  - Earth's magnetic field direction in NED frame [3x1]
%   bias      - magnetometer bias [3x1]
%   noiseStd  - magnetometer noise standard deviation
%
% Output:
%   mag.fieldBody     - measured magnetic field in body axes
%   mag.trueFieldBody - ideal magnetic field in body axes

phi   = x(7);
theta = x(8);
psi   = x(9);

% Normalize reference magnetic field.
fieldNED = fieldNED(:);
fieldNED = fieldNED / norm(fieldNED);

% Body-to-NED rotation matrix.
Rbn = euler321_body_to_ned(phi, theta, psi);

% Magnetic field expressed in body coordinates.
fieldBodyTrue = Rbn' * fieldNED;

% Add sensor bias and noise.
fieldBodyMeasured = ...
    fieldBodyTrue + bias(:) + noiseStd .* randn(3,1);

mag.trueFieldBody = fieldBodyTrue;
mag.fieldBody     = fieldBodyMeasured;
end


function Rbn = euler321_body_to_ned(phi, theta, psi)
%EULER321_BODY_TO_NED 3-2-1 body-to-NED rotation matrix.

cphi = cos(phi);
sphi = sin(phi);

ctheta = cos(theta);
stheta = sin(theta);

cpsi = cos(psi);
spsi = sin(psi);

Rbn = [ ...
    ctheta*cpsi, ...
    sphi*stheta*cpsi - cphi*spsi, ...
    cphi*stheta*cpsi + sphi*spsi; ...

    ctheta*spsi, ...
    sphi*stheta*spsi + cphi*cpsi, ...
    cphi*stheta*spsi - sphi*cpsi; ...

    -stheta, ...
    sphi*ctheta, ...
    cphi*ctheta ];
end