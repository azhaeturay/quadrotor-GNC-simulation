function [eulerHat, state] = attitude_estimator_step( ...
    gyroMeas, accelMeas, state, dt, tau)
%ATTITUDE_ESTIMATOR_STEP
% Complementary-filter attitude estimator.
%
% Estimates 3-2-1 Euler attitude:
%
%   eulerHat = [phi; theta; psi]
%
% where
%   phi   = roll  [rad]
%   theta = pitch [rad]
%   psi   = yaw   [rad]
%
% Coordinate convention:
%   Navigation frame: NED
%   Body frame:       x-forward, y-right, z-down
%
% Inputs:
%   gyroMeas  = [p; q; r] body angular rates [rad/s]
%   accelMeas = [ax; ay; az] specific force [m/s^2]
%   state     = estimator state structure
%   dt        = timestep [s]
%   tau       = complementary-filter time constant [s]
%
% Outputs:
%   eulerHat  = estimated [roll; pitch; yaw] [rad]
%   state     = updated estimator state


%% -------------------------------------------------------------
%  CURRENT ESTIMATE
% --------------------------------------------------------------

phi   = state.euler(1);
theta = state.euler(2);
psi   = state.euler(3);


%% -------------------------------------------------------------
%  GYROSCOPE BIAS CORRECTION
% --------------------------------------------------------------
% For now gyroBiasHat is initialized to zero.
%
% We are intentionally NOT estimating gyro bias yet.
% This means yaw should slowly drift during the test.

gyroCorrected = gyroMeas - state.gyroBiasHat;

p = gyroCorrected(1);
q = gyroCorrected(2);
r = gyroCorrected(3);


%% -------------------------------------------------------------
%  BODY RATES -> EULER ANGLE RATES
% -------------------------------------------------------------

cphi = cos(phi);
sphi = sin(phi);

ctheta = cos(theta);
stheta = sin(theta);

% Prevent numerical problems near pitch = +/- 90 deg
if abs(ctheta) < 1e-6
    ctheta = sign(ctheta + eps) * 1e-6;
end

phiDot = ...
    p + q*sphi*stheta/ctheta + r*cphi*stheta/ctheta;

thetaDot = ...
    q*cphi - r*sphi;

psiDot = ...
    q*sphi/ctheta + r*cphi/ctheta;


%% -------------------------------------------------------------
%  GYRO PREDICTION
% -------------------------------------------------------------

phiPred   = phi   + phiDot   * dt;
thetaPred = theta + thetaDot * dt;
psiPred   = psi   + psiDot   * dt;


%% -------------------------------------------------------------
%  ACCELEROMETER ATTITUDE MEASUREMENT
% -------------------------------------------------------------
%
% For a vehicle with little translational acceleration:
%
%        f_b ~= R_NB * [0; 0; -g]
%
% Therefore the gravity direction provides roll and pitch.

ax = accelMeas(1);
ay = accelMeas(2);
az = accelMeas(3);

phiAccel = atan2(-ay, -az);

thetaAccel = atan2( ...
    ax, ...
    sqrt(ay^2 + az^2));


%% -------------------------------------------------------------
%  COMPLEMENTARY FILTER
% -------------------------------------------------------------
%
% alpha near 1:
%   trust gyro heavily over short times
%
% 1-alpha:
%   slowly correct attitude using gravity

alpha = tau / (tau + dt);

rollInnovation = wrap_to_pi_local(phiAccel - phiPred);

pitchInnovation = wrap_to_pi_local(thetaAccel - thetaPred);

phiHat = ...
    phiPred + (1-alpha) * rollInnovation;

thetaHat = ...
    thetaPred + (1-alpha) * pitchInnovation;

% No absolute yaw sensor yet.
%
% Therefore yaw is gyro propagation ONLY.

psiHat = psiPred;


%% -------------------------------------------------------------
%  ANGLE WRAPPING
% -------------------------------------------------------------

phiHat   = wrap_to_pi_local(phiHat);
thetaHat = wrap_to_pi_local(thetaHat);
psiHat   = wrap_to_pi_local(psiHat);


%% -------------------------------------------------------------
%  SAVE STATE
% -------------------------------------------------------------

eulerHat = [phiHat; thetaHat; psiHat];

state.euler = eulerHat;

end


%% =============================================================
%  LOCAL HELPER
% =============================================================

function angle = wrap_to_pi_local(angle)

angle = mod(angle + pi, 2*pi) - pi;

end