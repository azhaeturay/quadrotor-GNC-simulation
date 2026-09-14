function eulerHat = attitude_estimator_mag_step( ...
    eulerHat, gyroMeas, accelMeas, ...
    magMeas, magAvailable, dt, ...
    alphaRP, alphaYaw, fieldNED)
%ATTITUDE_ESTIMATOR_MAG_STEP
%
% Complementary attitude estimator using:
%
%   Gyroscope     -> short-term attitude propagation
%   Accelerometer -> long-term roll/pitch correction
%   Magnetometer  -> long-term yaw correction
%
% Angles and gyro rates are in radians / radians per second.

    phi   = eulerHat(1);
    theta = eulerHat(2);
    psi   = eulerHat(3);

    p = gyroMeas(1);
    q = gyroMeas(2);
    r = gyroMeas(3);

    %% ================================================================
    % 1. GYRO PROPAGATION
    % =================================================================

    % Avoid numerical problems near theta = +/- 90 deg.
    cosTheta = cos(theta);

    if abs(cosTheta) < 1e-6
        cosTheta = sign(cosTheta) * 1e-6;
    end

    tanTheta = sin(theta) / cosTheta;

    phiDot = ...
        p + q*sin(phi)*tanTheta + r*cos(phi)*tanTheta;

    thetaDot = ...
        q*cos(phi) - r*sin(phi);

    psiDot = ...
        q*sin(phi)/cosTheta + r*cos(phi)/cosTheta;

    eulerPred = eulerHat + dt * ...
        [phiDot;
         thetaDot;
         psiDot];

    phiPred   = eulerPred(1);
    thetaPred = eulerPred(2);
    psiPred   = wrap_to_pi_local(eulerPred(3));

    %% ================================================================
    % 2. ACCELEROMETER ROLL / PITCH OBSERVATION
    % =================================================================

    ax = accelMeas(1);
    ay = accelMeas(2);
    az = accelMeas(3);

    phiAccel = atan2(-ay, -az);

    thetaAccel = atan2( ...
        ax, ...
        sqrt(ay^2 + az^2));

    % Complementary correction.
    phiHat = phiPred + ...
        (1 - alphaRP) * ...
        wrap_to_pi_local(phiAccel - phiPred);

    thetaHat = thetaPred + ...
        (1 - alphaRP) * ...
        wrap_to_pi_local(thetaAccel - thetaPred);

    %% ================================================================
    % 3. MAGNETOMETER YAW OBSERVATION
    % =================================================================

    psiHat = psiPred;

    if magAvailable

        mx = magMeas(1);
        my = magMeas(2);
        mz = magMeas(3);

        magBody = [mx; my; mz];

        % -------------------------------------------------------------
        % Tilt compensation
        %
        % Remove estimated roll and pitch from the magnetic-field
        % measurement before calculating heading.
        % -------------------------------------------------------------

        cphi = cos(phiHat);
        sphi = sin(phiHat);

        ctheta = cos(thetaHat);
        stheta = sin(thetaHat);

        % Roll/pitch portion of body-to-NED rotation.
        Rrp = [ ...
            ctheta,  stheta*sphi,  stheta*cphi;
            0,       cphi,        -sphi;
           -stheta,  ctheta*sphi,  ctheta*cphi ];

        magLevel = Rrp * magBody;

        % Magnetic heading measured in the leveled frame.
        magneticAngle = atan2( ...
            magLevel(2), ...
            magLevel(1));

        % Reference magnetic declination embedded in fieldNED.
        declination = atan2( ...
            fieldNED(2), ...
            fieldNED(1));

        % Recover yaw.
        psiMag = wrap_to_pi_local( ...
            declination - magneticAngle);

        % Heading innovation.
        yawError = wrap_to_pi_local( ...
            psiMag - psiPred);

        % Complementary yaw correction.
        psiHat = wrap_to_pi_local( ...
            psiPred + (1 - alphaYaw)*yawError);
    end

    %% Output

    eulerHat = [ ...
        wrap_to_pi_local(phiHat);
        wrap_to_pi_local(thetaHat);
        wrap_to_pi_local(psiHat)];

end


function angle = wrap_to_pi_local(angle)
    angle = mod(angle + pi, 2*pi) - pi;
end