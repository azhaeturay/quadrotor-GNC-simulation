function A = allocation_matrix(P)
% ALLOCATION_MATRIX
% Maps individual rotor thrusts to total thrust and body moments.
%
% [T; tau_x; tau_y; tau_z] = A * [T1; T2; T3; T4]

A = zeros(4,4);

for i = 1:4

    % Rotor position in body frame
    ri = P.rotorPos(i,:).';

    % Force direction per 1 N of rotor thrust
    % Rotor thrust acts upward = -z_B
    Fi_per_N = [0; 0; -1];

    % Roll/pitch moment produced per unit thrust
    moment_per_N = cross(ri, Fi_per_N);

    % Total thrust contribution
    A(1,i) = 1;

    % Roll moment coefficient
    A(2,i) = moment_per_N(1);

    % Pitch moment coefficient
    A(3,i) = moment_per_N(2);

    % Yaw reaction torque coefficient
    A(4,i) = P.rotorSpin(i) * P.kappa;

end

end