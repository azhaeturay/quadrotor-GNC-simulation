function R_BN = rotation_matrix(phi, theta, psi)
% ROTATION_MATRIX
% Returns the direction cosine matrix that transforms a vector
% expressed in the body frame (B) into the NED navigation frame (N).
%
% Convention:
%   3-2-1 Euler sequence
%   yaw (psi) -> pitch (theta) -> roll (phi)
%
% Inputs are in radians.

cphi = cos(phi);
sphi = sin(phi);

ctheta = cos(theta);
stheta = sin(theta);

cpsi = cos(psi);
spsi = sin(psi);

%% Elementary rotation matrices

Rx = [1,    0,     0;
    0,  cphi, -sphi;
    0,  sphi,  cphi];

Ry = [ ctheta, 0, stheta;
    0, 1,      0;
    -stheta, 0, ctheta];

Rz = [cpsi, -spsi, 0;
    spsi,  cpsi, 0;
    0,     0, 1];

%% Body -> Navigation transformation

R_BN = Rz * Ry * Rx;

end