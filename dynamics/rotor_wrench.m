function control = rotor_wrench(rotorThrust, P)
% ROTOR_WRENCH
% Converts individual rotor thrusts into total thrust and body moments.
%
% Input:
% rotorThrust = [T1; T2; T3; T4] [N]
%
% Output:
% control = [T; tau_x; tau_y; tau_z]

A = allocation_matrix(P);

control = A * rotorThrust;

end