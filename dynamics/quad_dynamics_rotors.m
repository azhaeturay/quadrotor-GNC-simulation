function xdot = quad_dynamics_rotors(t, x, rotorThrust, P)
% QUAD_DYNAMICS_ROTORS
% Propagates the nonlinear 6-DOF quadrotor dynamics using
% individual rotor thrusts as the actuator inputs.
%
% rotorThrust = [T1; T2; T3; T4] [N]

%% Convert rotor thrusts into total thrust and moments

control = rotor_wrench(rotorThrust, P);

%% Propagate the validated 6-DOF plant

xdot = quad_dynamics(t, x, control, P);

end