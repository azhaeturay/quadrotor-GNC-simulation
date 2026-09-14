function tauCmd = rate_controller(omegaCmd, omega, P)
% RATE_CONTROLLER
% Proportional body-rate controller.
%
% Inputs:
%   omegaCmd = [p_cmd; q_cmd; r_cmd] [rad/s]
%   omega    = [p; q; r]             [rad/s]
%
% Output:
%   tauCmd   = [tau_x; tau_y; tau_z] [N*m]

%% Rate error

rateError = omegaCmd - omega;

%% Proportional rate control

tauCmd = P.rateKp .* rateError;

end