function omegaCmd = attitude_controller(attitudeCmd, attitude, P)
% ATTITUDE_CONTROLLER
% Outer-loop attitude controller.
%
% Inputs:
%   attitudeCmd = [phi_cmd; theta_cmd; psi_cmd] [rad]
%   attitude    = [phi; theta; psi]             [rad]
%
% Output:
%   omegaCmd    = [p_cmd; q_cmd; r_cmd]         [rad/s]

%% Attitude error

attitudeError = attitudeCmd - attitude;

% Wrap yaw error to [-pi, pi]
attitudeError(3) = atan2( ...
    sin(attitudeError(3)), ...
    cos(attitudeError(3)));

%% Proportional attitude control

omegaCmd = P.attitudeKp .* attitudeError;

%% Rate-command limits

omegaCmd = max(omegaCmd, -P.maxBodyRate);
omegaCmd = min(omegaCmd,  P.maxBodyRate);

end