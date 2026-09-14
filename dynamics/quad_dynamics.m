function xdot = quad_dynamics(~, x, inputs, P)
% QUAD_DYNAMICS
% Nonlinear 6-DOF rigid-body equations of motion for a quadrotor.
%
% State vector:
%
% x = [xN yN zN u v w phi theta psi p q r]'
%
% where:
%
% xN, yN, zN     = NED position [m]
% u, v, w        = body-frame velocity [m/s]
% phi,theta,psi  = roll, pitch, yaw [rad]
% p, q, r        = body angular rates [rad/s]
%
% Inputs:
%
% inputs = [T tau_x tau_y tau_z]'
%
% T                  = total rotor thrust [N]
% tau_x,tau_y,tau_z  = body moments [N*m]


%% Extract states

% NED position
xN = x(1); %#ok<NASGU>
yN = x(2); %#ok<NASGU>
zN = x(3); %#ok<NASGU>

% Body-frame linear velocity
u = x(4);
v = x(5);
w = x(6);

% Euler angles
phi   = x(7);
theta = x(8);
psi   = x(9);

% Body angular rates
p = x(10);
q = x(11);
r = x(12);


%% Extract control inputs

T = inputs(1);

tau = [inputs(2);
       inputs(3);
       inputs(4)];


%% Useful vectors

vB = [u;
      v;
      w];

omegaB = [p;
          q;
          r];


%% 1. Position kinematics

R_BN = rotation_matrix(phi, theta, psi);

positionDot = R_BN * vB;


%% 2. Translational dynamics

% Gravity expressed in NED
gN = [0;
      0;
      P.g];

% Convert gravity acceleration into body coordinates
gB = R_BN.' * gN;

% Total rotor thrust acts upward, which is -z_B
thrustB = [0;
           0;
          -T];

% Body-frame translational acceleration
velocityDot = gB ...
            + thrustB / P.mass ...
            - cross(omegaB, vB);


%% 3. Euler-angle kinematics

E = [1, sin(phi)*tan(theta),  cos(phi)*tan(theta);
     0, cos(phi),            -sin(phi);
     0, sin(phi)/cos(theta),  cos(phi)/cos(theta)];

eulerDot = E * omegaB;


%% 4. Rotational dynamics

omegaDot = P.I \ ...
    (tau - cross(omegaB, P.I * omegaB));


%% Assemble state derivative

xdot = [positionDot;
        velocityDot;
        eulerDot;
        omegaDot];

end