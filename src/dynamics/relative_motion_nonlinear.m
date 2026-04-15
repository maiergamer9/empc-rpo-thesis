function dRho = relative_motion_nonlinear(Rho, X_t, mu)
% relative_motion_nonlinear.m
% Computes exact nonlinear rel. EoMs in Hill frame
%
% NO ASSUMPTIONS: Use this for Nonlinear Models or NMPC
%
% Equations of motion (Schaub Chapter 14, Eq. 14.13, no control input):
%   x_ddot = 2*f_dot*(y_dot - y*r_t_dot/r_t) + x*f_dot^2 + mu/r_t^2 - mu*(r_t+x)/r_d^3
%   y_ddot = -2*f_dot*(x_dot - x*r_t_dot/r_t) + y*f_dot^2 - mu*y/r_d^3
%   z_ddot = -mu*z/r_d^3
%
% where:
%   r_d = sqrt((r_t+x)^2 + y^2 + z^2)  chaser orbital radius
%   f_dot = theta_dot (true anomaly rate = true latitude rate for Keplerian orbits)
%
% Input:
%   Rho  - relative state [x; y; z; x_dot; y_dot; z_dot] [m, m/s]
%   X_t  - target ECI state [rx; ry; rz; vx; vy; vz] [m, m/s]
%   mu   - gravitational parameter [m^3/s^2]
%
% Output:
%   dRho - time derivative [x_dot; y_dot; z_dot; x_ddot; y_ddot; z_ddot]

% Extract relative state
x = Rho(1);
y = Rho(2);
z = Rho(3);
x_dot = Rho(4);
y_dot = Rho(5);
z_dot = Rho(6);

% Extract Target kinematics, same as before
[r_t, r_t_dot, f_dot, ~] = target_kinematics(X_t);

% Chaser orbital radius in Hill frame (from Schaub)
r_d = sqrt((r_t + x)^2 + y^2 + z^2);

% Equations of Motion (Schaub 14.13, p.773)
x_ddot = 2*f_dot*(y_dot - y*r_t_dot/r_t) + x*f_dot^2 + mu/r_t^2 - mu*(r_t+x)/r_d^3;
y_ddot = -2*f_dot*(x_dot - x*r_t_dot/r_t) + y*f_dot^2 - mu*y/r_d^3;
z_ddot = -mu*z/r_d^3;


dRho = [x_dot; y_dot; z_dot; x_ddot; y_ddot; z_ddot];


end
