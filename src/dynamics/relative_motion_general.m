function dRho = relative_motion_general(Rho, X_t, mu)
% relative_motion_general
% Computes relative EoMS in Hill frame for an elliptic target orbit.
%
% ASSUMPTIONS:
%   1. |rho| << r_t  (chaser-target separation small vs target orbit radius)
%   2. No assumption on target orbit shape (valid for elliptic orbits)
% NOTE: For Keplerian two-body motion, omega (argument of perigee) is constant.
% Therefore theta_dot = omega_dot + f_dot = f_dot
% and theta_ddot = f_ddot.
%
% Equations of motion (Schaub Chapter 14, no control input):
%   x_ddot = 2*f_dot*y_dot + f_ddot*y + (f_dot^2 + 2*mu/r_t^3)*x
%   y_ddot = -2*f_dot*x_dot - f_ddot*x + (f_dot^2 - mu/r_t^3)*y
%   z_ddot = -(mu/r_t^3)*z
%
% Input:
%   Rho  - relative state [x; y; z; x_dot; y_dot; z_dot] [m, m/s]
%   X_t  - target ECI state [rx; ry; rz; vx; vy; vz] [m, m/s]
%   mu   - gravitational parameter [m^3/s^2]
%
% Output:
%   dRho - time derivative [x_dot; y_dot; z_dot; x_ddot; y_ddot; z_ddot]


x = Rho(1);
y = Rho(2);
z = Rho(3);
x_dot = Rho(4);
y_dot = Rho(5);
z_dot = Rho(6);

% Different to CW we now need the target kinematics defined in
% target_kinematics.m
[r_t, ~, f_dot, f_ddot] = target_kinematics(X_t);


% Equations of motion (Schaub 14.20, p.776)

x_ddot = 2*f_dot*y_dot + f_ddot*y + (f_dot^2 + 2*mu/r_t^3)*x;
y_ddot = -2*f_dot*x_dot - f_ddot*x + (f_dot^2 - mu/r_t^3)*y;
z_ddot = -(mu/r_t^3) * z;

dRho = [x_dot; y_dot; z_dot; x_ddot; y_ddot; z_ddot];

end
