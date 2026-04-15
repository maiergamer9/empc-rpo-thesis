function dRho = clohessy_wiltshire(Rho, n)
% clohessy_wiltshire
% Relative EoMs using CW-equations in Hill frame

% ASSUMPTIONS:
%   1. |rho| << r_t  (chaser-target separation small vs target orbit radius)
%   2. Circular target orbit -> theta_dot = n = const, theta_ddot = 0
%      r_t = a = const, r_t_dot = 0
% Equations of motion (no control input):
%   x_ddot = 2*n*y_dot + 3*n^2*x
%   y_ddot = -2*n*x_dot
%   z_ddot = -n^2*z
%
% Input:
%   Rho - relative state [x; y; z; x_dot; y_dot; z_dot] [m, m/s]
%   n   - target mean motion sqrt(mu/a^3) [rad/s]
%
% Output:
%   dRho - time derivative [x_dot; y_dot; z_dot; x_ddot; y_ddot; z_ddot]


x = Rho(1);
y = Rho(2);
z = Rho(3);
x_dot = Rho(4);
y_dot = Rho(5);
z_dot = Rho(6);


% CWH-equations from "Analytical Mechanics of Space Systems" - H.Schaub

x_ddot = 2*n*y_dot + 3*n^2*x;
y_ddot = -2*n*x_dot;
z_ddot = -n^2*z;

dRho = [x_dot; y_dot; z_dot; x_ddot; y_ddot; z_ddot];

end