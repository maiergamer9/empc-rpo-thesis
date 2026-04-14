function dX = two_body(X, mu)
% two_body.m
% This script derives Newton's inertial equations of motions from the
% 2-body-problem.
%
%   r_ddot = a = -mu * r / ||r||^3
%
% "Analytical MEchanics of Space Systems"- H.Schaub, J.L.Junkins is as
% reference.

% Input:
%   X  - state vector [rx; ry; rz; vx; vy; vz] in ECI frame [m, m/s]
%   mu - gravitational parameter [m^3/s^2]
%
% Output:
%   dX - time derivative of the state [vx; vy; vz; ax; ay; az]

r = X(1:3);