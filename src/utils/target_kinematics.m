function [r_t, r_t_dot, f_dot, f_ddot] = target_kinematics(X_t)
% target_kinematics
% Extracts orbital kinematics of the target spacecraft from its ECI state.
% Required by the relative equations of motion in the Hill frame.
%
% NOTATION:
%   f     - true anomaly
%   theta - true latitude, theta = omega + f
%   For Keplerian two-body motion, omega is constant (no perturbations),
%   therefore theta_dot = f_dot and theta_ddot = f_ddot.
% Input:
%   X_t  - target ECI state [rx; ry; rz; vx; vy; vz] [m, m/s]
%   mu   - gravitational parameter [m^3/s^2]
%
% Output:
%   r_t       - target orbital radius (scalar) [m]
%   r_t_dot   - radial velocity dr/dt [m/s]
%   f_dot  - true latitude rate dtheta/dt [rad/s]
%   f_ddot - true latitude acceleration d²theta/dt² [rad/s^2]

r = X_t(1:3);
v = X_t(4:6);

r_t = norm(r);

r_t_dot = dot(r, v) / r_t;                          % Radial velocity

h_vec = cross(r, v);    
h = norm(h_vec);                                    % Specific angular momentum

f_dot = h / r_t^2;                              % True latitude rate
f_ddot = -2 * r_t_dot * f_dot / r_t;        % True latitude acceleration

end