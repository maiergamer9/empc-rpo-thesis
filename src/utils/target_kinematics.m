function [r_t, r_t_dot, theta_dot, theta_ddot] = target_kinematics(X_t, mu)
% target_kinematics
% Extracts orbital kinematics of the target spacecraft from its ECI state.
% Required by the relative equations of motion in the Hill frame.
%
% Input:
%   X_t  - target ECI state [rx; ry; rz; vx; vy; vz] [m, m/s]
%   mu   - gravitational parameter [m^3/s^2]
%
% Output:
%   r_t       - target orbital radius (scalar) [m]
%   r_t_dot   - radial velocity dr/dt [m/s]
%   theta_dot  - true latitude rate dtheta/dt [rad/s]
%   theta_ddot - true latitude acceleration d²theta/dt² [rad/s^2]


end