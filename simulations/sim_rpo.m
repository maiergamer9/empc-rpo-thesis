% sim_rpo.m
% Simulates relative motion between target and chaser spacecraft
% in the Hill frame (LVLH) using selectable dynamics models.
%
% DYNAMICS MODELS:
%   'nonlinear' - Exact nonlinear EoMs (Schaub 14.13)
%   'general'   - Linearized, elliptic target orbit (Schaub 14.20)
%   'CWH'       - Clohessy-Wiltshire-Hill, circular target orbit
%
% OUTPUT:
%   - Phase plot of relative trajectory in Hill frame
%   - Export to JSON for GUI visualization

clear all; clc;

%% Configuration (choose CWH/general/nonlinear, to be expanded)

dynamics_model = '...';

%% Constants
constants;

%% Simulation Parameters (as in two_body)
dt      = 10;               % [s]   timestep
n_steps = round(T / dt);    % [-]   one full target orbit
t       = (0:n_steps-1)*dt; % [s]   time vector

%% Target initial condition (circular orbit in equatorial plane)

rx0_t = a;
ry0_t = 0;
rz0_t = 0;
vc    = sqrt(mu / a);           % circular velocity
vx0_t = 0;
vy0_t = vc;
vz0_t = 0;

X_t0 = [rx0_t; ry0_t; rz0_t; vx0_t; vy0_t; vz0_t];

%% Integrate Target Orbit (like in two_body.m)

f_target = @(X) two_body(X, mu);
X_t = rk4_integrator(f_target, X_t0, dt, n_steps);

%% Integration of Relative Motion

% Switch Cases between the dynamic models
switch dynamics_model
    case 'nonlinear'
        f_rel = @(Rho, X_t_k) relative_motion_nonlinear(Rho, X_t_k, mu);
    case 'general'
        f_rel = @(Rho, X_t_k) relative_motion_general(Rho, X_t_k, mu);
    case 'CWH'
        f_rel = @(Rho, ~) clohessy_wiltshire(Rho, n);
    otherwise
        error('Unknown dynamics model: %s', dynamics_model);
end

Rho = zeros(6, n_steps);
Rho(:, 1) = Rho0;

for k = 1:n_steps-1
    X_t_k = X_t(:, k);
    Rho(:, k+1) = rk4_integrator(@(rho) f_rel(rho, X_t_k), Rho(:,k), dt, 2);
end
