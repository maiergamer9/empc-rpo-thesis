% sim_two_body.m
% Simulates the Keplaria orbit by calling the dynamics of the
% 2-body-problem together with the RK4-integrator.

clear; clc; close all;

%% Load Constants
constants;

%% Simulation Parameters
dt = 10;                % timestep [s]
n_steps = round(T/dt);  % one orbit [-]
t = (0:n_steps-1)*dt;   % time vector [s]

%% Initial conditions (circular robit in equatorial plane)

% position [m]
rx0 = a;                % perigee on x-axis
ry0 = 0;
rz0 = 0;

% velocity [m/s]
v_c = sqrt(mu/a);       % circular velocity [m/s]

vx0 = 0;
vy0 = v_c;
vz0 = 0;

%% Initial state as 1x6 vector
x0 = [rx0; ry0; rz0; vx0; vy0; vz0];

%% Integration

f = @(X) two_body(X, mu);
X = rk4_integrator(f, x0, dt, n_steps);

%% Plot
figure;
plot3(X(1,:)/1000, X(2,:)/1000, X(3,:)/1000, 'b', 'LineWidth', 1.5);
hold on;