% sim_two_body.m
% Simulates the Keplaria orbit by calling the dynamics of the
% 2-body-problem together with the RK4-integrator.

clear; clc; close all;

%% Load Constants
constants;

%% Simulation Parameters
dt = 10;
N = T/dt;

%% Initial conditions

% position
rx0 = a;                % semi-major axis points to perigee
ry0 = 0;
rz0 = 0;

% velocity
v_c = sqrt(mu/r);       % circular velocity

vx0 = 0;
vy0 = 0;
vz0 = v_c;

%% Initial state as 1x6 vector
x0 = [rx0; ry0; rz0; vx0; vy0; vz0];

%% Integration


X = rk4_integrator(f, x0, dt, N)