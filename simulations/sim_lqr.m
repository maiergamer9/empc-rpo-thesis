% sim_lqr.m
% Simulates LQR-controlled rendezvous in the Hill frame based on CW
% equations

clear; clc;

%% constants
constants;

%% Simulation Parameters
dt = 10;
t_final = 2*T;
n_steps = round(t_final / dt);
t = (0:n_steps-1) * dt;

%% CW Equations on cont. State-space
% with X=[x; y; z; x_dot; y_dot; z_dot] and simple u = [ux; uy; uz]

Ac = [0         0         0         1         0         0  ;
      0         0         0         0         1         0  ;
      0         0         0         0         0         1  ;
      3*n^2     0         0         0         2*n       0  ;
      0         0         0        -2*n       0         0  ;
      0         0        -n^2       0         0         0];

Bc = [zeros(3,3); eye(3)];

%% Discretize (ZOH)

[Ad, Bd] = discretize(Ac, Bc, dt);


%% LQR design

Q = diag([1e0, 1e0, 1e0, ...        % position weights
          1e0, 1e0, 1e0]);          % velocity weights

R = diag([1e0, 1e0, 1e0]);        % control weights  

[K, P] = lqr_controller(Ad, Bd, Q, R);

fprintf('LQR gain K computed.\n');
fprintf('Closed-loop eigenvalues:\n');
disp(eig(Ad - Bd*K));