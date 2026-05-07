% sim_mpc_regulation.m
% Simulates Regulation MPC rendezvous in the Hill frame using CWH equations.
%
% drives the relative state to zero (origin = target) subject to hard input constraints.
clear; clc;


%%  LOAD CONSTANTS
constants;

%%  SIMULATION PARAMETERS (as before)
dt      = 10;
t_final = 2 * T;
n_steps = round(t_final / dt);
t       = (0:n_steps-1) * dt;


%%  MPC PARAMETERS

N     = 20;       % [-]      prediction horizon (initial guess)
u_max = 1e-2;     % [m/s^2]  thrust limit (initial guess?)


%%  CWH SS

Ac = [0      0     0    1     0    0  ;
      0      0     0    0     1    0  ;
      0      0     0    0     0    1  ;
      3*n^2  0     0    0     2*n  0  ;
      0      0     0   -2*n   0    0  ;
      0      0    -n^2  0     0    0 ];

Bc = [zeros(3,3); eye(3)];


%%  DISCRETIZE
[Ad, Bd] = discretize(Ac, Bc, dt);


%%  COST MATRICES (same as LQR for direct comparison)
Q = diag([1e-2, 1e-2, 1e-2, 1e0, 1e0, 1e0]);
R = diag([1e4,  1e4,  1e4]);


%%  TERMINAL COST — P from LQR 
[~, P] = lqr_controller(Ad, Bd, Q, R);


%%  INITIAL CONDITIONS
X0 = [500; 1000; 200; 0; 0; 0];


%%  INTEGRATE TARGET ORBIT (as before)
X_t0     = [a; 0; 0; 0; sqrt(mu/a); 0];
f_target = @(X) two_body(X, mu);
X_t      = rk4_integrator(f_target, X_t0, dt, n_steps);


%%  CLOSED-LOOP MPC SIMULATION
X_hist = zeros(6, n_steps);
U_hist = zeros(3, n_steps);
X_hist(:, 1) = X0;

fprintf('Running Regulation MPC (%d steps, N=%d)...\n', n_steps, N);
for k = 1:n_steps-1
    if mod(k, 100) == 0
        fprintf('  Step %d / %d\n', k, n_steps);
    end
    [u_opt, ~, ~]  = mpc_regulation(X_hist(:,k), Ad, Bd, Q, R, P, N, u_max);
    U_hist(:, k)   = u_opt;
    X_hist(:, k+1) = Ad * X_hist(:,k) + Bd * u_opt;
end
U_hist(:,end) = mpc_regulation(X_hist(:,end), Ad, Bd, Q, R, P, N, u_max);

fprintf('Simulation complete.\n');