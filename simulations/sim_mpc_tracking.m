%sim_mpc_tracking.m
% Simulates tracking MPC rednezvous in the Hill frame by extending the
% regulation_mpc with a reference trajectory to track

clear; clc;

%% Constants
constants;


%%  SIMULATION PARAMETERS
dt      = 10;
t_final = 2 * T;            
n_steps = round(t_final / dt);
t       = (0:n_steps-1) * dt;
t_min   = t / 60;


%%  MPC PARAMETERS
N     = 20;       % [-]      prediction horizon
u_max = 1e-2;     % [m/s^2]  


%%  CWH STATE SPACE
Ac = [0      0     0    1     0    0  ;
      0      0     0    0     1    0  ;
      0      0     0    0     0    1  ;
      3*n^2  0     0    0     2*n  0  ;
      0      0     0   -2*n   0    0  ;
      0      0    -n^2  0     0    0 ];

Bc = [zeros(3,3); eye(3)];


%%  DISCRETIZE
[Ad, Bd] = discretize(Ac, Bc, dt);


%%  COST MATRICES (as before for comparison)
Q = diag([1e-2, 1e-2, 1e-2, 1e0, 1e0, 1e0]);
R = diag([1e8,  1e8,  1e8]);

[~, P] = lqr_controller(Ad, Bd, Q, R);


%%  INITIAL CONDITIONS

x0_ic = 500;    % [m] radial offset
y0_ic = 1000;   % [m] along-track offset
z0_ic = 0;      % [m] cross-track offset

X0 = [x0_ic; y0_ic; z0_ic; 0; 0; 0];


%%  REFERENCE TRAJECTORY

traj_mode = 'rbar_vbar';    % can also choose rbar or vbar only, but need to adapt the initial conditions
[R_ref, phase_switch_idx] = reference_trajectory(x0_ic, y0_ic, z0_ic, ...
                                                  n_steps, dt, traj_mode);

fprintf('Trajectory mode: %s\n', traj_mode);
fprintf('Phase switch at step %d (t = %.1f min)\n', ...
        phase_switch_idx, t_min(phase_switch_idx));


%%  INTEGRATE TARGET ORBIT
X_t0     = [a; 0; 0; 0; sqrt(mu/a); 0];
f_target = @(X) two_body(X, mu);
X_t      = rk4_integrator(f_target, X_t0, dt, n_steps);


%%  CLOSED-LOOP TRACKING MPC SIMULATION
X_hist = zeros(6, n_steps);
U_hist = zeros(3, n_steps);
E_hist = zeros(6, n_steps);   % tracking error history

X_hist(:, 1) = X0;
E_hist(:, 1) = X0 - R_ref(:, 1);

fprintf('Running Tracking MPC (%d steps, N=%d)...\n', n_steps, N);

for k = 1:n_steps-1
    if mod(k, 100) == 0
        fprintf('  Step %d / %d\n', k, n_steps);
    end

    % current reference
    r_k = R_ref(:, k);

    % solve tracking MPC
    [u_opt, ~, ~] = mpc_tracking(X_hist(:,k), r_k, Ad, Bd, Q, R, P, N, u_max);

    % apply control and propagate
    U_hist(:, k)   = u_opt;
    X_hist(:, k+1) = Ad * X_hist(:,k) + Bd * u_opt;
    E_hist(:, k+1) = X_hist(:, k+1) - R_ref(:, k+1);
end
U_hist(:,end) = mpc_tracking(X_hist(:,end), R_ref(:,end), Ad, Bd, Q, R, P, N, u_max);

fprintf('Simulation complete.\n');