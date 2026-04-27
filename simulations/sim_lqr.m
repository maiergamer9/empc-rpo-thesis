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

Q = diag([1e-2, 1e-2, 1e-2, ...        % position weights
          1e0, 1e0, 1e0]);          % velocity weights

R = diag([1e8, 1e8, 1e8]);        % control weights  

[K, P] = lqr_controller(Ad, Bd, Q, R);

fprintf('LQR gain K computed.\n');
fprintf('Closed-loop eigenvalues:\n');
disp(eig(Ad - Bd*K));

%% Initial Conditions

x0     =  500e3;  
y0     = 1000e3;   
z0     =  200e3;   
x_dot0 =    0;   
y_dot0 =    0;   
z_dot0 =    0;  

X0 = [x0; y0; z0; x_dot0; y_dot0; z_dot0];

%% Integrate target orbit (for JSON export and visualization)

rx0_t = a; ry0_t = 0; rz0_t = 0;
vc    = sqrt(mu / a);
X_t0  = [rx0_t; ry0_t; rz0_t; 0; vc; 0];
f_target = @(X) two_body(X, mu);
X_t = rk4_integrator(f_target, X_t0, dt, n_steps);

%% Closed Loop Simulation
X_hist = zeros(6, n_steps);   % state history
U_hist = zeros(3, n_steps);   % control history

X_hist(:, 1) = X0;

for k = 1:n_steps-1
    u = -K * X_hist(:, k);                          % LQR control law
    X_hist(:, k+1) = Ad * X_hist(:, k) + Bd * u;   
    U_hist(:, k)   = u;                             
end
U_hist(:, end) = -K * X_hist(:, end);               % last control input

%% Plots

figure('Color', [0.02 0.05 0.09], 'Position', [50 50 1200 700]);

% Colour scheme
c_pos = [0.00 0.78 1.00];   % cyan   — position states
c_vel = [1.00 0.55 0.00];   % orange — velocity states
c_u   = [0.22 1.00 0.08];   % green  — control inputs
ax_style = {'Color', [0.04 0.07 0.13], ...
            'XColor', [0.5 0.6 0.7], 'YColor', [0.5 0.6 0.7], ...
            'FontName', 'Courier New', 'FontSize', 7, 'GridColor', [0.2 0.3 0.4]};

t_min = t / 60;   % convert to minutes for readability

% ── Plot 1: Position states ───────────────────────────────────────────────
subplot(2, 3, 1);
set(gca, ax_style{:}); hold on; grid on;
plot(t_min, X_hist(1,:), 'Color', c_pos,         'LineWidth', 1.2, 'DisplayName', 'x (radial)');
plot(t_min, X_hist(2,:), 'Color', c_pos*0.7,     'LineWidth', 1.2, 'DisplayName', 'y (along-track)');
plot(t_min, X_hist(3,:), 'Color', c_pos.*[1 0.5 0.5], 'LineWidth', 1.2, 'DisplayName', 'z (cross-track)');
xlabel('Time [min]', 'Color', [0.7 0.8 0.9]);
ylabel('Position [m]', 'Color', [0.7 0.8 0.9]);
title('POSITION STATES', 'Color', c_pos, 'FontName', 'Courier New');
legend('TextColor', [0.7 0.8 0.9], 'Color', [0.04 0.07 0.13], 'EdgeColor', 'none', 'FontSize', 6);

% ── Plot 2: Velocity states ───────────────────────────────────────────────
subplot(2, 3, 2);
set(gca, ax_style{:}); hold on; grid on;
plot(t_min, X_hist(4,:), 'Color', c_vel,         'LineWidth', 1.2, 'DisplayName', 'x\_dot');
plot(t_min, X_hist(5,:), 'Color', c_vel*0.7,     'LineWidth', 1.2, 'DisplayName', 'y\_dot');
plot(t_min, X_hist(6,:), 'Color', c_vel.*[1 0.5 0.5], 'LineWidth', 1.2, 'DisplayName', 'z\_dot');
xlabel('Time [min]', 'Color', [0.7 0.8 0.9]);
ylabel('Velocity [m/s]', 'Color', [0.7 0.8 0.9]);
title('VELOCITY STATES', 'Color', c_vel, 'FontName', 'Courier New');
legend('TextColor', [0.7 0.8 0.9], 'Color', [0.04 0.07 0.13], 'EdgeColor', 'none', 'FontSize', 6);

% ── Plot 3: Relative distance ─────────────────────────────────────────────
subplot(2, 3, 3);
set(gca, ax_style{:}); hold on; grid on;
rel_dist = vecnorm(X_hist(1:3, :), 2, 1);
plot(t_min, rel_dist, 'Color', c_pos, 'LineWidth', 1.2);
xlabel('Time [min]', 'Color', [0.7 0.8 0.9]);
ylabel('Distance [m]', 'Color', [0.7 0.8 0.9]);
title('RELATIVE DISTANCE', 'Color', c_pos, 'FontName', 'Courier New');

% ── Plot 4: Control inputs ────────────────────────────────────────────────
subplot(2, 3, 4);
set(gca, ax_style{:}); hold on; grid on;
plot(t_min, U_hist(1,:)*1000, 'Color', c_u,         'LineWidth', 1.2, 'DisplayName', 'ux');
plot(t_min, U_hist(2,:)*1000, 'Color', c_u*0.7,     'LineWidth', 1.2, 'DisplayName', 'uy');
plot(t_min, U_hist(3,:)*1000, 'Color', c_u.*[1 0.5 0.5], 'LineWidth', 1.2, 'DisplayName', 'uz');
xlabel('Time [min]', 'Color', [0.7 0.8 0.9]);
ylabel('Acceleration [mm/s^2]', 'Color', [0.7 0.8 0.9]);
title('CONTROL INPUTS', 'Color', c_u, 'FontName', 'Courier New');
legend('TextColor', [0.7 0.8 0.9], 'Color', [0.04 0.07 0.13], 'EdgeColor', 'none', 'FontSize', 6);

% ── Plot 5: Hill frame trajectory (x vs y) ────────────────────────────────
subplot(2, 3, 5);
set(gca, ax_style{:}); hold on; grid on; axis equal;
plot(X_hist(2,:), X_hist(1,:), 'Color', c_pos, 'LineWidth', 1.2);
plot(X_hist(2,1), X_hist(1,1), 'o', 'Color', c_vel, 'MarkerFaceColor', c_vel, 'MarkerSize', 8);
plot(0, 0, '+', 'Color', c_pos, 'MarkerSize', 10, 'LineWidth', 2);
xlabel('Along-track y [m]', 'Color', [0.7 0.8 0.9]);
ylabel('Radial x [m]', 'Color', [0.7 0.8 0.9]);
title('HILL FRAME TRAJECTORY', 'Color', c_pos, 'FontName', 'Courier New');

% ── Plot 6: Cumulative delta-v ────────────────────────────────────────────
subplot(2, 3, 6);
set(gca, ax_style{:}); hold on; grid on;
dv = cumsum(vecnorm(U_hist, 2, 1) * dt);   % cumulative delta-v [m/s]
plot(t_min, dv, 'Color', c_u, 'LineWidth', 1.2);
xlabel('Time [min]', 'Color', [0.7 0.8 0.9]);
ylabel('\Delta v [m/s]', 'Color', [0.7 0.8 0.9]);
title('CUMULATIVE \Delta V', 'Color', c_u, 'FontName', 'Courier New');

sgtitle('LQR RENDEZVOUS — CWH MODEL', 'Color', [0 0.78 1], ...
        'FontName', 'Courier New', 'FontSize', 12);


%% JSON export

metadata = struct(...
    'a_km',           a / 1000, ...
    'e',              e, ...
    'i_deg',          rad2deg(i), ...
    'period_min',     T / 60, ...
    'altitude_km',    (a - R_earth) / 1000, ...
    'dt',             dt, ...
    'n_steps',        n_steps, ...
    'controller',     'LQR', ...
    'dynamics_model', 'CWH');

data = struct(...
    'metadata', metadata, ...
    't',        t, ...
    'X_t',      X_t', ...
    'Rho',      X_hist');

json_str = jsonencode(data, 'PrettyPrint', true);
fid = fopen('exports/scenarios/sim_lqr.json', 'w');
fprintf(fid, '%s', json_str);
fclose(fid);

fprintf('Simulation complete. JSON exported.\n');
fprintf('Total delta-v: %.4f m/s\n', dv(end));