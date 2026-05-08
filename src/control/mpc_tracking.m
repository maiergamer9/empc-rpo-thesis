function [u_opt, U_opt, X_pred] = mpc_tracking(x0, r_current, Ad, Bd, Q, R, P, N, u_max)
% mpc_tracking.m
% Solves finite-horizon tracking MPC with input constraints at each time
% step. Basically as the Regulation MPC, but with a reference trajectory to
% track on the way to x=0.
%
%   min   Σᵢ₌₀ᴺ⁻¹ (eᵢ'Qeᵢ + uᵢ'Ruᵢ) + eₙ'Peₙ
%    U
%   s.t.  x(i+1) = Ad*x(i) + Bd*u(i)    (dynamics)
%         -u_max ≤ u(i) ≤ u_max          (thrust limits)
%         x(0)   = x0                    (initial condition)
%
% where eᵢ = xᵢ - rᵢ is the tracking error at step i.
%
% Input:
%   x0        - current state [6x1]
%   r_current - current reference state [6x1]
%   Ad        - discrete state matrix [6x6]
%   Bd        - discrete input matrix [6x3]
%   Q         - state cost matrix [6x6]
%   R         - control cost matrix [3x3]
%   P         - terminal cost matrix [6x6]
%   N         - prediction horizon 
%   u_max     - symmetric thrust acceleration limit [m/s^2]
%
% Output:
%   u_opt  - optimal first control input [3x1]
%   U_opt  - full optimal input sequence [3N x 1]
%   X_pred - predicted state trajectory [6 x N+1]

n_x = size(Ad, 1);   % 6 states
n_u = size(Bd, 2);   % 3 inputs

%% Error state
e0 = x0 - r_current; 

%% Prediction matrices Sx and Su (as before)
Sx = zeros(n_x * N, n_x);
Su = zeros(n_x * N, n_u * N);

Ad_pow = eye(n_x);
for i = 1:N
    Ad_pow = Ad_pow * Ad;
    Sx((i-1)*n_x+1 : i*n_x, :) = Ad_pow;
    for j = 1:i
        Su((i-1)*n_x+1 : i*n_x, (j-1)*n_u+1 : j*n_u) = ...
            Ad^(i-j) * Bd;
    end
end

%% Build block diagonal cost matrices
Qbar = blkdiag(kron(eye(N-1), Q), P);
Rbar = kron(eye(N), R);

%% QP matrices (not with error e0 instead of x0, like in regulation_mpc)
H = 2 * (Su' * Qbar * Su + Rbar);
f = 2 * Su' * Qbar * Sx * e0;   
H = (H + H') / 2;

%% Input constraints
lb = -u_max * ones(n_u * N, 1);
ub =  u_max * ones(n_u * N, 1);

%% Solve QP 
opts = optimoptions('quadprog', 'Display', 'off');
[U_opt, ~, exitflag] = quadprog(H, f, [], [], [], [], lb, ub, [], opts);

if exitflag ~= 1
    warning('MPC QP did not solve to optimality. exitflag = %d', exitflag);
end

%% First control input
u_opt = U_opt(1:n_u);

%% Predicted trajectory
X_pred = zeros(n_x, N+1);
X_pred(:, 1) = x0;
for i = 1:N
    u_i = U_opt((i-1)*n_u+1 : i*n_u);
    X_pred(:, i+1) = Ad * X_pred(:, i) + Bd * u_i;
end

end