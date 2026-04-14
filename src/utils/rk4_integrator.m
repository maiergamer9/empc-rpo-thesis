function X = rk4_integrator(f, x0, dt, N)
% rk4_integrator.m
% Simulation of the continuous-time system using RK4 method.
%
%   x(k+1) = x(k) + (dt/6) * (k1 + 2*k2 + 2*k3 + k4)
%
% Input:
%   f   - function handle for dynamics
%   x0  - initial state
%   dt  - sampling time
%   N   - number of time steps
%
% Output:
%   X   - state trajectory, one state per column

nx = length(x0);
X = zeros(nx, N);
X(:, 1) = x0;

for k = 1:N-1
    xk = X(:, k);

    k1 = f(xk);
    k2 = f(xk + 0.5 * dt * k1);
    k3 = f(xk + 0.5 * dt * k2);
    k4 = f(xk + dt * k3);

    X(:, k+1) = xk + (dt / 6) * (k1 + 2*k2 + 2*k3 + k4);
end
end