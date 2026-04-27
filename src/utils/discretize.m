function [Ad, Bd] = discretize(Ac, Bc, dt)
% discretize.,
% Discretizes a continuous linear state space model using ZOH.
%
% Cont: Xdot = Ac * X + Bc * u
%
% Discrete: X(k+1) = Ad * X(k) + Bd * u(k)
%
% Uses c2d with ZOH method, which is valid even when Ac
% is singular (as is the case for the CWH equations, which have
% two zero eigenvalues in the in-plane subsystem).
%
% Reference is Wikipedia, Discretization of linear state space models
%   Ad = expm(Ac * dt)
%   Bd = inv(Ac) * (Ad - I) * Bc
%
% Input:
%   Ac - continuous state matrix (nxn)
%   Bc - continuous input matrix (nxm)
%   dt - discretization timestep [s]
%
% Output:
%   Ad - discrete state matrix (nxn)
%   Bd - discrete input matrix (nxm)

sys_c = ss(Ac, Bc, eye(size(Ac)), zeros(size(Ac,1), size(Bc,2)));
sys_d = c2d(sys_c, dt, 'zoh');

Ad = sys_d.A;
Bd = sys_d.B;

end
    