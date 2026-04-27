function [K, P] = lqr_controller(Ad, Bd, Q, R)
% lqr_controller.m
% Computes the discrete-time LQR gain matrix K for the system:
%
%   X(k+1) = Ad * X(k) + Bd * u(k)
%
% The optimal control law is:
%
%   u(k) = -K * X(k)
%
% K is found by solving the Discrete Algebraic Riccati Equation (DARE):
%
%   P = Qd + Ad'*P*Ad - Ad'*P*Bd * inv(R + Bd'*P*Bd) * Bd'*P*Ad
%
% The resulting closed-loop system is:
%
%   X(k+1) = (Ad - Bd*K) * X(k)
%
%
% Input:
%   Ad - discrete state matrix (n x n)
%   Bd - discrete input matrix (n x m)
%   Q  - state cost matrix (n x n), symmetric positive semi-definite
%   R  - control cost matrix (m x m), symmetric positive definite
%
% Output:
%   K  - optimal gain matrix (m x n)
%   P  - solution to dare, cost-to-go matrix (n x n)
%
%References: 
%   Model Predicitve Control - Theroy, Computation and Design (S.20
%   from Rawlings, Diehl, Mayne)
%   IST, Konzepte der Regelungstechnik, Kapitel 3 (LQR)


[K, P, ~] = dlqr(Ad, Bd, Q, R);

end
