function [K, P] = lqr_controller(Ad, Bd, Q, R)
% lqr_controller.m
% DARE from Model Predicitve Control - Theroy, Computation and Design (S.20
% from Rawlings, Diehl, Mayne
%
%   X(k+1) = Ad * X(k) + Bd * u(k)
%   u(k)   = -K * X(k)
%
% DARE:
%   P = Q + Ad'*P*Ad - Ad'*P*Bd * inv(R + Bd'*P*Bd) * Bd'*P*Ad
%
% Input:
%   Ad - discrete state matrix (n x n)
%   Bd - discrete input matrix (n x m)
%   Q  - state cost matrix (n x n), symmetric positive semi-definite
%   R  - control cost matrix (m x m), symmetric positive definite
%
% Output:
%   K  - optimal gain matrix (m x n)
%   P  - converged cost-to-go matrix (n x n)
%
%References: 
%   Model Predicitve Control - Theroy, Computation and Design (S.20
%   from Rawlings, Diehl, Mayne)
%   IST, Konzepte der Regelungstechnik, Kapitel 3 (LQR)

max_iter = 10000;         
tol = 1e-10;

P = Q;              % initial guess with state cost matrix

for i = 1:max_iter
    P_prev = P;

    S = R + Bd' * P * Bd;           
    K = S \ (Bd' * P * Ad);         % optimal gain at this iteration
    P = Q + Ad' * P * Ad - Ad' * P * Bd * K;   % Riccati update
end
warning('DARE did not converge within %d iterations.', max_iter);

end
