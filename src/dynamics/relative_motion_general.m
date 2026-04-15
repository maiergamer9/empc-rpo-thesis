function dRho = relative_motion_general(Rho, X_t, mu)
% relative_motion_general
% Computes relative EoMS in Hill frame for an elliptic target orbit.


x = Rho(1);
y = Rho(2);
z = Rho(3);
x_dot = Rho(4);
y_dot = Rho(5);
z_dot = Rho(6);

% Different to CW we now need the target kinematics defined in
% target_kinematics.m
[r_t, ~, theta_dot, theta_ddot] = target_kinematics(X_t);


% Equations of motion

x_ddot = 2*theta_dot*y_dot + theta_ddot*y + (theta_dot^2 + 2*mu/r_t^3)*x;
y_ddot = -2*theta_dot*x_dot - theta_ddot*x + (theta_dot^2 - mu/r_t^3)*y;
z_ddot = -(mu/r_t^3) * z;

dRho = [x_dot; y_dot; z_dot; x_ddot; y_ddot; z_ddot];

end
