function [R_ref, phase_switch_idx] = reference_trajectory(x0, y0, z0, n_steps, dt, mode)
% reference_trajectory
% Generates a reference trajectory for spacecraft rendezvous in the
% Hill frame (LVLH).
%
% AVAILABLE MODES:
%   'vbar'        — Pure V-bar: approach along along-track axis only
%   'rbar'        — Pure R-bar: approach along radial axis only
%   'rbar_vbar'   — Two-phase: R-bar first, then V-bar
%
% Input:
%   x0              - initial radial offset [m]
%   y0              - initial along-track offset [m]
%   z0              - initial cross-track offset [m]
%   n_steps         - total number of timesteps
%   dt              - timestep [s]
%   mode            - trajectory mode string (see above)
%
% Output:
%   R_ref           - reference trajectory [6 x n_steps]
%                     rows: [x; y; z; x_dot; y_dot; z_dot]
%   phase_switch_idx - timestep index where phase switches ([] if single phase)
%

R_ref = zeros(6, n_steps);
phase_switch_idx = [];

switch mode

    case 'vbar'
        vy = -y0 / (n_steps * dt);   % constant approach velocity [m/s]

        for k = 1:n_steps
            t_k         = (k-1) * dt;
            R_ref(1, k) = 0;                
            R_ref(2, k) = y0 + vy * t_k;   % y decreasing linearly
            R_ref(3, k) = 0;                
            R_ref(4, k) = 0;                
            R_ref(5, k) = vy;               % y_dot = constant approach vel
            R_ref(6, k) = 0;                
        end

    case 'rbar'
        vx = -x0 / (n_steps * dt);   % constant approach velocity [m/s]

        for k = 1:n_steps
            t_k         = (k-1) * dt;
            R_ref(1, k) = x0 + vx * t_k;   % x decreasing linearly
            R_ref(2, k) = 0;                
            R_ref(3, k) = 0;                
            R_ref(4, k) = vx;               % x_dot = constant approach vel
            R_ref(5, k) = 0;               
            R_ref(6, k) = 0;               
        end

    case 'rbar_vbar'

        n_phase1 = round(n_steps / 2);   % steps for R-bar phase
        n_phase2 = n_steps - n_phase1;   % steps for V-bar phase

        phase_switch_idx = n_phase1;

        vx_p1 = -x0 / (n_phase1 * dt);  % radial approach velocity [m/s]

        vy_p2 = -y0 / (n_phase2 * dt);  % along-track approach velocity [m/s]

        % R-bar
        for k = 1:n_phase1
            t_k         = (k-1) * dt;
            R_ref(1, k) = x0 + vx_p1 * t_k;   % x decreasing
            R_ref(2, k) = y0;                   % y constant
            R_ref(3, k) = 0;
            R_ref(4, k) = vx_p1;               % x_dot = approach vel
            R_ref(5, k) = 0;
            R_ref(6, k) = 0;
        end

        % Phase 2: V-bar
        for k = 1:n_phase2
            t_k         = (k-1) * dt;
            R_ref(1, n_phase1 + k) = 0;                    % x = 0
            R_ref(2, n_phase1 + k) = y0 + vy_p2 * t_k;    % y decreasing
            R_ref(3, n_phase1 + k) = 0;
            R_ref(4, n_phase1 + k) = 0;
            R_ref(5, n_phase1 + k) = vy_p2;                % y_dot = approach vel
            R_ref(6, n_phase1 + k) = 0;
        end

    otherwise
        error('Unknown trajectory mode: %s. Use vbar, rbar, or rbar_vbar.', mode);

end

% Ensure final reference is exactly zero (for the start, can be etended to
% nonzero setpoints)
R_ref(:, end) = zeros(6, 1);

end