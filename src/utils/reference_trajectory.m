function [R_ref, phase_switch_idx] = reference_trajectory(x0, y0, z0, ...
                                      n_steps, dt, mode, x_end, y_end, z_end)
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
%   x_end, y_end, z_end - final hold point [m] (default: 0, 0, 0)
%
% Output:
%   R_ref           - reference trajectory [6 x n_steps]
%                     rows: [x; y; z; x_dot; y_dot; z_dot]
%   phase_switch_idx - timestep index where phase switches ([] if single phase)
%
%% Default hold point
if nargin < 7; x_end = 0; end
if nargin < 8; y_end = 0; end
if nargin < 9; z_end = 0; end

R_ref = zeros(6, n_steps);
phase_switch_idx = [];

switch mode

    case 'vbar'
        vy = (y_end - y0) / (n_steps * dt);   % constant approach velocity [m/s]

        for k = 1:n_steps
            t_k         = (k-1) * dt;
            R_ref(1, k) = x_end;                
            R_ref(2, k) = y0 + vy * t_k;   % y decreasing linearly
            R_ref(3, k) = z_end;                
            R_ref(4, k) = 0;                
            R_ref(5, k) = vy;               % y_dot = constant approach vel
            R_ref(6, k) = 0;                
        end

    case 'rbar'
        vx = (x_end - x0) / (n_steps * dt);   % constant approach velocity [m/s]

        for k = 1:n_steps
            t_k         = (k-1) * dt;
            R_ref(1, k) = x0 + vx * t_k;   % x decreasing linearly
            R_ref(2, k) = y_end;                
            R_ref(3, k) = z_end;                
            R_ref(4, k) = vx;               % x_dot = constant approach vel
            R_ref(5, k) = 0;               
            R_ref(6, k) = 0;               
        end

    case 'rbar_vbar'

        dist_rbar  = abs(x0 - x_end);
        dist_vbar  = abs(y0 - y_end);
        dist_total = dist_rbar + dist_vbar;

        n_phase1 = round(n_steps * dist_rbar / dist_total); % steps for R-bar phase
        n_phase2 = n_steps - n_phase1;   % steps for V-bar phase

        phase_switch_idx = n_phase1;

        vx_p1 = (x_end - x0) / (n_phase1 * dt);  % radial approach velocity [m/s]

        vy_p2 = (y_end - y0) / (n_phase2 * dt);  % along-track approach velocity [m/s]

        % R-bar
        for k = 1:n_phase1
            t_k         = (k-1) * dt;
            R_ref(1, k) = x0 + vx_p1 * t_k;   % x decreasing
            R_ref(2, k) = y0;                   % y constant
            R_ref(3, k) = z_end;
            R_ref(4, k) = vx_p1;               % x_dot = approach vel
            R_ref(5, k) = 0;
            R_ref(6, k) = 0;
        end

        % Phase 2: V-bar
        for k = 1:n_phase2
            t_k         = (k-1) * dt;
            R_ref(1, n_phase1 + k) = x_end;                    % x = 0
            R_ref(2, n_phase1 + k) = y0 + vy_p2 * t_k;    % y decreasing
            R_ref(3, n_phase1 + k) = z_end;
            R_ref(4, n_phase1 + k) = 0;
            R_ref(5, n_phase1 + k) = vy_p2;                % y_dot = approach vel
            R_ref(6, n_phase1 + k) = 0;
        end

    otherwise
        error('Unknown trajectory mode: %s. Use vbar, rbar, or rbar_vbar.', mode);

end

% Ensure final reference is exactly at hold point
R_ref(:, end) = [x_end; y_end; z_end; 0; 0; 0];

end