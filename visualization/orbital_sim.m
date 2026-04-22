function orbital_sim()
% orbital_sim.m defines the orbital mechanics of the 2-body problem and
% displays it in a GUI
% all of the math and physics are derived from the book "Analytical
% Mechanics of Space Systems" - H.Schaub, J.L.Junkins

% constants
MU = 398600.4418;           %gravitational constant [m/s^2]
RE = 6378.137;              %Earth Radius [km]
nu_sc = 0;                  % initial true anomaly, looking at periapsis

%% Figure 
fig = figure('Name', 'Orbital Mechanics', ...
    'Color', [0.02 0.05 0.09], ...
    'Position', [100 100 1200 650], ...
    'NumberTitle', 'off');

%% 3D Axes
ax3d = axes('Parent', fig, ...
    'Position', [0.03 0.22 0.58 0.72], ...
    'Color',    [0.02 0.05 0.09], ...
    'XColor',   [0.29 0.42 0.53], ...
    'YColor',   [0.29 0.42 0.53], ...
    'ZColor',   [0.29 0.42 0.53], ...
    'FontSize', 7, 'FontName', 'Courier New');
hold(ax3d, 'on'); grid(ax3d, 'on'); axis(ax3d, 'equal');
view(ax3d, 35, 25);
xlabel(ax3d, 'X (km)', 'FontSize', 7);                  % ECI x-axis (points toward vernal equinox)
ylabel(ax3d, 'Y (km)', 'FontSize', 7);                  % ECI y-axis (completes right-hand frame)
zlabel(ax3d, 'Z (km)', 'FontSize', 7);                  % ECI z-axis (points toward North Pole)
title(ax3d, 'ORBITAL MECHANICS - 2-BODY SIMULATOR', ...
    'Color', [0 0.78 1], 'FontName', 'Courier New', 'FontSize', 10);


%% Earth (visualizing it as a sphere)
% ue = longitude angles (0 to 2pi), ve = latitude angles (0 to pi)
[ue, ve] = meshgrid(linspace(0, 2*pi, 36), linspace(0, pi, 18));
mesh(ax3d, RE*sin(ve).*cos(ue), RE*sin(ve).*sin(ue), RE*cos(ve), ...
    'EdgeColor', [0.10 0.23 0.35], 'FaceColor', [0.04 0.08 0.14], ...
    'EdgeAlpha', 0.4, 'FaceAlpha', 0.6);


%% PLOT ARTISTS
% These are graphics handles. They reference to drawn objects we update each frame
% Initialised with NaN so nothing is drawn until do_update() fills them in
h_orbit = plot3(ax3d, nan, nan, nan, 'Color', [0 0.78 1], 'LineWidth', 1.0);            %handle to the full orbit path line
h_sc    = plot3(ax3d, nan, nan, nan, 'o', 'Color', [1 0.55 0], ...                      %handle to the spacecraft marker
                'MarkerFaceColor', [1 0.55 0], 'MarkerSize', 8);
h_rad   = plot3(ax3d, nan, nan, nan, 'Color', [0.13 0.20 0.27], 'LineWidth', 0.8);      %handle to the radius vector line


%% TELEMETRY  
% Colour definitions reused across telemetry elements
bg  = [0.02 0.05 0.09];
dim = [0.29 0.42 0.53];
blu = [0.00 0.78 1.00];

% Names shown above each telemetry readout
tnames = {'ALTITUDE','VELOCITY','PERIOD','TRUE ANOMALY','ECCENTRICITY','PERIGEE ALT'};
tx = [0.66 0.66 0.66 0.84 0.84 0.84];
ty = [0.84 0.72 0.60 0.84 0.72 0.60];

h_tval = gobjects(1, 6);

for k = 1:6
    % static label above (e.g. "ALTITUDE")
    annotation(fig, 'textbox', [tx(k) ty(k)+0.04 0.18 0.04], ...
        'String', tnames{k}, 'Color', dim, 'FontName', 'Courier New', ...
        'FontSize', 7, 'EdgeColor', 'none', 'BackgroundColor', bg);
    % live value below (e.g. "400 km"), updated every tick in do_update()    
    h_tval(k) = annotation(fig, 'textbox', [tx(k) ty(k)-0.01 0.18 0.05], ...
        'String', '-', 'Color', blu, 'FontName', 'Courier New', ...
        'FontSize', 12, 'EdgeColor', 'none', 'BackgroundColor', bg);
end

%% Sliders 
sc_bg = [0.02 0.05 0.09];

    % mkslider: helper to create a slider uicontrol
    % returns s = handle to the slider
    function s = mkslider(lft, bot, mn, mx, val)
        s = uicontrol(fig, 'Style', 'slider', 'Units', 'normalized', ...
            'Position', [lft+0.07 bot 0.30 0.025], ...
            'Min', mn, 'Max', mx, 'Value', val, ...
            'BackgroundColor', [0.06 0.13 0.22]);
    end

    % mklabel: helper to create a parameter label + editable value box pair
    % editable value box on the right — Style 'edit' means user can type in it
    function vh = mklabel(lft, bot, name)
        uicontrol(fig, 'Style', 'text', 'Units', 'normalized', ...
            'Position', [lft bot 0.07 0.025], 'String', name, ...
            'HorizontalAlignment', 'right', 'BackgroundColor', sc_bg, ...
            'ForegroundColor', [0.69 0.80 0.91], ...
            'FontName', 'Courier New', 'FontSize', 8);
        vh = uicontrol(fig, 'Style', 'edit', 'Units', 'normalized', ...
            'Position', [lft+0.37 bot 0.10 0.025], 'String', '', ...
            'HorizontalAlignment', 'left', 'BackgroundColor', [0.06 0.13 0.22], ...
            'ForegroundColor', [1 0.55 0], ...
            'FontName', 'Courier New', 'FontSize', 8);
    end


rows = [0.155, 0.120, 0.085, 0.050, 0.015]; % bottom positions for each slider row (top to bottom)
L    = 0.03;                %left margin for all sliders

 
% sl_* = slider handles, vh_* = value box handles
% a    = semi-major axis [km]         — orbit size
% e    = eccentricity [-]             — orbit shape (0=circle, 1=parabola)
% i    = inclination [deg]            — tilt of orbit plane vs equator
% w    = argument of perigee [deg]    — rotation of perigee within orbit plane
% raan = RAAN right ascension of ascending node [deg] — orbit plane orientation
sl_a    = mkslider(L, rows(1), 6578, 42164, 6778);
sl_e    = mkslider(L, rows(2), 0,    0.80,  0);
sl_i    = mkslider(L, rows(3), 0,    180,   0);
sl_w    = mkslider(L, rows(4), 0,    360,   0);
sl_raan = mkslider(L, rows(5), 0,    360,   0);
 
vh_a    = mklabel(L, rows(1), 'a (km)');
vh_e    = mklabel(L, rows(2), 'e');
vh_i    = mklabel(L, rows(3), 'i (deg)');
vh_w    = mklabel(L, rows(4), 'w (deg)');
vh_raan = mklabel(L, rows(5), 'RAAN (deg)');

% speed slider — controls animation speed multiplier (1x to 100x real time)
sl_spd = uicontrol(fig, 'Style', 'slider', 'Units', 'normalized', ...
    'Position', [0.73 0.155 0.18 0.025], ...
    'Min', 1, 'Max', 100, 'Value', 20, ...
    'BackgroundColor', [0.06 0.13 0.22]);
uicontrol(fig, 'Style', 'text', 'Units', 'normalized', ...
    'Position', [0.66 0.155 0.07 0.025], 'String', 'Speed', ...
    'HorizontalAlignment', 'right', 'BackgroundColor', sc_bg, ...
    'ForegroundColor', [0.69 0.80 0.91], 'FontName', 'Courier New', 'FontSize', 8);

    % handle to speed value display label
vh_spd = uicontrol(fig, 'Style', 'text', 'Units', 'normalized', ...
    'Position', [0.91 0.155 0.07 0.025], 'String', '20x', ...
    'HorizontalAlignment', 'left', 'BackgroundColor', sc_bg, ...
    'ForegroundColor', [0.22 1 0.08], 'FontName', 'Courier New', 'FontSize', 8);

% button for resetting everything
btn_reset = uicontrol(fig, 'Style', 'pushbutton', 'Units', 'normalized', ...
    'Position', [0.66 0.100 0.10 0.040], 'String', 'RESET', ...
    'BackgroundColor', [0.04 0.08 0.14], 'ForegroundColor', [1 0.55 0], ...
    'FontName', 'Courier New', 'FontSize', 8);

%% UPDATE
% do_update: redraws everything based on current slider values and nu_sc
% called by tick() every frame AND by slider/edit callbacks on user input

    function do_update()
        % read current orbital element values from sliders
        a    = sl_a.Value;
        ecc  = sl_e.Value;
        i    = sl_i.Value;
        w    = sl_w.Value;
        raan = sl_raan.Value;
        
        % update value box strings to match current slider positions
        vh_a.String    = sprintf('%.0f', a);
        vh_e.String    = sprintf('%.3f', ecc);
        vh_i.String    = sprintf('%.1f', i);
        vh_w.String    = sprintf('%.0f', w);
        vh_raan.String = sprintf('%.0f', raan);
        vh_spd.String  = sprintf('%.0fx', sl_spd.Value);
 
        % Orbit path
        % nu_path = 512 evenly spaced true anomaly values covering one full orbit
        nu_path = linspace(0, 2*pi, 512);
        [op, dummy1, dummy2] = sv(a, ecc, i, w, raan, nu_path);
        % update the orbit line with the new XYZ coordinates
        set(h_orbit, 'XData', op(1,:), 'YData', op(2,:), 'ZData', op(3,:));
        
        % nu_now = current true anomaly wrapped to [0, pi]
        nu_now = mod(nu_sc, 2*pi);
        % sp = spacecraft position [3x1], sv2 = velocity [3x1], sr = radius [km]

        [sp, sv2, sr] = sv(a, ecc, i, w, raan, nu_now);
        % move spacecraft dot to current position
        set(h_sc,  'XData', sp(1), 'YData', sp(2), 'ZData', sp(3));
        % updte radius vector line from origin [0,0,0] to spacecraft position
        set(h_rad, 'XData', [0 sp(1)], 'YData', [0 sp(2)], 'ZData', [0 sp(3)]);
 
        %apoapsis distance * 1.15 for padding
        r_max = a * (1 + ecc) * 1.15;
        axis(ax3d, [-r_max r_max -r_max r_max -r_max r_max]);
 
        % Telemetry 
        T      = 2 * pi * sqrt(a^3 / MU);   % orbital period [s]
        sc_alt = sr - RE;                   % spacecraft altitude above surface [km]
        sc_spd = norm(sv2);                 % spacecraft speed = magnitude of velocity [km/s]

        nu_deg = mod(rad2deg(nu_now), 360); % true anomaly in degrees [deg]
        rp_alt = a * (1 - ecc) - RE;        % perigee altitude above surface [km]
 
        vals = {sprintf('%.0f km',   sc_alt), ...
                sprintf('%.3f km/s', sc_spd), ...
                sprintf('%.1f min',  T/60),   ...
                sprintf('%.1f deg',  nu_deg), ...
                sprintf('%.3f',      ecc),    ...
                sprintf('%.0f km',   rp_alt)};
        for k = 1:6
            h_tval(k).String = vals{k};
        end  

        drawnow limitrate;
    end

%% TIMER
    
    % tick: called every 50ms by the timer to advance the animation
    function tick(src, evt)
        a     = sl_a.Value;
        T     = 2 * pi * sqrt(a^3 / MU);
        % (2pi/T) = angular velocity [rad/s]
        % * sl_spd.Value = speed multiplier
        % * 0.05 = timer interval 
        nu_sc = nu_sc + (2*pi / T) * sl_spd.Value * 0.05;
        do_update();
    end

    % when pressing the reset button, the values will be those below
    function do_reset(src, evt)
        sl_a.Value    = 6778;
        sl_e.Value    = 0;
        sl_i.Value    = 0;
        sl_w.Value    = 0;
        sl_raan.Value = 0;
        sl_spd.Value  = 20;
        nu_sc         = 0;
        do_update();
    end
 
     % on_close: clean up timer when window is closed
    % without this the timer keeps running invisibly after the figure closes
    function on_close(src, evt)
        stop(t); delete(t); delete(fig);
    end
 
set(sl_a,    'Callback', @(src,evt) do_update());
set(sl_e,    'Callback', @(src,evt) do_update());
set(sl_i,    'Callback', @(src,evt) do_update());
set(sl_w,    'Callback', @(src,evt) do_update());
set(sl_raan, 'Callback', @(src,evt) do_update());
set(sl_spd,  'Callback', @(src,evt) do_update());
set(btn_reset, 'Callback', @do_reset);
set(fig,     'CloseRequestFcn', @on_close);

    % apply_edit: called when user types a value into a vh edit box and presses Enter
    function apply_edit(vh, sl, fmt)
        val = str2double(vh.String);
        if isnan(val)
            vh.String = sprintf(fmt, sl.Value);
            return;
        end
        val = max(sl.Min, min(sl.Max, val));
        sl.Value = val;
        do_update();        % redraw
    end

% wire edit box Enter key to apply_edit for each orbital element
set(vh_a,    'Callback', @(~,~) apply_edit(vh_a,    sl_a,    '%.0f'));
set(vh_e,    'Callback', @(~,~) apply_edit(vh_e,    sl_e,    '%.3f'));
set(vh_i,    'Callback', @(~,~) apply_edit(vh_i,    sl_i,    '%.1f'));
set(vh_w,    'Callback', @(~,~) apply_edit(vh_w,    sl_w,    '%.0f'));
set(vh_raan, 'Callback', @(~,~) apply_edit(vh_raan, sl_raan, '%.0f'));


%% Math
% Mean anomaly to eccentric anomaly
% using Newton-Raphson iteration
    % M   = mean anomaly [rad]  — grows linearly with time
    % ecc = eccentricity [-]
    % E   = eccentric anomaly [rad] — intermediate angle for position calculation
function E = kepler(M, ecc)
        % initial guess: for low eccentricity E ~ M is good
        % for high eccentricity start at pi (closer to true solution)
        if ecc < 0.8
            E = M;
        else
            E = pi * ones(size(M));
        end
        for iter = 1:60
            % Newton-Raphson step: dE = -f(E)/f'(E)
            % f(E)  = E - ecc*sin(E) - M
            % f'(E) = 1 - ecc*cos(E)
            dE = (M - E + ecc.*sin(E)) ./ (1 - ecc.*cos(E));
            E  = E + dE;
            if all(abs(dE) < 1e-10), break; end     % converged
        end
    end
 
% Transform from 6 orbital elements in perifocal frame to state vector
    % returns:
    %   pos = position vector in ECI frame [km], 3xN
    %   vel = velocity vector in ECI frame [km/s], 3xN
    %   r   = scalar distance from Earth center [km], 1xN
    function [pos, vel, r] = sv(a, ecc, id, wd, rd, nu)
        p  = a * (1 - ecc^2);                                       % semi-latus rectum [km]

        r  = p ./ (1 + ecc * cos(nu));                              % orbital radius at true anomaly nu [km]
        s  = sqrt(MU / p);                                          % velocity scale factor [km/s]
        
        % position and velocity in perifocal frame (2D, orbit plane)
        pf = [r.*cos(nu); r.*sin(nu); zeros(1, numel(nu))];
        vf = [-s*sin(nu); s*(ecc + cos(nu)); zeros(1, numel(nu))];
        
        % precompute trig for rotation matrix
        cO = cos(deg2rad(rd)); sO = sin(deg2rad(rd));
        cI = cos(deg2rad(id)); sI = sin(deg2rad(id));
        cW = cos(deg2rad(wd)); sW = sin(deg2rad(wd));

        % 313 Euler rotation matrix: perifocal -> ECI
        % combines three rotations: RAAN around Z, inclination around X, omega around Z
        R  = [cO*cW-sO*sW*cI, -cO*sW-sO*cW*cI,  sO*sI; ...
              sO*cW+cO*sW*cI, -sO*sW+cO*cW*cI, -cO*sI; ...
              sW*sI,           cW*sI,            cI];

        % rotate perifocal vectors into ECI frame      
        pos = R * pf;
        vel = R * vf;
    end
 
%% START
% create and start the animation timer
% ExecutionMode fixedRate = fires every Period seconds regardless of execution time
% Period 0.05 = 50ms interval = ~20 frames per second
t = timer('ExecutionMode', 'fixedRate', 'Period', 0.05, 'TimerFcn', @tick);
do_update();
start(t);

end