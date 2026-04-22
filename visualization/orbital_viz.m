function orbital_viz()
% orbital_viz.m — trajectory visualizer for thesis
% Imports a JSON file and replays the recorded ECI trajectory
% JSON format: { "metadata": {...}, "t": [N floats], "X": [Nx6 floats] }
% X columns: [rx, ry, rz, vx, vy, vz] in SI units (m, m/s)
% Reference: "Analytical Mechanics of Space Systems" - H.Schaub, J.L.Junkins

%% ── CONSTANTS ────────────────────────────────────────────────────────────
RE = 6378.137;      % Earth mean equatorial radius [km]

%% ── PLAYBACK STATE ───────────────────────────────────────────────────────
pb_t    = [];       % time vector [s], Nx1
pb_X    = [];       % state matrix [Nx6], SI units (m, m/s)
pb_idx  = 1;        % current frame index
pb_meta = [];       % metadata struct from JSON
running = false;    % is animation currently playing?

pb_tick_acc = 0;        % tick accumulator for real-time pacing

%% ── FIGURE ───────────────────────────────────────────────────────────────
fig = figure('Name', 'Trajectory Visualizer', ...
    'Color', [0.02 0.05 0.09], ...
    'Position', [100 100 1100 650], ...
    'NumberTitle', 'off');

%% ── 3D AXES ──────────────────────────────────────────────────────────────
ax3d = axes('Parent', fig, ...
    'Position', [0.03 0.12 0.68 0.82], ...
    'Color',    [0.02 0.05 0.09], ...
    'XColor',   [0.29 0.42 0.53], ...
    'YColor',   [0.29 0.42 0.53], ...
    'ZColor',   [0.29 0.42 0.53], ...
    'FontSize', 7, 'FontName', 'Courier New');
hold(ax3d, 'on'); grid(ax3d, 'on'); axis(ax3d, 'equal');
view(ax3d, 35, 25);
xlabel(ax3d, 'X (km)', 'FontSize', 7);
ylabel(ax3d, 'Y (km)', 'FontSize', 7);
zlabel(ax3d, 'Z (km)', 'FontSize', 7);
title(ax3d, 'ECI TRAJECTORY VISUALIZER', ...
    'Color', [0 0.78 1], 'FontName', 'Courier New', 'FontSize', 10);

%% ── EARTH ────────────────────────────────────────────────────────────────
[ue, ve] = meshgrid(linspace(0, 2*pi, 36), linspace(0, pi, 18));
mesh(ax3d, RE*sin(ve).*cos(ue), RE*sin(ve).*sin(ue), RE*cos(ve), ...
    'EdgeColor', [0.10 0.23 0.35], 'FaceColor', [0.04 0.08 0.14], ...
    'EdgeAlpha', 0.4, 'FaceAlpha', 0.6);

%% ── PLOT ARTISTS ─────────────────────────────────────────────────────────
% h_trail = full imported trajectory path (purple dashed)
h_trail = plot3(ax3d, nan, nan, nan, 'Color', [0.8 0.4 1], ...
                'LineWidth', 1.2, 'LineStyle', '--');
% h_sc    = spacecraft dot (orange)
h_sc    = plot3(ax3d, nan, nan, nan, 'o', 'Color', [1 0.55 0], ...
                'MarkerFaceColor', [1 0.55 0], 'MarkerSize', 8);
% h_rad   = radius vector from Earth center to spacecraft
h_rad   = plot3(ax3d, nan, nan, nan, 'Color', [0.13 0.20 0.27], 'LineWidth', 0.8);

%% ── TELEMETRY PANEL ──────────────────────────────────────────────────────
bg  = [0.02 0.05 0.09];
dim = [0.29 0.42 0.53];
blu = [0.00 0.78 1.00];

tnames = {'ALTITUDE', 'VELOCITY', 'ELAPSED TIME', 'PERIOD', 'ECCENTRICITY', 'PERIGEE ALT'};
tx = [0.74 0.74 0.74 0.88 0.88 0.88];
ty = [0.84 0.72 0.60 0.84 0.72 0.60];
h_tval = gobjects(1, 6);
for k = 1:6
    annotation(fig, 'textbox', [tx(k) ty(k)+0.04 0.12 0.04], ...
        'String', tnames{k}, 'Color', dim, 'FontName', 'Courier New', ...
        'FontSize', 7, 'EdgeColor', 'none', 'BackgroundColor', bg);
    h_tval(k) = annotation(fig, 'textbox', [tx(k) ty(k)-0.01 0.12 0.05], ...
        'String', '-', 'Color', blu, 'FontName', 'Courier New', ...
        'FontSize', 11, 'EdgeColor', 'none', 'BackgroundColor', bg);
end

% h_file_label = shows the loaded filename
h_file_label = annotation(fig, 'textbox', [0.74 0.91 0.24 0.05], ...
    'String', 'No file loaded', 'Color', dim, ...
    'FontName', 'Courier New', 'FontSize', 8, ...
    'EdgeColor', 'none', 'BackgroundColor', bg);

%% ── CONTROLS ─────────────────────────────────────────────────────────────
sc_bg = [0.02 0.05 0.09];

% btn_load  = open file browser to import JSON
btn_load = uicontrol(fig, 'Style', 'pushbutton', 'Units', 'normalized', ...
    'Position', [0.74 0.025 0.055 0.045], 'String', 'LOAD JSON', ...
    'BackgroundColor', [0.04 0.08 0.14], 'ForegroundColor', [0 0.78 1], ...
    'FontName', 'Courier New', 'FontSize', 8);

% btn_play  = start / resume playback
btn_play = uicontrol(fig, 'Style', 'pushbutton', 'Units', 'normalized', ...
    'Position', [0.80 0.025 0.055 0.045], 'String', 'PLAY', ...
    'BackgroundColor', [0.04 0.08 0.14], 'ForegroundColor', [0.22 1 0.08], ...
    'FontName', 'Courier New', 'FontSize', 8);

% btn_pause = pause playback
btn_pause = uicontrol(fig, 'Style', 'pushbutton', 'Units', 'normalized', ...
    'Position', [0.86 0.025 0.055 0.045], 'String', 'PAUSE', ...
    'BackgroundColor', [0.04 0.08 0.14], 'ForegroundColor', [1 0.55 0], ...
    'FontName', 'Courier New', 'FontSize', 8);

% btn_reset = rewind to first frame
btn_reset = uicontrol(fig, 'Style', 'pushbutton', 'Units', 'normalized', ...
    'Position', [0.92 0.025 0.055 0.045], 'String', 'RESET', ...
    'BackgroundColor', [0.04 0.08 0.14], 'ForegroundColor', [0.69 0.80 0.91], ...
    'FontName', 'Courier New', 'FontSize', 8);

% sl_spd  = playback speed slider (1x to 200x)
% vh_spd  = label showing current speed
uicontrol(fig, 'Style', 'text', 'Units', 'normalized', ...
    'Position', [0.74 0.110 0.05 0.025], 'String', 'Speed', ...
    'HorizontalAlignment', 'left', 'BackgroundColor', sc_bg, ...
    'ForegroundColor', [0.69 0.80 0.91], 'FontName', 'Courier New', 'FontSize', 8);
sl_spd = uicontrol(fig, 'Style', 'slider', 'Units', 'normalized', ...
    'Position', [0.74 0.085 0.20 0.022], ...
    'Min', 1, 'Max', 200, 'Value', 1, ...
    'BackgroundColor', [0.06 0.13 0.22]);
vh_spd = uicontrol(fig, 'Style', 'text', 'Units', 'normalized', ...
    'Position', [0.93 0.085 0.05 0.022], 'String', '1x', ...
    'HorizontalAlignment', 'left', 'BackgroundColor', sc_bg, ...
    'ForegroundColor', [0.22 1 0.08], 'FontName', 'Courier New', 'FontSize', 8);

set(sl_spd, 'Callback', @(~,~) update_speed());
    function update_speed()
        vh_spd.String = sprintf('%dx', round(sl_spd.Value));
    end

%% ── PLAYBACK UPDATE ──────────────────────────────────────────────────────
% pb_update: reads current frame from pb_X, moves spacecraft, updates telemetry

    function pb_update()
        if isempty(pb_X), return; end

        % loop back to start when end of data is reached
        if pb_idx > size(pb_X, 1)
            pb_idx = 1;
        end

        % current state row — convert SI (m, m/s) to km and km/s
        row     = pb_X(pb_idx, :);
        pos_km  = row(1:3) / 1000;      % position [km]
        vel_kms = row(4:6) / 1000;      % velocity [km/s]

        % update spacecraft position and radius vector
        set(h_sc,  'XData', pos_km(1), 'YData', pos_km(2), 'ZData', pos_km(3));
        set(h_rad, 'XData', [0 pos_km(1)], 'YData', [0 pos_km(2)], 'ZData', [0 pos_km(3)]);

        % telemetry
        sc_alt = norm(pos_km) - RE;     % altitude above surface [km]
        sc_spd = norm(vel_kms);         % speed [km/s]
        t_now  = pb_t(pb_idx);          % elapsed time [s]

        % pull metadata values if available
        if ~isempty(pb_meta) && isstruct(pb_meta)
            T_min  = pb_meta.period_min;
            ecc    = pb_meta.e;
            rp_alt = pb_meta.altitude_km;
        else
            T_min = nan; ecc = nan; rp_alt = nan;
        end

        vals = {sprintf('%.0f km',   sc_alt), ...
                sprintf('%.3f km/s', sc_spd), ...
                sprintf('%.1f s',    t_now),  ...
                sprintf('%.2f min',  T_min),  ...
                sprintf('%.4f',      ecc),    ...
                sprintf('%.0f km',   rp_alt)};
        for k = 1:6
            h_tval(k).String = vals{k};
        end

        % advance frame index based on real time
        sim_dt       = pb_t(pb_idx+1) - pb_t(pb_idx);
        ticks_needed = sim_dt / (0.05 * sl_spd.Value);
        pb_tick_acc  = pb_tick_acc + 1;
        if pb_tick_acc >= ticks_needed
            pb_idx      = pb_idx + max(1, round(pb_tick_acc / ticks_needed));
            pb_tick_acc = 0;
        end

        drawnow limitrate;
    end

%% ── JSON LOAD ────────────────────────────────────────────────────────────

    function load_json(src, evt)
        [fname, fpath] = uigetfile('*.json', 'Select trajectory JSON file');
        if isequal(fname, 0), return; end

        % read and decode JSON
        fid = fopen(fullfile(fpath, fname), 'r');
        raw = fread(fid, inf, 'uint8=>char')';
        fclose(fid);
        data = jsondecode(raw);

        % validate
        if ~isfield(data, 't') || ~isfield(data, 'X')
            errordlg('JSON must contain fields "t" and "X"', 'Invalid file');
            return;
        end

        % store data
        pb_t   = data.t;
        pb_X   = data.X;
        pb_idx = 1;
        pb_tick_acc = 0;
        pb_meta = [];
        if isfield(data, 'metadata')
            pb_meta = data.metadata;
        end

        % draw full trajectory path — convert all positions m -> km
        pos_all = pb_X(:, 1:3) / 1000;
        set(h_trail, ...
            'XData', pos_all(:,1)', ...
            'YData', pos_all(:,2)', ...
            'ZData', pos_all(:,3)');

        % fit axes to trajectory
        r_max = max(vecnorm(pos_all, 2, 2)) * 1.15;
        axis(ax3d, [-r_max r_max -r_max r_max -r_max r_max]);

        % update filename label
        h_file_label.String = fname;
        h_file_label.Color  = [0 0.78 1];

        % auto-start playback
        running = true;
    end

%% ── TIMER ────────────────────────────────────────────────────────────────

    function tick(src, evt)
        if running
            pb_update();
        end
    end

    function on_close(src, evt)
        stop(t); delete(t); delete(fig);
    end

%% ── BUTTON CALLBACKS ─────────────────────────────────────────────────────
set(btn_load,  'Callback', @load_json);
set(btn_play,  'Callback', @(~,~) set_running(true));
set(btn_pause, 'Callback', @(~,~) set_running(false));
set(btn_reset, 'Callback', @(~,~) do_reset());
set(fig,       'CloseRequestFcn', @on_close);

    function set_running(val)
        running = val;
    end

    function do_reset()
        pb_idx  = 1;
        pb_tick_acc = 0;
        running = false;
    end

%% ── START ────────────────────────────────────────────────────────────────
t = timer('ExecutionMode', 'fixedRate', 'Period', 0.05, 'TimerFcn', @tick);
start(t);

end