% constants.m
% This file contains all physical and mission relevant constants
% For consistency reasons, alls values are in SI units unless otherwise
% noted

%% Earth Parameters
mu = 3.986004418e14;            % gravitational parameter mu = G*M [m^3/s^2]
R_earth = 6371000;              % Earth Radius [m]
omega_earth = 7.2921150e-5;     % Earth Rotation Rate [rad/s]

%% Mission Parameters (ISS values, for start)
a = 6771000;                    % Semi-major axis [m] (400km altitude of ISS)
e = 0;                          % eccentricity [-] (circular orbit, assumption for HCW)
i = deg2rad(51.6);              % Inclination [rad] (ISS)

%% Derived Parameters
T = 2*pi*sqrt(a^3/mu);          % Orbital Period [s]
alt = a - R_earth;              % Altitude above surface [m]

n = sqrt(mu/a^3);                % Mean motion [rad/s], for HCW 
