% Script/Function Description:
%   Script which defines simulation constants


%------------------------------------------------------------------------------
% Sampling rate variables and simulation time
%------------------------------------------------------------------------------
constants.Fs  = 100;                % Sample frequency (Hz)
constants.dt  = 1/constants.Fs;     % Sample interval (sec)
constants.t_start = 0;              % Simulation start time (sec)
constants.t_end = 360;              % Simulation end time (sec)
%------------------------------------------------------------------------------
% Earth model parameters
%------------------------------------------------------------------------------
constants.w_ie = 72.92115167e-6;    % WGS84 Earth rate (rad/s)
constants.mu = 3.986004418e14;      % Earth's gravitational constant (m^3/s^2)
constants.J2 = 1.082627e-3;         % Earth's second gravitational constant
constants.R0 = 6378137.0;           % Earth's equatorial radius (meters)
constants.Rp = 6356752.3142;        % Earth's polar radius (meters)
constants.e = 0.0818191908425;      % Eccentricity

disp(constants)