% Script/Function Description:
%   Script which defines simulation constants


%------------------------------------------------------------------------------
% Sampling rate variables and simulation time
%------------------------------------------------------------------------------
constants.Fs  = 100;                % Sample frequency (Hz)
constants.dt  = 1/constants.Fs;     % Sample interval (sec)
constants.t_start = 0;              % Simulation start time (sec)
constants.t_end = 300;              % Simulation end time (sec)
%------------------------------------------------------------------------------
% Earth model parameters
%------------------------------------------------------------------------------
constants.w_ie = 72.92115167e-6;    % WGS84 Earth rate (rad/s)
constants.mu = 3.986004418e14;      % Earth's gravitational constant (m^3/s^2)
constants.J2 = 1.082627e-3;         % Earth's second gravitational constant
constants.R0 = 6378137.0;           % Earth's equatorial radius (meters)
constants.Rp = 6356752.3142;        % Earth's polar radius (meters)
constants.e = 0.0818191908425;      % Eccentricity
constants.f = 1 / 298.257223563;    % Flattening from page 38 of text

constants.Ohm_e__i_e = [    0 -1  0;
                            1  0  0;
                            0  0  0;    ] * constants.w_ie;

%--------------------------------------------------------------------------
% IMU error characterization values - KVH 1750 IMU
%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
% Gyro specific terms

% Bias terms
constants.gyro.b_g_FB = zeros(3, 1);      % Bias - Fixed Bias term (rad/s)
constants.gyro.b_g_BS_sigma = degtorad(1)/3600;  % Bias - Bias Stability Bias term 1-sigma (rad/s)
constants.gyro.b_g_BI_sigma = degtorad(0.1)/3600;  % Bias - Bias Instability Bias term 1-sigma (rad/s)
constants.gyro.b_g_BI_PSD = degtorad(0.1)^2 / 3600;
constants.gyro.BI.correlation_time = 3600; % Correlation time for the bias instability (sec)

% Noise terms
constants.gyro.ARW = 0.012;      % Gyro Angle Random Walk (deg/rt-hr)
constants.gyro.ARW_sigma = degtorad(constants.gyro.ARW) * sqrt(constants.Fs / 3600);
constants.gyro.ARW_PSD = degtorad(constants.gyro.ARW)^2 / 3600; % (rad/s)^2 / Hz

% Scale factor stability & misalignment terms
s_g_x = 50 * 1e-6;             % x-axis scale factor error (ppm * 1e-6)
s_g_y = 50 * 1e-6;             % y-axis scale factor error (ppm * 1e-6);
s_g_z = 50 * 1e-6;             % z-axis scale factor error (ppm * 1e-6);

m_g_xy =  1.0e-3;               % Misalignment of y-axis into x-axis (in rad)
m_g_xz =  0.5e-3;               % Misalignment of z-axis into x-axis
m_g_yx = -1.0e-3;               % Misalignment of x-axis into y-axis
m_g_yz =  0.2e-3;               % Misalignment of z-axis into y-axis
m_g_zx = -0.5e-3;               % Misalignment of x-axis into z-axis
m_g_zy =  0.7e-3;               % Misalignment of y-axis into z-axis

constants.gyro.M_g = ...
   [s_g_x , m_g_xy, m_g_xz; ... % The combined Misalignment / Scale Factor matrix (dimensionless)
    m_g_yx, s_g_y , m_g_yz; ...
    m_g_zx, m_g_zy, s_g_z ];
   
g_sens = degtorad(0.5)/9.81;  % Gyro G-sensitivity (rad/sec/g)

constants.gyro.G_g = ...        % The gyro G-sensitivity matrix (rad/sec/g)
   [g_sens , 0      , 0; ...    
    0      , g_sens , 0; ...
    0      , 0      , g_sens ];

%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
% Accelerometer specific terms

% Bias terms
constants.accel.b_a_FB = 0;                     % Bias - Fixed Bias term (m/s^2)
constants.accel.b_a_BS_sigma = 0;               % Bias - Bias Stability Bias term (m/s^2)
constants.accel.b_a_BI_sigma = 0.05E-3 * 9.81;  % Bias - Bias Instability Bias term 1-sigma (m/s^2)
constants.accel.b_a_BI_PSD = degtorad(0.05E-3 * 9.81)^2 / 3600;
constants.accel.BI.correlation_time = 3600;     % Correlation time for the bias instability (sec)

% Noise terms
constants.accel.VRW = 0.12E-3 * 9.81;           % Accel Angle Random Walk ((m/s^2)/rt-Hz)
constants.accel.VRW_sigma = constants.accel.VRW * sqrt(constants.Fs / 3600);
constants.accel.VRW_PSD = constants.accel.VRW^2 / 3600; % (m/s/s)^2 / Hz

% Scale factor stability & misalignment terms
s_a_x = 250 * 1e-6;             % x-axis scale factor error (ppm * 1e-6);
s_a_y = 250 * 1e-6;             % y-axis scale factor error (ppm * 1e-6);
s_a_z = 250 * 1e-6;             % z-axis scale factor error (ppm * 1e-6);

m_a_xy = -1.0e-3;               % Misalignment of y-axis into x-axis (in rad)
m_a_xz =  0.1e-3;               % Misalignment of z-axis into x-axis
m_a_yx =  0.5e-3;               % Misalignment of x-axis into y-axis
m_a_yz =  1.0e-3;               % Misalignment of z-axis into y-axis
m_a_zx =  0.5e-3;               % Misalignment of x-axis into z-axis
m_a_zy = -0.5e-3;               % Misalignment of y-axis into z-axis

constants.accel.M_a = ...
   [s_a_x , m_a_xy, m_a_xz; ... % The combined Misalignment / Scale Factor matrix (dimensionless)
    m_a_yx, s_a_y , m_a_yz; ...
    m_a_zx, m_a_zy, s_a_z ];

%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
% GPS specific terms

constants.gps.position_sigma = 3;       % (m)
constants.gps.velocity_sigma = 0.01;    % (m/s)
constants.gps.Fs = 1;                   % (Hz)
constants.gps.dt = 1/constants.gps.Fs;

disp(constants)
disp(constants.gyro)
disp(constants.accel)
disp(constants.gps)

%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
% Kalman Filter constants

zer = zeros(3,3);
I3 = eye(3);
        
% Initial Q: E{x * x'}
constants.Q = [ constants.gyro.ARW_PSD * I3     zer                         zer     zer                             zer;
                zer                             constants.accel.VRW_PSD*I3  zer     zer                             zer;
                zer                             zer                         zer     zer                             zer;
                zer                             zer                         zer     constants.accel.b_a_BI_PSD*I3   zer;
                zer                             zer                         zer     zer                             constants.gyro.b_g_BI_PSD*I3;   ];

% Initial R: E{z * z'}
constants.R = [ constants.gps.position_sigma^2 * I3     zer;
                zer                                     constants.gps.velocity_sigma^2 * I3;  ];

% Initial H: maps z to state error, x --> only take position and velocity terms
constants.H = [ zer zer I3  zer zer;
                zer I3  zer zer zer; ];

% Initial P: E{e * e'}
constants.P = [ 1e-9*I3 zer                                 zer                                 zer     zer;
                zer     constants.gps.velocity_sigma^2*I3   zer                                 zer     zer;
                zer     zer                                 constants.gps.position_sigma^2*I3   zer     zer;
                zer     zer                                 zer                                 1e-9*I3 zer;
                zer     zer                                 zer                                 zer     1e-9*I3;    ];


                