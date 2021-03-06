%% Script/Function Description:
%   Main script which generates the probabilities and calls the IMU functions
%
% NOTES:
%   - Matrix(row, col, time)
%   - Vector(row, time)
%
% REFERENCES:
%   Navigation Systems textbook by P. Groves
%
clear all   % Remove all items from workspace - release system memory
close all   % Close all open figures (i.e. plots)
clc         % Clear the command window

load_constants2;                     % Load a file of constants

Fs = constants.Fs;                  % Sample frequency (Hz)
dt = constants.dt;                  % Sample interval (sec)

%==============================================================================
% Generate simulation time vector
%==============================================================================
t_sec = (constants.t_start:constants.dt:constants.t_end)';        % Simulation time vector
N = length(t_sec);                  % Length of the simulation time vector
constants.N = N;

%==============================================================================
% Generate the ground truth data
%==============================================================================
%% ECI to ECEF Coordinate Frame
%--------------------------------------------------------------------------
% Generate Angular Velocity
w_ie = constants.w_ie;              % WGS84 Earth rate (rad/s)
w_i__i_e = [0; 0; w_ie];            % Angular velocity of {e} wrt {i} resolved in {i} (rad/s)
Ohm_i__i_e = [ 0   , -w_ie, 0; ...  % Skew symmetric version of w_i__i_e (rad/s)
               w_ie,  0   , 0; ...
               0   ,  0   , 0];
constants.Ohm_i__i_e = Ohm_i__i_e;  % will need later

% Generate Orientation (i.e. Attitude)
C_i__e = zeros(3,3,N);              % Initialize to zero
theta_GMST = 0;                     % Set theta GMST to be zero
theta_e = w_ie * t_sec + theta_GMST;% Rotational angle theta_e = w_ie t + theta_GMST
for i=1:N
    C_i__e(:,:,i) = [ cos(theta_e(i)), -sin(theta_e(i)), 0;     % Eqn. 2.94
                      sin(theta_e(i)), cos(theta_e(i)) , 0;
                      0              , 0               , 1];
end
% Generate Position
r_i__i_e = [0; 0; 0];               % Position of the origin of {e} wrt origin of {i} resolved in {i} (m)

% Generate Velocity
v_i__i_e = [0; 0; 0];               % Velocity of e-frame wrt i-frame (m/s)

% Generate Acceleration
a_i__i_e = [0; 0; 0];               % Acceleration of e-frame wrt i-frame (m/s^2)

%==========================================================================
%% ECEF to Navigation
%--------------------------------------------------------------------------
% The curvlinear coordinates of the origin of the radar tracking frame
L_t       = (  30+0.01*cos(0.01*t_sec))*pi/180; % Geodetic Latitude  of radar tracking frame (rad)
lambda_t  = (-135+0.01*sin(0.01*t_sec))*pi/180; % Geodetic Longitude of radar tracking frame (rad)
h_t       =    15+   2*sin(0.2*pi  *t_sec);         % Geodetic height    of radar tracking frame (m)

% Generate Position - Need to determine the ECEF position of the radar + offset from radar to aircraft
r_e__e_t = zeros(3,N);
for i=1:N
    r_e__e_t(:,i) = llh2xyz(constants, L_t(i), lambda_t(i), h_t(i));         % Position of the origin of the radar tracking frame wrt e-frame org resolved in the e-frame (m)
end
r_t__t_b = [ 10000 * cos(0.03 * t_sec');...      % Position of the origin of {b} wrt {t} org resolved in {t} (m)
             10000 * sin(0.03 * t_sec');...
             -5000 - 5 * t_sec'];
         
% Orientation of the Nav frame of the ship at the start wrt the ECEF frame
C_e__n_ship_start = [ -cos(lambda_t(1))*sin(L_t(1)), -sin(lambda_t(1)) , -cos(lambda_t(1))*cos(L_t(1));
                      -sin(lambda_t(1))*sin(L_t(1)),  cos(lambda_t(1)) , -sin(lambda_t(1))*cos(L_t(1));
                       cos(L_t(1))                 ,  0                , -sin(L_t(1))];
C_e__t = C_e__n_ship_start;                     % The orientation of the tracker frame stays constant wrt ECEF

r_e__e_b = r_e__e_t + C_e__t * r_t__t_b;        % Position of the origin of the body frame wrt t-frame org resolved in the t-frame (m)
r_e__e_n = r_e__e_b;                            % The origin of the body frame = origin of the nav frame (m)

% The curvlinear coordinates of the Navigation/Body frame
L_b      = zeros(1,N);  % Initialize the arrays
lambda_b = zeros(1,N);
h_b      = zeros(1,N);
for i=1:N
    [L_b(i), lambda_b(i), h_b(i)] = xyz2llh(constants,r_e__e_b(:,i));    % Geodetic Lat (rad), Lon (rad), height(m) of Body/Nav frame
end

% The (numeric) differential of curvlinear coordinates of the Navigation/Body frame
L_b_dot      = diff(L_b)/dt;                    % Geodetic Latitude  rate of Body/Nav frame (rad/s)
L_b_dot      = [L_b_dot, L_b_dot(end)];
lambda_b_dot = diff(lambda_b)/dt;               % Geodetic Longitude rate of Body/Nav frame (rad/s)
lambda_b_dot = [lambda_b_dot, lambda_b_dot(end)];
h_b_dot      = diff(h_b)/dt;                    % Geodetic height    rate of Body/Nav frame (m/s)
h_b_dot      = [h_b_dot, h_b_dot(end)];

% Generate Angular Velocity
w_e__e_n = zeros(3,N);                          % Angular velocity of {n} wrt {e} resolved in {e}
w_i__i_n = zeros(3,N);                          % Angular velocity of {n} wrt {i} resolved in {i}
for i=1:N
    w_e__e_n(:,i) = [ sin(lambda_b(i)) * L_b_dot(i); ...  % See handoun#2 eqn. (12)
                     -cos(lambda_b(i)) * L_b_dot(i); ...
                                        lambda_b_dot(i)];
    w_i__i_n(:,i) =   w_i__i_e +  C_i__e(:,:,i) *  w_e__e_n(:,i);
end
% Generate Orientation
C_e__n = zeros(3,3,N);              % Orientation of n-frame wrt e-frame
C_i__n = zeros(3,3,N);              % Orientation of n-frame wrt i-frame
for i=1:N                           % Groves Eqn 2.99
    C_e__n(:,:,i) = [ -cos(lambda_b(i))*sin(L_b(i)), -sin(lambda_b(i)) , -cos(lambda_b(i))*cos(L_b(i));
                      -sin(lambda_b(i))*sin(L_b(i)),  cos(lambda_b(i)) , -sin(lambda_b(i))*cos(L_b(i));
                       cos(L_b(i))                 ,  0                , -sin(L_b(i))];
    C_i__n(:,:,i) = C_i__e(:,:,i) * C_e__n(:,:,i);
end
% Generate Position 
r_i__i_n = zeros(3,N);                      % Position of n-frame wrt i-frame resolved in {i}
for i=1:N
    r_i__i_n(:,i) = C_i__e(:,:,i) * r_e__e_n(:,i); % r_i__i_n = r_i__i_e + C_i__e * r_e__e_n -> Eqn 2.96
end
% Generate Velocity
v_e__e_n = (r_e__e_n(:,2:end) - r_e__e_n(:,1:end-1))/dt;  % Velocity (numerically) of n-frame wrt e-frame in the e-frame
v_e__e_n = [v_e__e_n, v_e__e_n(:,end)];
v_i__i_n = zeros(3,N);                      % Velocity of n-frame wrt i-frame in the i-frame
for i=1:N
    v_i__i_n(:,i) = C_i__e(:,:,i) * (v_e__e_n(:,i) + Ohm_i__i_e * r_e__e_n(:,i) ); % Eqn 2.96
end
% Generate Acceleration
a_e__e_n = (v_e__e_n(:,2:end) - v_e__e_n(:,1:end-1))/dt;  % Acceleration (numerically) of n-frame wrt e-frame in the e-frame
a_e__e_n = [a_e__e_n, a_e__e_n(:,end)];
a_i__i_n = zeros(3,N);                      % Acceleration of n-frame wrt i-frame in the i-frame
for i=1:N                                   % Eqn 2.97
    a_i__i_n(:,i) = C_i__e(:,:,i)*(a_e__e_n(:,i) + 2*Ohm_i__i_e*v_e__e_n(:,i) + Ohm_i__i_e*Ohm_i__i_e*r_e__e_n(:,i)); % Eqn 2.96
end

%% Navigation to Body frame
%--------------------------------------------------------------------------
a=10000; b=5; w=0.03;     % Some constants for this example
den = sqrt(b^2 + a^2*w^2);
% Generate Orientation
C_i__b = zeros(3,3,N);              % Orientation of b-frame wrt i-frame
C_e__b = zeros(3,3,N);              % Orientation of b-frame wrt e-frame
C_n__b = zeros(3,3,N);              % Orientation of b-frame wrt n-frame

for i=1:N                           % From Problem #4 (with y-axis inverted)
    C_n__b(:,:,i) = [ -a*w*sin(t_sec(i)*w)/den, -cos(t_sec(i)*w) , -b*sin(t_sec(i)*w)/den;
                       a*w*cos(t_sec(i)*w)/den, -sin(t_sec(i)*w) ,  b*cos(t_sec(i)*w)/den;
                      -b                  /den,  0               ,  a*w              /den];
    C_i__b(:,:,i) = C_i__n(:,:,i) * C_n__b(:,:,i);
    C_e__b(:,:,i) = C_e__n(:,:,i) * C_n__b(:,:,i);
end
% Generate Angular Velocity
w_n__n_b = zeros(3,N);              % Angular velocity of b-frame wrt n-frame in the n-frame
w_i__i_b = zeros(3,N);              % Angular velocity of b-frame wrt i-frame in i-frame
w_e__e_b = zeros(3,N);              % Angular velocity of b-frame wrt i-frame in i-frame

for i=1:N
    w_n__n_b(:,i) = [ 0; ...
                      0; ...
                      w];
    w_i__i_b(:,i) = w_i__i_n(:,i) + C_i__n(:,:,i) * w_n__n_b(:,i);
    w_e__e_b(:,i) = w_e__e_n(:,i) + C_e__n(:,:,i) * w_n__n_b(:,i);
end
% Assuming that the Navigation and body frames have the same origins
% Generate Position
r_n__n_b = [0; 0; 0];              % Position of b-frame wrt n-frame

% Generate velocity
v_n__n_b = [0; 0; 0];              % Velocity of b-frame wrt n-frame

% Generate acceleration
a_n__n_b = [0; 0; 0];              % Acceleration of b-frame wrt n-frame

%==========================================================================
%% Generate the IMU measurements (Ground Truth - Error Free)
%--------------------------------------------------------------------------
%  Error free (i.e. ground truth) IMU measurements measurements
w_b__i_b = zeros(3,N);              % Angular velocity of b-frame wrt i-frame resolved in i-frame
f_b__i_b = zeros(3,N);              % Specific force   of b-frame wrt i-frame resolved in i-frame

for i=1:N
    w_b__i_b(:,i) =   C_i__b(:,:,i)' * w_i__i_b(:,i);   % Gyro measurements (ground truth)
    % Now compute the specific force vector
    r_i__i_b = r_i__i_n(:,i);                   % Nav and body frame have same origin
    gamma_i__i_b = gamma__i_b(constants, r_i__i_b);        % Compute gravitational attraction
    f_i__i_b = a_i__i_n(:,i) - gamma_i__i_b;    % NOTE: a_i__i_b = a_i__i_n
    f_b__i_b(:,i) = C_i__b(:,:,i)' * f_i__i_b;  % Eqn 2.76
end

%==========================================================================
%% Generate the IMU measurements (With Error)
%--------------------------------------------------------------------------
%  With-Error IMU measurements measurements
w_b__i_b_tilde = zeros(3,N);  % Angular velocity of b-frame wrt i-frame resolved in i-frame
f_b__i_b_tilde = zeros(3,N);  % Specific force   of b-frame wrt i-frame resolved in i-frame

for i=1:N
    [w_b__i_b_tilde(:,i), f_b__i_b_tilde(:,i)] = IMU(constants, w_b__i_b(:,i), f_b__i_b(:,i));
end

% plot_IMU(constants, w_b__i_b, f_b__i_b, w_b__i_b_tilde, f_b__i_b_tilde)

%==========================================================================
%% ECEF Mechanization (Ground Truth - Error Free)
%==========================================================================
% r_e__e_b_INS = zeros(3,N);
% v_e__e_b_INS = zeros(3,N);
% C_e__b_INS = zeros(3,3,N);
% 
% % Initialize the INS mechanization (Use ground truth)
% C_e__b_INS(:,:,1) = C_e__b(:,:,1);      % No errors in the initialization
% v_e__e_b_INS(:,1) = v_e__e_n(:,1);      % Remember: the origin of b-frame = origin of n-frame
% r_e__e_b_INS(:,1) = r_e__e_b(:,1);
% 
% for i=2:N  % Call the mechanization at each iteration: PVA(+) = ECEF_mech(PVA(-), w, f)
%     [r_e__e_b_INS(:,i)  , v_e__e_b_INS(:,i)  , C_e__b_INS(:,:,i)] = ECEF_mech(constants, ...
%      r_e__e_b_INS(:,i-1), v_e__e_b_INS(:,i-1), C_e__b_INS(:,:,i-1), ...
%      w_b__i_b(:,i), f_b__i_b(:,i), 'High'); % Error free IMU
% end
% 
% % Plot the ECEF PVA Ground truth, INS derived PVA, & Error betw the two
% plot_PVA(constants, r_e__e_n, v_e__e_n, C_e__b, r_e__e_b_INS, v_e__e_b_INS, C_e__b_INS, 'ECEF')
% plot_err(r_e__e_n, v_e__e_n, C_e__b, r_e__e_b_INS, v_e__e_b_INS, C_e__b_INS)

%==========================================================================
%% ECEF Mechanization (With Error)
%==========================================================================
r_e__e_b_INS = zeros(3,N);
v_e__e_b_INS = zeros(3,N);
C_e__b_INS = zeros(3,3,N);

% Initialize the INS mechanization (Use ground truth)
C_e__b_INS(:,:,1) = C_e__b(:,:,1);      % No errors in the initialization
v_e__e_b_INS(:,1) = v_e__e_n(:,1);      % Remember: the origin of b-frame = origin of n-frame
r_e__e_b_INS(:,1) = r_e__e_b(:,1);

for i=2:N  % Call the mechanization at each iteration: PVA(+) = ECEF_mech(PVA(-), w, f)
    [r_e__e_b_INS(:,i)  , v_e__e_b_INS(:,i)  , C_e__b_INS(:,:,i)] = ECEF_mech(constants, ...
     r_e__e_b_INS(:,i-1), v_e__e_b_INS(:,i-1), C_e__b_INS(:,:,i-1), ...
     w_b__i_b_tilde(:,i), f_b__i_b_tilde(:,i), 'High'); % Error free IMU
end

% Plot the ECEF PVA Ground truth, INS derived PVA, & Error betw the two
plot_PVA(constants, r_e__e_n, v_e__e_n, C_e__b, r_e__e_b_INS, v_e__e_b_INS, C_e__b_INS, 'ECEF')
plot_err(r_e__e_n, v_e__e_n, C_e__b, r_e__e_b_INS, v_e__e_b_INS, C_e__b_INS)