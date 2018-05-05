function [w_b__i_b_tilde, f_b__i_b_tilde, b_g, b_a] = IMU(constants,w_b__i_b, f_b__i_b)
% FUNCTION DESCRIPTION:
%   Simulates an IMU
%
% INPUTS:
%   w_b__i_b        = Ideal (error-free) Gyro measurements (rad/s)
%   f_b__i_b        = Ideal (error-free) Accel measurements (m/s^2)
%
% OUTPUTS:
%   w_b__i_b_tilde  = Gyro measurements (rad/s)
%   f_b__i_b_tilde  = Accel measurements (m/s^2)
%
%

Fs = constants.Fs;

persistent tau_s
if isempty(tau_s)
    tau_s = 0;
end

%--------------------------------------------------------------------------
% Generate the IMU measurements
%--------------------------------------------------------------------------

% Gyro Model:
%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

% BS vars
persistent b_g_BS
if isempty(b_g_BS)
    b_g_BS = sqrt(constants.gyro.b_g_BS_sigma*Fs) * randn(3,1);             % Bias - Bias Stability Bias term changes every run (rad/s)
end

% Markov BI vars
persistent b_g_BI
if isempty(b_g_BI)
    b_g_BI = zeros(3,1);
end
persistent w_g_BI
if isempty(w_g_BI)
    w_g_BI = zeros(3,1);
end

% Bias terms
b_g_FB = constants.gyro.b_g_FB;                                             % Bias - Fixed Bias term never changes ever
b_g_BI = exp(-tau_s/constants.gyro.BI.correlation_time)*b_g_BI + w_g_BI;    % Bias - Markov Instability Bias term (rad/s)
b_g = b_g_BI + b_g_FB + b_g_BS;                                             % All three bias terms (rad/s)

% Update Markov BI terms
% Q_d = constants.gyro.b_g_BI_sigma^2 * (1 - exp(-2 * tau_s / constants.gyro.BI.correlation_time));
Q_d = 2 * constants.gyro.b_g_BI_sigma^2 / constants.gyro.BI.correlation_time;
w_g_BI = sqrt(Q_d*Fs) * randn(3,1);

% Noise - ARW term (rad/s)
w_g_ARW = sqrt(constants.gyro.ARW_PSD*Fs) * randn(3,1);
w_b__i_b_tilde = b_g + (eye(3) + constants.gyro.M_g)*w_b__i_b + ...
                 constants.gyro.G_g*f_b__i_b + w_g_ARW;

% Accelerometer Model
%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

% BS vars
persistent b_a_BS
if isempty(b_a_BS)
    b_a_BS = sqrt(constants.accel.b_a_BS_sigma*Fs) * randn(3,1);            % Bias - Bias Stability Bias term (m/s^2)
end

% Markov BI vars
persistent b_a_BI
if isempty(b_a_BI)
    b_a_BI = zeros(3,1);
end
persistent w_a_BI
if isempty(w_a_BI)
    w_a_BI = zeros(3,1);
end

% Bias terms
b_a_FB = constants.accel.b_a_FB;
b_a_BI = exp(-tau_s/constants.accel.BI.correlation_time)*b_a_BI + w_a_BI;    % Bias - Markov Instability Bias term (rad/s)
% b_a_BI = constants.accel.b_a_BI_sigma;  % Bias - Markov Bias Instability Bias term (m/s^2)
b_a = b_a_BI + b_a_FB + b_a_BS;         % All three bias terms (m/s^2)

% Update Markov BI terms
% Q_d = constants.accel.b_a_BI_sigma^2 * (1 - exp(-2 * tau_s / constants.accel.BI.correlation_time));
Q_d = 2 * constants.accel.b_a_BI_sigma^2 / constants.accel.BI.correlation_time;
w_a_BI = sqrt(Q_d*Fs) * randn(3,1);

% Noise - VRW term (m/s/s)
w_a_VRW = sqrt(constants.accel.VRW_PSD*Fs) * randn(3,1);
f_b__i_b_tilde = b_a + (eye(3) + constants.gyro.M_g)*f_b__i_b + w_a_VRW;


% Time increment
%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
tau_s = tau_s + constants.dt;

end
