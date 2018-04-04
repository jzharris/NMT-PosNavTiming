function [w_b__i_b_tilde, f_b__i_b_tilde] = IMU(constants,w_b__i_b, f_b__i_b)
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
b_g_BS = sqrt(constants.gyro.b_g_BS_sigma*Fs) * randn(3,1);                 % Bias - Bias Stability Bias term changes every run (rad/s)
b_g_BI = exp(-tau_s/constants.gyro.BI.correlation_time)*b_g_BI + w_g_BI;    % Bias - Markov Instability Bias term (rad/s)
b_g = b_g_BI + b_g_FB + b_g_BS;                                             % All three bias terms (rad/s)

% Markov ARW vars
persistent b_g_ARW
if isempty(b_g_ARW)
    b_g_ARW = zeros(3,1);
end
persistent w_g_ARW
if isempty(w_g_ARW)
    w_g_ARW = zeros(3,1);
end

% Noise - Markov ARW term (rad/s)
b_g_ARW = exp(-tau_s/constants.gyro.BI.correlation_time)*b_g_ARW + w_g_ARW;
w_b__i_b_tilde = b_g + (eye(3) + constants.gyro.M_g)*w_b__i_b + ...
                 constants.gyro.G_g*f_b__i_b + b_g_ARW;
% W_g = b_g_ARW;
             
% Update Markov BI terms
Q_d = constants.gyro.b_g_BI_sigma^2 * (1 - exp(-2 * tau_s / constants.gyro.BI.correlation_time));
w_g_BI = sqrt(Q_d*Fs) * randn(3,1);

% Update Markov ARW terms
Q_d = constants.gyro.ARW_sigma^2 * (1 - exp(-2 * tau_s / constants.gyro.BI.correlation_time));
w_g_ARW = sqrt(Q_d*Fs) * randn(3,1);

% Accelerometer Model
%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

% Bias terms
b_a_FB = constants.accel.b_a_FB;
b_a_BS = constants.accel.b_a_BS;        % Bias - Bias Stability Bias term (m/s^2)
b_a_BI = constants.accel.b_a_BI_sigma;  % Bias - Markov Bias Instability Bias term (m/s^2)
b_a = b_a_BI + b_a_FB + b_a_BS;         % All three bias terms (m/s^2)

% Noise terms
% Q_d = b_a_BI^2 * (1 - exp(-2 * tau_s / constants.accel.BI.correlation_time));
% w_a = sqrt(Q_d) * randn(3,1);
% b_k.accel = exp(-tau_s/constants.accel.BI.correlation_time)*b_k.accel + w_a;

f_b__i_b_tilde = b_a + (eye(3) + constants.gyro.M_g)*f_b__i_b;% + b_k.accel;


% Time increment
%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
tau_s = tau_s + constants.dt;

end
