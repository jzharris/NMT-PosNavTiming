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

persistent b_k
if isempty(b_k)
    b_k.gyro = zeros(3,1);
    b_k.accel = zeros(3,1);
end

persistent tau_s
if isempty(tau_s)
    tau_s = 0;
end

%--------------------------------------------------------------------------
% Generate the IMU measurements
%--------------------------------------------------------------------------

% Gyro Model:
%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

% Bias terms
b_g_FB = constants.gyro.b_g_FB;     % Bias - Fixed Bias term never changes ever
b_g_BS = constants.gyro.b_g_BS;     % Bias - Bias Stability Bias term changes every run (rad/s)
b_g_BI = constants.gyro.b_g_BI;     % Bias - Markov Bias Instability Bias term (rad/s)
b_g = b_g_BI + b_g_FB + b_g_BS;     % All three bias terms (rad/s)

% Noise terms
w_g = sqrt(contants.Fs * constants.gyro.Q) * randn(3,1);
b_k.gyro = e^(-tau_s/constants.gyro.BI.correlation_time)*b_k.gyro + w_g;

w_b__i_b_tilde =    b_g + (eye(3) + constants.gyro.M_g)*w_b__i_b + ...
                    constants.gyro.G_g*f_b__i_b + b_k.gyro;

% Accelerometer Model
%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

% Bias terms
b_a_FB = constants.accel.b_a_FB;
b_a_BS = constants.accel.b_a_BS;    % Bias - Bias Stability Bias term (m/s^2)
b_a_BI = constants.accel.b_a_BI;    % Bias - Markov Bias Instability Bias term (m/s^2)
b_a = b_a_BI + b_a_FB + b_a_BS;     % All three bias terms (m/s^2)

% Noise terms
w_a = sqrt(contants.Fs * constants.accel.Q) * randn(3,1);
b_k.accel = e^(-tau_s/constants.accel.BI.correlation_time)*b_k.accel + w_a;

f_b__i_b_tilde =    b_a + (eye(3) + constants.gyro.M_g)*f_b__i_b + b_k.accel;


% Time increment
%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
tau_s = tau_s + constants.dt;

end
