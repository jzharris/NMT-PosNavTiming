function [Phi, Q_d] = discrete_FQ (constants, F, G)

Phi = eye(15) + F * constants.dt;

% Q_d = constants.gyro.b_g_BI_sigma^2 * (1 - exp(-2 * tau_s / constants.gyro.BI.correlation_time));
% w_g_BI = sqrt(Q_d*Fs) * randn(3,1);
Q_d = 0.5 * (Phi * G * constants.Q_t * G' * Phi' + G * constants.Q_t * G') * constants.dt;

end