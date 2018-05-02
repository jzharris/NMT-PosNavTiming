function [Phi, Q_d] = discrete_FQ (constants, F, G)

zer = zeros(3,3);
I3 = eye(3);

%--------------------------------------------------------------------------
% Generate the new Phi matrix
%--------------------------------------------------------------------------

Phi = eye(15) + F * constants.gps.dt;

%--------------------------------------------------------------------------
% Generate the new Q matrix
%--------------------------------------------------------------------------

persistent tau_s
if isempty(tau_s)
    tau_s = 0;
end

Q_d_g = constants.gyro.b_g_BI_sigma^2 * (1 - exp(-2 * tau_s / constants.gyro.BI.correlation_time));
Q_d_a = constants.accel.b_a_BI_sigma^2 * (1 - exp(-2 * tau_s / constants.accel.BI.correlation_time));

Q_11 = (Q_d_g * constants.Fs)^2 * I3;
Q_22 = (Q_d_a * constants.Fs)^2 * I3;
Q_44 = constants.gyro.b_g_BI_PSD^2 * I3;
Q_55 = constants.accel.b_a_BI_PSD^2 * I3;

Q_t = [   Q_11    zer     zer     zer     zer;
          zer     Q_22    zer     zer     zer;
          zer     zer     zer     zer     zer;
          zer     zer     zer     Q_44    zer;
          zer     zer     zer     zer     Q_55;   ];

Q_d = 0.5 * (Phi * G * Q_t * G' * Phi' + G * Q_t * G') * constants.gps.dt;

% Time increment
%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
tau_s = tau_s + constants.gps.dt;

end