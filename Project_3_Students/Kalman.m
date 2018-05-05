function [x_kf_est, P] = Kalman(constants, x_kf_est, P, z, Phi, Q)

H = constants.H;
R = constants.R;
I = eye(15);

% Compute Kalman gain
K = (P * H.') / (H * P * H.' + R);

% Update estimate with measurement z
x_kf_est = x_kf_est + K * (z - H * x_kf_est);

% Update error covariance
P = (I - K * H) * P * (I - K * H).' + K * R * K.';

% Project ahead
x_kf_est = Phi * x_kf_est;
P = Q + (Phi * P * Phi.');

end