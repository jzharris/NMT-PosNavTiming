function [F, G] = determine_FG (constants, C_e__b_meas, f_b__i_b_meas, r_e__e_b_meas)

zer = zeros(3,3);
I3 = eye(3);

% Quick access to constants
e = constants.e;
R0 = constants.R0;

% Determin L_b and R_E
[L_b_meas, ~, ~] = xyz2llh(constants, r_e__e_b_meas);
RE = R0 / sqrt(1 - (e^2*(sin(L_b_meas)^2)));

% Find g_0 and r_e__e_S for delta_g_e__b
g_0 = 9.7803253359 * (1 +  0.00193185*(sin(L_b_meas)^2)) / sqrt(1 - (e^2)*(sin(L_b_meas)^2));
r_e__e_S = RE * sqrt(cos(L_b_meas)^2 + (1 - e^2)^2*sin(L_b_meas)^2);

% Determine delta_g_e__b
delta_g_e__b = (2 * g_0 / r_e__e_S) * (r_e__e_b_meas / norm(r_e__e_b_meas)^2) * r_e__e_b_meas';

% Create the F and G matrices
F_phi_g = constants.gyro.ARW;                       % Relates angle to angular accel error
F_va = constants.accel.VRW;                         % Relates velocity to accel error
F_aa = - 1 / constants.accel.BI.correlation_time;   % b_a = (-1/T_c)*b + w_b
F_gg = - 1 / constants.gyro.BI.correlation_time;    % b_g = (-1/T_c)*b + w_b

F_11 = -constants.Ohm_e__i_e;
F_15 = -C_e__b_meas*F_phi_g;
F_21 = -vec2ss(C_e__b_meas*f_b__i_b_meas);
F_22 = -2*constants.Ohm_e__i_e;
F_23 = delta_g_e__b;
F_24 = -C_e__b_meas * F_va;
F_44 = F_aa * I3;
F_55 = F_gg * I3;
                                        % Units:
F = [   F_11 zer  zer  zer  F_15;       % [rad]
        F_21 F_22 F_23 F_24 zer;        % [m/s]
        zer  I3   zer  zer  zer;        % [m]
        zer  zer  zer  F_44 zer;        % [m/s^2]
        zer  zer  zer  zer  F_55;   ];  % [rad/s]
    
G_11 = -C_e__b_meas;
G_22 = -C_e__b_meas;
G_44 = I3;
G_55 = I3;
                                        % Units:
G = [   G_11 zer  zer  zer  zer;        % [rad]
        zer  G_22 zer  zer  zer;        % [m/s]
        zer  zer  zer  zer  zer;        % [m]
        zer  zer  zer  G_44 zer;        % [m/s^2]
        zer  zer  zer  zer  G_55;   ];  % [rad/s]

end