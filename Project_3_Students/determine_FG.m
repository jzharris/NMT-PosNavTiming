function [F, G] = determine_FG (constants, C_e__b_meas, f_b__i_b_meas, r_e__e_b_meas)

zer = zeros(3,3);
I3 = eye(3);

[L_b_meas, ~, ~] = xyz2llh(constants, r_e__e_b_meas);
g_0 = 9.7803253359 * (1 +  0.00193185*(sin(L_b_meas)^2)) / sqrt(1 - (constants.e^2)*(sin(L_b_meas)^2));
r_e__e_S = g_0;
delta_g_e__b = (2 * g_0 / r_e__e_S) * (r_e__e_b_meas / norm(r_e__e_b_meas)^2) * r_e__e_b_meas';

F_phi_g = constants.gyro.G_g;                       % F_phi_g = g-sensitivity
F_va = constants.accel.VRW;                         % Relates velocity to accel error
F_aa = - 1 / constants.accel.BI.correlation_time;   % b_a = (-1/T_c)*b + w_b
F_gg = - 1 / constants.gyro.BI.correlation_time;    % b_g = (-1/T_c)*b + w_b


F = [   -constants.Ohm_e__i_e               zer                     zer             zer                     -C_e__b_meas*F_phi_g;
        -vec2ss(C_e__b_meas*f_b__i_b_meas)  -2*constants.Ohm_e__i_e delta_g_e__b    -C_e__b_meas * F_va     zer;
        zer                                 I3                      zer             zer                     zer;
        zer                                 zer                     zer             F_aa                    zer;
        zer                                 zer                     zer             zer                     F_gg;                   ];
    
G = [   -C_e__b_meas    zer             zer     zer     zer;
        zer             -C_e__b_meas    zer     zer     zer;
        zer             zer             zer     zer     zer;
        zer             zer             zer     I3      zer;
        zer             zer             zer     zer     I3;     ];

end