function [r_e__e_b_CAL, v_e__e_b_CAL, C_e__b_CAL] = calibrate_mech(x_kf_est, r_e__e_b_INS, v_e__e_b_INS, C_e__b_INS)

x_kf_est = -x_kf_est;

%--------------------------------------------------------------------------
% Calibrate C_e__b_INS
%--------------------------------------------------------------------------

R_x = [ 1   0                   0;
        0   cos(x_kf_est(1))    -sin(x_kf_est(1));
        0   sin(x_kf_est(1))    cos(x_kf_est(1));   ];
        
R_y = [ cos(x_kf_est(2))    0   sin(x_kf_est(2));
        0                   1   0;
        -sin(x_kf_est(2))   0   cos(x_kf_est(2));   ];

R_z = [ cos(x_kf_est(3))    -sin(x_kf_est(3))   0;
        sin(x_kf_est(3))    cos(x_kf_est(3))    0;
        0                   0                   1;  ];

C_e__b_CAL = R_z * R_y * R_x * C_e__b_INS;
q_e__b = dcm2quat(C_e__b_CAL);

% normalize if this rotation is closer to C_e__b_INS than original
diff1 = norm(C_e__b_CAL - C_e__b_INS);
diff2 = norm(quat2dcm(q_e__b / norm(q_e__b)) - C_e__b_INS);

if diff2 - diff1 < 1
    C_e__b_CAL = quat2dcm(q_e__b / norm(q_e__b));
end

% C_e__b_CAL = C_e__b_INS;

%--------------------------------------------------------------------------
% Calibrate v_e__e_b_INS
%--------------------------------------------------------------------------

v_e__e_b_CAL = v_e__e_b_INS - x_kf_est(4:6);

%--------------------------------------------------------------------------
% Calibrate r_e__e_b_INS
%--------------------------------------------------------------------------

r_e__e_b_CAL = r_e__e_b_INS - x_kf_est(7:9);

end