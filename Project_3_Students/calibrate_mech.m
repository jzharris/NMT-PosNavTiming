function [r_e__e_b_CAL, v_e__e_b_CAL, C_e__b_CAL] = calibrate_mech(x_kf_est, r_e__e_b_INS, v_e__e_b_INS, C_e__b_INS)

x_kf_est = -x_kf_est;

%--------------------------------------------------------------------------
% Calibrate C_e__b_INS
%--------------------------------------------------------------------------

C_e__b_CAL = (eye(3) + vec2ss(x_kf_est(1:3))) * C_e__b_INS;

% normalize if this rotation is closer to C_e__b_INS than original
q_e__b = dcm2quat(C_e__b_CAL);
diff1 = norm(C_e__b_CAL - C_e__b_INS);
diff2 = norm(quat2dcm(q_e__b / norm(q_e__b)) - C_e__b_INS);

if diff2 - diff1 < 1
    C_e__b_CAL = quat2dcm(q_e__b / norm(q_e__b));
end

%--------------------------------------------------------------------------
% Calibrate v_e__e_b_INS
%--------------------------------------------------------------------------

v_e__e_b_CAL = v_e__e_b_INS - x_kf_est(4:6);

%--------------------------------------------------------------------------
% Calibrate r_e__e_b_INS
%--------------------------------------------------------------------------

r_e__e_b_CAL = r_e__e_b_INS - x_kf_est(7:9);

end