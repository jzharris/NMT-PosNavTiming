function [r_e__e_b_GPS, v_e__e_b_GPS] = GPS(constants, r_e__e_b, v_e__e_b)

Fs = constants.Fs;

r_e__e_b_GPS = r_e__e_b + sqrt(constants.gps.position_sigma*Fs) * randn(3,1);
v_e__e_b_GPS = v_e__e_b + sqrt(constants.gps.velocity_sigma*Fs) * randn(3,1);

end