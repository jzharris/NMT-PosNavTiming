function [L_b, lambda_b, h_b] = xyz2llh(Constants, r_e__e_b)

x_e__e_b = r_e__e_b(1);
y_e__e_b = r_e__e_b(2);
z_e__e_b = r_e__e_b(3);

sqrt_e = sqrt(1 - Constants.exp^2);
sqrt_xy = sqrt(x_e__e_b^2 + y_e__e_b^2);
e_R0 = Constants.exp^2*Constants.R_0;

gamma_b = atan2(z_e__e_b, sqrt_e*sqrt_xy);

L_b = atan2(z_e__e_b*sqrt_e + e_R0*sin(gamma_b)^3, sqrt_e * (sqrt_xy - e_R0*cos(gamma_b)^3));
R_E = Constants.R_0 / sqrt(1 - Constants.exp^2*sin(L_b)^2);

lambda_b = atan2(y_e__e_b, x_e__e_b);
h_b = sqrt_xy/cos(L_b) - R_E;