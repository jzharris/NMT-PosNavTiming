function [r_e__e_b] = llh2xyz(Constants, L_b, lambda_b, h_b)

R_E = Constants.R_0 / sqrt(1 - Constants.exp^2*sin(L_b)^2);

x_e__e_b = (R_E+h_b) * cos(L_b)*cos(lambda_b);
y_e__e_b = (R_E+h_b) * cos(L_b)*sin(lambda_b);
z_e__e_b = (R_E*(1 - Constants.exp^2)+h_b) * sin(L_b);

r_e__e_b = [x_e__e_b; y_e__e_b; z_e__e_b];