function g_n__bD = llh2grav (Constants, L_b, h_b)

g_0 = 9.7803253359 * (1 + 0.001931853*sin(L_b)^2)/sqrt(1-Constants.exp^2*sin(L_b)^2);

g_n__bD = g_0 * (1 - 2/Constants.R_0*(1 + Constants.f*(1 - 2*sin(L_b)^2) + Constants.w__i_e^2*Constants.R_0^2*Constants.R_p/Constants.mu)*h_b + 3*h_b^2/Constants.R_0^2);