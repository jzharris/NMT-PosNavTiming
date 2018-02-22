function gamma_e__i_b = xyz2grav (Constants, r_e__e_b)

n = norm(r_e__e_b);
m = [ 1-5*(r_e__e_b(3)/n)^2*r_e__e_b(1);
      1-5*(r_e__e_b(3)/n)^2*r_e__e_b(2);
      3-5*(r_e__e_b(3)/n)^2*r_e__e_b(3);  ];

  gamma_e__i_b = - Constants.mu/n^3 * (r_e__e_b + 1.5*Constants.J_2*Constants.R_0^2/n^2*m);