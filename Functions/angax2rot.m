function rot = angax2rot(angax)

theta = angax(1);
k_1 = angax(2);
k_2 = angax(3);
k_3 = angax(4);

V_theta = 1 - cos(theta);

rot = [ k_1*k_1*V_theta + cos(theta)        k_1*k_2*V_theta - k_3*sin(theta)    k_1*k_3*V_theta - k_2*sin(theta);
        k_1*k_2*V_theta + k_3*sin(theta)    k_2*k_2*V_theta + cos(theta)        k_2*k_3*V_theta - k_1*sin(theta);
        k_1*k_3*V_theta - k_2*sin(theta)    k_2*k_3*V_theta + k_1*sin(theta)    k_3*k_3*V_theta + cos(theta);       ];
