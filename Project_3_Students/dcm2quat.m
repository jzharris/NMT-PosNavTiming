function quat = dcm2quat (C)

q_s = sqrt(1 + C(1,1) + C(2,2) + C(3,3))/2;
q_x = (C(3,2) - C(2,3))/(4 * q_s);
q_y = (C(1,3) - C(3,1))/(4 * q_s);
q_z = (C(2,1) - C(1,2))/(4 * q_s);

quat = [q_s; q_x; q_y; q_z];