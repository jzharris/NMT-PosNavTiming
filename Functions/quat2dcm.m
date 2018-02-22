function C = quat2dcm(quat)

q_s = quat(1);
q_x = quat(2);
q_y = quat(3);
q_z = quat(4);

C =   [ q_s^2+q_x^2-q_y^2-q_z^2     2*(q_x*q_y - q_s*q_z)   2*(q_x*q_z + q_s*q_y);
        2*(q_x*q_y + q_s*q_z)       q_s^2-q_x^2+q_y^2-q_z^2 2*(q_y*q_z - q_s*q_x);
        2*(q_x*q_z - q_s*q_y)       2*(q_y*q_z + q_s*q_x)   q_s^2-q_x^2-q_y^2+q_z^2;    ];