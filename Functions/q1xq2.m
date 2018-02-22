function q = q1xq2 (q1, q2)

q1_s = q1(1);
q1_x = q1(2);
q1_y = q1(3);
q1_z = q1(4);

q1X = [ q1_s -q1_x -q1_y -q1_z;
        q1_x  q1_s -q1_z  q1_y;
        q1_y  q1_z  q1_s -q1_x;
        q1_z -q1_y  q1_x  q1_s; ];
    
q = q1X * q2;