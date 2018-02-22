function [C_e__n] = llh2dcm(L_b, lambda_b)

C_e__n = [  -sin(lambda_b)              cos(lambda_b)               0;
            -cos(lambda_b)*sin(L_b-pi)  -sin(lambda_b)*sin(L_b-pi)  cos(L_b-pi);
            cos(lambda_b)*cos(L_b-pi)   sin(lambda_b)*cos(L_b-pi)   sin(L_b-pi);    ];
            