function [P_out, V_out, A_out] = Nav_mech(constants,P_in, V_in, A_in, w_b__i_b_tilde, f_b__i_b_tilde)
% FUNCTION DESCRIPTION:
%   Implements the low-fidelity ECEF Mechanization
%
% INPUTS:
%   constants = structure containing constants
%   P_in    = Past position r_n__e_b(-) (meters)
%   V_in    = Past velocity v_n__e_b(-) (meters/sec)
%   A_in    = Past attitude/orientation as a rotation matrix C_n__b(-)
%   w_b__i_b_tilde = Current Gyro measurements
%   f_b__i_b_tilde = Current Accel measurements
%
% OUTPUTS:
%   P_out   = Updated position r_n__e_b(+) (meters)
%   V_out   = Updated velocity v_n__e_b(+) (meters/sec)
%   A_out   = Updated attitude/orientation as a rotation matrix C_n__b(+)

dt = constants.dt;      % Time step
e = constants.e;
R0 = constants.R0;

% [L_b, lambda_b, h_b] = xyz2llh(constants, P_in);
L_b = P_in(1);
lambda_b = P_in(2);
h_b = P_in(3);

RE = R0 / sqrt(1 - (e^2*(sin(L_b)^2)));
RN = (1-e^2)*R0 / ((1 - (e^2*(sin(L_b)^2)))^1.5);

% ECEF Mechanization: One iteration of the mechanization

%--------------------------------------------------------------------------
% STEP 1.) Attitude Update

Ohm_b__i_b = vec2ss(w_b__i_b_tilde);

w_n__i_e = constants.w_ie * [cos(L_b); 0; -sin(L_b)];
Ohm_n__i_e = vec2ss(w_n__i_e);

w_n__e_n = [V_in(2)/(RE + h_b); -V_in(1)/(RN + h_b); -tan(L_b)*V_in(2)/(RE + h_b)];
Ohm_n__e_n = vec2ss(w_n__e_n);

A_out = (A_in*(eye(3) + (Ohm_b__i_b * dt))) - ((Ohm_n__i_e + Ohm_n__e_n) * A_in * dt);

% q_e__b = dcm2quat(A_out);
% A_out = quat2dcm(q_e__b / norm(q_e__b));

q_e__b = dcm2quat(A_out);

% normalize if this rotation is closer to A_in than original
diff1 = norm(A_out - A_in);
diff2 = norm(quat2dcm(q_e__b / norm(q_e__b)) - A_in);

if diff2 - diff1 < 1
    A_out = quat2dcm(q_e__b / norm(q_e__b));
end

%--------------------------------------------------------------------------
% STEP 2.) Specific Force Update
f_n__i_b = A_out * f_b__i_b_tilde;

%--------------------------------------------------------------------------
% STEP 3.) Velocity Update

% Orientation of the Nav frame of the ship wrt the ECEF frame
C_e__n = [ -cos(lambda_b)*sin(L_b), -sin(lambda_b) , -cos(lambda_b)*cos(L_b);
           -sin(lambda_b)*sin(L_b),  cos(lambda_b) , -sin(lambda_b)*cos(L_b);
           cos(L_b)               ,  0             , -sin(L_b)];

g_e__i_b = gamma__i_b(constants, P_in);
g_n__i_b = C_e__n' * g_e__i_b;
a_n__e_b = f_n__i_b + g_n__i_b - (Ohm_n__e_n + 2 * (C_e__n'*Ohm_n__i_e) * V_in);

V_out = V_in + (a_n__e_b * dt);

%--------------------------------------------------------------------------
% STEP 4.) Position Update
L_out = L_b + (dt*V_in(1)/(RN + h_b));
lambda_out = lambda_b + (dt*V_in(2)/(cos(L_b)*(RE + h_b)));
h_out = h_b - (dt*V_in(3));

P_out = [L_out; lambda_out; h_out];

end
