function [P_out, V_out, A_out] = ECI_mech(constants,P_in, V_in, A_in, w_b__i_b_tilde, f_b__i_b_tilde)
% FUNCTION DESCRIPTION:
%   Implements the low-fidelity ECEF Mechanization
%
% INPUTS:
%   constants = structure containing constants
%   P_in    = Past position r_i__i_b(-) (meters)
%   V_in    = Past velocity v_i__i_b(-) (meters/sec)
%   A_in    = Past attitude/orientation as a rotation matrix C_i__b(-)
%   w_b__i_b_tilde = Current Gyro measurements
%   f_b__i_b_tilde = Current Accel measurements
%
% OUTPUTS:
%   P_out   = Updated position r_i__i_b(+) (meters)
%   V_out   = Updated velocity v_i__i_b(+) (meters/sec)
%   A_out   = Updated attitude/orientation as a rotation matrix C_i__b(+)

dt = constants.dt;      % Time step

% ECEF Mechanization: One iteration of the mechanization

%--------------------------------------------------------------------------
% STEP 1.) Attitude Update
Ohm_b__i_b = vec2ss(w_b__i_b_tilde);

A_out = A_in*(eye(3) + (Ohm_b__i_b * dt));

q_i__b = dcm2quat(A_out);
q_i__b = q_i__b / norm(q_i__b);

A_out = quat2dcm(q_i__b);

%--------------------------------------------------------------------------
% STEP 2.) Specific Force Update
f_i__i_b = A_out * f_b__i_b_tilde;

%--------------------------------------------------------------------------
% STEP 3.) Velocity Update
a_i__i_b = f_i__i_b + gamma__i_b(constants, P_in);

V_out = V_in + (a_i__i_b * dt);

%--------------------------------------------------------------------------
% STEP 4.) Position Update
P_out = P_in + (V_in * dt) + (a_i__i_b * dt^2 / 2);

end