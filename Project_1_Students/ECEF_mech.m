function [P_out, V_out, A_out] = ECEF_mech(constants,P_in, V_in, A_in, w_b__i_b_tilde, f_b__i_b_tilde)
% FUNCTION DESCRIPTION:
%   Implements the low-fidelity ECEF Mechanization
%
% INPUTS:
%   constants = structure containing constants
%   P_in    = Past position r_e__e_b(-) (meters)
%   V_in    = Past velocity v_e__e_b(-) (meters/sec)
%   A_in    = Past attitude/orientation as a rotation matrix C_e__b(-)
%   w_b__i_b_tilde = Current Gyro measurements
%   f_b__i_b_tilde = Current Accel measurements
%
% OUTPUTS:
%   P_out   = Updated position r_e__e_b(+) (meters)
%   V_out   = Updated velocity v_e__e_b(+) (meters/sec)
%   A_out   = Updated attitude/orientation as a rotation matrix C_e__b(+)

dt = constants.dt;      % Time step

% ECEF Mechanization: One iteration of the mechanization

%--------------------------------------------------------------------------
% STEP 1.) Attitude Update
Ohm_b__i_b = vec2ss(w_b__i_b_tilde);
Ohm_e__i_e = [0 -1  0;
              1  0  0;
              0  0  0;] * constants.w_ie;

A_out = A_in*(eye + Ohm_b__i_b * dt) - Ohm_e__i_e * A_in * dt

%--------------------------------------------------------------------------
% STEP 2.) Specific Force Update
f_e__i_b = A_out * f_b__i_b_tilde;

%--------------------------------------------------------------------------
% STEP 3.) Velocity Update
g_e__i_b = gamma__i_b(constants, P_in);
a_e__e_b = f_e__i_b + g_e__i_b - 2 * Ohm_e__i_e * V_in;

V_out = V_in + a_e__e_b * dt

%--------------------------------------------------------------------------
% STEP 4.) Position Update
P_out = P_in + V_in * dt + a_e__e_b * dt^2 / 2

end
