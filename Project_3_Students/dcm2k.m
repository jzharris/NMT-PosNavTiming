function k = dcm2k(C)
% Function Description:
%   Converts a 3X3 Rotation matrix C to 3X1 angle-axis vector k
%
% INPUTS:
%   C = 3x3 rotation matrix
%
% OUTPUTS:
%   k = 3X1 angle-axis vector equivalent of the DCM
%     = [k1 k2 k3] Transpose = e_vector * sin(Theta/2)
%   where: The e_vector is the 3X1 unit axis of rotation and 
%         theta is the angle of rotation about e_vector
%
% NOTES:
%   - The algorithm can become ill-conditioned if theta becomes "too small"
%   - Using MATLAB defined eps = 2.2204e-016 as ~ zero

  Tr  = C(1,1)+C(2,2)+C(3,3);   % Trace(C) = 1 + 2Cos(theta)
  
  if(Tr >= (3-eps))             % degenerate case Tr=3 => theta = 0
        k  = [0; 0; 0];
  elseif (Tr <= (-1+eps))       % degenerate case Tr=-1 => theta = Pi
        [Mx,i] = max([C(1,1),C(2,2),C(3,3)]);  % Compute the maximum element of e
        switch i
            case 1  % if( Mx == C(1,1) )  % Use column 1
                e1 = sqrt((C(1,1) + 1.0) / 2.0);
                e2 = C(2,1)/(2*e1);  
                e3 = C(3,1)/(2*e1);
            case 2 % if ( Mx == C(2,2) )  % Use column 2
                e2 = sqrt((C(2,2) + 1.0) / 2.0);
                e1 = C(1,2)/(2*e2);  
                e3 = C(3,2)/(2*e2);
            case 3 % if ( Mx == C(3,3) )  % Use column 3
                e3 = sqrt((C(3,3) + 1.0) / 2.0);
                e1 = C(1,3)/(2*e3);  
                e2 = C(2,3)/(2*e3);
        end
        k  = pi* [e1; e2; e3];
  else    % Neither degenerate case: if -1 < trace(C) < 3
        theta = acos((Tr-1)/2);
        tmp   = 1/(2 * sin(theta));
        e1    = (C(3,2) - C(2,3)) * tmp;
        e2    = (C(1,3) - C(3,1)) * tmp;
        e3    = (C(2,1) - C(1,2)) * tmp;
  
        k     = theta * [e1; e2; e3];
  end