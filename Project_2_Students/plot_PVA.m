function plot_PVA(constants, P_truth, V_truth, A_truth, P_est, V_est, A_est, mech)
%
% FUNCTION DESCRIPTION:
%   Plots PVA truth. If PVA estimates/measurements provides will also plot
%   same and PVA errors
%
% INPUTS:
%   P_truth = True position (meters)
%   V_truth = True velocity (meters/sec)
%   A_truth = True attitude/orientation as a rotation matrix
%   P_est   = Estimated/measured position (meters)
%   V_est   = Estimated/measured velocity (meters/sec)
%   A_est   = Estimated/measured attitude/orientation as a rotation matrix
%   mech    = String label of the mechanization case ('ECI', 'ECEF', or 'NAV')
%
% OUTPUTS:

switch mech     % Define labels for the plots
   case 'ECI'
      P_label = 'r^i_{ib}';
      V_label = 'v^i_{ib}';
      A_label = 'C^i_b';
   case 'ECEF'
      P_label = 'r^e_{eb}';
      V_label = 'v^e_{eb}';
      A_label = 'C^e_b';
   case 'NAV'
      P_label = 'L_b, \gamma_b, h_b';
      V_label = 'v^n_{eb}';
      A_label = 'C^n_b';
   otherwise
      error('Please choose ECI, ECEF, or NAV for the mechanization');
end

I3 = eye(3);        % 3X3 Indentiy matrix
t_sec = (constants.t_start:constants.dt:constants.t_end)';        % Simulation time vector
N = constants.N;            % Length of the simulation time vector

% Plot the position
if isempty(P_est)  % PVA Estimate/Measurement is NOT Provided
    figure;
    subplot(3,1,1)
    plot(t_sec, P_truth(1,:),'r')
    ylabel('r_x (m)')
    title([mech,' Mechanization Position ', P_label],'interpreter','tex');
    subplot(3,1,2)
    plot(t_sec, P_truth(2,:),'g')
    ylabel('r_y (m)')
    subplot(3,1,3)
    plot(t_sec, P_truth(3,:),'b')
    ylabel('r_z (m)')
    xlabel('Time (sec)')
else    % PVA Estimate/Measurement is Provided
    figure;
    subplot(3,1,1)
    plot(t_sec, P_truth(1,:),'r', t_sec, P_est(1,:),':k')
    ylabel('r_x (m)')
    legend('Truth','Est')
    title([mech,' Mechanization Position ', P_label],'interpreter','tex');
    subplot(3,1,2)
    plot(t_sec, P_truth(2,:),'g', t_sec, P_est(2,:),':k')
    ylabel('r_y (m)')
    legend('Truth','Est')
    subplot(3,1,3)
    plot(t_sec, P_truth(3,:),'b',t_sec, P_est(3,:),':k')
    ylabel('r_z (m)')
    legend('Truth','Est')
    xlabel('Time (sec)')
    
    figure;
    subplot(3,1,1)
    plot(t_sec, P_truth(1,:) - P_est(1,:),'r')
    ylabel('\deltar_x (m)')
    title([mech,' Mechanization Position Error \delta', P_label],'interpreter','tex');
    subplot(3,1,2)
    plot(t_sec, P_truth(2,:) - P_est(2,:),'g')
    ylabel('\deltar_y (m)')
    subplot(3,1,3)
    plot(t_sec, P_truth(3,:) - P_est(3,:),'b')
    ylabel('\deltar_z (m)')
    xlabel('Time (sec)')    
end

% Plot the velocity
if isempty(V_est)  % PVA Estimate/Measurement is NOT Provided
    figure;
    subplot(3,1,1)
    plot(t_sec, V_truth(1,:),'r')
    ylabel('v_x (m/s)')
    title([mech,' Mechanization Velocity ', V_label],'interpreter','tex');
    subplot(3,1,2)
    plot(t_sec, V_truth(2,:),'g')
    ylabel('v_y (m/s)')
    subplot(3,1,3)
    plot(t_sec, V_truth(3,:),'b')
    ylabel('v_z (m/s)')
    xlabel('Time (sec)')
else    % PVA Estimate/Measurement is Provided
    figure;
    subplot(3,1,1)
    plot(t_sec, V_truth(1,:),'r', t_sec, V_est(1,:),':k')
    ylabel('v_x (m/s)')
    legend('Truth','Est')
    title([mech,' Mechanization Velocity ', V_label],'interpreter','tex');
    subplot(3,1,2)
    plot(t_sec, V_truth(2,:),'g', t_sec, V_est(2,:),':k')
    ylabel('v_y (m/s)')
    legend('Truth','Est')
    subplot(3,1,3)
    plot(t_sec, V_truth(3,:),'b',t_sec, V_est(3,:),':k')
    ylabel('v_z (m/s)')
    legend('Truth','Est')
    xlabel('Time (sec)')
    
    figure;
    subplot(3,1,1)
    plot(t_sec, V_truth(1,:) - V_est(1,:),'r')
    ylabel('\deltav_x (m/s)')
    title([mech,' Mechanization Velocity Error \delta', V_label],'interpreter','tex');
    subplot(3,1,2)
    plot(t_sec, V_truth(2,:) - V_est(2,:),'g')
    ylabel('\deltav_y (m/s)')
    subplot(3,1,3)
    plot(t_sec, V_truth(3,:) - V_est(3,:),'b')
    ylabel('\deltav_z (m/s)')
    xlabel('Time (sec)')    
end

% Plot the attitude
if isempty(A_est)  % PVA Estimate/Measurement is NOT Provided
    k_vec = zeros(3,N);                     % Angle/axis representation of A_truth (i.e. C matrix)
    for i=1:N 
        k_vec(:,i)     = dcm2k(A_truth(:,:,i)); % Convert Rotation matrix to an Angle/axis k-vector
    end
    figure;
    subplot(3,1,1)
    plot(t_sec, k_vec(1,:),'r')
    ylabel('k_x')
    title([mech,' Mechanization Attitude ', A_label,' as an (Angle/Axis) k vector'],'interpreter','tex');
    subplot(3,1,2)
    plot(t_sec, k_vec(2,:),'g')
    ylabel('k_y')
    subplot(3,1,3)
    plot(t_sec, k_vec(3,:),'b')
    ylabel('k_z')
    xlabel('Time (sec)')
else    % PVA Estimate/Measurement is Provided
    k_vec = zeros(3,N);                         % Angle/axis representation of A_truth (i.e. C matrix)
    for i=1:N 
        k_vec(:,i)     = dcm2k(A_truth(:,:,i)); % Convert Rotation matrix to an Angle/axis k-vector
    end
    k_vec_est = zeros(3,N);                     % Angle/axis representation of A_est (i.e. C matrix)
    for i=1:N 
        k_vec_est(:,i) = dcm2k(A_est(:,:,i));   % Convert Rotation matrix to an Angle/axis k-vector
    end
    
    figure;
    subplot(3,1,1)
    plot(t_sec, k_vec(1,:),'r', t_sec, k_vec_est(1,:),':k')
    ylabel('k_x')
    legend('Truth','Est')
    title([mech,' Mechanization Attitude ', A_label,' as an (Angle/Axis) k vector'],'interpreter','tex');
    subplot(3,1,2)
    plot(t_sec, k_vec(2,:),'g', t_sec, k_vec_est(2,:),':k')
    ylabel('k_y')
    legend('Truth','Est')
    subplot(3,1,3)
    plot(t_sec, k_vec(3,:),'b',t_sec, k_vec_est(3,:),':k')
    ylabel('k_z')
    legend('Truth','Est')
    xlabel('Time (sec)')
    
    psi_ERR = zeros(3,N);                       % Initialize the attitude error vector
    for i=1:N                                   %                  T
        dC = A_truth(:,:,i) * A_est(:,:,i)' ;   % Delta_C = C C_est

        Psi_ERR_cross = dC - I3;                % The approx skew-symmetric matrix version of Psi_ERR
        psi_ERR(:,i)  = [Psi_ERR_cross(3,2);    % Row 3 Col 2 = x-component of vector
                         Psi_ERR_cross(1,3);    % Row 1 Col 3 = y-component of vector
                         Psi_ERR_cross(2,1)];   % Row 2 Col 1 = z-component of vector
    end

    figure;
    subplot(3,1,1)
    plot(t_sec, psi_ERR(1,:)*180/pi,'r')
    ylabel('\psi_x (°)')
    title([mech,' Mechanization Attitude Error \Delta', A_label,' as a \psi vector'],'interpreter','tex');
    subplot(3,1,2)
    plot(t_sec, psi_ERR(2,:)*180/pi,'g')
    ylabel('\psi_y (°)')
    subplot(3,1,3)
    plot(t_sec, psi_ERR(3,:)*180/pi,'b')
    ylabel('\psi_z (°)')
    xlabel('Time (sec)')    
end

end
