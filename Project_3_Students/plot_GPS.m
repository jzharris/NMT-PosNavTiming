function plot_GPS(constants, G_truth, G_est)
%
% FUNCTION DESCRIPTION:
%   Plots GPS truth. If GPS estimates/measurements provides will also plot
%   same and GPS errors

P_label = 'r^e_{eb}';

I3 = eye(3);        % 3X3 Indentiy matrix
t_sec = (constants.t_start:constants.dt:constants.t_end)';        % Simulation time vector
N = constants.N;            % Length of the simulation time vector

% Plot the position
if isempty(G_est)  % PVA Estimate/Measurement is NOT Provided
    figure;
    subplot(3,1,1)
    plot(t_sec, G_truth(1,:),'r')
    ylabel('r_x (m)')
    title([' GPS Position ', P_label],'interpreter','tex');
    subplot(3,1,2)
    plot(t_sec, G_truth(2,:),'g')
    ylabel('r_y (m)')
    subplot(3,1,3)
    plot(t_sec, G_truth(3,:),'b')
    ylabel('r_z (m)')
    xlabel('Time (sec)')
else    % PVA Estimate/Measurement is Provided
    figure;
    subplot(3,1,1)
    plot(t_sec, G_truth(1,:),'r', t_sec, G_est(1,:),':k')
    ylabel('r_x (m)')
    legend('Truth','Est')
    title([' GPS Position ', P_label],'interpreter','tex');
    subplot(3,1,2)
    plot(t_sec, G_truth(2,:),'g', t_sec, G_est(2,:),':k')
    ylabel('r_y (m)')
    legend('Truth','Est')
    subplot(3,1,3)
    plot(t_sec, G_truth(3,:),'b',t_sec, G_est(3,:),':k')
    ylabel('r_z (m)')
    legend('Truth','Est')
    xlabel('Time (sec)')
    
    figure;
    subplot(3,1,1)
    plot(t_sec, G_truth(1,:) - G_est(1,:),'r')
    ylabel('\deltar_x (m)')
    title([' GPS Position Error \delta', P_label],'interpreter','tex');
    subplot(3,1,2)
    plot(t_sec, G_truth(2,:) - G_est(2,:),'g')
    ylabel('\deltar_y (m)')
    subplot(3,1,3)
    plot(t_sec, G_truth(3,:) - G_est(3,:),'b')
    ylabel('\deltar_z (m)')
    xlabel('Time (sec)')    
end

end
