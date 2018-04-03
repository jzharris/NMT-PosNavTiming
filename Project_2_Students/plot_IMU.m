function plot_IMU(constants, w_b__i_b, f_b__i_b, w_b__i_b_tilde, f_b__i_b_tilde)
%
% FUNCTION DESCRIPTION:
%   Plots IMU measurements vs true angular velociyt and true specific force
%
% INPUTS:
%   w_b__i_b        = Ideal (error-free) Gyro measurements (rad/s)
%   f_b__i_b        = Ideal (error-free) Accel measurements (m/s^2)
%   w_b__i_b_tilde  = Gyro measurements (rad/s)
%   f_b__i_b_tilde  = Accel measurements (m/s^2)
%


t_sec = (constants.t_start:constants.dt:constants.t_end)';        % Simulation time vector

% Plot the angular velocity

w_b__i_b        = w_b__i_b       * 180/pi;  % Convert to deg/s
w_b__i_b_tilde  = w_b__i_b_tilde * 180/pi;  % Convert to deg/s

figure;
subplot(3,1,1)
plot(t_sec, w_b__i_b_tilde(1,:),'k.',t_sec, w_b__i_b(1,:),'r', 'MarkerSize',2)
ylabel('\omega_x (°/s)')
legend('Meas','Truth')
title(['Error free vs measured Angular velocity (\omega^b_{ib}) '],'interpreter','tex');
subplot(3,1,2)
plot(t_sec, w_b__i_b_tilde(2,:),'k.', t_sec, w_b__i_b(2,:),'g', 'MarkerSize',2)
ylabel('\omega_y (°/s)')
legend('Meas','Truth')
subplot(3,1,3)
plot(t_sec, w_b__i_b_tilde(3,:),'k.', t_sec, w_b__i_b(3,:),'b', 'MarkerSize',2)
ylabel('\omega_z (°/s)')
legend('Meas','Truth')
xlabel('Time (sec)')

figure;
subplot(3,1,1)
plot(t_sec, w_b__i_b(1,:) - w_b__i_b_tilde(1,:),'r.', 'MarkerSize',2)
ylabel('\delta\omega_x (°/s)')
title(['Error free - measured Angular velocity (\delta\omega^b_{ib}) '],'interpreter','tex');
subplot(3,1,2)
plot(t_sec, w_b__i_b(2,:) - w_b__i_b_tilde(2,:),'g.', 'MarkerSize',2)
ylabel('\delta\omega_y (°/s)')
subplot(3,1,3)
plot(t_sec, w_b__i_b(3,:) - w_b__i_b_tilde(3,:),'b.', 'MarkerSize',2)
ylabel('\delta\omega_z (°/s)')
xlabel('Time (sec)')

% Plot the specific force

figure;
subplot(3,1,1)
plot(t_sec, f_b__i_b_tilde(1,:),'k.', t_sec, f_b__i_b(1,:),'r', 'MarkerSize',2)
ylabel('a_x (m/s^2)')
legend('Meas','Truth')
title(['Error free vs measured Specific Force (f ^b_{ib}) '],'interpreter','tex');
subplot(3,1,2)
plot(t_sec, f_b__i_b_tilde(2,:),'k.', t_sec, f_b__i_b(2,:),'g', 'MarkerSize',2)
ylabel('a_y (m/s^2)')
legend('Meas','Truth')
subplot(3,1,3)
plot(t_sec, f_b__i_b_tilde(3,:),'k.', t_sec, f_b__i_b(3,:),'b', 'MarkerSize',2)
ylabel('a_z (m/s^2)')
legend('Meas','Truth')
xlabel('Time (sec)')

figure;
subplot(3,1,1)
plot(t_sec, f_b__i_b(1,:) - f_b__i_b_tilde(1,:),'r.', 'MarkerSize',2)
ylabel('\deltaa_x (m/s^2)')
title(['Error free - measured Specific Force (\deltaf ^b_{ib}) '],'interpreter','tex');
subplot(3,1,2)
plot(t_sec, f_b__i_b(2,:) - f_b__i_b_tilde(2,:),'g.', 'MarkerSize',2)
ylabel('\deltaa_y (m/s^2)')
subplot(3,1,3)
plot(t_sec, f_b__i_b(3,:) - f_b__i_b_tilde(3,:),'b.', 'MarkerSize',2)
ylabel('\deltaa_z (m/s^2)')
xlabel('Time (sec)')

end
