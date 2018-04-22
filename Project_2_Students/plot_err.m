function plot_err(P_truth, V_truth, A_truth, P_est, V_est, A_est)

P_err = P_truth - P_est;
V_err = V_truth - V_est;
A_err = A_truth - A_est;

std_dev = [ std(P_err(1,:)) std(P_err(2,:)) std(P_err(3,:)); ...
            std(V_err(1,:)) std(V_err(2,:)) std(V_err(3,:)); ...
            std(A_err(1,:)) std(A_err(2,:)) std(A_err(3,:))]
        
avg_err = [mean(P_err(1,:)) mean(P_err(2,:)) mean(P_err(3,:)); ...
           mean(V_err(1,:)) mean(V_err(2,:)) mean(V_err(3,:)); ...
           mean(A_err(1,:)) mean(A_err(2,:)) mean(A_err(3,:))]

end