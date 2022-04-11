function [] = plot_costs(Q_list, qo_list, qc_list, acc_cost_list, filename)
    
    % init
    num_sampled_traj = length(Q_list);

    % plot costs
    figure
    plot(1:num_sampled_traj, Q_list, 'DisplayName','Q'); hold on;
    plot(1:num_sampled_traj, qo_list, 'DisplayName','q_o'); hold on;
    plot(1:num_sampled_traj, qc_list, 'DisplayName','q_c'); hold on;
    plot(1:num_sampled_traj, acc_cost_list, 'DisplayName', '^{1}/_{2}\theta^TR\theta');
    xlabel("Iteration");
    ylabel("Cost");
    legend();
    saveas(gcf,filename);
end
