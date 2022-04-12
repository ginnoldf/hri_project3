% Input: 
%   num_sampled_traj: number of sampled trajectories
%       double value
%   sigma： sample covariance matrix
%       2dim matrix of size (num_waypoints, num_waypoints)
%   theta: mean trajectory from last iteration
%       2dim matrix of size (num_joints, num_waypoints)
% Output:
%   theta_samples: sampled trajectories
%       3dim matrix with size (num_sampled_traj, num_joints, num_waypoints)
%   epsilon: sampled Gaussian noise for each joint
%       3dim matrix with size (num_sampled_traj, num_joints, num_waypoints)

function [theta_samples, epsilon] = stompSamples(num_sampled_traj, sigma, theta)
    
    % get dimensions
    [num_joints, num_waypoints] = size(theta);
    
    % sample noisy trajectories per trajectory, per joint
    % return noise and noisy trajectories
    epsilon = zeros(num_sampled_traj, num_joints, num_waypoints);
    theta_samples = zeros(num_sampled_traj, num_joints, num_waypoints);
    for traj_idx = 1:num_sampled_traj
        for joint_idx = 1:num_joints
            
            % epsilon is 0 for start and end position
            epsilon(traj_idx,joint_idx,2:end-1) = mvnrnd(zeros(1, num_waypoints-2), sigma, 1);

            % convert epsilon for joint to vector so we can add it to theta
            curr_epsilon_vector = squeeze(epsilon(traj_idx,joint_idx,:))';
            theta_samples(traj_idx,joint_idx,:) = theta(joint_idx,:) + curr_epsilon_vector;
        end
    end
end