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
    % Sample theta (joints angles) trajectory 
    % 
    % [nJoints, nDiscretize] = size(theta);
    % 
    % em = cell(1,nJoints);
    % ek = cell(1,nSamplePaths);
    % 
    % theta_paths = cell(1, nSamplePaths);
    % mu=zeros(1,length(sigma));
    % 
    % 
    % for m = 1 : nJoints
    %     % Each joint is sampled independently The starting q0 and final qT are
    %     % fixed, so set the sample to 0 sample from multivariable Gaussian
    %     % distribution
    %    
    % end
    % 
    % % regroup it by samples
    % emk = [em{:}];
    % for k=1:nSamplePaths
    %     ek{k} = reshape(emk(k,:),nDiscretize, nJoints)';
    %     theta_paths{k} = theta + ek{k};
    %     
    % end
    
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