% Input:
%   traj_probs: probability of states per trajectory, per waypoint
%       3dim matrix with size (num_sampled_traj, num_waypoints)
%   epsilon: sampled noise per trajectory, per waypoint, per joint
%       3dim matrix with size (num_sampled_traj, num_joints, num_waypoints)
% Output:
%   dtheta: delta theta per joint, per waypoint
%       2dim matrix with size (num_joints, num_waypoints)

function dtheta = stompDTheta(traj_probs, epsilon)

    % get dimensions
    [~, num_joints, num_waypoints] = size(epsilon);

    % iterate over all joints and waypoints and apply the formula from STOMP
    % algorithm step 3)
    dtheta = zeros(num_joints, num_waypoints);

    for joint_idx = 1:num_joints
        for waypoint_idx = 1:num_waypoints
            % elementwise multiplication
            dtheta(joint_idx, waypoint_idx) = traj_probs(:, waypoint_idx)' * epsilon(:, joint_idx, waypoint_idx);
        end

    end
