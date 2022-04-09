% Input: 
%   traj_probs: probability of states per trajectory, per waypoint
%       3dim matrix with size (num_sampled_traj, num_waypoints)
%   epsilon: sampled noise per trajectory, per waypoint, per joint
%       3dim matrix with size (num_sampled_traj, num_waypoints, num_joints)
% Output:
%   dtheta: delta theta per joint, per waypoint
%       2dim matrix with size (num_joints, num_waypoints)

function dtheta = stompDTheta(traj_probs, epsilon)

% get dimensions
[~, num_waypoints, num_joints] = size(epsilon);

% iterate over all joints and waypoints and apply the formula from STOMP
% algorithm step 3)
dtheta = zeros(num_joints, num_waypoints);
for joint = 1:num_joints
    for waypoint = 1:num_waypoints
        % elementwise multiplication
        dtheta(joint, waypoint) = traj_probs(:,waypoint) .* epsilon(:, waypoint, joint);
    end
end
