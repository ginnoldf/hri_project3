% Input: 
%   robot_struct: the robot structure
%       1x1 rigidBodyTree
%   theta: the trajectory for which the cost needs to be calculated
%       2dim matrix of size (num_joints, num_waypoints)
%   R: R matrix for calculating the smoothness term
%       2dim matrix of size(num_waypoints-2, num_waypoints-2)
%   voxel_world: voxel world 
%       struct
% Output:
%   S: local cost of the waypoints of a trajectory
%       2dim matrix of size (1, num_waypoints)
%   Q: cost of the trajectory
%       double value
%
% Compute the local trajectory cost at each waypoint, as well as the 
% overall trajecotry cost Q
function [S, Q, acc_cost] = stompTrajCost(robot_struct, theta, R, voxel_world)

    % get number of waypoints
    [~, num_waypoints] = size(theta);

    % iterate over waypoints to calculate the obstacle and constraint costs
    qo = zeros(1, num_waypoints);
    qc = zeros(1, num_waypoints);
    for waypoint_idx = 1:num_waypoints

        % get the coordinates and spheres of joints in World frame
        [joint_positions, ~] = updateJointsWorldPosition(robot_struct, theta(:, waypoint_idx));
        [sphere_centers, radi] = stompRobotSphere(joint_positions);

        % initial velocity at the sphere centers around the manipulator is 0
        if waypoint_idx == 1
            vel = zeros(length(sphere_centers), 1);
        else
            vel = vecnorm(sphere_centers_prev - sphere_centers,2,2);
        end
        
        % calculate cost of the robots position and velocity for this waypoint
        qo(waypoint_idx) = stompObstacleCost(sphere_centers, radi, voxel_world, vel);
        
        % we need to save the sphere centers for the next iteration
        sphere_centers_prev = sphere_centers;
    end

    % local trajectory cost
    S = 1000 * qo + qc;

    % sum over time and add the smoothness cost, do not consider start and end waypoint
    theta_reduced = theta(:, 2:end-1);
    acc_cost = 1/2 * sum(theta_reduced * R * theta_reduced', "all");
    Q = sum(S) + acc_cost;

end


    