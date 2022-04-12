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
%   acc_cost: acceleration cost
%       double value
%   world_pos_time: time needed to calculate the joints world coordinates
%       double value
%   spheres_time: time needed to calculate the spheres in s
%       double value
%   qo_time: time needed to calculate the obstacle cost in s
%       double value
%
% Compute the local trajectory cost at each waypoint, as well as the 
% overall trajecotry cost Q
function [S, Q, qo, qc, acc_cost, world_pos_time, spheres_time, qo_time] = stompTrajCost(robot_struct, theta, R, voxel_world)

    % get number of waypoints
    [num_joints, num_waypoints] = size(theta);

    % iterate over waypoints to calculate the obstacle and constraint costs
    qo_list = zeros(1, num_waypoints);
    qc_list = zeros(1, num_waypoints);
    world_pos_time = 0;
    spheres_time = 0;
    qo_time = 0;
    [~, base_joint_T] = updateJointsWorldPosition(robot_struct, zeros(num_joints));
    for waypoint_idx = 1:num_waypoints

        % get the coordinates of joints in world frame
        world_pos_timer = tic;
        %[joint_positions, ~] = updateJointsWorldPosition(robot_struct, theta(:, waypoint_idx));
        [joint_positions, ~] = updateJointsWorldPositionPOE(robot_struct, theta(:, waypoint_idx), base_joint_T);
        world_pos_time = world_pos_time + toc(world_pos_timer);
        
        % get the spheres of joints in world frame
        spheres_timer = tic;
        [sphere_centers, radi] = stompRobotSphere(joint_positions);
        spheres_time = spheres_time + toc(spheres_timer);

        % initial velocity at the sphere centers around the manipulator is 0
        if waypoint_idx == 1
            vel = zeros(length(sphere_centers), 1);
        else
            vel = vecnorm(sphere_centers_prev - sphere_centers,2,2);
        end
        
        % calculate cost of the robots position and velocity for this waypoint
        qo_timer = tic;
        qo_list(waypoint_idx) = stompObstacleCost(sphere_centers, radi, voxel_world, vel);
        qo_time = qo_time + toc(qo_timer);
        
        % we need to save the sphere centers for the next iteration
        sphere_centers_prev = sphere_centers;

        % use constraint cost
        %qc_list(waypoint_idx) = stompConstraintCost(robot_struct, theta);
    end

    % local trajectory cost
    qo_factor = 1000;
    qc_factor = 1000;
    qo_list = qo_list * qo_factor;
    qc_list = qc_list * qc_factor;
    S = qo_list + qc_list;

    % sum over time and add the smoothness cost, do not consider start and end waypoint
    theta_reduced = theta(:, 2:end-1);
    qo = sum(qo_list);
    qc = sum(qc_list);
    acc_cost = 1/2 * sum(theta_reduced * R * theta_reduced', "all");
    Q = sum(S) + acc_cost;

end
