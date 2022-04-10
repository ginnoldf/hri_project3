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
% Given a trajectory, calculate its cost
function [S, Q] = stompTrajCost(robot_struct, theta, R, voxel_world)
    % Compute the local trajectory cost at each discretization theta point, as 
    % well as the overall trajecotry cost (the Qtheta)

    % Costi = stompCompute_Cost(robot, theta, Env);
    % Compute the cost of all discretized points on one trajectory
    [~, num_waypoints] = size(theta);
    % Obstacle cost 
    qo = zeros(1, num_waypoints);
    % Constraint costs
    qc = zeros(1, num_waypoints);

    % Get the coordinates of joints in World frame
    [X, ~] = updateJointsWorldPosition(robot_struct, theta(:, 1));
    % Construct the spheres around the robot manipulator for collision
    % avoidance
    [sphere_centers,radi] = stompRobotSphere(X);
    % Initial velocity at the sphere centers around the manipulator is 0
    vel = zeros(length(sphere_centers), 1);
    qo(1) = stompObstacleCost(sphere_centers, radi, voxel_world, vel);

    for waypoint_idx = 2 : num_waypoints
        sphere_centers_prev = sphere_centers;
        % Calculate the kinematics of the manipulator, given the
        % configuration theta values at different time (i=2:nDiscretize)
        [X, ~] = updateJointsWorldPosition(robot_struct, theta(:, waypoint_idx));
        [sphere_centers, radi] = stompRobotSphere(X);
        % xb: 3D workspace position of sphere b at the current time
        % Approximate the speed (xb_dot) using the finite difference of the current and
        % the previous position
        vel = vecnorm(sphere_centers_prev - sphere_centers,2,2);
        qo(waypoint_idx) = stompObstacleCost(sphere_centers, radi, voxel_world, vel);
        
    end

    % local trajectory cost
    S = 1000 * qo + qc;

    % sum over time and add the smoothness cost
    theta = theta(:, 2:end-1);
    Q = sum(S) + 1/2 * sum(theta * R * theta', "all");

end


    