% Input: 
%   robot_struct: the robot structure
%       1x1 rigidBodyTree
%   theta: the trajectory for which the cost needs to be calculated
%       2dim matrix of size (num_joints, num_waypoints)
% Output:
%   cost: cost the given robot configuration
%       double value
%
function cost = stompConstraintCost(robot_struct, theta)

    % prepare getTransform
    theta_cell = num2cell(theta);
    t_configuration = robot_struct.homeConfiguration;
    [t_configuration.JointPosition] = theta_cell{:};

    % get base endeffector transformation
    base_endeffector_T = getTransform(robot_struct, t_configuration, 'EndEffector_Link');

    % get rotation matrix from base endeffector transformation
    rotation_mat = base_endeffector_T(1:3,1:3);

    % calculate rotatet y-axis
    endeffector_y_axis = rotation_mat * [0 1 0];

    % calculate y-axis angle to desired y-axis (0 1 0)
    % https://www.analyzemath.com/stepbystep_mathworksheets/vectors/vector3D_angle.html
    desired_y_axis = [0 1 0]';
    angle = dot(endeffector_y_axis, desired_y_axis) / ...
        (norm(endeffector_y_axis) * norm(endeffector_y_axis));

    % calculate cost for the angle
    cost = (1 - cos(angle)) / 2;
end
