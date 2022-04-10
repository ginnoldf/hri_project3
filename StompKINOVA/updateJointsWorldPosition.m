%% forward kinematics
% Input: 
%       robot_struct: the Matlab robot structure object
%       theta: the joints rotation angles
% Output:
%       joint_positions: Joints' positions in the world frame
%       T: Homogeneous Transformation from the Joint frame to the base frame
%
% In this sample code, we directly call the MATLAB built-in function 
% getTransform to calculate the forward kinemetics
function [joint_positions, T] = updateJointsWorldPosition(robot_struct, theta)

    % Update the robot configuration structure to use getTransform
    % Refer to: https://www.mathworks.com/matlabcentral/answers/31273-how-to-update-struct-array-fields-with-mutiple-values
    theta_cell = num2cell(theta);
    t_configuration = robot_struct.homeConfiguration;
    [t_configuration.JointPosition] = theta_cell{:};

    % iterate over joints and get their base transformations and world coordinates
    num_joints = size(theta,1);
    T = cell(1, num_joints);
    joint_positions = zeros(num_joints, 4); 
    for joint_idx = 1:num_joints

        % use getTransform
        bodyname = robot_struct.Bodies{1, joint_idx}.Name;
        T{joint_idx} = getTransform(robot_struct, t_configuration, bodyname);

        % Get joint's world coordinates
        joint_positions(joint_idx,:) = T{joint_idx}(:,4)';
        
    end
end
