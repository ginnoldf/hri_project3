%% forward kinematics
% Input: 
%       robot_struct: the Matlab robot structure object
%       theta: the joints rotation angles
%           vector of length num_joints
% Output:
%       joint_positions: Joints' positions in the world frame
%           matrix of size (num_joints, 4)
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

    % calculate transformation and position for first joint
    T{1} = robot_struct.Bodies{1, 1}.Joint.JointToParentTransform;
    joint_positions(1,:) = T{1}(:,4)';
    
    % iterate over joints from second joint and use previous transformation
    for joint_idx = 2:num_joints

        % get joint to parent transform and calcualte final transform
        joint_to_parent_T = robot_struct.Bodies{1, joint_idx}.Joint.JointToParentTransform;
        base_to_parent_T = [T{joint_idx-1}(1:3,1:3)' -T{joint_idx-1}(1:3,4)
            zeros(3,1)' 1];
        T{joint_idx} =  base_to_parent_T * joint_to_parent_T;

        % Get joint's world coordinates
        joint_positions(joint_idx,:) = T{joint_idx}(:,4)';
    end
end
