%% forward kinematics
% Input: 
%       robot_struct: the Matlab robot structure object
%       theta: the joints rotation angles
% Output:
%       X: Joints' positions in the world frame
%       T: Homogeneous Transformation from the Joint frame to the base
%       frame
%
% In this sample code, we directly call the MATLAB built-in function getTransform
% to calculate the forward kinemetics
function [X, T] = updateJointsWorldPosition(robot_struct, theta)

    % Update the robot configuration structure used by Matlab
    % Refer to: https://www.mathworks.com/matlabcentral/answers/31273-how-to-update-struct-array-fields-with-mutiple-values
    theta_cell = num2cell(theta);

    % Because the getTranform() function can only takes in structure array
    % robot Configuration, we have to construct it first by copying the structure from the homeConfiguration
    % robot_struct = loadrobot(robot_name); % by default, it loads the robot with structure data format
    t_configuration = robot_struct.homeConfiguration; % copy the home configuration struct
    [t_configuration.JointPosition] = theta_cell{:}; % update the Joint position using theta

    % get the number of joints
    num_joints = size(theta,1);
    T = cell(1, num_joints);
    X = zeros(num_joints, 4); 

    for k = 1:num_joints
        % get the homegeneous transformation from kth joint's frame to the
        % base frame
        % getTransform can only takes in structure array Configuration
        bodyname = robot_struct.Bodies{1, k}.Name;
        T{k} = getTransform(robot_struct, t_configuration, bodyname);
        % Get joint's world coordinates
        X(k,:) = T{k}(:,4)';
    end
    
end