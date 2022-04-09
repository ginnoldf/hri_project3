%% forward kinematics
% INPUT: 
%       robot_struct: the Matlab robot structure object
%       theta: the joints rotation angles
% OUTPUT:
%       X: Joints' positions in the world frame
%       T: Homogeneous Transformation from the Joint frame to the base
%       frame
function [X, T] = updateJointsWorldPosition(robot_struct, theta)

% In this sample code, we directly call the MATLAB built-in function getTransform
% to calculate the forward kinemetics
    

% Update the robot configuration structure used by Matlab
% Refer to: https://www.mathworks.com/matlabcentral/answers/31273-how-to-update-struct-array-fields-with-mutiple-values
theta_cell = num2cell(theta);
% Because the getTranform() function can only takes in structure array
% robot Configuration, we have to construct it first by copying the structure from the homeConfiguration
% robot_struct = loadrobot(robot_name); % by default, it loads the robot with structure data format
tConfiguration= robot_struct.homeConfiguration; % copy the home configuration struct
[tConfiguration.JointPosition]= theta_cell{:}; % update the Joint position using theta
% get the number of joints
% EDITED
%nJoints = length(theta);
nJoints = size(theta,1);
T = cell(1,nJoints);
X = zeros(nJoints, 4); 

for k=1:nJoints
    % get the homegeneous transformation from kth joint's frame to the
    % base frame
    % getTransform can only takes in structure array Configuration
    % DONE
    bodyname = robot_struct.Bodies{1,k}.Name;
    T{k} = getTransform(robot_struct,tConfiguration,bodyname);
    % Get joint's world coordinates
    % DONE
    X(k,:) = T{k}(:,4)';
end
    
end