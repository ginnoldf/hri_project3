%% Construct the sphere around the robot maniputor
% It requires knowledge of the robot geometry. Here we roughly set the
% sphere radius to be 0.05m to cover the KINOVA Gen3 robot. There are 7
% joints and 8 links of KINOVA Gen3.
% Input:
%   X: joints (x,y,z,1) position in the world frame (num_joints by 4)
%       num_joints cells that contain a 3dim vector each
% Output:
%   centers: joint centers
%       2dim matrix of size (num_joints, 4)
%   radi: radii around the centers, currently 0.05
%       TODO vector of size num_joints
%
function [centers, radi] = stompRobotSphere(X)
    num_joints = size(X,1);
    center_cell = cell(num_joints,1);
    radi_cell = cell(num_joints,1);

    % consturct the spheres for all links
    for joint_idx = 1:num_joints
        if joint_idx == 1
            parent_joint_position = [0, 0, 0]; % The base coordinate
        else
            parent_joint_position = X(joint_idx-1, 1:3);
        end
        child_joint_poisition = X(joint_idx, 1:3);
 
        % calculate the number of spheres to cover the link
        rad = 0.05;  % radius of the sphere, tuning parameters
        num_spheres = ceil(norm(child_joint_poisition - parent_joint_position) / rad) + 1; % one sphere every rad
        
        % Calculate the centers of the spheres
        curr_center_cell = arrayfun(@(x1, x2) linspace(x1, x2, num_spheres), parent_joint_position', child_joint_poisition', 'UniformOutput', false);
        % xyz coordinates of each sphere
        center_cell{joint_idx} = cell2mat(curr_center_cell)';
        % radius of each sphere
        radi_cell{joint_idx} = rad * ones(size(center_cell{joint_idx}, 1), 1);
    end

    centers = cell2mat(center_cell);
    radi = cell2mat(radi_cell);

end
