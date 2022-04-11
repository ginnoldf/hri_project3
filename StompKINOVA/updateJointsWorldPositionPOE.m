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
function [joint_positions, T] = updateJointsWorldPositionPOE(robot_struct, theta)

    % init
    num_joints = size(theta,1);
    T = cell(1, num_joints);
    joint_positions = zeros(num_joints, 4); 
    
    % joint coordinates for theta = zeros(num_joints, 7)
    [~, joint_base_T] = updateJointsWorldPosition(robot_struct, zeros(num_joints, 1), true);

    % hardcode rpy of joints from udfr file for Kinova3Gen
    rpy = [0 0 0;
        0 0 0;
        0 0 0;
        0 0 0;
        0 0 0;
        0 0 0;
        0 0 0];

    % to save time we save all PoE's and start with the identity for joint 1
    mat_exps = zeros(num_joints+1, 4, 4);
    mat_exps(1,:,:) = eye(4);

    % iterate over joints and use PoE to get their base transformations
    for joint_idx = 1:num_joints

        % calculate skrew axis
        skrew_axis_v = robot_struct.Bodies{1, joint_idx}.Joint.JointAxis';

        % write skrew axis in matrix form
        skrew_axis_mat = [0 -skrew_axis_v(3) skrew_axis_v(2);
            skrew_axis_v(3) 0 -skrew_axis_v(1);
            -skrew_axis_v(2) skrew_axis_v(1) 0];

        % Rodrigues' rotation formula
        curr_rot_mat = eye(3) + skrew_axis_mat * sin(theta(joint_idx)) + ...
            skrew_axis_mat.^2 * (1 - cos(theta(joint_idx)));

        % compose and save matrix exponential
        mat_exp = [curr_rot_mat joint_base_T{joint_idx}(1:3, 4);
            zeros(3,1)' 1];
        mat_exps(joint_idx+1,:,:) = squeeze(mat_exps(joint_idx,:,:)) * mat_exp; 

        % product of exponentials formula (4.14) from the textbook
        T{joint_idx} = mat_exps(joint_idx+1) * joint_base_T{joint_idx};
        joint_positions(joint_idx,:) = T{joint_idx}(:,4)';
    end

    T{3}(1:3,1:3)
    pause(29);
end
