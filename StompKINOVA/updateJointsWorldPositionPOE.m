%% forward kinematics
% Input: 
%       robot_struct: the Matlab robot structure object
%       theta: the joints rotation angles
%           vector of length num_joints
%       joint_base_T: transformations from joint frames to base frames
%           cell with joint_num matrices of size (4,4)
% Output:
%       joint_positions: Joints' positions in the world frame
%           matrix of size (num_joints, 4)
%       T: Homogeneous Transformation from the Joint frame to the base frame
%           cell with joint_num matrices of size (4,4)
%
% In this sample code, we directly call the MATLAB built-in function 
% getTransform to calculate the forward kinemetics
function [joint_positions, T] = updateJointsWorldPositionPOE(robot_struct, theta, base_joint_T)

    % init
    num_joints = size(theta, 1);
    T = cell(1, num_joints);
    joint_positions = zeros(num_joints, 4);

    % set joint axis in joint frame
    joint_skrew_axis = [0 0 1]';

    % to save the matrix exponentials
    mat_exps = zeros(num_joints, 4, 4);

    % iterate over joints and use PoE to get their base transformations
    for joint_idx = 1:num_joints

        % get the base joint position q from joint base transformation
        q = base_joint_T{joint_idx}(1:3, 4);

        % transform skrew axis to base frame
        base_skrew_axis = base_joint_T{joint_idx}(1:3,1:3) * joint_skrew_axis;

        % write skrew axis [w] in matrix form
        rotation_mat = [0 -base_skrew_axis(3) base_skrew_axis(2);
            base_skrew_axis(3) 0 -base_skrew_axis(1);
            -base_skrew_axis(2) base_skrew_axis(1) 0];

        % calculate translation vector v
        v = cross(-base_skrew_axis, q);

        % compose skrew axis matrix S
        skrew_axis_mat = [ rotation_mat v;
            zeros(3,1)' 0 ];

        % compose and save matrix exponential
        mat_exp = expm(skrew_axis_mat * theta(joint_idx));
        mat_exps(joint_idx,:,:) = mat_exp;

        % product of exponentials formula (4.14) from the textbook
        T{joint_idx} = eye(4);
        for i = 1:joint_idx
            T{joint_idx} = T{joint_idx} * squeeze(mat_exps(joint_idx,:,:));
        end
        T{joint_idx} = T{joint_idx} * base_joint_T{joint_idx};
        
        % get joint position from transformation
        joint_positions(joint_idx,:) = T{joint_idx}(:,4)';

    end

end
