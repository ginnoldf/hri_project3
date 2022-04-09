% Input: 
%   theta: mean trajectory from last iteration
%       2dim matrix of size (num_joints, num_waypoints)
%   dtheta: gradient estimation for this step
%       2dim matrix of size (num_joints, num_waypoints)
%   M: to make dtheta smpoother
%       2dim matrix of size(num_waypoints-2, num_waypoints-2)
% Output:
%   theta: new trajectory
%       2dim matrix of size (num_joints, num_waypoints)
%   dtheta_smoothed: smoothed dtheta
%       2dim matrix of size (num_joints, num_waypoints)

function [theta, dtheta_smoothed] = stompUpdateTheta(theta, dtheta, M)

    % Smoothed delta theta
    dtheta_smoothed = M * dtheta(:, 2:end - 1)';

    % Update theta
    theta(:,2:end-1) = theta(:,2:end-1) + dtheta_smoothed';

end
