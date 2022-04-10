% Input: 
%   S: local cost of trajectories per waypoint
%       2dim matrix of size (num_sampled_traj, num_waypoints)
% Output:
%   traj_probs: new trajectory
%       2dim matrix of size (num_sampled_traj, num_waypoints)
%
% Compute the exp(-1/lambda * S{ki}) form for trajectory probability
% Equation (11) in the ICRA 2011 paper

function traj_probs = stompUpdateProb(S)
    % sensitivity
    h = 10;

    % max and min local cost at each time step
    maxS = max(S, [], 1);
    minS = min(S, [], 1);

    % exp calculates the element-wise exponential
    expCost = exp(-h * (S - minS) ./ (maxS - minS));

    % To handle the case where maxS = minS = 0. This is possible when local
    % trajectory cost only includes obstacle cost, which at the end of the time
    % duration, the speed is 0, or when there is no collision.
    expCost(isnan(expCost)) = 0;

    % normalize the exponentialized cost to have the probabilities
    traj_probs = expCost./ sum(expCost, 1);
    traj_probs(isnan(traj_probs)) = 0;

end
