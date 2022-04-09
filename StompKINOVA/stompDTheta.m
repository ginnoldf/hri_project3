% update theta 
% em: 1 by num_joints cell, each cell is num_sampled_traj by num_waypoints matrix
function dtheta = stompDTheta(trajProb, em)

num_joints = length(em);
num_sampled_traj = size(trajProb, 2);

dtheta = zeros(num_joints, num_sampled_traj);
% iterate over all joints
