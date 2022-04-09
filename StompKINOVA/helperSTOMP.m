%Parameters
% T = 5;
num_waypoints = 20; % number of discretized waypoint
num_sampled_traj = 20; % number of sample trajectories
convergence_threshold = 0.1; % convergence threshhold

% Initial guess of joint angles theta is just linear interpolation of theta_0 and theta_T
theta_0 = currentRobotJConfig;
theta_T = finalRobotJConfig;
num_joints = length(theta_0);
theta = zeros(num_joints, num_waypoints);
for k = 1:length(theta_0)
    theta(k,:) = linspace(theta_0(k), theta_T(k), num_waypoints);
end

% by default, it loads the robot with the structure data format
robot_struct = loadrobot(robot_name); 

%% for calculating the acceleration of theta
% Precompute
A_k = eye(num_waypoints - 1, num_waypoints - 1);
A = -2 * eye(num_waypoints, num_waypoints);
A(1:num_waypoints - 1, 2:num_waypoints) = A(1:num_waypoints - 1, 2:num_waypoints) + A_k;
A(2:num_waypoints, 1:num_waypoints - 1) = A(2:num_waypoints, 1:num_waypoints - 1) + A_k;
A = A(:, 2:end-1); 
R = A' * A;
Rinv = inv(R);
M = 1 / num_waypoints * Rinv ./ max(Rinv, [], 1); % normalized by each column, no longer symmetric
Rinv = 1.5*Rinv/sum(sum(Rinv)); % normalized R inverse, so that the sample is still within the voxel world

%%
%Planner
Q_time = [];   % Trajectory cost Q(theta), t-vector
RAR_time = [];

[~, Q] = stompTrajCost(robot_struct, theta, R, voxel_world);
Q_old = 0;

iter=0;
while abs(Q - Q_old) > convergence_threshold
    iter = iter + 1;
    % overall cost: Q
    Q_old = Q;
    % use tic and toc for printing out the running time
    tic
    % TODO
    % sample noisy trajectories
    [theta_samples, epsilon] = stompSamples(num_sampled_traj, Rinv, theta);

    % iterate over trajectories and calculate their local cost
    S = zeros(num_sampled_traj, num_waypoints);
    for traj = 1:num_sampled_traj
        [S(traj,:), ~] = stompTrajCost(robot_struct, theta_samples(traj), R, voxel_world);
    end
    
    % Given the local traj cost, calculate local trajectory probability
    traj_probs = zeros(num_sampled_traj, num_waypoints);
    for traj = 1:num_sampled_traj
        traj_probs(traj,:) = stompUpdateProb(S(traj,:));
    end
    
    % Compute delta theta (aka gradient estimator)
    d_theta = stompDTheta(traj_probs, epsilon);

    % Calculate the new theta trajectory
    [theta, ~] = stompUpdateTheta(theta, d_theta, M);

    % Compute the cost of the new trajectory
    [~, Q] = stompTrajCost(robot_struct, theta, R, voxel_world);

    toc

    Q_time = [Q_time Q];
    % control cost
    RAR = 1/2 * sum(sum(theta(:, 2:nDiscretize-1) * R * theta(:, 2:nDiscretize-1)'));
    RAR_time = [RAR_time RAR];
    Qtheta % display overall cost
    RAR  % display control cost

    % Stop iteration criteria:
    if iter > 30 || sum(dtheta_smoothed,'all') == 0
        disp('Maximum iteration (30) has reached.')
        break
    end

    if sum(dtheta_smoothed,'all') == 0
        disp('Estimated gradient is 0.')
        break
    end

end

disp('STOMP Finished.');






%% Plot trajectory
% axis tight manual % this ensures that getframe() returns a consistent size
for t=1:size(theta,2)
    show(robot, theta(:,t),'PreservePlot', false, 'Frames', 'off');
    drawnow;
    pause(1/50);
end


%% save data
filename = ['Theta_nDisc', num2str(nDiscretize),'_nPaths_', num2str(num_sampled_traj), '.mat'];
save(filename,'theta')

