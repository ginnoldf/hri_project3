%% Parameters
num_waypoints = 20; % number of discretized waypoint
num_sampled_traj = 20; % number of sample trajectories
convergence_threshold = 0.1; % convergence threshhold

%% Initial guess of joint angles theta - linear interpolation
theta_0 = currentRobotJConfig;
theta_T = finalRobotJConfig;
theta = zeros(num_joints, num_waypoints);
num_joints = length(theta_0);
for joint_idx = 1:num_joints
    theta(joint_idx,:) = linspace(theta_0(joint_idx), theta_T(joint_idx), num_waypoints);
end

%% Load the robot
robot_struct = loadrobot(robot_name); 

%% Precalculations
A_k = eye(num_waypoints - 1, num_waypoints - 1);
A = -2 * eye(num_waypoints, num_waypoints);
A(1:num_waypoints - 1, 2:num_waypoints) = A(1:num_waypoints - 1, 2:num_waypoints) + A_k;
A(2:num_waypoints, 1:num_waypoints - 1) = A(2:num_waypoints, 1:num_waypoints - 1) + A_k;
A = A(:, 2:end-1); 
R = A' * A;
Rinv = inv(R);
M = 1 / num_waypoints * Rinv ./ max(Rinv, [], 1); % normalized by each column, no longer symmetric
Rinv = 1.5 * Rinv / sum(sum(Rinv)); % normalized R inverse, so that the sample is still within the voxel world

%% Planner
Q_list = [];
qo_list = [];
qc_list = [];
acc_cost_list = [];

% get initial Q and set Q_old
[~, Q, ~, ~, ~, ~, ~, ~] = stompTrajCost(robot_struct, theta, R, voxel_world);
Q_old = Q + convergence_threshold + 100; % we do not want to meet the convergence threshold in the first iteration

% timing lists
iter_time_list = [];
sample_time_list = [];
cost_time_list = [];
world_pos_time_list = [];
spheres_time_list = [];
qo_time_list = [];
probs_time_list = [];
dtheta_time_list = [];
new_traj_time_list = [];

% start loop
iter = 1;
max_iter = 30;
Q_difference = 100;
while Q_difference > convergence_threshold

    iter_timer = tic;
    fprintf(['Timings for Iteration ', num2str(iter), ':\n']);

    % sample noisy trajectories and take time
    sample_timer = tic;
    %variance_factor = Q_difference/30;
    variance_factor = 1;
    [theta_samples, epsilon] = stompSamples(num_sampled_traj, variance_factor*Rinv, theta);
    sample_time = toc(sample_timer);
    fprintf(['Sampling trajectories: ', num2str(sample_time), 's\n']);

    % iterate over trajectories and calculate their local cost and take time
    cost_timer = tic;
    S = zeros(num_sampled_traj, num_waypoints);
    world_pos_time = 0;
    spheres_time = 0;
    qo_time = 0;
    for traj_idx = 1:num_sampled_traj
        traj = squeeze(theta_samples(traj_idx,:,:));
        [S(traj_idx,:), ~, ~, ~, ~, traj_world_pos_time, traj_spheres_time, traj_qo_time] = stompTrajCost(robot_struct, traj, R, voxel_world);
        world_pos_time = world_pos_time + traj_world_pos_time;
        spheres_time = spheres_time + traj_spheres_time;
        qo_time = qo_time + traj_qo_time;
    end
    cost_time = toc(cost_timer);
    fprintf(['Calculating costs: ', num2str(cost_time), 's\n']);
    fprintf(['  Avg cost time per trajectory: ', num2str(cost_time/num_sampled_traj), 's\n']);
    fprintf(['  Calculating joint world positions: ', num2str(world_pos_time), 's\n']);
    fprintf(['  Calculating spheres: ', num2str(spheres_time), 's\n']);
    fprintf(['  Calculating obstacle costs: ', num2str(qo_time), 's\n']);

    % Given the local traj costs, calculate local trajectory probabilities
    probs_timer = tic;
    traj_probs = stompUpdateProb(S);
    probs_time = toc(probs_timer);
    fprintf(['Calculating trajectory probabilities: ', num2str(probs_time), 's\n']);

    % Compute delta theta (aka gradient estimator)
    dtheta_timer = tic;
    d_theta = stompDTheta(traj_probs, epsilon);
    dtheta_time = toc(dtheta_timer);
    fprintf(['Calculating delta theta: ', num2str(dtheta_time), 's\n']);

    % Calculate the new theta trajectory
    new_traj_timer = tic;
    [theta, dtheta_smoothed] = stompUpdateTheta(theta, d_theta, M);
    new_traj_time = toc(new_traj_timer);
    fprintf(['Calculating new trajectory: ', num2str(new_traj_time), 's\n']);

    % Compute the cost of the new trajectory
    [~, Q, qo, qc, acc_cost, ~, ~] = stompTrajCost(robot_struct, theta, R, voxel_world);

    iter_time = toc(iter_timer);
    fprintf(['In total: ', num2str(iter_time), 's\n\n']);

    %% Evaluation of iteration result
    Q_list = [Q_list Q];
    qo_list = [qo_list qo];
    qc_list = [qc_list qc];
    acc_cost_list = [acc_cost_list acc_cost];

    % keep all timings for evaluation
    iter_time_list = [iter_time_list iter_time];
    sample_time_list = [sample_time_list sample_time];
    cost_time_list = [cost_time_list cost_time];
    world_pos_time_list = [world_pos_time_list world_pos_time];
    spheres_time_list = [spheres_time_list spheres_time];
    qo_time_list = [qo_time_list qo_time];
    probs_time_list = [probs_time_list probs_time];
    dtheta_time_list = [dtheta_time_list dtheta_time];
    new_traj_time_list = [new_traj_time_list new_traj_time] ;

    %% Stopping conditions
    % max iterations reached
    if iter >= max_iter
        disp('Maximum iteration (30) has reached.')
        break
    % estimated gradient is 0
    elseif sum(dtheta_smoothed,'all') == 0
        disp('Estimated gradient is 0.')
        break
    end

    % prepare next iteration
    iter = iter + 1;
    Q_difference = abs(Q - Q_old);
    Q_old = Q;

end

disp(['STOMP Finished after ', num2str(iter), ' iterations.']);

% print timing results
fprintf(['Mean iteration time: ', num2str(mean(iter_time_list)), 's\n']);
fprintf(['Mean sample time: ', num2str(mean(sample_time_list)), 's\n']);
fprintf(['Mean cost time: ', num2str(mean(cost_time_list)), 's\n']);
fprintf(['Mean world position time: ', num2str(mean(world_pos_time_list)), 's\n']);
fprintf(['Mean spheres time: ', num2str(mean(spheres_time_list)), 's\n']);
fprintf(['Mean obstacle cost time: ', num2str(mean(qo_time_list)), 's\n']);
fprintf(['Mean probs time: ', num2str(mean(probs_time_list)), 's\n']);
fprintf(['Mean dtheta time: ', num2str(mean(dtheta_time_list)), 's\n']);
fprintf(['Mean new trajectory time: ', num2str(mean(new_traj_time_list)), 's\n']);

% print cost evolution
Q_list
qo_list
qc_list
acc_cost_list

%% Nice plots
plot_costs(Q_list, qo_list, qc_list, acc_cost_list, 'task1_run1.png');

%% Plot trajectory
for waypoint_idx = 1:num_waypoints
    show(robot, theta(:,waypoint_idx),'PreservePlot', false, 'Frames', 'off');
    drawnow;
    pause(1/5);
end

%% Save data
filename = ['Theta_nDisc', num2str(num_waypoints),'_nPaths_', num2str(num_sampled_traj), '.mat'];
save(filename,'theta')
