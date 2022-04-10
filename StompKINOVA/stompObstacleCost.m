% Input: 
%   sphere_centers: centers of the spheres
%       2dim matrix of size (num_joints, 4)
%   radi: radii around the centers, currently 0.05
%       TODO vector of size num_joints
%   voxel_world: voxel world 
%       struct
%   vel: velocities of the joints
%       vector of length num_joints
% Output:
%   cost: cost for each joint in the given robot configuration
%       vector of length num_joints
%
function cost = stompObstacleCost(sphere_centers, radi, voxel_world, vel)

    safety_margin = 0.05; % the safety margin distance, unit: meter
    cost = 0;

    % signed distance function of the world
    voxel_world_sEDT = voxel_world.sEDT;
    world_size = voxel_world.world_size;

    % calculate which voxels the sphere centers are in. idx is the grid xyz subscripts
    % in the voxel world.
    env_corner = voxel_world.Env_size(1, :); % [xmin, ymin, zmin] of the metric world
    env_corner_vec = repmat(env_corner, length(sphere_centers), 1); % copy it to be consistent with the size of sphere_centers
    vw_idx = ceil((sphere_centers - env_corner_vec) ./ voxel_world.voxel_size);

    % Eq (13) in the STOMP conference paper
    try
        num_spheres = length(sphere_centers);
        cost_array = zeros(num_spheres, 1);

        for sphere_idx = 1:num_spheres
            cost_array(sphere_idx) = max([safety_margin + radi(sphere_idx) - voxel_world_sEDT(vw_idx(sphere_idx)), 0]) * abs(vel(sphere_idx));
        end

        cost = sum(cost_array);
    catch
        vw_idx = ceil((sphere_centers - env_corner_vec) ./ voxel_world.voxel_size);
    end
