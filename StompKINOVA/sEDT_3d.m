% Input:
%   A: binary map of voxels
%       2dim matrix with size (num_joints, num_joints-2)
% Output:
%   Q: 3D signed EDT
%       double value

function Q = sEDT_3d(A)

    % Compute 3D signed EDT, where A is the binary map of voxels
    % Note that no 0 cost boundary voxels are used for simplicity

    Ainv = 1 - A;

    Q = bwdist(A, 'Euclidean') - bwdist(Ainv, 'Euclidean');

end
