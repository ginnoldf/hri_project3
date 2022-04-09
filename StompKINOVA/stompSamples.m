% Input: 
%   sigma： sample covariance matrix
%   theta: mean trajectory from last iteration
% Output:
%   theta_paths: sampled trajectories
%   em: sampled Gaussian trajectory for each joint

function [theta_paths, em]=stompSamples(nSamplePaths,sigma,theta)
% Sample theta (joints angles) trajectory 

[nJoints, nDiscretize] = size(theta);

em = cell(1,nJoints);
ek = cell(1,nSamplePaths);

theta_paths = cell(1, nSamplePaths);
mu=zeros(1,length(sigma));


for m = 1 : nJoints
    % Each joint is sampled independently
    % The starting q0 and final qT are fixed, so set the sample to 0
    % sample from multivariable Gaussian distribution
   
end

% regroup it by samples
emk = [em{:}];
for k=1:nSamplePaths
    ek{k} = reshape(emk(k,:),nDiscretize, nJoints)';
    theta_paths{k} = theta + ek{k};
    
end