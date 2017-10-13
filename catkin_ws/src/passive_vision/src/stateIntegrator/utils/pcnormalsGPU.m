function normals = pcnormalsGPU(ptCloud, varargin)
%PCNORMALS Estimate normal vectors for a point cloud.
%  normals = PCNORMALS(ptCloud) returns an M-by-3 or M-by-N-by-3 matrix
%  that stores a normal for each point in ptCloud. 6 neighboring points are
%  used to fit a local plane to determine the normal vector. 
%
%  normals = PCNORMALS(ptCloud, K) additionally lets you specify
%  K, the number of points used for local plane fitting. K must be greater
%  or equal to 3. Smaller values of K result in faster computation at the
%  expense of accuracy.
%
%  Notes:
%  ------
%  The normal vectors are computed locally. Their directions cannot be
%  determined automatically. However, they can be set based on the
%  knowledge of how the points were acquired. An example of how to set the
%  direction is shown below, where the normal vectors are pointing towards
%  the sensor.
%
%  Example : Estimate normals of a point cloud
%  ------------------------------------------- 
%  load('object3d.mat'); 
% 
%  % Estimate the normal vectors
%  normals = pcnormals(ptCloud);
% 
%  figure
%  pcshow(ptCloud)
%  title('Estimated normals of a point cloud')
%  hold on
% 
%  x = ptCloud.Location(1:10:end, 1:10:end, 1);
%  y = ptCloud.Location(1:10:end, 1:10:end, 2);
%  z = ptCloud.Location(1:10:end, 1:10:end, 3);
%  u = normals(1:10:end, 1:10:end, 1);
%  v = normals(1:10:end, 1:10:end, 2);
%  w = normals(1:10:end, 1:10:end, 3);
% 
%  % Plot the normal vectors
%  quiver3(x, y, z, u, v, w);
%  hold off
% 
%  % Flip the normals to point towards the sensor location. This is only
%  % necessary when the inward/outward direction of surface needs to be
%  % determined.
%  sensorCenter = [0, -0.3, 0.3]; % in X,Y,Z coordinates
%  for k = 1 : numel(x)
%      p1 = sensorCenter - [x(k), y(k), z(k)];
%      p2 = [u(k), v(k), w(k)];
%      % Flip the normal vector if it is not pointing towards the sensor
%      angle = atan2(norm(cross(p1, p2)), p1*p2');
%      if angle > pi/2 || angle < -pi/2
%          u(k) = -u(k);
%          v(k) = -v(k);
%          w(k) = -w(k);
%      end
%  end
% 
%  figure
%  pcshow(ptCloud)
%  title('Adjusted normals of a point cloud')
%  hold on
%  quiver3(x, y, z, u, v, w);
%  hold off
%
% See also pointCloud, pcshow, pctransform, pcdownsample, pcregrigid,
%          pcfitcylinder
 
%  Copyright 2015 The MathWorks, Inc.
%
%  References:
%  -----------
%  Hugues Hoppe et al, "Surface reconstruction from unorganized points", In
%  Proc. ACM SIGGRAPH 1992.

if ~isa(ptCloud, 'pointCloudGPU')
    error(message('vision:pointcloudGPU:notPointCloudObject', 'ptCloud'));
end

parser = inputParser;
parser.CaseSensitive = false;

parser.addOptional('K', 6, @(x)validateattributes(x, {'single', 'double'}, ...
                        {'real', 'integer', 'scalar', '>=', 3}));    
parser.parse(varargin{:});

K = parser.Results.K;
    
% Find normal vectors for each point
normals = surfaceNormalImpl(ptCloud, K);
