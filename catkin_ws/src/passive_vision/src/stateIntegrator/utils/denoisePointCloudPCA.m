function objSegmPts = denoisePointCloudPCA(objSegmPts,precent)
% Remove outlier points from the principal components (computed from PCA)
% of a noisy point cloud
%
% function objSegPts = denoisePointCloud(objSegPts)
% Input:
%   objSegPts - 3xN float array of 3D points
% Output:
%   objSegPts - 3xN float array of 3D points
%
% Compute PCA for removing outliers from segmented point cloud
coeffPCAoutlier = pca(objSegmPts');
currObjSegPtsAligned = (inv(coeffPCAoutlier) * objSegmPts)';

% Find outliers from first principal component (local minima < 5%, maxima > 50%)
PCRange = prctile(currObjSegPtsAligned,precent);
objSegmPts = objSegmPts(:,find((currObjSegPtsAligned(:,1) > PCRange(1,1)) & (currObjSegPtsAligned(:,1) < PCRange(2,1)) & ...
                             (currObjSegPtsAligned(:,2) > PCRange(1,2)) & (currObjSegPtsAligned(:,2) < PCRange(2,2)) & ...
                             (currObjSegPtsAligned(:,3) > PCRange(1,3)) & (currObjSegPtsAligned(:,3) < PCRange(2,3))));

end

