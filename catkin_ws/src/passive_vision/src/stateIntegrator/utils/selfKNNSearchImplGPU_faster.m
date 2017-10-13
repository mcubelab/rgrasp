function [indices] = selfKNNSearchImplGPU_faster(loc, K, OrganizedLoc, UnorganizedToOrganizedInd)
% loc: n*3 pointcloud
% K: nearest neighbor

% indices: K * n indices

global selfKNNSearchGPU_faster_ptx

n = size(loc,1);
numThreadsPerBlock = 1024;
numBlocks = ceil(n / numThreadsPerBlock);
selfKNNSearchGPU_ptx.GridSize = [numBlocks 1];
selfKNNSearchGPU_ptx.ThreadBlockSize = [numThreadsPerBlock 1];

resultsGPU = gpuArray(int32(zeros(1, K * n)) );

argsGPU = gpuArray(int32([numBlocks, numThreadsPerBlock, K, n]));

%pcGPU1 = gpuArray(single([ptCloudB.Location(:,1)' ptCloudB.Location(:,2)' ptCloudB.Location(:,3)']));
pcGPU = gpuArray(single(reshape(loc', n * 3, 1)));

pcGPU_org = gpuArray(single(reshape(OrganizedLoc, 640*480 * 3, 1)));

unorg2org = gpuArray(int32(reshape(UnorganizedToOrganizedInd, n*1, 1)) -1 );  % -1 to go to c++ index

results = feval(selfKNNSearchGPU_faster_ptx, resultsGPU, argsGPU, pcGPU, pcGPU_org, unorg2org);

indices = uint32(gather(results));
indices = reshape(indices, K, n)+1;  % add one to move to matlab indices

end
