function predObjPose = estimatePoseICP(objModelCloud,info, changeptsGrid,useGPU,initRT)
         icpWorstRejRatio = 0.9; % ratio of outlier points to ignore during ICP
         if ~exist('initRT','var')
             [coeffPCA,scorePCA,latentPCA] = pca(changeptsGrid.Location);
             if size(latentPCA,1) < 3
                latentPCA = [latentPCA;0];
             end
             coeffPCA = [coeffPCA(:,1),coeffPCA(:,2),cross(coeffPCA(:,1),coeffPCA(:,2))]; % Follow righthand rule
             if ~isempty(info)
                 % Check whether the bottom is up 
                 bottomRotate = coeffPCA*info.bottomDir;
                 if bottomRotate(3)>0.1
                    coeffPCA = [coeffPCA(:,1),-coeffPCA(:,2),cross(coeffPCA(:,1),-coeffPCA(:,2))]; 
                 end
             end
             surfPCAPoseBin = [[coeffPCA median(changeptsGrid.Location,1)']; 0 0 0 1];

             pushBackAxis = [0; 0; -1];
             pushBackVal = min(0.1,0.5*max([objModelCloud.XLimits(2),objModelCloud.YLimits(2),objModelCloud.ZLimits(2)]));
             surfPCAPoseBin(1:3,4) = surfPCAPoseBin(1:3,4) + pushBackAxis.*pushBackVal;
         else
             surfPCAPoseBin = initRT;
         end
         
         tmpObjModelPts =  pointCloud(transformPointCloud(objModelCloud.Location',surfPCAPoseBin)');
         if useGPU
%             try 
%                 [tform,movingReg,icpRmse] = pcregrigidGPU(changeptsGrid,tmpObjModelPts,'InlierRatio',icpWorstRejRatio,'MaxIterations',200,'Tolerance',[0.0001 0.0009],'Verbose',false,'Extrapolate',true,'Metric','pointToPlane');
%             catch 
                %fprintf('use Point to Point')
                [tform,movingReg,icpRmse] = pcregrigidGPU(changeptsGrid,tmpObjModelPts,'InlierRatio',icpWorstRejRatio,'MaxIterations',200,'Tolerance',[0.0001 0.0009],'Verbose',false,'Extrapolate',true);
%             end
         else
%              try
%                  [tform,movingReg,icpRmse] = pcregrigid(changeptsGrid,tmpObjModelPts,'InlierRatio',icpWorstRejRatio,'MaxIterations',200,'Tolerance',[0.001 0.009],'Verbose',false,'Extrapolate',true,'Metric','pointToPlane');
%              catch
                 %fprintf('use Point to Point')
                 [tform,movingReg,icpRmse] = pcregrigid(changeptsGrid,tmpObjModelPts,'InlierRatio',icpWorstRejRatio,'MaxIterations',200,'Tolerance',[0.0001 0.0009],'Verbose',false,'Extrapolate',true);
%              end
         end
         icpRt = inv(tform.T');
         
         predObjPose = icpRt * surfPCAPoseBin;
end