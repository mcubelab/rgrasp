function [newobject,succ] = AddOneObject(message,cameraData, ARCUnit, MetaData,useGPU)
         % find the new object point could by change detection 
         tic;
         succ = 0;
         changepts = cell(1,length(ARCUnit.cameraIds));
         changemask = cell(1,length(ARCUnit.cameraIds));
         missingratio = zeros(1,length(ARCUnit.cameraIds));
         
         newobject.objectName = message.objectName;
         newobject.Pose = [eye(3,3),[0;0;0]];
         newobject.conf = 0;
         newobject.visibility  = 0;
         newobject.topobj = {};
         [objModelCloud,info,objMeta] = getModelPC(MetaData,message.objectName);
         for i = 1:length(ARCUnit.cameraIds)
             changemask{i} = (abs(cameraData.depth{i}-ARCUnit.Curr.depth{i})>min(0.02,0.9*min(objMeta.dimensions))&...
                          sum(abs(cameraData.color{i}-ARCUnit.Curr.color{i}),3)>60)...
                          |(sum(abs(cameraData.color{i}-ARCUnit.Curr.color{i}),3)>300 ...
                          &abs(cameraData.depth{i}-ARCUnit.Curr.depth{i})>0.01);
             changepts{i} = cameraData.points3d{i}(:, changemask{i} );
             addlocation  =  changemask{i}(:)'&...
                            (cameraData.points3d{i}(3,:) > ARCUnit.Curr.points3d{i}(3,:)...
                            |isnan(cameraData.points3d{i}(3,:)) & ~isnan(ARCUnit.Curr.points3d{i}(3,:)));
             changepts{i} = cameraData.points3d{i}(:,addlocation);
             missingratio(i) = sum(cameraData.depth{i}(changemask{i})==0)/sum(changemask{i}(:));
         end
         changepts = cell2mat(changepts);
         
         
         
         if mean(missingratio)>0.7 
            fprintf('Treated as missing depth object\n');
            [vol,gridxyzWord] = getMissingDepthVolume(cameraData, ARCUnit,changemask);
            TargetGrids = gridxyzWord(vol,:);
            %{
            if isfield(message,'dropxyz')&&length(message.dropxyz)==3
               range = 0.6*max(objMeta.dimensions);
               TargetGrids = removePointsOutBound(changepts,message.dropxyz,[range,range,Inf]);
            end
            %}
            if length(TargetGrids)<20
               fprintf('There is no change (%d). fail to add\n',length(TargetGrids));
               return;
            end
            TargetGrids = denoisePointCloud(TargetGrids');
            [coeffPCA,scorePCA,latentPCA] = pca(TargetGrids');
            coeffPCA = [coeffPCA(:,1),coeffPCA(:,2),cross(coeffPCA(:,1),coeffPCA(:,2))]; % Follow righthand rule
            predObjPose = [[coeffPCA median(TargetGrids,2)]; 0 0 0 1];
         else
             changepts = changepts(:,~isnan(changepts(1,:)));
             %{
             if isfield(message,'dropxyz')&&length(message.dropxyz)==3
                 range = 0.6*max(objMeta.dimensions);
                 changepts = removePointsOutBound(changepts,message.dropxyz,[range,range,Inf]);
             end
             %}
             if length(changepts)<20
                fprintf('There is no change. fail to add\n');
                return;
             end
             changepts = denoisePointCloud(changepts);
             % algin point could 
             changeptsGrid = pointCloud(changepts');
             gridStep = 0.002; % grid size for downsampling point clouds
             changeptsGrid = pcdownsample(changeptsGrid,'gridAverage',gridStep);
             predObjPose = estimatePoseICP(objModelCloud, info, changeptsGrid,useGPU);
         end
         
         newobject.Pose = predObjPose;
         newobject.conf = 1;
         newobject.visibility  = 1;
         newobject.topobj = {};
         succ = 1;
         fprintf('ADDED: %s ',message.objectName); toc;
         %%
         if 0
             clf;
             vis_point_cloud(cameraData.points3d{2}','r');hold on;
             try vis_point_cloud(gridxyzWord(vol(:),:),'b');end
             try vis_point_cloud(changepts','k');end
             [objModelCloud,info] =getModelPC(MetaData,message.objectName);
             ptCloudAligned = transformPointCloud(objModelCloud.Location',predObjPose);
             vis_point_cloud(ptCloudAligned','k');
         end
end         
         