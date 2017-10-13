function instanceseg = getInstanceSegmentMask(cameraData, ARcUnit, MetaData,useGPU)
         allModelPots = cell(1,length(cameraData.objectList));
         allModelIds = cell(1,length(cameraData.objectList));
         for i = 1:length(cameraData.objectList)
             objModelCloud =getModelPC(MetaData,cameraData.objectList(i).objectName);
             ptCloudAligned = transformPointCloud(objModelCloud.Location',cameraData.objectList(i).Pose);
             allModelPots{i} = double(ptCloudAligned);
             allModelIds{i} = i*ones(1,length(ptCloudAligned));
         end
         allModelPots = cell2mat(allModelPots);
         allModelIds = cell2mat(allModelIds);
         instanceseg = cell(1,length(cameraData.points3d));
         for i = 1:length(cameraData.points3d)
             notNaNIds = find(~isnan(cameraData.points3d{i}(1,:)));
             if useGPU
                [indicesNN,distsNN] = multiQueryKNNSearchImplGPU(pointCloud(allModelPots'),cameraData.points3d{i});
             else
                [indicesNN,distsNN] = multiQueryKNNSearchImpl(pointCloud(allModelPots'),cameraData.points3d{i}(:,notNaNIds)',1);
             end
             instance_seg = zeros(1,length(cameraData.points3d{i}));
             point_threshold = 0.0009;
             instance_seg(notNaNIds(distsNN<point_threshold)) = allModelIds(indicesNN(distsNN<point_threshold));
             instanceseg{i} = instance_seg;
         end
         
end