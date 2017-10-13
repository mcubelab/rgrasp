function [instanceseg,depthprojected] = getInstanceSegmentMaskProj(cameraData, ACRUnit, MetaData)
    allModelPots = cell(1,length(cameraData.objectList));
    allModelIds = cell(1,length(cameraData.objectList));
    for i = 1:length(cameraData.objectList)
        objModelCloud = getModelPC(MetaData,cameraData.objectList(i).objectName, 1);
        ptCloudAligned = transformPointCloud(objModelCloud.Location', cameraData.objectList(i).Pose);
        allModelPots{i} = double(ptCloudAligned);
        allModelIds{i} = i * ones(1, length(ptCloudAligned));
    end
    instanceseg = cell(1,length(cameraData.points3d));
    depthprojected = cell(1,length(cameraData.points3d));
    imSize = size(cameraData.depth{1});

    mat_allModelPots = cell2mat(allModelPots);
    mat_allModelIds = cell2mat(allModelIds);
    for i = 1:length(ACRUnit.cameraIds)
        [depth_projected, instance_seg] = proj3DPtsTo2D(mat_allModelPots,...
            ACRUnit.cameraparam{i}.ext, ACRUnit.cameraparam{i}.int, imSize, mat_allModelIds) ;
        instanceseg{i} = imclose(uint8(instance_seg),strel('disk',3));
        depthprojected{i} = depth_projected;
    end
end