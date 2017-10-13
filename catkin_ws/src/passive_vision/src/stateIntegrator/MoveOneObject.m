function [changeId,newobject,succ] = MoveOneObject( message,cameraData, ARCUnit,MetaData,useGPU)
    % find the remove points and added points
    tic;
    succ = 0;
    newobject =[];
    changeId = [];
    removedpts = cell(1,length(ARCUnit.cameraIds));
    addeddpts = cell(1,length(ARCUnit.cameraIds));
    for i = 1:length(ARCUnit.cameraIds)
        changemask = abs(cameraData.depth{i} - ARCUnit.Curr.depth{i}) > 0.02 & ...
                     sum(abs(cameraData.color{i} - ARCUnit.Curr.color{i}), 3) > 50;

        removelocation = changemask(:)' & (cameraData.points3d{i}(3,:) < ARCUnit.Curr.points3d{i}(3,:));
        removedpts{i}  = ARCUnit.Curr.points3d{i}(:,removelocation);
        addlocation  = changemask(:)' & (cameraData.points3d{i}(3,:) > ARCUnit.Curr.points3d{i}(3,:));
        addeddpts{i} = cameraData.points3d{i}(:,addlocation);
    end
    removedpts = cell2mat(removedpts);
    addeddpts = cell2mat(addeddpts);

    %try removedpts = denoisePointCloud(removedpts); end
    %try addeddpts = denoisePointCloud(addeddpts); end
    if length(removedpts) < 20 || length(addeddpts) < 20
        fprintf('There is NO change!\n')
        changeId = [];
        return;
    end
    % find the changeId
    ObjIdsInlist = find(ismember({cameraData.objectList.objectName}, message.objectName));
    length_ObjIdsInlist = length(ObjIdsInlist);
    if length_ObjIdsInlist == 0
        fprintf('There is NO this object in the unit!\n')
        return;
    elseif length_ObjIdsInlist == 1
        removeId = ObjIdsInlist;
    else
        centerofremoved = median(removedpts,2);
        distance = zeros(1, length_ObjIdsInlist);
        for i = 1:length_ObjIdsInlist
            c1 = cameraData.objectList(ObjIdsInlist(i)).Pose(1:3,4);
            distance(i) = sum((c1-centerofremoved).*(c1-centerofremoved));
        end
        [~,closer_id] = min(distance);
        removeId = ObjIdsInlist(closer_id);
    end

    %% try to align the new location
    [objModelCloud, info] = getModelPC(MetaData,message.objectName);
    addeddptsGrid = pointCloud(addeddpts');
    gridStep = 0.004; % grid size for downsampling point clouds
    addeddptsGrid = pcdownsample(addeddptsGrid,'gridAverage',gridStep);
    predObjPose = estimatePoseICP(objModelCloud, info, addeddptsGrid, useGPU);

    changeId = removeId;
    newobject = cameraData.objectList(changeId);
    newobject.Pose = predObjPose;
    succ = 1;
    if 0
        clf;
        vis_point_cloud(cameraData.points3d{1}','r');
        hold on;
        vis_point_cloud(addeddpts','g');
        vis_point_cloud(removedpts','b');


        ptCloudAligned = transformPointCloud(objModelCloud.Location',predObjPose);
        vis_point_cloud(ptCloudAligned','k');
        vis_point_cloud(tmpObjModelPts.Location,'m');
    end

    fprintf('[MATLAB Timing Moved %s]: ',message.objectName); toc;
end