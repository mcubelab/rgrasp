function [newObjectList,updateLocal,sceneChange] = LocalUpdateSIFTflow(cameraData, ARCUnit,MetaData,ignoredObjectIds, useGPU)
% detect the changed 
newObjectList = cameraData.objectList;
sceneChange = 0;
changeThre = 0.2;
% don't update the object that are occluded to much 
visibilityIgnore = find([cameraData.objectList.visibility]<0.5);
ignoredObjectIds = [ignoredObjectIds,visibilityIgnore];
if isfield(cameraData, 'instancemapping')
    for i = 1:cameraData.numodobject
        [old_id]= find(cameraData.instancemapping==i);
        if ~isempty(old_id)
            % cannot observe in previouse frame 
            if old_id>0&&(ARCUnit.Curr.objectList(old_id).visibility<0.3)
                ignoredObjectIds = [ignoredObjectIds i];
            end
            if ~isempty(cameraData.objectList(i).topobj)
                ignoredObjectIds = [ignoredObjectIds i];
            end
        end
    end
end
ignoredObjectIds = unique(ignoredObjectIds);

updateLocal = 0;
if ~isfield(ARCUnit.Curr,'instanceseg')
    return;
end

changemask = cell(1,length(ARCUnit.cameraIds));
for i = 1:length(ARCUnit.cameraIds)
    changemask{i} = abs(cameraData.depth{i}-ARCUnit.Curr.depth{i})>0.02&...
                    sum(abs(cameraData.color{i}-ARCUnit.Curr.color{i}),3)>50;
end
objChange_pixCount = zeros(1,cameraData.numodobject);
for obj_i = 1:cameraData.numodobject
    if ismember(obj_i,ignoredObjectIds)
        change_ratio = changeThre;
    else
        change_pix = 0;
        total_pix = 0;
        for cam_i = 1:length(ARCUnit.cameraIds)
            objchange = changemask{cam_i};
            objMask =  cameraData.instanceseg{cam_i} == obj_i;
            change_pix = change_pix+sum(objchange(:)&objMask(:));
            total_pix = total_pix+sum(objMask(:));
        end
        change_ratio = change_pix./total_pix;
    end
    objChange_pixCount(obj_i) = change_ratio;
end
changedObjId = find(objChange_pixCount>changeThre);
changedObjId(ismember(changedObjId,ignoredObjectIds)) =[];
if ~any(~ismember(find(objChange_pixCount>changeThre),ignoredObjectIds))
    return;
end
sceneChange = mean(objChange_pixCount(objChange_pixCount>changeThre));

fprintf('LocalUpdate\n');
for i = 1:length(changedObjId)
    fprintf('%s\n',cameraData.objectList(changedObjId(i)).objectName);
end

tic;
[change_sort,change_sortId] = sort(objChange_pixCount);
instancemapping = [0,cameraData.instancemapping];
label_warps = cell(1,length(cameraData.color));
for i = 1:length(cameraData.color)
    [lable_des]=computeSIFTflow(cameraData.color{i}, ARCUnit.Curr.color{i}, double(ARCUnit.Curr.instanceseg{i}));
    lable_des_map = instancemapping(round(lable_des)+1);
    label_warps{i} = lable_des_map(:);
end
toc;
% remove tote
combinePoits = cell2mat(cameraData.points3d);
combinePoits = combinePoits';
combinefgIdx = cell2mat(cameraData.fgIdx');
label_warps  = cell2mat(label_warps');


% form the least changed obj to most do local ICP
for obj_i = change_sortId
    [objModelCloud,info] =getModelPC(MetaData,cameraData.objectList(obj_i).objectName);
    
    if objChange_pixCount(obj_i)>changeThre&&~ismember(obj_i,ignoredObjectIds)
    
        objMeta =getModelMeta(MetaData,cameraData.objectList(obj_i).objectName);

        range = max(0.05,min(0.12,0.4*max(objMeta.dimensions)));
        
        
        [old_id]= find(cameraData.instancemapping==obj_i);
        if ~isempty(old_id)&&ARCUnit.Curr.objectList(old_id).visibility>0.6
            range = 1.1*range;
            valid_warp = label_warps == obj_i; 
        else
            valid_warp = ones(size(label_warps)); 
        end
        
        localRage = [cameraData.objectList(obj_i).Pose(1:3,4)-range,...
                     cameraData.objectList(obj_i).Pose(1:3,4)+range];
        local_ids = combinePoits(:,1)>localRage(1,1)&combinePoits(:,1)<localRage(1,2)&...
                                combinePoits(:,2)>localRage(2,1)&combinePoits(:,2)<localRage(2,2)&... 
                                combinePoits(:,3)>localRage(3,1)&combinePoits(:,3)<localRage(3,2)& combinefgIdx>0;
        validLocal = valid_warp&local_ids;
        
        
        if sum(validLocal)>10
           localcombinePoits = pointCloud(combinePoits(validLocal,:));
           gridStep = 0.004;
           localcombinePoits = pcdownsample(localcombinePoits,'gridAverage',gridStep);
           predObjPose = estimatePoseICP(objModelCloud,info,localcombinePoits,useGPU,cameraData.objectList(obj_i).Pose);
           updateLocal =1;
        else
            predObjPose = cameraData.objectList(obj_i).Pose;
        end
    else
        predObjPose = cameraData.objectList(obj_i).Pose;
    end
    newObjectList(obj_i).Pose = predObjPose;
    ptCloudAligned = transformPointCloud(objModelCloud.Location',predObjPose);
    if useGPU
        [indicesNN,distsNN] = multiQueryKNNSearchImplGPU(pointCloud(ptCloudAligned'),combinePoits);
    else
        [indicesNN,distsNN] = multiQueryKNNSearchImpl(pointCloud(ptCloudAligned'),combinePoits,1);
    end
    combinePoits(distsNN<0.0003,:) =[];
    combinefgIdx(distsNN<0.0003,:) =[];
    label_warps(distsNN<0.0003,:) =[];
end

%{
clf;
vis_point_cloud(combinePoits,'b');
vis_point_cloud(cell2mat(cameraData.points3d)','k');

hold on;
vis_point_cloud(ptCloudAligned','r');
vis_point_cloud(localcombinePoits.Location,'g');
%}

end