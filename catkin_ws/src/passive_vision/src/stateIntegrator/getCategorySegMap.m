function [segMap,segConfMap, categoryThreshold] = getCategorySegMap(cameraData, ARCUnit ,MetaData)
% segMap for each camera 
segConfMap = cell(1,length(cameraData.points3d));
segMap = cell(1,length(cameraData.points3d));
categoryThreshold = zeros(1,length(cameraData.objectList));
for i = 1:length(segConfMap)
    segConfMap{i} = zeros(size(cameraData.color{1},1),size(cameraData.color{1},2),length(cameraData.objectList));
    
end

for obj_id = 1:length(cameraData.objectList)
    confMap = getSegConfience(cameraData,cameraData.objectList(obj_id).objectName, MetaData);
    confMapCombine = cell2mat(confMap);
    categoryThreshold(obj_id) = prctile(confMapCombine(confMapCombine>0.00),97);
    
    for cam_id  = 1:length(segConfMap)
        confMap{cam_id}(confMap{cam_id}<categoryThreshold(obj_id)) =0;
        segConfMap{cam_id}(:,:,obj_id) = confMap{cam_id};
    end
end

 for cam_id  = 1:length(segConfMap)
     [Conf,segMap{cam_id}] = max(segConfMap{cam_id},[],3);
     segMap{cam_id}(Conf<0.001) = 0;
end





end