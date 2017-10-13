function [newObjectList] = getConfidence(cameraData, ARCUnit)
%{
% check the depth differece
weight = [0.3,0.7];
validmask = cell(1,length(ARCUnit.cameraIds));
for i = 1:length(ARCUnit.cameraIds)
    depthdiffMap = abs(cameraData.depth_comb{i}-cameraData.depthprojected{i});
    validmask{i} = depthdiffMap<0.02|(cameraData.depth_comb{i}==0&cameraData.depthprojected{i}>0); 
    conf_Map = 1-depthdiffMap;
    conf_Map(conf_Map<0) = 0;
    confMap{i} = conf_Map;
end

objconf_depthDiff = zeros(1,cameraData.numodobject);
for obj_i = 1:cameraData.numodobject
    valid_pix = 0;
    total_pix = 0;
    for cam_i = 1:length(ARCUnit.cameraIds)
        objMask =  cameraData.instanceseg{cam_i} == obj_i;
        valid_pix = valid_pix+sum(validmask{cam_i}(:)&objMask(:));
        total_pix = total_pix+sum(objMask(:));
    end
    objconf_depthDiff(obj_i) = valid_pix./total_pix;
end

objconf_seg = zeros(1,cameraData.numodobject);
if isfield(cameraData,'categorySegMap')
   for obj_i = 1:cameraData.numodobject
       allconf =[];
       
       for cam_i = 1:length(ARCUnit.cameraIds)
           viewconf = cameraData.categorySegConfMap{cam_i}(:,:,obj_i);
           allconf = [allconf(:);viewconf(cameraData.instanceseg{cam_i} == obj_i)];
       end
       objconf_seg(obj_i) = prctile(allconf,95);
   end 
end

objconf_depthDiff(isnan(objconf_depthDiff)) = 0.0001;
objconf_seg(isnan(objconf_seg)) = 0.0001;

newObjectList = cameraData.objectList;
for obj_i = 1:cameraData.numodobject
    newObjectList(obj_i).conf = weight(1)*objconf_depthDiff(obj_i)...
                                +weight(2)*objconf_seg(obj_i);
end
%}
newObjectList = cameraData.objectList;
for obj_i = 1:cameraData.numodobject
    newObjectList(obj_i).conf = ARCUnit.binconf;
end

end