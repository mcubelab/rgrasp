function [heightMap,heightMapSeg,newObjectList] = checkVisibility(cameraData, ARCUnit, MetaData)
     newObjectList = cameraData.objectList;
     allModelPots = cell(1,length(cameraData.objectList));
     allModelIds = cell(1,length(cameraData.objectList));
     for i = 1:length(cameraData.objectList)
         objModelCloud =getModelPC(MetaData,cameraData.objectList(i).objectName,1);
         ptCloudAligned = transformPointCloud(objModelCloud.Location',cameraData.objectList(i).Pose);
         allModelPots{i} = double(ptCloudAligned);
         allModelIds{i} = i*ones(1,length(ptCloudAligned));
     end
     [heightMap,heightMapSeg] = projHeightMap(cell2mat(allModelPots),cell2mat(allModelIds),ARCUnit);
     heightMapSeg = imclose(heightMapSeg,strel('disk',2));
     
     heightMapObjs = cell(1,cameraData.numodobject);
     allvisibility = zeros(1,cameraData.numodobject);
     
     for obj_i = 1:cameraData.numodobject    
         heightMapObjs{obj_i} = projHeightMap(allModelPots{obj_i},allModelIds{obj_i},ARCUnit);
         heightMapObjs{obj_i} = imclose(~isnan(heightMapObjs{obj_i}),strel('disk',2));
         allvisibility(obj_i) = sum(heightMapSeg(:) ==obj_i)/sum(heightMapObjs{obj_i}(:));
     end
     allvisibility(isnan(allvisibility)) = 0;
     for obj_i = 1:cameraData.numodobject    
         heightMapObj = heightMapObjs{obj_i};
         objMask =  heightMapSeg == obj_i;
         occluderIds = heightMapSeg(heightMapObj&~objMask);
         occluderIds = occluderIds(occluderIds>0);
         occluderIds_u = unique(occluderIds);
         topobj =[];
         for j = 1:length(occluderIds_u)
             
             % check 1 : occluder is mostly visible = visible/fullprojection > threshlod 
             check1 = allvisibility(occluderIds_u(j))>0.6;
             % check 2: occluded siginificant amount 
             threshould =min(0.3*sum(heightMapObj(:)),0.7*sum(heightMapObjs{occluderIds_u(j)}(:)));
             check2 = sum(occluderIds==occluderIds_u(j))>threshould;
             %put in later 
             check3 = occluderIds_u(j)>obj_i;
             if check2 && check1 &&check3
               topobj(end+1) = occluderIds_u(j);
             end
         end
         newObjectList(obj_i).topobj = topobj;
         newObjectList(obj_i).visibility =allvisibility(obj_i);
     end
         
    %{
    % check 2 view project 
    for obj_i = 1:cameraData.numodobject
        visible_view = 0;
        occluder = cell(1,length(ACRUnit.cameraIds));
        for cam_i = 1:length(ACRUnit.cameraIds)
            [depth_obj]=proj3DPtsTo2D(allModelPots{obj_i},ACRUnit.cameraparam{cam_i}.ext,ACRUnit.cameraparam{cam_i}.int,imSize,[]);
            objproject = imclose(~isnan(depth_obj),strel('disk',4));
            objMask =  instanceseg{cam_i} == obj_i;
            occluderIds = instanceseg{cam_i}(objproject&~objMask);
            occluderIds = occluderIds(occluderIds>0);
            occluderIds_u = unique(occluderIds);
            for j = 1:length(occluderIds_u)
                threshould =min(500,sum(instanceseg{cam_i}(:) == occluderIds_u(j)));
                if sum(occluderIds==occluderIds_u(j))>threshould
                   occluder{cam_i}(end+1) = occluderIds_u(j);
                end
            end
            visible_view = visible_view+sum(objproject(:)&objMask(:))/sum(objproject(:));
        end

        common = find(ismember(occluder{1},occluder{end}));
        topobj = {};
        for i = 1:length(common)
            topobj{end+1} =cameraData.objectList(occluder{1}(common(i))).objectName;
        end
        newObjectList(obj_i).topobj = topobj;
        newObjectList(obj_i).visibility = visible_view/length(ACRUnit.cameraIds);
    end
    %}
end