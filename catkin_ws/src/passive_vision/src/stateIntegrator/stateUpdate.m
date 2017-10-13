function [ARCUnit,succ] = stateUpdate(arcmesg, ARCUnit,MetaData,fastMode,useGPU)
    [cameraData,succ] = loadImagedata(arcmesg.imagesData,ARCUnit, MetaData);
    if ~succ
        fprintf('[stateUpdate] FAIL TO LOAD IMAGES!!! NO UPDATES\n');
        return;
    end
    
    %% initialize new state with new images and old object state
    totalStateUpdate = tic;
    cameraData = addImagedataInfo(cameraData,ARCUnit, MetaData,useGPU);
    cameraData.numodobject =  ARCUnit.Curr.numodobject;
    if isfield(ARCUnit.Curr,'objectList')
       cameraData.objectList = ARCUnit.Curr.objectList;
       cameraData.instancemapping = 1:cameraData.numodobject;
    end
    %% update the objectList
    ignoredObjectIds = [];
    
    if strcmp(arcmesg.mode,'add')
        [newobject,succ] = AddOneObject(arcmesg,cameraData, ARCUnit,MetaData,useGPU);
        cameraData.objectList(cameraData.numodobject+1) = newobject;
        ignoredObjectIds = cameraData.numodobject+1;
        cameraData.numodobject = cameraData.numodobject+1;
        if ~fastMode 
            ARCUnit.binconf = 0.9*ARCUnit.binconf;
        end
    elseif strcmp(arcmesg.mode,'rm')
        [removeId,succ] = RemoveOneObject(arcmesg,cameraData, ARCUnit);
        cameraData.objectList(removeId) = [];
        cameraData.numodobject = cameraData.numodobject-1;
        cameraData.instancemapping(removeId) = 0;
        cameraData.instancemapping(removeId+1:end) = cameraData.instancemapping(removeId+1:end)-1;
        ARCUnit.binconf = 0.9*ARCUnit.binconf;
    elseif strcmp(arcmesg.mode,'mv')
        [changeId,newobject,succ] = MoveOneObject(arcmesg,cameraData, ARCUnit,MetaData,useGPU);
        if succ
           cameraData.objectList(changeId) = [];
           cameraData.objectList(cameraData.numodobject) = newobject;
           ignoredObjectIds = cameraData.numodobject;
           ARCUnit.binconf = 0.8*ARCUnit.binconf;
        end
    end

    %% update the state objectList won't change length in the following operation
    if ~isempty(cameraData.segmentLabel{1})
        [categorySegMap,categorySegConfMap,categoryThreshold] = getCategorySegMap(cameraData, ARCUnit ,MetaData);
        cameraData.categorySegMap = categorySegMap;
        cameraData.categorySegConfMap = categorySegConfMap;
        cameraData.categoryThreshold = categoryThreshold;
    end
    
    %% Local update and post processing 
    if cameraData.numodobject>0
        [instanceseg,depthprojected] = getInstanceSegmentMaskProj(cameraData, ARCUnit, MetaData);
        cameraData.instanceseg = instanceseg;
        cameraData.depthprojected = depthprojected;

        [heightMapProjected,heightMapSegProjected,objectList_vis] = checkVisibility(cameraData, ARCUnit, MetaData);
        cameraData.objectList = objectList_vis;
        cameraData.heightMapProjected = heightMapProjected;
        cameraData.heightMapSegProjected = heightMapSegProjected;


        [newObjectList,updateLocal] = LocalUpdate(cameraData, ARCUnit,MetaData, ignoredObjectIds, useGPU);
        %[newObjectList,updateLocal,sceneChange] = LocalUpdateSIFTflow(cameraData, ARCUnit,MetaData,ignoredObjectIds, useGPU);

        if updateLocal
            cameraData.objectList = newObjectList;
            [instanceseg,depthprojected] = getInstanceSegmentMaskProj(cameraData, ARCUnit, MetaData);
            cameraData.instanceseg = instanceseg;
            cameraData.depthprojected = depthprojected;

            [heightMapProjected,heightMapSegProjected,objectList_vis] = checkVisibility(cameraData, ARCUnit, MetaData);
            cameraData.objectList = objectList_vis;
            cameraData.heightMapProjected = heightMapProjected;
            cameraData.heightMapSegProjected = heightMapSegProjected;
            ARCUnit.binconf = 0.7*ARCUnit.binconf;
        end



        [newObjectList] = getConfidence(cameraData, ARCUnit);
        cameraData.objectList = newObjectList;
        
        if isfield(cameraData,'instanceseg')
           for camId = 1:length(cameraData.instanceseg)
               newinstanceseg = zeros(size(cameraData.instanceseg{camId}));
               for cid = 1:length(cameraData.objectList)
                   newinstanceseg(imerode(cameraData.instanceseg{camId}==cid,strel('disk',3))) = cid;
               end
               cameraData.instanceseg{camId} = newinstanceseg;
           end
        end
        %}
        assert(cameraData.numodobject==length(cameraData.objectList));
    end
    fprintf('[MATLAB stateUpdate ] '); toc(totalStateUpdate);
    

    %% update the state
    ARCUnit.Curr = cameraData;
    ARCUnit.num_step = ARCUnit.num_step+1;
    %StateHistory(end+1) = sceneChange;
end
