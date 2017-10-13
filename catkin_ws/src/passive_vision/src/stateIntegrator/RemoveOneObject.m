function [removeId,succ] = RemoveOneObject( message,cameraData, ARCUnit)
         % check whether it is the only one 
         succ = 0;
         ObjIdsInlist = find(ismember({cameraData.objectList.objectName},message.objectName));
         if isempty(ObjIdsInlist)
             fprintf('There is NO this object in the uint!\n')
             removeId =[];
             
         elseif length(ObjIdsInlist) ==1
             removeId = ObjIdsInlist;
             succ = 1;
         else
             % find the remove points   
             removedpts = cell(1,length(ARCUnit.cameraIds));
             for i = 1:length(ARCUnit.cameraIds)
                 changemask = abs(cameraData.depth{i}-ARCUnit.Curr.depth{i})>0.02&...
                              sum(abs(cameraData.color{i}-ARCUnit.Curr.color{i}),3)>50;
                 removelocation = changemask(:)&(cameraData.points3d{i}(3,:)' < ARCUnit.Curr.points3d{i}(3,:)');
                 removedpts{i} = ARCUnit.Curr.points3d{i}(:,removelocation');
             end
             removedpts = cell2mat(removedpts);
             centerofchange = median(removedpts,2);
             distance = zeros(1,length(ObjIdsInlist));
             for i = 1:length(ObjIdsInlist)
                 c1 = cameraData.objectList(ObjIdsInlist(i)).Pose(1:3,4);
                 distance(i) = sum((c1-centerofchange).*(c1-centerofchange));
             end
             [~,closer_id] = min(distance);
             removeId = ObjIdsInlist(closer_id);
             succ = 1;
         end
            
        
         
end