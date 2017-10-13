function confMap = getSegConfience(cameraData,objectName, MetaData)
  ObjIdsInlist = find(ismember({MetaData.CategoryInfo.Name},objectName));
  confMap  =[];
  if ObjIdsInlist>0&&ObjIdsInlist+1<=size(cameraData.segmentLabel{1},3)
      for i = 1:length(cameraData.segmentLabel)
           confMap{i} = cameraData.segmentLabel{i}(:,:,ObjIdsInlist+1);
      end
  end
end