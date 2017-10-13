function objMeta = getModelMeta(MetaData,objectName)
         [~,cid]=ismember(objectName,{MetaData.CategoryInfo.Name});
         objMeta = MetaData.CategoryInfo(cid).objMeta;

end