function [objModelCloud,info,objMeta] =getModelPC(MetaData,objectName,dense)
         if ~exist('dense','var')
            dense = 0;
         end
         [~,cid]=ismember(objectName,{MetaData.CategoryInfo.Name});
         objModelCloud = MetaData.CategoryInfo(cid).objModelCloud;
         info = MetaData.CategoryInfo(cid).info;
         objMeta = MetaData.CategoryInfo(cid).objMeta;
         if ~dense
            gridStep = 0.002;
            objModelCloud = pcdownsample(objModelCloud,'gridAverage',gridStep);
         end
end