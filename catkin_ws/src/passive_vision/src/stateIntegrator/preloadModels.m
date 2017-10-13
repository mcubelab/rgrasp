function MetaData = preloadModels(MetaData)
    enumeration = tdfread(MetaData.categorylist,',');
    for objId = 1:size(enumeration.FolderName,1)
        objectFolderName = enumeration.FolderName(objId,:);
        objectFolderName(objectFolderName ==' ') = [];
        objectName = lower(objectFolderName);
        MetaData.CategoryInfo(objId).Name = objectName;
        MetaData.CategoryInfo(objId).Folderame = objectFolderName;
        MetaData.CategoryInfo(objId).itemprodimag = dir(fullfile(MetaData.category_dir,objectFolderName,['/*.png']));
        matfilename = dir(fullfile(MetaData.category_dir,objectFolderName,['/*_bdir.mat']));
        tmp = load(fullfile(MetaData.category_dir,objectFolderName,matfilename(1).name) );
        MetaData.CategoryInfo(objId).objModelCloud  = tmp.objModelCloud;
        MetaData.CategoryInfo(objId).info  = tmp.info;
        MetaData.CategoryInfo(objId).objMeta = loadjson(fullfile(MetaData.category_dir,objectFolderName,[objectName '.json']) );
    end
end