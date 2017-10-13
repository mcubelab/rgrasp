MetaData = setMetadata();
ARCUnit.cameraIds = {'616205002665','613204000977'};
rootFolder = '1495220550';
imageListFile = ['/n/fs/sun3d/arc/data/state_testdata/images/' rootFolder '/imageList.txt'];

for i = 1:length(ARCUnit.cameraIds )
    imageList{i} = dir(fullfile(MetaData.image_dir,rootFolder,[ARCUnit.cameraIds{i} '/*.color.png']));
end
fid = fopen(imageListFile,'w');
for i = 1:length(imageList{1})
    fprintf(fid,'%s\n',fullfile(MetaData.image_dir,rootFolder,ARCUnit.cameraIds{1},imageList{1}(i).name));
    fprintf(fid,'%s\n',fullfile(MetaData.image_dir,rootFolder,ARCUnit.cameraIds{2},imageList{2}(i).name));
end
fclose(fid);

mkdir(fullfile(MetaData.seg_dir,rootFolder))
cmd = sprintf('iterTest=10001 imgPathFile=%s result_path=%s th test.lua',imageListFile,fullfile(MetaData.seg_dir,rootFolder));
fprintf('%s\n',cmd)