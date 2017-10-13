function [allBinsData,allBinName] = readbinfile(MetaData)
allBinsData = {};
fid = fopen(MetaData.uintfile,'r');
tline = fgets(fid);
numofbin =0;
while ischar(tline)
    alldata = strsplit(tline);
    if ~isempty(alldata)
        if strcmp(alldata{1},'bin')
            
            numofbin = numofbin+1;
            
            allBinsData{numofbin}.name = alldata{2};
            allBinsData{numofbin}.cameraIds = cell(1,2);
            allBinsData{numofbin}.Tote.Size = [0.6,0.4,0.22];
            allBinsData{numofbin}.heightmapSize = [300,200];
            
        elseif strcmp(alldata{1},'pose')
            allBinsData{numofbin}.Tote.Center = [str2double(alldata{2}),str2double(alldata{3}),...
                                                 str2double(alldata{4})+ allBinsData{numofbin}.Tote.Size(3)*0.5];
            allBinsData{numofbin}.Tote.Range = [allBinsData{numofbin}.Tote.Center-0.5*allBinsData{numofbin}.Tote.Size,...
                                                allBinsData{numofbin}.Tote.Center+0.5*allBinsData{numofbin}.Tote.Size];
            allBinsData{numofbin}.heightmapGridSize =  0.002;%(allBinsData{numofbin}.Tote.Range(4)-allBinsData{numofbin}.Tote.Range(1))/allBinsData{numofbin}.heightmapSize(1);
        
        elseif strcmp(alldata{1},'tf')
            allBinsData{numofbin}.cameraIds{1} = alldata{2};
        elseif strcmp(alldata{1},'tn')
            allBinsData{numofbin}.cameraIds{2} = alldata{2};
        end
    end
    tline = fgets(fid);
end

fclose(fid);
allBinName = cell(1,length(allBinsData));
for bin_id = 1:length(allBinsData)
    allBinName{bin_id} = allBinsData{bin_id}.name;
    for i = 1:length(allBinsData{bin_id}.cameraIds)
        camposefile = fullfile(MetaData.camera_dir ,[allBinsData{bin_id}.cameraIds{i} '.pose.txt']);
        camKfile = fullfile(MetaData.camera_dir ,[allBinsData{bin_id}.cameraIds{i} '.intrinsics.txt']);
        if exist(camposefile,'file')
            camera.ext = dlmread(camposefile);
            camera.ext = camera.ext(1:4,1:4);
            camera.int = dlmread(camKfile);
            camera.int = camera.int(1:3,1:3);
            allBinsData{bin_id}.cameraparam{i} = camera;
            allBinsData{bin_id}.active = 1;
        else
            allBinsData{bin_id}.active = 0;
        end
        allBinsData{bin_id}.binconf = 1;
    end
end

end