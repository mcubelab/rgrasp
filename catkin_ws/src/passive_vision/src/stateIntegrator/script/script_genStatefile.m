mode = 'debug';
MetaData = setMetadata(mode);
MetaData.image_dir = '~/arcdata/loopdata/seq19_comb/';
statefile = [MetaData.image_dir '/states.txt'];
allfolders = dir(fullfile(MetaData.image_dir,'1*'));
fid = fopen(statefile,'w');
commandline_prev = '1 - -';
for i = 1:length(allfolders)
    
    folderName = allfolders(i).name;
    clf;
    for binId = 0:3
        subplot(1,4,binId+1)
        filepath = fullfile(MetaData.image_dir,sprintf('%s/passive-vision-input.%d.%d.depth.png',...
                            folderName,binId,0));
        if exist(filepath,'file')
           try imagesc(imread(filepath)); end
        end
    end
    %{
    prompt={'binid','cmd','objectName'};
    answer=inputdlg(prompt);
    fprintf(fid,'%s %s %s %s\n',folderName, answer{1},answer{2},answer{3});
    %}
    if exist(fullfile(MetaData.image_dir,sprintf('%s/state-command.txt',folderName)),'file')
       fid_cmd= fopen( fullfile(MetaData.image_dir,sprintf('%s/state-command.txt',folderName)),'r');
       commandline_curr = fgets(fid_cmd);
       commandline_curr = commandline_curr(1:end-1);
       fclose(fid_cmd);
       if ~strcmp(commandline_curr,commandline_prev)
           fprintf(fid,'%s %s\n',folderName, commandline_curr);
           fprintf('%s %s\n',folderName, commandline_curr);
           pause;
       end
       commandline_prev = commandline_curr;
    end
end
fclose(fid);

%{
MetaData = setMetadata();
ACRUnit.cameraIds = {'616205002665','613204000977'};
rootFolder = '1495220550';
statefile = ['/n/fs/sun3d/arc/data/state_testdata/images/' rootFolder '/states.txt'];

for i = 1:length(ACRUnit.cameraIds )
    imageList{i} = dir(fullfile(MetaData.image_dir,rootFolder,[ACRUnit.cameraIds{i} '/*.color.png']));
end
fid = fopen(statefile,'w');

for i = 1:length(imageList{1})
    clf;
    if i>1
    subplot(2,1,1);
    imshow(fullfile(MetaData.image_dir,rootFolder,ACRUnit.cameraIds{1},imageList{1}(i-1).name))
    end
    subplot(2,1,2);
    imshow(fullfile(MetaData.image_dir,rootFolder,ACRUnit.cameraIds{1},imageList{1}(i).name))
    prompt={'cmd','objectName'};
    answer=inputdlg(prompt);
    imageName1 = imageList{1}(i).name(1:end-length('.color.png'));
    imageName2 = imageList{2}(i).name(1:end-length('.color.png'));
    fprintf(fid,'%s %s %s %s %s %s %s %s\n',rootFolder, ACRUnit.cameraIds{1},imageName1, ...
                                   rootFolder, ACRUnit.cameraIds{2},imageName2,...
                                   answer{1},answer{2});
end
fclose(fid);
%}