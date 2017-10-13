function writeStateTofile(MetaData,ARCUnit,bin_id,stateBinaryOutputPath,stateTextOutputPath)
dataPath = MetaData.image_dir;
% stateBinaryOutputPath = fullfile(dataPath,sprintf('passive-vision-state.%d.output.bin',bin_id-1));
% stateTextOutputPath = fullfile(dataPath,sprintf('passive-vision-state.%d.output.txt',bin_id-1));
objectState = zeros(9,ARCUnit.Curr.numodobject);

for i = 1:ARCUnit.Curr.numodobject
    objectState(:,i) = [rotm2quat(ARCUnit.Curr.objectList(i).Pose(1:3,1:3))';...
                        ARCUnit.Curr.objectList(i).Pose(1:3,4);...
                        ARCUnit.Curr.objectList(i).conf;...
                        ARCUnit.Curr.objectList(i).visibility];
end

fileID = fopen(stateBinaryOutputPath,'wb');
fwrite(fileID,[size(objectState,2);objectState(:)],'single');	
fclose(fileID);

fileID = fopen(stateTextOutputPath,'w');
for i = 1:ARCUnit.Curr.numodobject
    fprintf(fileID,'%s ',ARCUnit.Curr.objectList(i).objectName);
    fprintf(fileID,'%d ',length(ARCUnit.Curr.objectList(i).topobj));
    for j = 1:length(ARCUnit.Curr.objectList(i).topobj)
        fprintf(fileID,'%s ',ARCUnit.Curr.objectList(ARCUnit.Curr.objectList(i).topobj(j)).objectName);
    end         
end
fclose(fileID);

end