function allBinsData = initBinBG(allBinsData,MetaData,arcmesg)
         for bin_id = 1:length(allBinsData)
             if allBinsData{bin_id}.active
                 if strcmp(MetaData.mode,'offline')
                    imagesData = arcmesg.imagesData;
                 elseif strcmp(MetaData.mode,'debug')
                    imagesData{1} = {arcmesg.imagesData{1}{1}, allBinsData{bin_id}.name,'0'};
                    imagesData{2} = {arcmesg.imagesData{1}{1}, allBinsData{bin_id}.name,'1'};
                 else
                    imagesData{1} = {'/', allBinsData{bin_id}.name,'0'};
                    imagesData{2} = {'/', allBinsData{bin_id}.name,'1'};
                 end
                 [cameraData,succ] = loadImagedata(imagesData,allBinsData{bin_id}, MetaData,1);
             else
                 succ = 0;
             end
             
             if succ
                cameraData.numodobject = 0;
                allBinsData{bin_id}.BG = cameraData;
                allBinsData{bin_id}.Curr = cameraData;
                allBinsData{bin_id}.active = 1;
                fprintf('BIN %s is active\n',allBinsData{bin_id}.name)
             else
                allBinsData{bin_id}.active = 0;
                fprintf('BIN %s is inactive\n',allBinsData{bin_id}.name)
             end
             allBinsData{bin_id}.num_step = 0;
         end  

end