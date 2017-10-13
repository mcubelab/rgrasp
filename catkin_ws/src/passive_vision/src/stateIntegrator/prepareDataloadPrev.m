function prepareDataloadPrev(allBinsData,MetaData)
        for bin_id = 1:length(allBinsData)
             if allBinsData{bin_id}.active
                 %{
                 for cam_id = 1:2
                    bgfilepath = fullfile(MetaData.image_dir,sprintf('/passive-vision-background.%d.%d',bin_id-1,cam_id-1));
                    inputfilepath = fullfile(MetaData.image_dir,sprintf('/passive-vision-input.%d.%d',bin_id-1,cam_id-1)); 

                    imwrite(uint8(allBinsData{bin_id}.BG.color{cam_id}),[bgfilepath '.color.png']);
                    imwrite(uint16(allBinsData{bin_id}.BG.depth{cam_id}*10000),[bgfilepath '.depth.png']);
                    try imwrite(uint8(allBinsData{bin_id}.Curr.color{cam_id}),[inputfilepath '.color.png']); end
                    try imwrite(uint16(allBinsData{bin_id}.Curr.depth{cam_id}),[inputfilepath '.depth.png']); end
                 end
                %}
                 fprintf('[ MATLAB ]: load state for bin %d.\n',bin_id-1);
                 
                 filename{1} = fullfile(MetaData.image_dir,sprintf('passive-vision-state.%d.%d.debug.png',bin_id-1,0));
                 filename{2} = fullfile(MetaData.image_dir,sprintf('passive-vision-state.%d.%d.debug.png',bin_id-1,1));
                 param = struct('show2D',1,'show3D',0,'pause',0,'saveImage',1);
                 visulizeState(allBinsData{bin_id},MetaData,param,filename);       
             end
        end
end