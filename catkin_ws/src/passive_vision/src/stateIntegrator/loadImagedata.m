function [cameraData,succ] = loadImagedata(ImagesData,ARCUnit, MetaData, backgroundImage)
         succ = 0;
         if ~exist('backgroundImage','var')
              backgroundImage =0;
         end
         if strcmp(MetaData.mode,'offline')
            for cam_id = 1:length(ARCUnit.cameraIds)
                filepath = fullfile(MetaData.image_dir,ImagesData{cam_id}{1},ImagesData{cam_id}{2},ImagesData{cam_id}{3});
                if exist([filepath '.depth.png'],'file')&&exist([filepath '.color.png'],'file')

                    cameraData.depth{cam_id} = double(imread([filepath '.depth.png']))/10000;
                    cameraData.color{cam_id} = double(imread([filepath '.color.png']));
                    segfileName = fullfile(MetaData.seg_dir,ImagesData{cam_id}{1},[ImagesData{cam_id}{3}  '.color.png.hd5f']);
                    if exist(segfileName,'file')
                       seg = hdf5read(segfileName,'result');
                       cameraData.segmentLabel{cam_id} = permute(seg(:,:,2:end),[2,1,3]);
                    else
                       cameraData.segmentLabel{cam_id} =[];
                    end
                else
                     cameraData = [];
                     return;
                end
             end
         else
             for cam_id = 1:length(ARCUnit.cameraIds)
                 if backgroundImage
                    filepath = fullfile(MetaData.image_dir,sprintf('%s/passive-vision-background.%s.%d',ImagesData{cam_id}{1},ImagesData{cam_id}{2},cam_id-1));
                 else
                    filepath = fullfile(MetaData.image_dir,sprintf('%s/passive-vision-input.%s.%d',ImagesData{cam_id}{1},ImagesData{cam_id}{2},cam_id-1));
                 end
                 if exist([filepath '.depth.png'],'file')&&exist([filepath '.color.png'],'file')
                    cameraData.depth{cam_id} = double(imread([filepath '.depth.png']))/10000;
                    cameraData.color{cam_id} = double(imread([filepath '.color.png']));
                    cameraData.segmentLabel{cam_id} =[];
                 else
                     fprintf('file not exist: %s \n',filepath);
                     cameraData = [];
                     return;
                end
             end   
         end
         
         for cam_id = 1:length(ARCUnit.cameraIds)
             [~,points3dcam]=read_3d_pts_general(cameraData.depth{cam_id},[], ARCUnit.cameraparam{cam_id}.int);
             points3dcam = points3dcam';
             points3dwrd = bsxfun(@plus,ARCUnit.cameraparam{cam_id}.ext(1:3,1:3)*points3dcam,ARCUnit.cameraparam{cam_id}.ext(1:3,4));

             outOfUintIdx = checkOutUinit(ARCUnit, points3dwrd);
             points3dwrd(:,outOfUintIdx)  = NaN;
             cameraData.depth{cam_id}(reshape(outOfUintIdx,[480,640]))=0;
             cameraData.points3d{cam_id} = points3dwrd; 
         end
         succ = 1;
         
end