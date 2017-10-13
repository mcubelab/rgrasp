function cameraData = addImagedataInfo(cameraData,ARCUnit, MetaData,useGPU)
        for cam_i = 1:length(ARCUnit.cameraIds)
           % fill in depth from the other view 
            for cam_j = 1:length(ARCUnit.cameraIds)
                if cam_j~=cam_i
                   [depth_projected]=proj3DPtsTo2D(cameraData.points3d{cam_j},...
                                            ARCUnit.cameraparam{cam_j}.ext,ARCUnit.cameraparam{cam_j}.int,...
                                            size(cameraData.depth{cam_j}),[]);
                                        
                   cameraData.depth_comb{cam_i} = cameraData.depth{cam_i};
                   cameraData.depth_comb{cam_i}(cameraData.depth{cam_i}==0)=depth_projected(cameraData.depth{cam_i}==0);                     
                end
            end
        end
        
        for cam_i = 1:length(ARCUnit.cameraIds)
            bgmask = abs(cameraData.depth{cam_i}-ARCUnit.BG.depth{cam_i})<0.015&...
                         sum(abs(cameraData.color{cam_i}-ARCUnit.BG.color{cam_i}),3)<100;
            
            cameraData.fgIdx{cam_i} = ~bgmask(:);
        end
            
        %{
        % 3D back ground removeal
        combinePoitsBG = cell2mat(ACRUnit.BG.points3d);
        combinePoitsBG = combinePoitsBG(:,~isnan(combinePoitsBG(1,:)));

        for cam_i = 1:length(ACRUnit.cameraIds)
            % fill in depth from the other view 
            for cam_j = 1:length(ACRUnit.cameraIds)
                if cam_j~=cam_i
                   [depth_projected]=proj3DPtsTo2D(cameraData.points3d{cam_j},...
                                            ACRUnit.cameraparam{cam_j}.ext,ACRUnit.cameraparam{cam_j}.int,...
                                            size(cameraData.depth{cam_j}),[]);
                                        
                   cameraData.depth_comb{cam_i} = cameraData.depth{cam_i};
                   cameraData.depth_comb{cam_i}(cameraData.depth{cam_i}==0)=depth_projected(cameraData.depth{cam_i}==0);                     
                end
            end
            % back ground removal
            validInds = find(~isnan(cameraData.points3d{cam_i}(1,:)));
            if useGPU
                [indicesNN,distsNN] = multiQueryKNNSearchImplGPU(pointCloud(combinePoitsBG'),cameraData.points3d{cam_i}(:,validInds)');
            else
                [indicesNN,distsNN] = multiQueryKNNSearchImpl(pointCloud(combinePoitsBG'),cameraData.points3d{cam_i}(:,validInds)',1);
            end
            cameraData.fgIdx{cam_i} = ones(1,length(cameraData.points3d{cam_i}));
            cameraData.fgIdx{cam_i}(validInds(distsNN<0.0004)) = 0;
            
             
             % compute height map
             %{
             points3dwrd = cameraData.points3d{i};
             points3dwrd = points3dwrd(:,~isnan(points3dwrd(1,:)));
             grid = round(bsxfun(@minus,points3dwrd([1:2],:),ACRUnit.Tote.Range([1,2])')/ACRUnit.heightmapGridSize);
             %vis_point_cloud(points3dwrd')
             valid = grid(1,:)>0&grid(1,:)<=ACRUnit.heightmapSize(1)&grid(2,:)>0&grid(2,:)<=ACRUnit.heightmapSize(2);
             cameraData.heightmap(sub2ind(ACRUnit.heightmapSize,grid(1,valid(:)),grid(2,valid(:)))) = points3dwrd(3,valid(:));
             %}
        end
        %}
             
end