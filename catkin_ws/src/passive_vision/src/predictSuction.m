function predictSuction(binId,ARCUnit,showDebug)
t = tic();
if ~exist('binId','var')
    binId = 1;
end
if ~exist('showDebug','var')
    showDebug = 0;
end
dataPath = '/home/mcube/arcdata/tmpdata';
cameraInfoPath = '/home/mcube/arc/catkin_ws/src/passive_vision/camerainfo';

predictions = [];
predObjectNames = {};
predictionsSide =[];
predSideObjectNames ={};
suctionOutputPath = fullfile(dataPath,sprintf('passive-vision-suction.%d.output.bin',binId));
suctionOutputObjectsPath = fullfile(dataPath,sprintf('passive-vision-suction-objects.%d.output.txt',binId));

suctionSideOutputPath = fullfile(dataPath,sprintf('passive-vision-suctionside.%d.output.bin',binId));
suctionSideOutputObjectsPath =  fullfile(dataPath,sprintf('passive-vision-suctionside-objects.%d.output.txt',binId));

 
%{
suctionVisImgPath = cell(1,2);
suctionVisImg = cell(1,2);

camInfoFilePath = fullfile(cameraInfoPath,'bins.txt');
binMiddleBottom = dlmread(camInfoFilePath,' ',[(binId*6+1),1,(binId*6+1),3]);
degFromGravityDir = 40;
rad_degFromGravityDir = deg2rad(degFromGravityDir);

colorMapJet = jet;
for camIdx = 0:1
   
    colorImgPath = fullfile(dataPath,sprintf('passive-vision-input.%d.%d.color.png',binId, camIdx));
    depthImgPath = fullfile(dataPath,sprintf('passive-vision-input.%d.%d.depth.png',binId, camIdx));
    bgColorImgPath = fullfile(dataPath,sprintf('passive-vision-background.%d.%d.color.png',binId, camIdx));
    bgDepthImgPath = fullfile(dataPath,sprintf('passive-vision-background.%d.%d.depth.png',binId, camIdx));
    camKFilePath = fullfile(dataPath,sprintf('passive-vision-camera.%d.%d.intrinsics.txt',binId, camIdx));
    camPoseFilePath = fullfile(dataPath,sprintf('passive-vision-camera.%d.%d.pose.txt',binId, camIdx));
    deepSuctionOutputPath = fullfile(dataPath,sprintf('suction-prediction-output.%d.%d.hdf5',binId,camIdx));
    suctionVisImgPath{camIdx+1} = fullfile(dataPath,sprintf('passive-vision-suction.%d.%d.debug.png',binId, camIdx));

    % Read RGB-D image files
    depthImg = double(imread(depthImgPath))./10000;
    bgColorImg = im2double(imread(bgColorImgPath));
    bgDepthImg = double(imread(bgDepthImgPath))./10000;
    colorImg = imread(colorImgPath);
    camK = dlmread(camKFilePath);
    camPose = dlmread(camPoseFilePath);
    try 
        depthImgFill = fill_depth_cross_bf(imresize(colorImg,0.25), imresize(depthImg,0.25,'nearest')); 
        depthImgFill = imresize(depthImgFill,4);
        depthImg(depthImg==0) = depthImgFill(depthImg==0);
    catch ee
        ee
    end
    colorImg = im2double(colorImg);
    % Read Segmentation 
    try
       instanceSeg = ARCUnit.Curr.instanceseg{camIdx+1};
       objNameList = ['NULL',{ARCUnit.Curr.objectList.objectName}];
       objConfList = [0; [ARCUnit.Curr.objectList.conf]'];
    catch
       objNameList ={'NULL'};
       objConfList = [0];
       instanceSeg = zeros(size(bgDepthImg));
    end
    
    % Do background subtraction
    fgMaskColor = ~(sum(abs(colorImg-bgColorImg) < 0.3,3) == 3);
    fgMaskDepth = bgDepthImg ~= 0 & abs(depthImg-bgDepthImg) > 0.02;
    fgMask = (fgMaskColor | fgMaskDepth);

    % Project depth into camera space
    [~,camX,camY,camZ] = depth2CamPts(depthImg,camK);

    % Only use points with valid depth
    validDepth = find(fgMask & camZ ~= 0);
    %camPts_org = [camX,camY,camZ]';
    camPts = [camX(validDepth),camY(validDepth),camZ(validDepth)]';

    % Get foreground point cloud normals
    fgCloud = pointCloud(camPts');
    %fgCloudGPU = pointCloudGPU(camPts', 'OrganizedLoc', camPts_org, 'UnorganizedToOrganizedInd', validDepth);
    fgNormals = pcnormals(fgCloud,15);
    %fgNormalsGPU = pcnormalsGPU(fgCloudGPU,15);

    % Flip normals to point towards sensor    
    %flip_normal_time = tic;
    p1s = -camPts;  % Matrix of column vectors of points; assume sensor center is [0,0,0]
    p2s = fgNormals'; % Matrix of column vectors of normals; assume number of normal is the same as camPts 
    p1_dot_p2s = sum(p1s .* p2s, 1); % dot products of each p1's and p2's
    flip_ind = find(p1_dot_p2s < 0); % get the indeces that has negative dot product
    fgNormals(flip_ind, :) = -fgNormals(flip_ind, :);
    %fprintf('flip the normals faster '); toc(flip_normal_time);

    % Project normals back to image plane
    pixX = round(camPts(1,:)*camK(1,1)./camPts(3,:)+camK(1,3));
    pixY = round(camPts(2,:)*camK(2,2)./camPts(3,:)+camK(2,3));
    fgNormalsImg = zeros(size(colorImg));
    fgNormalsImg(sub2ind(size(fgNormalsImg),pixY,pixX,ones(size(pixY)))) = fgNormals(:,1);
    fgNormalsImg(sub2ind(size(fgNormalsImg),pixY,pixX,2*ones(size(pixY)))) = fgNormals(:,2);
    fgNormalsImg(sub2ind(size(fgNormalsImg),pixY,pixX,3*ones(size(pixY)))) = fgNormals(:,3);

    % Update invalid points (sum is 0, or far from gravity direction)
    fgNormalsImgX = fgNormalsImg(:,:,1);
    fgNormalsImgY = fgNormalsImg(:,:,2);
    fgNormalsImgZ = fgNormalsImg(:,:,3);
    suctionNormals = camPose(1:3,1:3)*[fgNormalsImgX(:),fgNormalsImgY(:),fgNormalsImgZ(:)]';
%     suctionNormalsZ = suctionNormals(3,:);
%     suctionNormalsZ(sum(fgNormalsImg,3) == 0) = 1;
%     suctionNormals(3,:) = suctionNormalsZ(:)';
    
%     invalidPts = (sum(fgNormalsImg,3) == 0) | ~reshape(acos(suctionNormals(3,:)) < rad_degFromGravityDir,480,640);

%     % Compute standard deviation of local normals
%     meanStdNormals = mean(stdfilt(fgNormalsImg,ones(25,25)),3);
%     suctionConf = 1 - meanStdNormals./max(meanStdNormals(:));
%     suctionConf(invalidPts) = 0;
    
    % Get subsample
%     validPts = ~invalidPts;
%     downsampleMapCell = zeros(8,8);
%     downsampleMapCell(4,4) = 1;
%     validPts = validPts & repmat(downsampleMapCell,size(validPts,1)/8,size(validPts,2)/8);
    
    % Remove points outside bin
    worldPts = (camPose(1:3,1:3)*[camX(:),camY(:),camZ(:)]' + repmat(camPose(1:3,4),1,size(camZ(:),1)))';
    ptsWithinBounds = worldPts(:,1) > (binMiddleBottom(1)-0.27) & worldPts(:,1) < (binMiddleBottom(1)+0.27) & ...
                      worldPts(:,2) > (binMiddleBottom(2)-0.18) & worldPts(:,2) < (binMiddleBottom(2)+0.18) & ...
                      worldPts(:,3) < 0.1;
    acos_suctionNormals = acos(suctionNormals(3,:));
    upNormal = (acos_suctionNormals < rad_degFromGravityDir);
    if binId<2             
       ptsOnTheSide =  abs(worldPts(:,1) - (binMiddleBottom(1)-0.27))< 0.05  | ...
                       abs(worldPts(:,1) - (binMiddleBottom(1)+0.27))< 0.05  | ...
                       abs(worldPts(:,2) - (binMiddleBottom(2)-0.18))< 0.05  | ...
                       abs(worldPts(:,2) - (binMiddleBottom(2)+0.18))< 0.05;
       ptsOnTheSide = ptsOnTheSide & (~upNormal' & suctionNormals(3,:)' > 0);         
       ptsOnTheSide = ptsOnTheSide & ptsWithinBounds;
    else
        
       ptsOnTheSide = zeros(size(worldPts,1),1);
    end           
%     validPts = validPts & reshape(ptsWithinBounds,480,640);
    
%     % Apply gaussian smooth suction confidence
%     suctionConf = imgaussfilt(suctionConf, 20); %6
%     suctionConf(invalidPts) = 0;
%     suctionConf(~ptsWithinBounds) = 0;
    
    % Read deep suction results
    deepSuctionOutput = hdf5read(deepSuctionOutputPath,'output');
    deepSuctionOutput = permute(deepSuctionOutput,[2,1,3]);

    deepSuctionConf = imresize(deepSuctionOutput(:,:,2),8);
    deepSuctionConf(deepSuctionConf >= 1) = 0.9999;
    deepSuctionConf(deepSuctionConf < 0) = 0;
    invalidPts = reshape(~upNormal & ~ptsOnTheSide',480,640);
    
    %invalidPts = imdilate(invalidPts,strel('disk',2));
    deepSuctionConf(invalidPts) = 0;
    deepSuctionConf(~ptsWithinBounds) = 0;
    deepSuctionConf = imgaussfilt(deepSuctionConf, 7);
    %deepSuctionConf(1:2:end,1:2:end) = 0; % remove half of points directly
    suctionConf = deepSuctionConf;
    suctionConf = nmsRange2(suctionConf,4,0.2);
    %suctionConfMax = ordfilt2(suctionConf,5,true(3));
    %MaxInds = abs(suctionConfMax-suctionConf)<0.0000001;
    
    %suctionConf = nmsRange(suctionConf,3,0.1);
    % Save colormap visualization
    colorScale = floor(suctionConf.*size(colorMapJet,1))+1;
    suctionConfVis = permute(reshape(colorMapJet(colorScale(:),:)',3,480,640),[2,3,1]);
    suctionVisImg{camIdx+1} = colorImg.*0.3 + suctionConfVis.*0.7;

    % Get suction points, normals, and confidence
    fgNormalsImgX = fgNormalsImg(:,:,1);
    fgNormalsImgY = fgNormalsImg(:,:,2);
    fgNormalsImgZ = fgNormalsImg(:,:,3);
    
    validPts =  reshape(ptsWithinBounds(:) & upNormal(:),480,640) & suctionConf>0.1;
    suctionNormals = [fgNormalsImgX(validPts),fgNormalsImgY(validPts),fgNormalsImgZ(validPts)];
    suctionPts = [camX(validPts),camY(validPts),camZ(validPts)];
    suctionPtsConf = suctionConf(validPts);
    instancePtsSeg = instanceSeg(validPts);
    % Convert results to world coordinates
    suctionNormals = camPose(1:3,1:3)*suctionNormals';
    suctionPts = camPose(1:3,1:3)*suctionPts' + repmat(camPose(1:3,4),1,size(suctionPts,1));
    
    predictions = [predictions; suctionPts',suctionNormals',suctionPtsConf,objConfList(instancePtsSeg(:)+1)];
    predObjectNames = [predObjectNames, objNameList(instancePtsSeg(:)' + 1) ];
    
    %% suction side 
    side_validPts =  reshape(ptsWithinBounds & ptsOnTheSide,480,640)&suctionConf>0.1;
    side_suctionNormals = [fgNormalsImgX(side_validPts),fgNormalsImgY(side_validPts),fgNormalsImgZ(side_validPts)];
    side_suctionPts = [camX(side_validPts),camY(side_validPts),camZ(side_validPts)];
    side_suctionPtsConf = suctionConf(side_validPts);
    side_instanceSeg = instanceSeg(side_validPts);
    % Convert results to world coordinates
    side_suctionNormals = camPose(1:3,1:3)*side_suctionNormals';
    side_suctionPts = camPose(1:3,1:3)*side_suctionPts' + repmat(camPose(1:3,4),1,size(side_suctionPts,1));
    
    predictionsSide = [predictionsSide; side_suctionPts',side_suctionNormals',side_suctionPtsConf,objConfList(side_instanceSeg(:)+1)];
    predSideObjectNames = [predSideObjectNames, objNameList(side_instanceSeg(:)'+1)];
    
    % % Debug
    % figure(); pcshow(fgCloud); hold on;
    % for i = 1:(size(predictions,1))
    %     fprintf('%d/%d\n',i,size(predictions,1));
    %     quiver3(predictions(i,1),predictions(i,2),predictions(i,3),predictions(i,4),predictions(i,5),predictions(i,6),'LineWidth',3,'MaxHeadSize',1,'AutoScaleFactor',0.1);
    % end

end

if ~isempty(predictions)
    
    % Slightly bias suction predictions based on location (higher = better)
    predictions(:,7) = 0.8 * predictions(:,7) + min(max(predictions(:,3) - binMiddleBottom(3),0),0.2);
    
    % Sort suction predictions by confidence
    [~,suctionSortInd] = sortrows(predictions,-7);
    predictions = predictions(suctionSortInd,:);
    predObjectNames = predObjectNames(suctionSortInd)';
    
    % Use only the top 10k suction predictions
    maxnumberofPoints = 10000;
    predictions = predictions(1:min(maxnumberofPoints,size(predictions,1)),:);  
    predObjectNames = predObjectNames(1:min(maxnumberofPoints,length(predObjectNames)));
    
    % do the same for suction side
    [~,suctionSortInd] = sortrows(predictionsSide,-7);
    predictionsSide = predictionsSide(suctionSortInd,:);
    predSideObjectNames = predSideObjectNames(suctionSortInd)';
    predictionsSide = predictionsSide(1:min(maxnumberofPoints,size(predictionsSide,1)),:);  
    predSideObjectNames = predSideObjectNames(1:min(maxnumberofPoints,length(predSideObjectNames)));
end
if showDebug
    % Draw top 2 suction points
    if size(predictions,1) >= 2 
        for camIdx = 0:1
            camKFilePath = fullfile(dataPath,sprintf('passive-vision-camera.%d.%d.intrinsics.txt',binId, camIdx));
            camPoseFilePath = fullfile(dataPath,sprintf('passive-vision-camera.%d.%d.pose.txt',binId, camIdx));

            % Read RGB-D image files
            camK = dlmread(camKFilePath);
            camPose = dlmread(camPoseFilePath);
            world2Cam = inv(camPose);

            suctionPts = predictions(1:2,1:3);
            suctionPts = world2Cam(1:3,1:3)*suctionPts' + repmat(world2Cam(1:3,4),1,size(suctionPts,1));
            pixX = suctionPts(1,:)*camK(1,1)./suctionPts(3,:)+camK(1,3);
            pixY = suctionPts(2,:)*camK(2,2)./suctionPts(3,:)+camK(2,3);

            suctionVisImg{camIdx+1} = insertShape(suctionVisImg{camIdx+1},'circle',[pixX(1) pixY(1) 5],'LineWidth',10,'Color','white');
  
            imwrite(suctionVisImg{camIdx+1},[suctionVisImgPath{camIdx+1},'.ppm']);
            ppmtopng_and_remove_ppm_nowait([suctionVisImgPath{camIdx+1},'.ppm'], suctionVisImgPath{camIdx+1});
        end
    end
end
%}
% Save prediction data in row major order
predictions = predictions';
fileID = fopen(suctionOutputPath,'wb');
fwrite(fileID,[size(predictions,2);predictions(:)],'single');	
fclose(fileID);

fileID = fopen(suctionOutputObjectsPath,'w');
for i = 1:length(predObjectNames)
    fprintf(fileID,'%s\n',predObjectNames{i});
end
fclose(fileID);


predictionsSide = predictionsSide';
fileID = fopen(suctionSideOutputPath,'wb'); 
fwrite(fileID,[size(predictionsSide,2);predictionsSide(:)],'single');	
fclose(fileID);

fileID = fopen(suctionSideOutputObjectsPath,'w');
for i = 1:length(predSideObjectNames)
    fprintf(fileID,'%s\n',predSideObjectNames{i});
end
fclose(fileID);

% % Debug
% pcwrite(pointCloud(predictions(1:3,:)'),fullfile(dataPath,'test'),'PLYformat','binary');

fprintf('[MATLAB Timing PredictSuction]: '); toc(t);

% imagesc(suctionVisImg);


end
