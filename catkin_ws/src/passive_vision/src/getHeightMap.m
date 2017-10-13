function getHeightMap(binId)

if ~exist('binId','var')
    binId = 0;
end

dataPath = '/home/mcube/arcdata/tmpdata';
% dataPath = '/home/mcube/arcdata/loopdata/passive_vision/1497965159';
cameraInfoPath = '/home/mcube/arc/catkin_ws/src/passive_vision/camerainfo';

heightMapOutputPath = fullfile(dataPath,sprintf('passive-vision-height-map.%d.output.bin',binId));
heightmapColorFile = fullfile(dataPath,sprintf('passive-vision-height.%d.color.png',binId));
heightmapDepthFile = fullfile(dataPath,sprintf('passive-vision-height.%d.depth.png',binId));

heightMaps = {};
missingHeightMaps = {};

voxelSize = 0.002;
heightMapColor = zeros(200*300,3);
for camIdx = 0:1
    colorImgPath = fullfile(dataPath,sprintf('passive-vision-input.%d.%d.color.png',binId,camIdx));
    depthImgPath = fullfile(dataPath,sprintf('passive-vision-input.%d.%d.depth.png',binId,camIdx));
    bgColorImgPath = fullfile(dataPath,sprintf('passive-vision-background.%d.%d.color.png',binId,camIdx));
    bgDepthImgPath = fullfile(dataPath,sprintf('passive-vision-background.%d.%d.depth.png',binId,camIdx));
    camKFilePath = fullfile(dataPath,sprintf('passive-vision-camera.%d.%d.intrinsics.txt',binId,camIdx));
    camPoseFilePath = fullfile(dataPath,sprintf('passive-vision-camera.%d.%d.pose.txt',binId,camIdx));
    camInfoFilePath = fullfile(cameraInfoPath,'bins.txt');

    binMiddleBottom = dlmread(camInfoFilePath,' ',[(binId*6+1),1,(binId*6+1),3]);
    
    
    % Read RGB-D image files
    colorImg = double(imread(colorImgPath))./255;
    depthImg = double(imread(depthImgPath))./10000;
    bgColorImg = double(imread(bgColorImgPath))./255;
    bgDepthImg = double(imread(bgDepthImgPath))./10000;
    camK = dlmread(camKFilePath);
    camPose = dlmread(camPoseFilePath);

    % Do background subtraction
    fgMaskColor = ~(sum(abs(colorImg-bgColorImg) < 0.3,3) == 3);
    fgMaskDepth = bgDepthImg ~= 0 & abs(depthImg-bgDepthImg) > 0.02;
    fgMask = (fgMaskColor | fgMaskDepth);

    [camPts,~,~,camZ] = depth2CamPts(depthImg,camK);
    %
    % Transform points to world coordinates
    worldPts = (camPose(1:3,1:3)*camPts' + repmat(camPose(1:3,4),1,size(camPts,1)))';
    
    % Get height map
    heightMap = zeros(200,300);
    gridOrigin = [binMiddleBottom(1)-0.3,binMiddleBottom(2)-0.2,binMiddleBottom(3)];
    gridMapping = [round((worldPts(:,1)-gridOrigin(1))./voxelSize), round((worldPts(:,2)-gridOrigin(2))./voxelSize), worldPts(:,3) - binMiddleBottom(3)];
    
    validPix = gridMapping(:,1) > 0 & gridMapping(:,1) <= 300 & gridMapping(:,2) > 0 & gridMapping(:,2) <= 200; %& gridMapping(:,3) > 0;
    colorPts = [reshape(colorImg(:,:,1),[],1),reshape(colorImg(:,:,2),[],1),reshape(colorImg(:,:,3),[],1)];
    
    heightMapColor(sub2ind(size(heightMap),gridMapping(validPix,2),gridMapping(validPix,1)),:) = colorPts(validPix,:);
    
    
    % real heigth map with bcakground removel
    validPix = gridMapping(:,1) > 0 & gridMapping(:,1) <= 300 & gridMapping(:,2) > 0 & gridMapping(:,2) <= 200& gridMapping(:,3) > 0;
    validDepth = (fgMask & camZ ~= 0);
    gridMapping = gridMapping(validPix&validDepth(:),:);
    heightMap(sub2ind(size(heightMap),gridMapping(:,2),gridMapping(:,1))) = gridMapping(:,3);
    
    
    % Find missing depth and project background depth into camera space
    missingDepth = depthImg == 0 & bgDepthImg > 0;
    [pixX,pixY] = meshgrid(1:640,1:480);
    camX = (pixX-camK(1,3)).*bgDepthImg/camK(1,1);
    camY = (pixY-camK(2,3)).*bgDepthImg/camK(2,2);
    
    camZ = bgDepthImg;
    
    missingCamPts = [camX(missingDepth),camY(missingDepth),camZ(missingDepth)];
    missingWorldPts = (camPose(1:3,1:3)*missingCamPts' + repmat(camPose(1:3,4),1,size(missingCamPts,1)))';
    
    % Get missing depth height map
    missingHeightMap = zeros(200,300);
    gridOrigin = [binMiddleBottom(1)-0.3,binMiddleBottom(2)-0.2,binMiddleBottom(3)];
    gridMapping = [round((missingWorldPts(:,1)-gridOrigin(1))./voxelSize), round((missingWorldPts(:,2)-gridOrigin(2))./voxelSize), missingWorldPts(:,3) - binMiddleBottom(3)];
    validPix = gridMapping(:,1) > 0 & gridMapping(:,1) <= 300 & gridMapping(:,2) > 0 & gridMapping(:,2) <= 200;
    gridMapping = gridMapping(validPix,:);
    missingHeightMap(sub2ind(size(missingHeightMap),gridMapping(:,2),gridMapping(:,1))) = 1;
    
    noisePix = ~bwareaopen(missingHeightMap > 0,50);
    missingHeightMap(noisePix) = 0;
    
    % Denoise height map
    noisePix = ~bwareaopen(heightMap > 0,50);
    heightMap(noisePix) = 0;
    
    heightMaps{camIdx+1} = heightMap;
    missingHeightMaps{camIdx+1} = missingHeightMap;
end
heightMap = max(heightMaps{1},heightMaps{2});
heightMapColor = reshape(heightMapColor,[200,300,3]);

% Height cannot exceed 30cm above bottom of tote
heightMap = min(heightMap,ones(size(heightMap)).*0.3);
rawHeightMap = heightMap;

% Fill in missing depth holes
heightMap(heightMap == 0 & (missingHeightMaps{1} & missingHeightMaps{2})) = 0.03;

% Save height map into binary file
heightMapOutput = heightMap';
fileID = fopen(heightMapOutputPath,'wb');
fwrite(fileID,heightMapOutput(:),'single');	
fclose(fileID);

% Save height map and reprojected color for Torch
colorData = zeros(224,320,3);
depthData = uint16(zeros(224,320));
colorData(13:212,11:310,:) = heightMapColor;
depthData(13:212,11:310) = uint16(rawHeightMap.*10000);
imwrite(colorData,heightmapColorFile);
imwrite(depthData,heightmapDepthFile);

end

