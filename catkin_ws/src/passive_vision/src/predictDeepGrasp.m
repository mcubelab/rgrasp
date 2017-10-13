function predictDeepGrasp(binId,ARCUnit,showDebug)
%close all;
t = tic();

if ~exist('binId','var')
    binId = 0;
end
if ~exist('showDebug','var')
    showDebug = 1;
end

dataPath = '/home/mcube/arcdata/tmpdata';
% dataPath = '/home/mcube/arcdata/1498248596'
cameraInfoPath = '/home/mcube/arc/catkin_ws/src/passive_vision/camerainfo';

graspPredictions = [];
flushGraspPredictions = [];

graspVis = [];
flushGraspVis = [];
graspObjId = [];
flushGraspObjId = [];

graspOutputPath = fullfile(dataPath,sprintf('passive-vision-grasp.%d.output.bin',binId));
flushGraspOutputPath = fullfile(dataPath,sprintf('passive-vision-flush-grasp.%d.output.bin',binId));
%heightMapOutputPath = fullfile(dataPath,sprintf('passive-vision-height-map.%d.output.bin',binId));
graspVisImgPath = fullfile(dataPath,sprintf('passive-vision-grasp.%d.debug.png',binId));
%graspObjVisImgPath = fullfile(dataPath,sprintf('passive-vision-grasp-object.%d.debug.png',binId));
flushVisImgPath = fullfile(dataPath,sprintf('passive-vision-flush-grasp.%d.debug.png',binId));

graspOutputObjectsPath = fullfile(dataPath,sprintf('passive-vision-grasp-objects.%d.output.txt',binId));
flushOutputObjectsPath = fullfile(dataPath,sprintf('passive-vision-flush-grasp-objects.%d.output.txt',binId));

%deepGraspOutputPath = fullfile(dataPath,sprintf('grasp-prediction-output.%d.hdf5',binId));

heightMaps = {};
missingHeightMaps = {};

voxelSize = 0.002;
% Read Seg HeightMap 
try
   heightSeg = ARCUnit.Curr.heightMapSegProjected';
   objNameList = ['NULL',{ARCUnit.Curr.objectList.objectName}];
   objConfList = [0; [ARCUnit.Curr.objectList.conf]'];
catch
   objNameList ={'NULL'};
   objConfList = [0];
   heightSeg = zeros(200,300); 
end

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

    % Project depth into camera space
    [camPts,~,~,camZ] = depth2CamPts(depthImg,camK);

    % Only use points with valid depth
    validDepth = find(fgMask & camZ ~= 0);
    camPts = camPts(validDepth(:),:);

    % Transform points to world coordinates
    worldPts = (camPose(1:3,1:3)*camPts' + repmat(camPose(1:3,4),1,size(camPts,1)))';
    
    % Get height map
    heightMap = zeros(200,300);
    gridOrigin = [binMiddleBottom(1)-0.3,binMiddleBottom(2)-0.2,binMiddleBottom(3)];
    gridMapping = [round((worldPts(:,1)-gridOrigin(1))./voxelSize), round((worldPts(:,2)-gridOrigin(2))./voxelSize), worldPts(:,3) - binMiddleBottom(3)];
    validPix = gridMapping(:,1) > 0 & gridMapping(:,1) <= 300 & gridMapping(:,2) > 0 & gridMapping(:,2) <= 200 & gridMapping(:,3) > 0;
    gridMapping = gridMapping(validPix,:);
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

% Height cannot exceed 30cm above bottom of tote
heightMap = min(heightMap,ones(size(heightMap)).*0.3);

% Fill in missing depth holes
heightMap(heightMap == 0 & (missingHeightMaps{1} & missingHeightMaps{2})) = 0.03;

% % Save height map into binary file
% heightMapOutput = heightMap';
% fileID = fopen(heightMapOutputPath,'wb');
% fwrite(fileID,heightMapOutput(:),'single');	
% fclose(fileID);

% Load deep grasp prediction results
% deepGraspData = hdf5read(deepGraspOutputPath,'output');
% deepGraspData = permute(deepGraspData,[4 3 2 1]);
% deepGraspConf = {};
% for rotIdx = 1:8
%     tmpGraspConf = reshape(deepGraspData(rotIdx,2,:,:),size(deepGraspData,3),size(deepGraspData,4));
%     tmpGraspConf = imresize(tmpGraspConf,4);
%     tmpGraspConf = imgaussfilt(tmpGraspConf, 5);
%     tmpGraspConf(tmpGraspConf < 0) = 0;
%     tmpGraspConf(tmpGraspConf >= 1) = 0.9999;
%     tmpGraspConf = tmpGraspConf(13:212,11:310);
%     deepGraspConf{rotIdx} = tmpGraspConf;
% end

fingerWidth = 0.06;
fingerThickness = 0.036;

% Use image filtering for grasp detection
% fingerMask = zeros(75,75);
% fingerMask(66:75,23:52) = 1;
% fingerMask(1:10,23:52) = 1;
% fingerMask = imrotate(fingerMask,90,'crop');
% fingerMax = ordfilt2(heightMap,sum(fingerMask(:)),fingerMask);
% figure(); imagesc(fingerMax); axis equal;
% figure(); imagesc(heightMap > (fingerMax + 0.02)); axis equal;

% Predict normal grasps
sizeHeightMap = size(heightMap);
[topFingerX_orig,topFingerY_orig] = meshgrid(1:(fingerWidth/voxelSize),(fingerThickness/voxelSize):-1:1);
[botFingerX_orig,botFingerY_orig] = meshgrid(1:(fingerWidth/voxelSize),1:(fingerThickness/voxelSize));
theta_rad_range = 0:deg2rad(45/2):deg2rad(179);
%third_quartile = @(y)median(y(y>median(y)));
for dx = 5:25
    for dy = 6:15
        localHeightMap = heightMap((dy*10-9):(dy*10),(dx*10-9):(dx*10));
%         localHeightMapSeg = heightSeg((dy*10-9):(dy*10),(dx*10-9):(dx*10));
        if sum(localHeightMap(:) > 0) > 75
            x_coord = dx*10-5;
            y_coord = dy*10-5;
            xy_coord_rep = repmat([x_coord;y_coord],1,size(topFingerX_orig(:),1));
            
            median_localHeightMap = median(localHeightMap(:));
            surfaceCentroid = [x_coord*voxelSize+gridOrigin(1),y_coord*voxelSize+gridOrigin(2),median_localHeightMap+gridOrigin(3)];
            %for theta = 0:(45/2):179
            for theta_rad = theta_rad_range
                rotMat = [cos(theta_rad),-sin(theta_rad);sin(theta_rad),cos(theta_rad)];
                topFingerX = topFingerX_orig - (fingerWidth/voxelSize)/2;
                botFingerX = botFingerX_orig - (fingerWidth/voxelSize)/2;
                for graspSpace = 0.03:0.02:0.11
                    graspSpace_div_voxelSize_div_2 = (graspSpace/voxelSize)/2;
                    topFingerY = topFingerY_orig + graspSpace_div_voxelSize_div_2;
                    topFingerPix = [topFingerX(:),topFingerY(:)];
                    
                    botFingerY = -botFingerY_orig - graspSpace_div_voxelSize_div_2;
                    botFingerPix = [botFingerX(:),botFingerY(:)];
                    
                    % change coordinate
                    topFingerPix = round(rotMat * topFingerPix' + xy_coord_rep)';
                    botFingerPix = round(rotMat * botFingerPix' + xy_coord_rep)';
                    try
                        fingerInd = sub2ind2d(sizeHeightMap, [topFingerPix(:,2);botFingerPix(:,2)],[topFingerPix(:,1);botFingerPix(:,1)]);

                        % If middle surface centroid is higher than finger lowpoints, set valid grasp
%                         topFingerHeight = mean(heightMap(fingerInd));
%                         topFingerHeight = sort(heightMap(fingerInd),'descend');
%                         topFingerHeight = mean(topFingerHeight(1:floor(size(topFingerHeight,1)/25)));

                        heightMap_fingerInd = heightMap(fingerInd);
                        
                        if median_localHeightMap > median(heightMap_fingerInd) + 0.02 && ...  %% median 50% is O(n) so can filter quickly
                                 median_localHeightMap > prctile(heightMap_fingerInd,90) + 0.02 %% prctile is O(nlogn) because of sorting
                            % put the following 6 lines inside if to prevent
                            % unnecessary computation
                            [midGraspX,midGraspY] = meshgrid(1:(fingerWidth/voxelSize),1:(graspSpace/voxelSize));
                            midGraspX = midGraspX - (fingerWidth/voxelSize)/2;
                            midGraspY = midGraspY - (graspSpace/voxelSize)/2;
                            midGraspPix = [midGraspX(:),midGraspY(:)];
                            midGraspPix = round(rotMat * midGraspPix' + repmat([x_coord;y_coord],1,size(midGraspPix,1)))';  
                            graspInd = sub2ind2d(size(heightMap),midGraspPix(:,2),midGraspPix(:,1));
                            
                            surfacePts = heightMap(graspInd) > median(heightMap(fingerInd));
%                             graspConf = max((0.9*sum(surfacePts(:))./size(graspInd,1) + 0.1*deepGraspConf{(theta/(45/2))+1}(y_coord,x_coord))-0.2,0);
                            graspConf = max((sum(surfacePts(:))./size(graspInd,1))-0.2,0);
                            colorMapJet = jet;
                            colorScale = colorMapJet(floor(graspConf.*63)+1,:);
%                             plot([topFingerPix(150,1);botFingerPix(151,1)],[topFingerPix(150,2);botFingerPix(151,2)],'LineWidth',2,'Color',colorScale);
                            graspDirection = [0,0,-1];
                            graspDepth = min(0.15, median_localHeightMap - prctile(heightMap(fingerInd),10));
                            graspJawWidth = min(0.11,graspSpace+0.03);
                            gripperAngleDirection = [0,1,0];
                            gripperAngleAxis = [0,0,1];
                            gripperRotm = vrrotvec2mat([gripperAngleAxis,theta_rad]);
                            gripperAngleDirection = (gripperRotm*gripperAngleDirection')';
%                             surfaceCentroid = surfaceCentroid + gripperAngleDirection*0.02; % Fix gripper shifting
                            graspPredictions = [graspPredictions;[surfaceCentroid,graspDirection,graspDepth,graspJawWidth,gripperAngleDirection,graspConf]];
                            graspVis = [graspVis;[topFingerPix(255,1),topFingerPix(255,2),botFingerPix(256,1),botFingerPix(256,2),graspConf,colorScale]];
                            
                            graspObjId = [graspObjId;mode(heightSeg(sub2ind2d(size(heightSeg),midGraspPix(:,2),midGraspPix(:,1))))];
                            break;
                        end
                    catch
                    end
                end
            end
        end
    end
end

% Predict flush grasps
for dx=5:25
    
    % Flush grasps on one side
    for dy = [4,5]
        localHeightMap = heightMap((dy*10-9):(dy*10),(dx*10-9):(dx*10));
%         localHeightMapSeg = heightSeg((dy*10-9):(dy*10),(dx*10-9):(dx*10));
        if sum(localHeightMap(:) > 0) > 75
            x_coord = dx*10-5;
            y_coord = dy*10-5;
            median_localHeightMap = median(localHeightMap(:));
            surfaceCentroid = [x_coord*voxelSize+gridOrigin(1),y_coord*voxelSize+gridOrigin(2), median_localHeightMap+gridOrigin(3)];
            for graspSpace = 0.03:0.02:0.11
                [botFingerX,botFingerY] = meshgrid((x_coord-fingerWidth/(voxelSize*2)):(x_coord+fingerWidth/(voxelSize*2)),(21+(graspSpace/voxelSize)):(20+(graspSpace/voxelSize)+(fingerThickness/voxelSize)));
                botFingerPix = [botFingerX(:),botFingerY(:)];
                try
                    fingerInd = sub2ind(size(heightMap),botFingerPix(:,2),botFingerPix(:,1));

                    % If middle surface centroid is higher than finger lowpoints, set valid grasp
                    %topFingerHeight = mean(heightMap(fingerInd));
%                     topFingerHeight = sort(heightMap(fingerInd),'descend');
%                     topFingerHeight = mean(topFingerHeight(1:floor(size(topFingerHeight,1)/25)));
                    if median_localHeightMap > prctile(heightMap(fingerInd),90) + 0.02
                        % move the following 3 lines in to avoid unecessary
                        % computation
                        [midGraspX,midGraspY] = meshgrid((x_coord-fingerWidth/(voxelSize*2)):(x_coord+fingerWidth/(voxelSize*2)),20:(20+(graspSpace/voxelSize)));
                        midGraspPix = [midGraspX(:),midGraspY(:)];
                        graspInd = sub2ind(size(heightMap),midGraspPix(:,2),midGraspPix(:,1));
                        
                        surfacePts = heightMap(graspInd) > median(heightMap(fingerInd));
                        
%                         graspConf = max((0.9*sum(surfacePts(:))./size(graspInd,1) + 0.1*deepGraspConf{1}(y_coord,x_coord))-0.2,0);
                        graspConf = max((sum(surfacePts(:))./size(graspInd,1))-0.2,0);
                        colorMapJet = jet;
                        colorScale = colorMapJet(floor(graspConf.*63)+1,:);
%                         hold on; plot([x_coord;botFingerPix(151,1)],[20;botFingerPix(151,2)],'LineWidth',2,'Color',colorScale);
                        graspJawWidth = min(0.11,graspSpace+0.01);
                        graspDepth = min(0.15, median_localHeightMap - prctile(heightMap(fingerInd),10));
                        flushGraspPredictions = [flushGraspPredictions;[surfaceCentroid,graspDepth,graspJawWidth,graspConf]];
                        flushGraspVis = [flushGraspVis;[x_coord,20,botFingerPix(256,1),botFingerPix(256,2),graspConf,colorScale]];
                        flushGraspObjId = [flushGraspObjId;mode(heightSeg(sub2ind(size(heightSeg),midGraspPix(:,2),midGraspPix(:,1))))];
                            
                        break;
                    end
                catch errorMsg
                    display(errorMsg);
                end
            end
        end
    end
    
    % Flush grasps on other side
    for dy = [16,17]
        localHeightMap = heightMap((dy*10-9):(dy*10),(dx*10-9):(dx*10));
%         localHeightMapSeg = heightSeg((dy*10-9):(dy*10),(dx*10-9):(dx*10));
        if sum(localHeightMap(:) > 0) > 75
            x_coord = dx*10-5;
            y_coord = dy*10-5;
            surfaceCentroid = [x_coord*voxelSize+gridOrigin(1),y_coord*voxelSize+gridOrigin(2),heightMap(y_coord,x_coord)+gridOrigin(3)];
            for graspSpace = 0.03:0.02:0.11
                [midGraspX,midGraspY] = meshgrid((x_coord-fingerWidth/(voxelSize*2)):(x_coord+fingerWidth/(voxelSize*2)),(180-(graspSpace/voxelSize)):180);
                midGraspPix = [midGraspX(:),midGraspY(:)];
                [topFingerX,topFingerY] = meshgrid((x_coord+1-fingerWidth/(voxelSize*2)):(x_coord+fingerWidth/(voxelSize*2)),(180-(graspSpace/voxelSize)-(fingerThickness/voxelSize)):(181-(graspSpace/voxelSize)));
                topFingerPix = [topFingerX(:),topFingerY(:)];
                try
                    fingerInd = sub2ind(size(heightMap),topFingerPix(:,2),topFingerPix(:,1));
                    graspInd = sub2ind(size(heightMap),midGraspPix(:,2),midGraspPix(:,1));

                    % If middle surface centroid is higher than finger lowpoints, set valid grasp
%                     topFingerHeight = mean(heightMap(fingerInd));
%                     topFingerHeight = sort(heightMap(fingerInd),'descend');
%                     topFingerHeight = mean(topFingerHeight(1:floor(size(topFingerHeight,1)/25)));
                    if heightMap(y_coord,x_coord) > prctile(heightMap(fingerInd),90) + 0.02
                        surfacePts = heightMap(graspInd) > median(heightMap(fingerInd));
%                         graspConf = max((0.9*sum(surfacePts(:))./size(graspInd,1) + 0.1*deepGraspConf{1}(y_coord,x_coord))-0.2,0);
                        graspConf = max((sum(surfacePts(:))./size(graspInd,1))-0.2,0);
                        colorMapJet = jet;
                        colorScale = colorMapJet(floor(graspConf.*63)+1,:);
                        graspJawWidth = min(0.11,graspSpace+0.01);
                        graspDepth = min(0.15,heightMap(y_coord,x_coord) - prctile(heightMap(fingerInd),10));
                        flushGraspPredictions = [flushGraspPredictions;[surfaceCentroid,graspDepth,graspJawWidth,graspConf]];
                        flushGraspVis = [flushGraspVis;[x_coord,180,topFingerPix(180,1),topFingerPix(180,2),graspConf,colorScale]]; % FIX THIS LATER
                        flushGraspObjId = [flushGraspObjId;mode(heightSeg(sub2ind(size(heightSeg),midGraspPix(:,2),midGraspPix(:,1))))];
                        break;
                    end
                catch errorMsg
                    display(errorMsg);
                end
            end
        end
    end
end

% Associate object names to grasp and flush grasp proposals
graspObjId(isnan(graspObjId)) = 0;
flushGraspObjId(isnan(flushGraspObjId)) = 0;
graspObjectNames = objNameList(graspObjId(:)+1);
flushObjectNames = objNameList(flushGraspObjId(:)+1);
graspObjectConf = objConfList(graspObjId(:)+1);
flushObjectConf = objConfList(flushGraspObjId(:)+1);
graspPredictions = [graspPredictions,graspObjectConf];
flushGraspPredictions = [flushGraspPredictions,flushObjectConf];

if ~isempty(graspPredictions)
    % Remove grasp proposals with confidence < 10%
    graspPredictions = graspPredictions(find(graspPredictions(:,12) > 0.1),:);
    
    % Sort grasp proposals based on confidence
    [~,graspSortInd] = sortrows(graspPredictions,-12);
    graspPredictions = graspPredictions(graspSortInd,:);
    graspVis = graspVis(graspSortInd,:);
    graspObjectNames = graspObjectNames(graspSortInd);
end

if ~isempty(flushGraspPredictions)
    % Remove flush grasp proposals with confidence < 10%
    flushGraspPredictions = flushGraspPredictions(find(flushGraspPredictions(:,6) > 0.1),:);

    % Sort flush grasp proposals based on confidence
    [~,flushGraspSortInd] = sortrows(flushGraspPredictions,-6);
    flushGraspPredictions = flushGraspPredictions(flushGraspSortInd,:);
    flushGraspVis = flushGraspVis(flushGraspSortInd,:);
    flushObjectNames = flushObjectNames(flushGraspSortInd);
end

% Adjust grasp proposals location based on finger shift
if ~isempty(graspPredictions)
    graspPredictions(:,1:3) = graspPredictions(:,1:3) - graspPredictions(:,9:11)*0.02;
end

% Save grasp proposals in row major order
graspPredictions = graspPredictions';
fileID = fopen(graspOutputPath,'wb');
fwrite(fileID,[size(graspPredictions,2);graspPredictions(:)],'single');	
fclose(fileID);

% Save object names associated with grasp proposals
fileID = fopen(graspOutputObjectsPath,'w');
for i = 1:length(graspObjectNames)
    fprintf(fileID,'%s\n',graspObjectNames{i});
end
fclose(fileID);

% Save flush grasp proposals in row major order
flushGraspPredictions = flushGraspPredictions';
fileID = fopen(flushGraspOutputPath,'wb');
fwrite(fileID,[size(flushGraspPredictions,2);flushGraspPredictions(:)],'single');	
fclose(fileID);

% Save object names associated with flush grasp proposals
fileID = fopen(flushOutputObjectsPath,'w');
for i = 1:length(flushObjectNames)
    fprintf(fileID,'%s\n',flushObjectNames{i});
end
fclose(fileID);

if showDebug
    % Save grasp proposals debug visualization
    colorMapJet = gray;
    colorScale = floor((heightMap./0.3).*63)+1;
    heightMapVis = permute(reshape(colorMapJet(colorScale(:),:)',3,200,300),[2,3,1]);
    if ~isempty(graspPredictions) 
        graspVis = sortrows(graspVis,5);
        heightMapVis = insertShape(heightMapVis, 'Line', graspVis(:,1:4), 'LineWidth', 2, 'Color', graspVis(:,6:8));
    end
    imwrite(flipud(heightMapVis),graspVisImgPath);
    %imwrite(flipud(getImagesc(heightSeg)),graspObjVisImgPath);

    % Save flush grasp proposals debug visualization
    colorMapJet = gray;
    colorScale = floor((heightMap./0.3).*63)+1;
    heightMapVis = permute(reshape(colorMapJet(colorScale(:),:)',3,200,300),[2,3,1]);
    if ~isempty(flushGraspPredictions) 
        flushGraspVis = sortrows(flushGraspVis,5);
        heightMapVis = insertShape(heightMapVis, 'Line', flushGraspVis(:,1:4), 'LineWidth', 2, 'Color', flushGraspVis(:,6:8));
    end
    imwrite(flipud(heightMapVis),flushVisImgPath);
end

fprintf('[MATLAB Timing PredictDeepGrasp] '); toc(t);

end
