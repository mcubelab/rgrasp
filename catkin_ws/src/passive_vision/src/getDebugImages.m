function getDebugImages()
clear all; close all;

while (true)
    try
        dataDir = '~/arcdata/tmpdata';
        cameraInfoPath = '/home/mcube/arc/catkin_ws/src/passive_vision/camerainfo';
    %     tic();
        debugCanvas = [];
        colorScale = jet;

        for binId = 0:3

            binCanvas = uint8(zeros(930,415,3));

            % Check if bin is active
            try
                bgColor0 = imread(fullfile(dataDir,sprintf('passive-vision-background.%d.0.color.png',binId)));
            catch
                debugCanvas = [debugCanvas,binCanvas];
                continue;
            end

            % Get tote corners
            camInfoFilePath = fullfile(cameraInfoPath,'bins.txt');
            binMiddleBottom = dlmread(camInfoFilePath,' ',[(binId*6+1),1,(binId*6+1),3]);
            toteCorners = [[binMiddleBottom(1)-0.27;binMiddleBottom(2)-0.18;0.0],...
                           [binMiddleBottom(1)-0.27;binMiddleBottom(2)+0.18;0.0],...
                           [binMiddleBottom(1)+0.27;binMiddleBottom(2)+0.18;0.0],...
                           [binMiddleBottom(1)+0.27;binMiddleBottom(2)-0.18;0.0]];

            % Load background and foreground RGB-D images
            bgDepth0 = double(imread(fullfile(dataDir,sprintf('passive-vision-background.%d.0.depth.png',binId))))./10000;
            bgDepth0 = bgDepth0./max(bgDepth0(:));
            bgDepth0 = uint8(reshape(colorScale(floor(bgDepth0(:).*63)+1,:),480,640,3).*255);

            % Project background
            camKFilePath = fullfile(dataDir,sprintf('passive-vision-camera.%d.0.intrinsics.txt',binId));
            camPoseFilePath = fullfile(dataDir,sprintf('passive-vision-camera.%d.0.pose.txt',binId));
            camK = dlmread(camKFilePath);
            camPose = dlmread(camPoseFilePath);
            camExtrinsics = inv(camPose);
            toteCornersCam = camExtrinsics(1:3,1:3)*toteCorners + repmat(camExtrinsics(1:3,4),1,4);
            totePix = camK*toteCornersCam;
            totePix(1,:) = totePix(1,:)./totePix(3,:);
            totePix(2,:) = totePix(2,:)./totePix(3,:);
            bgColor0 = insertShape(bgColor0,'line',[totePix(1,1),totePix(2,1),totePix(1,2),totePix(2,2);
                                                    totePix(1,2),totePix(2,2),totePix(1,3),totePix(2,3);
                                                    totePix(1,3),totePix(2,3),totePix(1,4),totePix(2,4);
                                                    totePix(1,4),totePix(2,4),totePix(1,1),totePix(2,1)],'LineWidth',10,'Color',[0,255,0]);

            bgColor0 = imresize(bgColor0,200/640);
            bgDepth0 = imresize(bgDepth0,200/640);
            binCanvas(6:155,6:205,:) = bgColor0;
            binCanvas(6:155,211:410,:) = bgDepth0;

            bgColor1 = imread(fullfile(dataDir,sprintf('passive-vision-background.%d.1.color.png',binId)));
            bgDepth1 = double(imread(fullfile(dataDir,sprintf('passive-vision-background.%d.1.depth.png',binId))))./10000;
            bgDepth1 = bgDepth1./max(bgDepth1(:));
            bgDepth1 = uint8(reshape(colorScale(floor(bgDepth1(:).*63)+1,:),480,640,3).*255);

            % Project background
            camKFilePath = fullfile(dataDir,sprintf('passive-vision-camera.%d.1.intrinsics.txt',binId));
            camPoseFilePath = fullfile(dataDir,sprintf('passive-vision-camera.%d.1.pose.txt',binId));
            camK = dlmread(camKFilePath);
            camPose = dlmread(camPoseFilePath);
            camExtrinsics = inv(camPose);
            toteCornersCam = camExtrinsics(1:3,1:3)*toteCorners + repmat(camExtrinsics(1:3,4),1,4);
            totePix = camK*toteCornersCam;
            totePix(1,:) = totePix(1,:)./totePix(3,:);
            totePix(2,:) = totePix(2,:)./totePix(3,:);
            bgColor1 = insertShape(bgColor1,'line',[totePix(1,1),totePix(2,1),totePix(1,2),totePix(2,2);
                                                    totePix(1,2),totePix(2,2),totePix(1,3),totePix(2,3);
                                                    totePix(1,3),totePix(2,3),totePix(1,4),totePix(2,4);
                                                    totePix(1,4),totePix(2,4),totePix(1,1),totePix(2,1)],'LineWidth',10,'Color',[0,255,0]);

            bgColor1 = imresize(bgColor1,200/640);
            bgDepth1 = imresize(bgDepth1,200/640);
            binCanvas(161:310,6:205,:) = bgColor1;
            binCanvas(161:310,211:410,:) = bgDepth1;

            try
                graspDebug0 = imread(fullfile(dataDir,sprintf('passive-vision-grasp.%d.debug.png',binId)));
                graspDebug0 = imrotate(graspDebug0,-90);
                flushGraspDebug0 = imread(fullfile(dataDir,sprintf('passive-vision-flush-grasp.%d.debug.png',binId)));
                flushGraspDebug0 = imrotate(flushGraspDebug0,-90);

                binCanvas(316:615,6:205,:) = graspDebug0;
                binCanvas(316:615,211:410,:) = flushGraspDebug0;
            catch
            end

            try
                suctionDebug0 = imread(fullfile(dataDir,sprintf('passive-vision-suction.%d.0.debug.png',binId)));
                suctionDebug1 = imread(fullfile(dataDir,sprintf('passive-vision-suction.%d.1.debug.png',binId)));

                suctionDebug0 = imresize(suctionDebug0,200/640);
                suctionDebug1 = imresize(suctionDebug1,200/640);
                binCanvas(621:770,6:205,:) = suctionDebug0;
                binCanvas(621:770,211:410,:) = suctionDebug1;
            catch
            end

            try
                segDebug0 = imread(fullfile(dataDir,sprintf('passive-vision-state.%d.0.debug.png',binId)));
                segDebug1 = imread(fullfile(dataDir,sprintf('passive-vision-state.%d.1.debug.png',binId)));

                segDebug0 = imresize(segDebug0,200/640);
                segDebug1 = imresize(segDebug1,200/640);
                binCanvas(776:925,6:205,:) = segDebug0;
                binCanvas(776:925,211:410,:) = segDebug1;
            catch
            end

            debugCanvas = [debugCanvas,binCanvas];

        end

        imwrite(debugCanvas,fullfile(dataDir,'debug.png'));
        % delete(fullfile(dataDir, '*.debug.png'));
        % toc();
    catch ee
%         fprintf('Debug image error!');
%         display(ee)
%         display(ee.stack)
    end
    
end
    
    
end

