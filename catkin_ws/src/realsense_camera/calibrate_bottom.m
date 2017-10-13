clear all; close all;
% NOTE: prior to running this script, start ROS service for RealSense
% camera streaming

dataDir = '/home/mcube/arcdata';
cameraInfoDir = '/home/mcube/rgrasp/catkin_ws/src/passive_vision/camerainfo';


% Start Matlab ROS
fprintf('Starting Matlab ROS.\n');
try
    rosinit
catch
    rosshutdown
    rosinit
end
addpath(fullfile(pwd,'matlab_gen/msggen'));

% Start ROS client for RealSense cameras
fprintf('Starting ROS client for Realsense cameras.\n');
realsenseClient = rossvcclient('/realsense_camera/capture');

% Parse RealSense camera serial numbers from unit setup
binInfo = textread(fullfile(cameraInfoDir,'bins.txt'),'%s','delimiter','\n');
middlePointHeight = 0.4;
% Full point cloud for visualization
fullCloudPoints = [];
fullCloudColors = [];

% Calibrate each RealSense camera
for binFileIdx=1:6%length(binInfo)
    if mod(binFileIdx,6) == 2
        binPosition = str2num(binInfo{binFileIdx}(6:end));
        fprintf('Check that bin %s has a calibration board. Press any key to continue.\n',binInfo{binFileIdx-1}(5)); pause();
    end
    if mod(binFileIdx,6) == 4 || mod(binFileIdx,6) == 0
        if ~strcmp(binInfo{binFileIdx}(4:end),'nan')
            tmpCameraSerialNumber = binInfo{binFileIdx}(4:end);
            cameraSerialNumber = '            ';
            cameraSerialNumber(1:length(tmpCameraSerialNumber)) = tmpCameraSerialNumber;
            fprintf('Calibrating camera: %s\n',cameraSerialNumber);
 
            % Fetch data from RealSense cameras
            realsenseReq = rosmessage(realsenseClient);
            realsenseReq.CameraSerialNumber = cameraSerialNumber;
            realsenseData = call(realsenseClient,realsenseReq,'Timeout',3);
            cloudData = permute(reshape(double(realsenseData.PointCloudXyz),3,640,480),[3,2,1]);
            colorData = permute(reshape(realsenseData.PointCloudRgb,3,640,480),[3,2,1]);

            % Find center yellow dot of tote
            colorDataHSV = rgb2hsv(double(reshape(permute(colorData,[3,1,2]),3,480*640)')./255);
            depthData = cloudData(:,:,3);
            validYellowPix = find(colorDataHSV(:,1) > 30/360 & colorDataHSV(:,1) < 70/360 & depthData(:) <1);
            yellowDotMask = boolean(zeros(size(colorData,1),size(colorData,2)));
            yellowDotMask(validYellowPix) = 1;
            yellowDotMask(:,1:200) = 0;
            yellowDotMask(:,440:end) = 0;
            yellowDotCenters = imfindcircles(yellowDotMask,[10,30]);
            yellowDotCenters = sortrows(yellowDotCenters,2);

            figure(1);
            clf;imagesc(yellowDotMask); 
            hold on;
            scatter(yellowDotCenters([1,end],1),yellowDotCenters([1,end],2))

            yellowDotPt1 = reshape(cloudData(round(yellowDotCenters(1,2)),round(yellowDotCenters(1,1)),:),[3,1]);
            yellowDotPt2 = reshape(cloudData(round(yellowDotCenters(end,2)),round(yellowDotCenters(end,1)),:),[3,1]);
           
            toteAxisZ = [yellowDotPt1-yellowDotPt2]/norm(yellowDotPt1-yellowDotPt2);
            toteAxisY = [-1;0;0];
            toteAxisX = cross(toteAxisY,toteAxisZ);
            pcaRotation = [toteAxisX,toteAxisY,toteAxisZ];
            middlePoint = yellowDotPt2;

            if mod(binFileIdx,6) == 0
               cam2tote = [[inv(pcaRotation),[0;0;0]];...
                            [0,0,0,1]]*[[eye(3),-reshape(middlePoint,3,1)];...
                            [0,0,0,1]];
            else
               cam2tote = [[rotz(180)*inv(pcaRotation),[0;0;0]];[0,0,0,1]]*[[eye(3),-reshape(middlePoint,3,1)];[0,0,0,1]];
            end
            binPosition_tf = binPosition;
            binPosition_tf(3) = binPosition_tf(3) + middlePointHeight;
            tote2world = [[eye(3),binPosition_tf'];[0,0,0,1]];
            totePoints = (cam2tote(1:3,1:3)*reshape(permute(cloudData,[3,1,2]),3,480*640)+repmat(cam2tote(1:3,4),1,480*640))';

            cam2world = tote2world*cam2tote;
            worldPoints = (cam2world(1:3,1:3)*reshape(permute(cloudData,[3,1,2]),3,480*640)+repmat(cam2world(1:3,4),1,480*640))';
            pcwrite(pointCloud(totePoints,'Color',reshape(permute(colorData,[3,1,2]),3,480*640)'),fullfile(cameraInfoDir,strtrim(cameraSerialNumber)),'PLYformat','binary');
            fullCloudPoints = [fullCloudPoints;worldPoints];
            fullCloudColors = [fullCloudColors;reshape(permute(colorData,[3,1,2]),3,480*640)'];
            
            pcwrite(pointCloud(worldPoints,'Color',reshape(permute(colorData,[3,1,2]),3,480*640)'),fullfile(cameraInfoDir,'debug2'),'PLYformat','binary');

            dlmwrite(fullfile(cameraInfoDir,sprintf('%s.pose.txt',strtrim(cameraSerialNumber))),cam2world,'delimiter',' ');
        end
    end
end
pcwrite(pointCloud(fullCloudPoints,'Color',fullCloudColors),fullfile(cameraInfoDir,'debug'),'PLYformat','binary');

