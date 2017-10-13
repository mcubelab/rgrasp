clear all; close all;
% NOTE: prior to running this script, start ROS service for RealSense
% camera streaming

dataDir = '/home/mcube/arcdata';
cameraInfoDir = '/home/mcube/rgrasp/catkin_ws/src/passive_vision/camerainfo';

% Camera setup per unit
% % tote
% unit{1}.upperFarCam = ''; % upper camera, far from robot
% unit{1}.upperNearCam = ''; % upper camera, near robot
% unit{1}.lowerNearCam = '';
% unit{1}.worldCoords = [];
% % storage system part 1
% unit{2}.upperFarCam = ''; 
% unit{2}.upperNearCam = '';
% unit{2}.lowerNearCam = '';
% unit{2}.worldCoords = [];
% % sotrage system part 2
% unit{3}.upperFarCam = '614203000682';
% unit{3}.upperNearCam = ''; 
% unit{3}.lowerNearCam = '';
% unit{3}.worldCoords = [0.63051 + 0.33, 0.41, -0.1943]; % 0.63051 + 0.33, 0.41, -0.1943

% Start Matlab ROS
fprintf('Starting Matlab ROS.\n');
try
    rosinit
catch
    rosshutdown
    rosinit
end

% Start ROS client for RealSense cameras
fprintf('Starting ROS client for Realsense cameras.\n');
realsenseClient = rossvcclient('/realsense_camera/capture');

% Parse RealSense camera serial numbers from unit setup
binInfo = textread(fullfile(cameraInfoDir,'bins.txt'),'%s','delimiter','\n');

% Full point cloud for visualization
fullCloudPoints = [];
fullCloudColors = [];

% Calibrate each RealSense camera
for binFileIdx=[1:6]%length(binInfo)
    if mod(binFileIdx,6) == 2
        binPosition = str2num(binInfo{binFileIdx}(6:end));
        fprintf('Check that bin %s has a calibration board. Press any key to continue.\n',binInfo{binFileIdx-1}(5)); pause();
    end
    if mod(binFileIdx,6) == 3 || mod(binFileIdx,6) == 5
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
            validYellowPix = find(colorDataHSV(:,1) > 30/360 & colorDataHSV(:,1) < 80/360 & depthData(:) < 1.0 & depthData(:) > 0.5);
            yellowDotMask = boolean(zeros(size(colorData,1),size(colorData,2)));
            yellowDotMask(validYellowPix) = 1;
            yellowDotMask(:,1:200) = 0;
            yellowDotMask(:,440:end) = 0;
            yellowDotCenters = imfindcircles(yellowDotMask,[10,20]);
            yellowDotCenters = sortrows(yellowDotCenters,2);
            yellowDotPt1 = reshape(cloudData(round(yellowDotCenters(1,2)),round(yellowDotCenters(1,1)),:),[3,1]);
            yellowDotPt2 = reshape(cloudData(round(yellowDotCenters(2,2)),round(yellowDotCenters(2,1)),:),[3,1]);
            yellowDotPt3 = reshape(cloudData(round(yellowDotCenters(3,2)),round(yellowDotCenters(3,1)),:),[3,1]);
            toteAxisX = [yellowDotPt1-yellowDotPt3]/norm(yellowDotPt1-yellowDotPt3);
            toteMiddleBottom = yellowDotPt2;

            % Find bottom of tote and calibrate camera to tote to world
            cloudPoints = reshape(permute(cloudData,[3,1,2]),3,480*640)';
            [planeModel,inlierIndices,outlierIndices] = pcfitplane(pointCloud(cloudPoints),0.01,[0,0,-1],60);
            toteAxisY = -cross(toteAxisX,planeModel.Normal');
            pcaBottomPoints = [toteAxisX,toteAxisY,planeModel.Normal'];

            if mod(binFileIdx,6) == 5
                cam2tote = [[inv(pcaBottomPoints),[0;0;0]];...
                            [0,0,0,1]]*[[eye(3),-reshape(toteMiddleBottom,3,1)];...
                            [0,0,0,1]];
            else
                cam2tote = [[rotz(180)*inv(pcaBottomPoints),[0;0;0]];[0,0,0,1]]*[[eye(3),-reshape(toteMiddleBottom,3,1)];[0,0,0,1]];
            end
            tote2world = [[eye(3),binPosition'];[0,0,0,1]];
            totePoints = (cam2tote(1:3,1:3)*reshape(permute(cloudData,[3,1,2]),3,480*640)+repmat(cam2tote(1:3,4),1,480*640))';
            cam2world = tote2world*cam2tote;
            worldPoints = (cam2world(1:3,1:3)*reshape(permute(cloudData,[3,1,2]),3,480*640)+repmat(cam2world(1:3,4),1,480*640))';
            pcwrite(pointCloud(totePoints,'Color',reshape(permute(colorData,[3,1,2]),3,480*640)'),fullfile(cameraInfoDir,strtrim(cameraSerialNumber)),'PLYformat','binary');
            fullCloudPoints = [fullCloudPoints;worldPoints];
            fullCloudColors = [fullCloudColors;reshape(permute(colorData,[3,1,2]),3,480*640)'];
            dlmwrite(fullfile(cameraInfoDir,sprintf('%s.pose.txt',strtrim(cameraSerialNumber))),cam2world,'delimiter',' ');

        end
    end
end

pcwrite(pointCloud(fullCloudPoints,'Color',fullCloudColors),fullfile(cameraInfoDir,'debug'),'PLYformat','binary');
% pcwrite(pointCloud(fullCloudPoints,'Color',fullCloudColors),'debug','PLYformat','binary');

% Camera Calibration
% Run script, save cam2world matrix for each camera
% realsense copy cam2world matrix, pass through ros message



