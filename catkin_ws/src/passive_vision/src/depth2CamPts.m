function [camPts,camX,camY,camZ] = depth2CamPts(depthImg,camK)
    % Project depth into camera space
    [pixX,pixY] = meshgrid(1:640,1:480);
    camX = (pixX-camK(1,3)).*depthImg/camK(1,1);
    camY = (pixY-camK(2,3)).*depthImg/camK(2,2);
    camZ = depthImg;
    %camY(camY>0) = camY(camY>0)-0.005;
    %camZ(camZ>0) = camZ(camZ>0)+0.005;
    camPts = [camX(:),camY(:),camZ(:)];