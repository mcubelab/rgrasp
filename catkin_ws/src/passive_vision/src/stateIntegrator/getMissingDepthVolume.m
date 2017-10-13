function [vol,gridxyzWord] = getMissingDepthVolume(cameraData, ARCUnit,mask)
grideSize = 0.01;
volSize = ARCUnit.Tote.Size/grideSize;

[gridx,gridy,gridz] = ndgrid(1:volSize(1),1:volSize(2),1:volSize(3));
gridxyz = [gridx(:),gridy(:),gridz(:)];
gridxyzWord = bsxfun(@plus,(gridxyz - 1)*grideSize,ARCUnit.Tote.Range(1:3));
% preject them into each view 
Volviews = cell(1,length(ARCUnit.cameraIds));
imSize = size(cameraData.depth{1});
for i = 1:length(ARCUnit.cameraIds)
    [depth_projected,~,proj_xyz]=proj3DPtsTo2D(gridxyzWord',...
                                                 ARCUnit.cameraparam{i}.ext,ARCUnit.cameraparam{i}.int,...
                                                 imSize,[]) ;
                                                 
    isValid = find(proj_xyz(1,:)>=1&proj_xyz(1,:)<imSize(2)&...
                   proj_xyz(2,:)>=1&proj_xyz(2,:)<imSize(1));

    inMask = mask{i}(sub2ind(size(depth_projected),proj_xyz(2,isValid),proj_xyz(1,isValid)));  
    %depthRead = cameraData.depth{i}(sub2ind(size(depth_projected),proj_xyz(2,isValid),proj_xyz(1,isValid)));  
    %hasDepth = depthRead(:)>0;
    %onSurface = depthRead(:)'-proj_xyz(3,isValid)<0.005;
    vol_inMask = zeros(volSize);
    vol_inMask((sub2ind(size(vol_inMask),gridxyz(isValid(inMask),1),gridxyz(isValid(inMask),2),gridxyz(isValid(inMask),3)))) = 1;
    Volviews{i} = vol_inMask;
end
vol = Volviews{1}&Volviews{2};
end
%{
vis_point_cloud(gridxyz(isValid(onSurface),:),'g');
hold on;
vis_point_cloud(gridxyz(isValid(inMask),:),'r');
%}