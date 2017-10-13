function [heightMap,heightMapSeg] = projHeightMap(allModelPoints_combine,allModelIds_combine,ARCUnit)
    heightMap = nan(ARCUnit.heightmapSize);
    heightMapSeg  = zeros(ARCUnit.heightmapSize);
    if size(allModelPoints_combine,2)>1
        allModelPoints_combine_grid = round(bsxfun(@minus,allModelPoints_combine([1:2],:),...]
                                      ARCUnit.Tote.Range([1,2])')/ARCUnit.heightmapGridSize);
        [~,heightInd]= sort(allModelPoints_combine(3,:),'ascend');    

        allModelPoints_combine_grid = allModelPoints_combine_grid(:,heightInd);
        allModelIds_combine = allModelIds_combine(:,heightInd);
        allModelPoints_combine  = allModelPoints_combine(:,heightInd);
        isValid = allModelPoints_combine_grid(1,:)>0&allModelPoints_combine_grid(1,:)<=ARCUnit.heightmapSize(1)...
                  &allModelPoints_combine_grid(2,:)>0&allModelPoints_combine_grid(2,:)<=ARCUnit.heightmapSize(2);


        heightMap(sub2ind(ARCUnit.heightmapSize,allModelPoints_combine_grid(1,isValid),allModelPoints_combine_grid(2,isValid))) = allModelPoints_combine(isValid);
        heightMapSeg(sub2ind(ARCUnit.heightmapSize,allModelPoints_combine_grid(1,isValid),allModelPoints_combine_grid(2,isValid))) = allModelIds_combine(isValid);
    end
end