function [depth_projected,instance_seg,allModelPots_p] =proj3DPtsTo2D(allPts,RT,K,imSize,allIds) % ACRUnit.cameraparam{i}.int
         depth_projected = nan(imSize);
         instance_seg = zeros(imSize);
        
         if length(allPts)>1
            allModelPots_t = transformPointCloud(allPts, inv(RT));
            allModelPots_p = K*allModelPots_t;
            allModelPots_p(1:2,:) = bsxfun(@rdivide,allModelPots_p(1:2,:),allModelPots_p(3,:));
            allModelPots_p(1:2,:) = round(allModelPots_p(1:2,:));
            depth = allModelPots_p(3,:);


            [~,depthInd]= sort(depth,'descend');
            proj_xy =allModelPots_p(1:2,depthInd);
            depth =depth(depthInd);


            isValid = proj_xy(1,:)>=1&proj_xy(1,:)<imSize(2)&...
                   proj_xy(2,:)>=1&proj_xy(2,:)<imSize(1);

            depth_projected(sub2ind(size(depth_projected),proj_xy(2,isValid),proj_xy(1,isValid))) = depth(isValid);

            if ~isempty(allIds)
                allModelIds_sort = allIds(depthInd);
                instance_seg(sub2ind(size(depth_projected),proj_xy(2,isValid),proj_xy(1,isValid))) = allModelIds_sort(isValid);
            end
         end
end