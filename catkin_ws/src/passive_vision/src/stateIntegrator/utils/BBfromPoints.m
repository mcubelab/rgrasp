function [bb3dAlginedZ,bb3dTight] = BBfromPoints(objPts)
        % objPts is 3xN point could
        [coeffPCA,scorePCA,latentPCA] = pca(objPts');
        Rot = [coeffPCA(:,1),coeffPCA(:,2),cross(coeffPCA(:,1),coeffPCA(:,2))]; % Follow righthand rule
        
        Vproj = Rot'*objPts;
        [projmin] = min(Vproj,[],2);
        [projmax] = max(Vproj,[],2);
        
        
        centroid = Rot*[0.5*(projmax(1)+projmin(1));...
                          0.5*(projmax(2)+projmin(2)); ...
                          0.5*(projmax(3)+projmin(3))];
       
        bb3dTight.basis = Rot';
        bb3dTight.coeffs = [projmax(1)-projmin(1), projmax(2)-projmin(2), projmax(3)-projmin(3)]/2;
        bb3dTight.centroid = centroid';

        bb3dAlginedZ = getBBAlginedZ(objPts);
end


function bb3dAlginedZ = getBBAlginedZ(objPts)
        zmin = min(objPts(3,:));
        zmax = max(objPts(3,:));
        [C,V]= pca(objPts(1:2,:)');
        Vproj = C'*objPts(1:2,:);
        [projmin] = min(Vproj,[],2);
        [projmax] = max(Vproj,[],2);
        bb3dAlginedZ = [];
        bb3dAlginedZ.basis = [C' [0;0]; 0 0 1]; % one row is one basis
        centroid2D = C*[0.5*(projmax(1)+projmin(1)); 0.5*(projmax(2)+projmin(2))];
        bb3dAlginedZ.coeffs = [projmax(1)-projmin(1), projmax(2)-projmin(2), zmax-zmin]/2;
        bb3dAlginedZ.centroid = [centroid2D(1), centroid2D(2), 0.5*(zmax+zmin)];
end