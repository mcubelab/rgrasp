 function changepts = removePointsOutBound(changepts,centerxyz,ranges)
 minBound = centerxyz - ranges;
 maxBound = centerxyz + ranges;
 outInd =  changepts(1,:)<minBound(1)|changepts(1,:)>maxBound(1)|...
           changepts(2,:)<minBound(2)|changepts(2,:)>maxBound(2)|...
           changepts(3,:)<minBound(3)|changepts(3,:)>maxBound(3);
changepts(:,outInd) =[];
 
 end
 