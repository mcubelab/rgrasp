function finalPick = nmsRange(scores,range,minvalue)
tic;
finalPick = zeros(size(scores));
changeScore = scores;

while sum(changeScore(:)>minvalue)>0
    [currMax,currInd] = max(changeScore(:));
    [x,y] =  ind2sub(size(scores),currInd);
    finalPick(x,y) = currMax;
    changeScore(max(x-range,1):min(x+range,size(changeScore,1)),...
                max(y-range,1):min(y+range,size(changeScore,2))) = 0;
end
fprintf('[Matlab nmsRange] '); toc
end