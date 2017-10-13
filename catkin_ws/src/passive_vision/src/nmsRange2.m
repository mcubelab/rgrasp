% A sped up version of nmsRange()
function finalPick = nmsRange2(scores,range,minvalue)
%tic;
finalPick = zeros(size(scores));
changeScore = scores;
changeScore(changeScore<=minvalue) = 0.0; % suppres the scores that are under minvalue

xdim = size(changeScore,1);
ydim = size(changeScore,2);

ind_havevalue = find(changeScore(:));
score_ind_list = [changeScore(ind_havevalue), ind_havevalue];  % [[score1, ind1];[score2, ind2];[score2, ind2]]

sorted_score_ind_list = sortrows(score_ind_list, 1, 'descend');  % sort the scores first and retrive them from high to low

for i = 1:size(sorted_score_ind_list,1)
    currMax = sorted_score_ind_list(i,1);
    currInd = sorted_score_ind_list(i,2);
       
    y = fix((currInd-1) / xdim) + 1;  % manually doing the conversion is faster than ind2sub
    x = currInd - (y-1)*xdim;
    if(changeScore(x,y) < 1e-8)  % already got suppressed
        continue;
    end
    
    finalPick(x,y) = currMax;
    xstart = max(x-range,1);
    xend = min(x+range,xdim);
    ystart = max(y-range,1);
    yend = min(y+range,ydim);
    changeScore(xstart:xend, ystart:yend) = 0.0;
end
%fprintf('[Matlab Timing nmsRange2] '); toc
end
