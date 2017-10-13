load('scores.mat');
finalPick = nmsRange(scores, 4, 0.2);
finalPick2 = nmsRange2(scores, 4, 0.2);
diff = finalPick2 - finalPick;
if(max(abs(diff(:))) == 0.0)
    fprintf('Pass 1\n');
end

scores = scores * 0.0;
finalPick = nmsRange(scores, 4, 0.2);
finalPick2 = nmsRange2(scores, 4, 0.2);
diff = finalPick2 - finalPick;
if(max(abs(diff(:))) == 0.0)
    fprintf('Pass 2\n');
end
