% a faster version for sub2ind for 2d case
function linIndex = sub2ind2d(sz, rowSub, colSub)
  linIndex = (colSub-1) * sz(1) + rowSub;

