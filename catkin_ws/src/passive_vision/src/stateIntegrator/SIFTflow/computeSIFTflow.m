function [lable_des]=computeSIFTflow(im_des, im_src, lable_src)

im_des=double(im_des)/255;
im_src=double(im_src)/255;
im_des = imfilter(im_des,fspecial('gaussian',7,1.),'same','replicate');
im_src = imfilter(im_src,fspecial('gaussian',7,1.),'same','replicate');

im_des = imresize(im_des,0.5,'bicubic');
im_src = imresize(im_src,0.5,'bicubic');
lable_src = imresize(lable_src,0.5,'nearest');

cellsize=3;
gridspacing=1;

sift1 = mexDenseSIFT(im_des,cellsize,gridspacing);
sift2 = mexDenseSIFT(im_src,cellsize,gridspacing);

SIFTflowpara.alpha=2*255;
SIFTflowpara.d=40*255;
SIFTflowpara.gamma=0.005*255;
SIFTflowpara.nlevels=2;
SIFTflowpara.wsize=2;
SIFTflowpara.topwsize=10;
SIFTflowpara.nTopIterations = 60;
SIFTflowpara.nIterations= 30;


[vx,vy,energylist]=SIFTflowc2f(sift1,sift2,SIFTflowpara);
lable_des = warpImage(lable_src,vx,vy,'nearest');
lable_des = imresize(lable_des,2,'nearest');
end