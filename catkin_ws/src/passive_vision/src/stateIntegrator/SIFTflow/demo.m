im_des=imread('1494989516.color.png');
im_src=imread('1494989544.color.png');

im_des=imresize(imfilter(im_des,fspecial('gaussian',7,1.),'same','replicate'),0.5,'bicubic');
im_src=imresize(imfilter(im_src,fspecial('gaussian',7,1.),'same','replicate'),0.5,'bicubic');

im_des=im2double(im_des);
im_src=im2double(im_src);

%figure;imshow(im1);figure;imshow(im2);

cellsize=3;
gridspacing=1;

addpath(fullfile(pwd,'mexDenseSIFT'));
addpath(fullfile(pwd,'mexDiscreteFlow'));

sift1 = mexDenseSIFT(im_des,cellsize,gridspacing);
sift2 = mexDenseSIFT(im_src,cellsize,gridspacing);

SIFTflowpara.alpha=2*255;
SIFTflowpara.d=40*255;
SIFTflowpara.gamma=0.005*255;
SIFTflowpara.nlevels=4;
SIFTflowpara.wsize=2;
SIFTflowpara.topwsize=10;
SIFTflowpara.nTopIterations = 60;
SIFTflowpara.nIterations= 30;


tic;[vx,vy,energylist]=SIFTflowc2f(sift1,sift2,SIFTflowpara);toc

warpI2=warpImage(im_src,vx,vy);
figure;imshow(im_des);
figure;imshow(warpI2);
figure;imshow(im_src);
% display flow
clear flow;
flow(:,:,1)=vx;
flow(:,:,2)=vy;
figure;imshow(flowToColor(flow));

return;

% this is the code doing the brute force matching
tic;[flow2,energylist2]=mexDiscreteFlow(Sift1,Sift2,[alpha,alpha*20,60,30]);toc
figure;imshow(flowToColor(flow2));
