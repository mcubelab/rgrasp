function visulizeState(ARCUnit,MetaData,param,filename)
tic;
% know whether we have desktop or not. No desktop -> real run -> don't need to display images
havedesktop = usejava('desktop'); 

im_vis =  cell(1,length(ARCUnit.Curr.color));
for i = 1:length(ARCUnit.Curr.color)
    im = ARCUnit.Curr.color{i}/255;
    if isfield(ARCUnit.Curr,'instanceseg')
        instanceseg = getImagesc(ARCUnit.Curr.instanceseg{i});
        im_vis{i}  = 0.4*im+0.7*instanceseg;
        if param.saveImage
            imwrite(im_vis{i},[filename{i},'.ppm']); 
            ppmtopng_and_remove_ppm_nowait([filename{i},'.ppm'], filename{i});
        end
    end
    
end

if havedesktop && param.show2D
    figure(1);clf;
    subplot(2,2,1); imshow(im_vis{1});
    try subplot(2,2,2); imagesc(ARCUnit.Curr.categorySegMap{1}); axis equal;end
    
    
    subplot(2,2,3); imshow(im_vis{2});
    try subplot(2,2,4); imagesc(ARCUnit.Curr.categorySegMap{2}); axis equal;end
    %{
            try figure(3);clf;
                imagesc(ARCUnit.Curr.heightMapSegProjected);
                %imagesc(ARCUnit.Curr.heightMapProjected);
                axis equal;
            end
    %}
end

fprintf('\nSummary: [obj] [confidence] [visibility]\n')
for i = 1:ARCUnit.Curr.numodobject
    fprintf('%s: %.2f %.2f',ARCUnit.Curr.objectList(i).objectName,...
        ARCUnit.Curr.objectList(i).conf, ...
        ARCUnit.Curr.objectList(i).visibility)
    for j = 1:length(ARCUnit.Curr.objectList(i).topobj)
        fprintf(' covered by %s ',ARCUnit.Curr.objectList(ARCUnit.Curr.objectList(i).topobj(j)).objectName);
    end
    fprintf('\n')
end

if param.show3D && havedesktop
    figure(3)
    clf;
    for i = 1:length(ARCUnit.cameraIds)
        vis_point_cloud(ARCUnit.Curr.points3d{i}','r');
        hold on;
    end
    for i = 1:ARCUnit.Curr.numodobject
        objModelCloud =getModelPC(MetaData,ARCUnit.Curr.objectList(i).objectName);
        ptCloudAligned = transformPointCloud(objModelCloud.Location',ARCUnit.Curr.objectList(i).Pose);
        vis_point_cloud(ptCloudAligned',repmat(rand(1,3),[length(ptCloudAligned),1]));
        hold on;
    end
    view(-90,90);
end

if havedesktop
    if param.pause
        pause;
    else
        drawnow;
    end
end

fprintf('[Matlab Timing visulizeState]'); toc

end
