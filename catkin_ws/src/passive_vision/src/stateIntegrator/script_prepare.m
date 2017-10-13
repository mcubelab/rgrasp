clear all;
genPointCould = 1;

%category_dir = '/n/fs/sun3d/arc/data/itemdata/';
category_dir = '/home/mcube/arcdata/itemdata/';
itemDataDir = category_dir;

addpath(genpath('utils'));

% generate object list 
allfolder = dir(category_dir);
allfolder= allfolder([allfolder.isdir]&~ismember({allfolder.name},{'.','..'}));
categorylist = 'objectlist.csv';
fid = fopen(categorylist,'w');
fprintf(fid,'Index,FolderName,Weight,dx,dy,dz\n');
for i = 1:length(allfolder)
    if ~strcmpi(allfolder(i).name,'empty')
        tmp = loadjson(fullfile(category_dir,allfolder(i).name,[lower(allfolder(i).name) '.json']) );
        fprintf(fid,'%d,%s,%s,%f,%f,%f\n',i-1,allfolder(i).name,tmp.weight,tmp.dimensions(1),tmp.dimensions(2),tmp.dimensions(3));
    end
end

% Create a product image for 'Empty' class
fprintf('Preparing product images: Empty\n');
mkdir(itemDataDir,'Empty');
emptyFilename = fullfile(itemDataDir,'Empty','Empty_Top_01.png');
if exist(emptyFilename, 'file') ~= 2
    imwrite(uint8(zeros(3000,3000,3)),emptyFilename);
end
%{
% Augment product images with mirrors and flips
objList = dir(itemDataDir);
objList = objList(3:end);
for objIdx = 1:length(objList)
    fprintf('Preparing: %s\n',objList(objIdx).name);
    imgList = dir(fullfile(itemDataDir,objList(objIdx).name,'*_0*.png'));
    for imgIdx = 1:length(imgList)
        colorFilename = fullfile(itemDataDir,objList(objIdx).name,imgList(imgIdx).name);
        k = strfind(colorFilename,'Flip');
        if isempty(k)
            img = imread(colorFilename);
            img = imresize(img,[224,224]);

            flipXFilename = [colorFilename(1:(end-7)),'_FlipX_',colorFilename((end-5):(end-4)),'.png'];
            if exist(flipXFilename, 'file') ~= 2
                flipX = fliplr(img);
                imwrite(flipX,flipXFilename);
            end
            flipYFilename = [colorFilename(1:(end-7)),'_FlipY_',colorFilename((end-5):(end-4)),'.png'];
            if exist(flipYFilename, 'file') ~= 2
                flipY = flipud(img);
                imwrite(flipY,flipYFilename);
            end
            flipXYFilename = [colorFilename(1:(end-7)),'_FlipXY_',colorFilename((end-5):(end-4)),'.png'];
            if exist(flipXYFilename, 'file') ~= 2
                flipXY = flipud(fliplr(img));
                imwrite(flipXY,flipXYFilename);
            end
        end
    end
end
%}



    
fprintf('Image Data preparation complete.\n');
%% processe point could
if genPointCould
    for objId = 1:length(allfolder)
        if strcmpi(allfolder(objId).name,'empty')
            continue;
        end
        fprintf('Preparing PointCould: %d %s\n',objId, allfolder(objId).name);
        tic;
        objectFolderName = allfolder(objId).name;
        objectName = lower(allfolder(objId).name);

        itemprodimag = dir(fullfile(category_dir,objectFolderName,['/*.png']));
        plyfilename = dir(fullfile(category_dir,objectFolderName,['/*.pcd']));
        if isempty(plyfilename)
           plyfilename = dir(fullfile(category_dir,objectFolderName,['/*.ply']));
        end
        outFile = fullfile(category_dir,objectFolderName,[plyfilename(1).name(1:end-4) '_bdir.mat']);
        if ~strcmpi(allfolder(i).name,'empty')%~exist(outFile,'file')&&~strcmpi(allfolder(i).name,'empty')
            pointCould_raw = pcread(fullfile(category_dir,objectFolderName,plyfilename(1).name));
            % denose it 
            try 
                pts = denoisePointCloud(pointCould_raw.Location');
            catch 
                pts = pointCould_raw.Location';
            end
            % center it 
            [coeffPCA,scorePCA,latentPCA] = pca(pts');
             if size(latentPCA,1) < 3
                latentPCA = [latentPCA;0];
             end
             coeffPCA = [coeffPCA(:,1),coeffPCA(:,2),cross(coeffPCA(:,1),coeffPCA(:,2))]; % Follow righthand rule
             bottomDirOrg = [0;0;1];
             bottomDir = coeffPCA'*bottomDirOrg;
             pts_algined = coeffPCA'*pointCould_raw.Location';
             pts_algined_centered = bsxfun(@minus, pts_algined, median(pts_algined,2));

             % algin PCA in x, y
             objModelCloud = pointCloud(pts_algined_centered');
             normals = pcnormals(objModelCloud,100); 
             objModelCloud.Normal = normals;

             info.bottomDir = bottomDir;
             save(fullfile(category_dir,objectFolderName,[plyfilename(1).name(1:end-4) '_bdir.mat']),'objModelCloud','info');
             %pcwrite(objModelCloud,fullfile(category_dir,objectFolderName,[plyfilename(1).name(1:end-4) '_bdir.ply']));
             toc;
         end
         %{ 
            clf
            x = objModelCloud.Location(1:100:end,1);
            y = objModelCloud.Location(1:100:end,2);
            z = objModelCloud.Location(1:100:end,3);
            u = objModelCloud.Normal(1:100:end,1);
            v = objModelCloud.Normal(1:100:end,2);
            w = objModelCloud.Normal(1:100:end,3);
            quiver3(x,y,z,u,v,w);
            hold off; axis equal;


            clf
            x = objModelCloud.Location(1:end,1);
            y = objModelCloud.Location(1:end,2);
            z = objModelCloud.Location(1:end,3);
            u = objModelCloud.Normal(1:end,1);
            v = objModelCloud.Normal(1:end,2);
            w = objModelCloud.Normal(1:end,3);
            quiver3(x,y,z,u,v,w);
            hold off; axis equal;
         %}

         
    end
end

%exit();