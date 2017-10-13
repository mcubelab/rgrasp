%script_yaml2bins
% read bin center
yaml_filefolder = '/home/mcube/arc/catkin_ws/src/apc_config';
passive_vision_folder = '//home/mcube/arc/catkin_ws/src/passive_vision/';
binCenter = zeros(4,3);
for binid = 0:3
    fid = fopen(fullfile(yaml_filefolder,['bin' num2str(binid) '_pose.yaml']),'r');
    tline = fgets(fid);
    while ischar(tline)
        data = strsplit(tline);
        if strcmp(data{1},['/bin' num2str(binid) '_pose/x:'])
            binCenter(binid+1,1) = str2double(data{2});
        end

        if strcmp(data{1},['/bin' num2str(binid) '_pose/y:'])
            binCenter(binid+1,2) = str2double(data{2});
        end
        if strcmp(data{1},['/bin' num2str(binid) '_pose/z:'])
            binCenter(binid+1,3) = str2double(data{2});
        end
        tline = fgets(fid);
    end
    fclose(fid);
end
binsHeight = [0.205 ,0.205 ,0.105,0.105];
for binid = 0:3
    binCenter(binid+1,3) = binCenter(binid+1,3)-binsHeight(binid+1);
end
% read camera 
cameraNames = {'tf','bf','tn','bn'};
binCamera = cell(4,4);
% bn bottom near = active vision 
fid = fopen(fullfile(yaml_filefolder,['cameras_per_bin.yaml']),'r');
tline = fgets(fid);
while ischar(tline)
    data = strsplit(tline);
    for binid = 0:3
        if strcmp(data{1},['/camera/bin' num2str(binid) '_passive_far:'])
           binCamera{binid+1,1} = (data{5}(2:end-2));
        end
        if strcmp(data{1},['/camera/bin' num2str(binid) '_active_far:'])
           binCamera{binid+1,2} = (data{5}(2:end-2));
        end
        if strcmp(data{1},['/camera/bin' num2str(binid) '_passive_near:'])
           binCamera{binid+1,3} = (data{5}(2:end-2));
        end
        if strcmp(data{1},['/camera/bin' num2str(binid) '_active_near:'])
           binCamera{binid+1,4} = (data{5}(2:end-2));
        end
    end
    tline = fgets(fid);
end
fclose(fid);

fid = fopen(fullfile(passive_vision_folder,'camerainfo/bins_new.txt'),'w');
for binid = 0:3
    fprintf(fid,'bin %d\n',binid);
    fprintf(fid,'pose %f %f %f\n',binCenter(binid+1,:));
    for camId = 1:4
        fprintf(fid,'%s %s\n',cameraNames{camId},binCamera{binid+1,camId});
    end
end
fclose(fid);