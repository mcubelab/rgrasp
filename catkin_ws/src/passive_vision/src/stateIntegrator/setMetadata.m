function MetaData = setMetadata(mode) 
    
    if strcmp(mode,'offline')
             dataRoot = '/n/fs/sun3d/arc/data/';
             %dataRoot = '~/Documents/ARC/data/';
             dataRoot ='/home/mcube/arcdata/';
             MetaData.category_dir = fullfile(dataRoot,'itemdata/');
             MetaData.camera_dir = fullfile(dataRoot,'state_testdata/camerainfo/');
             MetaData.uintfile = fullfile(dataRoot,'state_testdata/camerainfo/bins.txt');
             MetaData.image_dir = fullfile(dataRoot,'state_testdata/images/');
             MetaData.seg_dir = fullfile(dataRoot,'state_testdata/segmentation/');
             MetaData.categorylist = 'objectlist.csv';
    elseif strcmp(mode,'debug')
        dataPath = '/home/mcube/shuran_debug/SEQ1/';
        MetaData.image_dir = '/home/mcube/shuran_debug/SEQ1/';
        MetaData.category_dir = '/home/mcube/arcdata/itemdata/';
        MetaData.camera_dir = '/home/mcube/arc/catkin_ws/src/passive_vision/camerainfo/';
        MetaData.uintfile =  '/home/mcube/arc/catkin_ws/src/passive_vision/camerainfo/bins.txt';
        MetaData.seg_dir = fullfile(dataPath,'/segmentation/');
        MetaData.categorylist = 'objectlist.csv';
    else
        dataPath = '/home/mcube/arcdata/tmpdata';
        MetaData.image_dir = dataPath;
        MetaData.category_dir = '/home/mcube/arcdata/itemdata/';
        MetaData.camera_dir = '/home/mcube/arc/catkin_ws/src/passive_vision/camerainfo/';
        MetaData.uintfile =  '/home/mcube/arc/catkin_ws/src/passive_vision/camerainfo/bins.txt';
        MetaData.seg_dir = fullfile(dataPath,'/segmentation/');
        MetaData.categorylist = 'objectlist.csv';
    end
    MetaData.mode = mode;

end