%% MetaData holds data about object, path
clear all;
addpath('utils');
addpath('SIFTflow');
addpath('SIFTflow/mexDiscreteFlow');
addpath('SIFTflow/mexDenseSIFT');
warning('off','images:imshow:magnificationMustBeFitForDockedFigure');

useGPU = 1;
if useGPU
    global KNNSearchGPU;
    fprintf('Setting up CUDA kernel functions...\n');
    KNNSearchGPU = parallel.gpu.CUDAKernel('KNNSearch.ptx','KNNSearch.cu');
    fprintf('Done setting up CUDA kernel functions...\n');
end
mode ='offline';

MetaData = setMetadata(mode);
MetaData = preloadModels(MetaData);
[allBinsData,allBinName] = readbinfile(MetaData);
StateHistory(1) = 0;
%statefile = fullfile(MetaData.image_dir, '1494989337/states.txt');
%statefile = fullfile(MetaData.image_dir, '1495220549/states.txt');
statefile = fullfile(MetaData.image_dir, '1495220550/states.txt');
%% loop through all the states
hasInit = 0;
fid = fopen(statefile);
tline = fgets(fid);
while ischar(tline)
    fprintf('%s\n',tline);
    arcmesg = parseMessage(tline,mode);
    if ~hasInit 
        allBinsData = initBinBG(allBinsData,MetaData,arcmesg);
        hasInit = 1;
        fprintf('init first frame\n');
    else
        %% compute current state
        [~,bin_id]=ismember(arcmesg.binname,allBinName);
        [ARCunitNew,succ] = stateUpdate(arcmesg, allBinsData{bin_id},MetaData,StateHistory,useGPU);
        
        %% visulize 
        filename{1} = fullfile(MetaData.image_dir,sprintf('passive-vision-state.%s.%d.debug.png',arcmesg.binname,0));
        filename{2} = fullfile(MetaData.image_dir,sprintf('passive-vision-state.%s.%d.debug.png',arcmesg.binname,1));
        param = struct('show2D',1,'show3D',1,'pause',1,'saveImage',0);    
        visulizeState(ARCunitNew,MetaData,param,filename)
        %%
        allBinsData{bin_id} = ARCunitNew;
        writeStateTofile(MetaData,allBinsData{bin_id},bin_id)
    end
    %% go to next state
    tline = fgets(fid);
end
fclose(fid);



