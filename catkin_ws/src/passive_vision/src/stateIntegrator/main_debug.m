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
mode ='debug';

MetaData = setMetadata(mode);
MetaData = preloadModels(MetaData);
[allBinsData,allBinName] = readbinfile(MetaData);
%MetaData.image_dir = '/home/mcube/shuran_debug/seq3/';
MetaData.image_dir = '~/arcdata/loopdata/seq19_comb/';
dbstop if error 
statefile = fullfile(MetaData.image_dir, 'states.txt');

%% loop through all the states
hasInit = 0;
fid = fopen(statefile,'r');
tline = fgets(fid);
while ischar(tline)
    fprintf('%s\n',tline);
    arcmesg = parseMessage(tline,mode);
    tmp_statefile = fullfile(MetaData.image_dir,'allBinsData.mat');
    if strcmp(arcmesg.mode,'loadprev') && exist(tmp_statefile,'file')
       fprintf('[ MATLAB ]: load previous state.\n');
       load(tmp_statefile)
       hasInit = 1;
   elseif strcmp(arcmesg.mode,'restart')
        for bin_id = 1:length(allBinsData)
            allBinsData{bin_id}.Curr = allBinsData{bin_id}.BG;
            allBinsData{bin_id}.numodobject = 0;
            fprintf('[ MATLAB ]: Bin  %s is restarted\n',bin_id);
        end
    else
        if ~hasInit 
            allBinsData = initBinBG(allBinsData,MetaData,arcmesg);
            hasInit = 1;
            fprintf('init first frame\n');
        end
        %% compute current state
        [~,bin_id]=ismember(arcmesg.binname,allBinName);
        [ARCunitNew,succ] = stateUpdate(arcmesg, allBinsData{bin_id},MetaData,1,useGPU);

        %% visulize 
        filename{1} = fullfile(MetaData.image_dir,sprintf('passive-vision-state.%s.%d.debug.png',arcmesg.binname,0));
        filename{2} = fullfile(MetaData.image_dir,sprintf('passive-vision-state.%s.%d.debug.png',arcmesg.binname,1));
        param = struct('show2D',1,'show3D',0,'pause',1,'saveImage',1);    
        visulizeState(ARCunitNew,MetaData,param,filename)
        %%
        allBinsData{bin_id} = ARCunitNew;
        writeStateTofile(MetaData,allBinsData{bin_id},bin_id)
        tic;save(tmp_statefile,'allBinsData','-v6');toc; 
    end

    %% go to next state
    tline = fgets(fid);
end
fclose(fid);



