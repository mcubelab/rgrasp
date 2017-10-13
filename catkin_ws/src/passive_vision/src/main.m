fprintf('[MATLAB]: Matlab loaded.\n');

% % Set cleanup function (to exit Matlab if ctrl-c is pressed)
% cleanupObj = onCleanup(@cleanupFun);

% Set up state integrator
addpath('stateIntegrator/utils');
addpath('stateIntegrator/');
addpath('stateIntegrator/SIFTflow');
addpath('stateIntegrator/SIFTflow/mexDiscreteFlow');
addpath('stateIntegrator/SIFTflow/mexDenseSIFT');
addpath('bxf');

useGPU = 1;
showDebug = 1;
saveState = 1;

if useGPU
   global KNNSearchGPU;
   KNNSearchGPU = parallel.gpu.CUDAKernel('KNNSearch.ptx','KNNSearch.cu');
   fprintf('[MATLAB]: Done setting up CUDA kernel functions...\n');
end

mode ='online';
MetaData = setMetadata(mode);
MetaData = preloadModels(MetaData);
[allBinsData,allBinName] = readbinfile(MetaData);

% Start TCP server
tcpPort = 5007;
tcpConn = tcpip('127.0.0.1', tcpPort, 'NetworkRole', 'server');

% Wait for active connection
fopen(tcpConn);
fprintf('[MATLAB]: Connected to 127.0.0.1:%d\n',tcpPort);

starttime = char(datetime('now'));
starttime(starttime==' ')='_';
hasInit = 0;
% Process incoming signals
while true
    if tcpConn.BytesAvailable > 0
        tcpData = fread(tcpConn, tcpConn.BytesAvailable);
        tcpMsg = char(tcpData');
        fprintf('[MATLAB]: tcpMsg: %s\n',tcpMsg);
        
        
        % Process height map
        if strcmp(tcpMsg(1:2),'HM')
            binId = str2double(tcpMsg(4));
            getHeightMap(binId);
            
        % Compute suction and grasp predictions
        elseif strcmp(tcpMsg(1:4),'NULL')
            try
                binId = str2double(tcpMsg(6));
                if allBinsData{binId+1}.active
                    predictSuction(binId,allBinsData{binId+1},showDebug);
                    predictDeepGrasp(binId,allBinsData{binId+1},showDebug);
                end
            catch ee
                ee
            end
        
        % Process state update command
        else
            fprintf('[MATLAB]: Processing command: %s\n',tcpMsg);
            try
                fastMode =  strcmp(tcpMsg(1:4),'fast');
                if fastMode
                   tcpMsgState = tcpMsg(6:end);
                else
                   tcpMsgState = tcpMsg;
                end
                
                fprintf('[MATLAB]: State command: %s\n',tcpMsgState);
                arcmesg = parseMessage(tcpMsgState,mode);
                tmp_statefile = fullfile(MetaData.image_dir,'allBinsData.mat');
                if strcmp(arcmesg.mode,'loadprev') && exist(tmp_statefile,'file')
                   fprintf('[MATLAB]: load previous state.\n');
                   load(tmp_statefile)
                   prepareDataloadPrev(allBinsData,MetaData)
                   hasInit = 1;
                elseif strcmp(arcmesg.mode,'restart')
                       allBinsData = initBinBG(allBinsData,MetaData);
                       hasInit = 1;
                       fprintf('[MATLAB]: all Bins are restarted');
                else
                    if ~hasInit 
                        allBinsData = initBinBG(allBinsData,MetaData);
                        hasInit = 1;
                        fprintf('[MATLAB]: Init first frame for state.\n');
                    end

                    [~,bin_id]=ismember(arcmesg.binname,allBinName);
                    if bin_id>0 && allBinsData{bin_id}.active
                        [allBinsData{bin_id}] = stateUpdate(arcmesg, allBinsData{bin_id},MetaData,fastMode,useGPU);
                        if showDebug
                            filename{1} = fullfile(MetaData.image_dir,sprintf('passive-vision-state.%d.%d.debug.png',bin_id-1,0));
                            filename{2} = fullfile(MetaData.image_dir,sprintf('passive-vision-state.%d.%d.debug.png',bin_id-1,1));
                            param = struct('show2D',1,'show3D',0,'pause',0,'saveImage',1);
                            visulizeState(allBinsData{bin_id},MetaData,param,filename);
                        end
                        stateBinaryOutputPath = fullfile(MetaData.image_dir,sprintf('passive-vision-state.%d.output.bin',bin_id-1));
                        stateTextOutputPath = fullfile(MetaData.image_dir,sprintf('passive-vision-state.%d.output.txt',bin_id-1));
                        writeStateTofile(MetaData,allBinsData{bin_id},bin_id,stateBinaryOutputPath,stateTextOutputPath);
                        save(tmp_statefile,'allBinsData'); 
                        if fastMode
                            save([tmp_statefile '.fast.mat'],'allBinsData'); 
                           
                            stateBinaryOutputPath = fullfile(MetaData.image_dir,sprintf('passive-vision-state.%d.output.fast.bin',bin_id-1));
                            stateTextOutputPath = fullfile(MetaData.image_dir,sprintf('passive-vision-state.%d.output.fast.txt',bin_id-1));
                            writeStateTofile(MetaData,allBinsData{bin_id},bin_id,stateBinaryOutputPath,stateTextOutputPath);
%                            try
%                                copyfile('passive-vision-state.1.output.bin','passive-vision-state.1.output.fast.bin');
%                                copyfile('passive-vision-state.2.output.bin','passive-vision-state.2.output.fast.bin');
%                                copyfile('passive-vision-state.3.output.bin','passive-vision-state.3.output.fast.bin');
%                                copyfile('passive-vision-state.4.output.bin','passive-vision-state.4.output.fast.bin');
%                                copyfile('passive-vision-state.1.output.txt','passive-vision-state.1.output.fast.txt');
%                                copyfile('passive-vision-state.2.output.txt','passive-vision-state.2.output.fast.txt');
%                                copyfile('passive-vision-state.3.output.txt','passive-vision-state.3.output.fast.txt');
%                                copyfile('passive-vision-state.4.output.txt','passive-vision-state.4.output.fast.txt');
%                            catch ee
%                                ee
%                            end
                        end
                    else
                        fprintf('[ stateUpdate ] Try to update inactive Bin %s.\n',arcmesg.binname);
                    end
                    
                    if ~fastMode
                        binId = str2double(arcmesg.binname);
                        if allBinsData{binId+1}.active
                            predictSuction(binId,allBinsData{binId+1},showDebug);
                            predictDeepGrasp(binId,allBinsData{binId+1},showDebug);
                        end
                    end
                    
                end
                
                
                
                
            catch ee
                display(ee)
                display(ee.stack(1))
                display(ee.stack(end))
            end
        end
        fwrite(tcpConn, 'Done.');
    else
        pause(0.01);
    end
end
