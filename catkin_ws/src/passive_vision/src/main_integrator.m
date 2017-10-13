fprintf('[MATLAB]: Matlab loaded\n');

% % Set cleanup function (to exit Matlab if ctrl-c is pressed)
% cleanupObj = onCleanup(@cleanupFun);

% Start TCP server
tcpConn = tcpip('127.0.0.1', 5007, 'NetworkRole', 'server');

% Wait for active connection
fopen(tcpConn);
fprintf('[MATLAB]: Connected to 127.0.0.1:5007\n');


% set up integrator
addpath('stateIntegrator/utils');
addpath('stateIntegrator/');
addpath('stateIntegrator/SIFTflow');
addpath('stateIntegrator/SIFTflow/mexDiscreteFlow');
addpath('stateIntegrator/SIFTflow/mexDenseSIFT');


useGPU = 0;
if useGPU
    global KNNSearchGPU;
    fprintf('Setting up CUDA kernel functions...\n');
    KNNSearchGPU = parallel.gpu.CUDAKernel('KNNSearch.ptx','KNNSearch.cu');
    fprintf('Done setting up CUDA kernel functions...\n');
end

mode ='online';
MetaData = setMetadata(mode);
MetaData = preloadModels(MetaData);
[allBinsData,allBinName] = readbinfile(MetaData);
StateHistory(1) = 0;

% Process incoming signals
while true
    if tcpConn.BytesAvailable > 0
       if ~isempty(tcpConn.mesag)
          arcmesg = parseMessage(tcpConn.mesag,mode);
          [~,bin_id]=ismember(arcmesg.binname,allBinName);
          [ARCUnitNew,cameraData,StateHistory] = stateUpdate(arcmesg, allBinsData{bin_id},MetaData,StateHistory,useGPU);
          allBinsData{bin_id} = ARCUnitNew;
          param = struct('show2D',1,'show3D',0,'pause',0,'saveImage',0);
          visulizeState(allBinsData{bin_id},MetaData,param)
       end
       
       tcpData = fread(tcpConn, tcpConn.BytesAvailable);
       predictSuction(0);
       predictGrasp(0);
       fwrite(tcpConn, 'finished suction prediction');
       
    end
end
