require 'cutorch';
require 'cunn';
require 'cudnn'
require 'image'

-- require 'hdf5'

require 'utils'

require 'nn'
require 'nnx'

require 'inn'

require 'hdf5'


-- Constants
local arcDataDir = '/home/mcube/arcdata'
local tmpDataDir = '/home/mcube/arcdata/tmpdata'
local colorMean = {0.485,0.456,0.406}
local colorStd = {0.229,0.224,0.225}
local depthMean = {0.1,0.1,0.1}
local depthStd = {0.05,0.05,0.05}
local imH = 480
local imW = 640

local dg = false

-- Load trained model
local model = torch.load(paths.concat(arcDataDir,'suction-model.t7'))
model:add(cudnn.SpatialSoftMax())
model = model:cuda()
model:evaluate()

local graspModel
if dg then
    graspModel = torch.load(paths.concat(arcDataDir,'grasp-model.t7'))
    for i=1,8 do
      graspModel:get(5):get(i):add(cudnn.SpatialSoftMax())
    end
    graspModel = graspModel:cuda()
    graspModel:evaluate()
end

-- print(model)

-- Start TCP server
local socket = require("socket")
local host = "127.0.0.1"
local port = 5006
server = assert(socket.bind(host,port))
i,p   = server:getsockname()
assert(i, p)
print("[TORCH]: Torch loaded.")
client = assert(server:accept())
print("[TORCH]: Connected to "..i..":"..p)
l,e = client:receive()
-- print('begin:'..l)
binId = tonumber(string.sub(l,1,1))
while not e do
    l,e = client:receive()
    -- print('tic:'..l)
    -- print('[TORCH]: Recieved suction point prediction request for bin #'..binId..'.')

    if dg then

        -- Load and pre-process color image
        local filename = paths.concat(tmpDataDir,'passive-vision-height.'..binId..'.color.png')
        local colorImg = image.load(filename)
        for c=1,3 do
            colorImg[c]:add(-colorMean[c])
            colorImg[c]:div(colorStd[c])
        end

        -- Load and pre-process depth image
        filename = paths.concat(tmpDataDir,'passive-vision-height.'..binId..'.depth.png')
        local depth = image.load(filename)
        depth = depth*65536/10000
        depth = depth:clamp(0.0,1.2) -- Depth range of Intel RealSense SR300
        local depthImg = depth:cat(depth,1):cat(depth,1)
        for c=1,3 do
            depthImg[c]:add(-depthMean[c])
            depthImg[c]:div(depthStd[c])
        end

        local input = {colorImg:reshape(1, 3, 224, 320),depthImg:reshape(1, 3, 224, 320)}

        input[1] = input[1]:cuda()
        input[2] = input[2]:cuda()

        local tmpResult = graspModel:forward(input)

        local output = tmpResult[1]:float()
        for i=2,8 do
          output = output:cat(tmpResult[i]:float(),1)
        end

        -- Save grasp predictions to HDF5 file
        graspPredPath = paths.concat(tmpDataDir,'grasp-prediction-output.'..binId..'.hdf5')
        graspPredFile = hdf5.open(graspPredPath, 'w')
        graspPredFile:write('output', output)
        graspPredFile:close()

    end

    -- Load and pre-process color image
    local filename = paths.concat(tmpDataDir,'passive-vision-input.'..binId..'.0.color.png')
    local colorImg = image.load(filename)
    for i=1,3 do
        colorImg[i]:add(-colorMean[i])
        colorImg[i]:div(colorStd[i])
    end
    colorImg = colorImg:reshape(1,3,imH,imW)

    -- Load and pre-process depth image
    filename = paths.concat(tmpDataDir,'passive-vision-input.'..binId..'.0.depth.png')
    local depth = image.load(filename)
    depth = depth*65536/10000
    depth = depth:clamp(0.0,1.2) -- Depth range of Intel RealSense SR300
    local depthImg = depth:cat(depth,1):cat(depth,1)
    for c=1,3 do
        depthImg[c]:add(-colorMean[c])
        depthImg[c]:div(colorStd[c])
    end
    depthImg = depthImg:reshape(1,3,imH,imW)

    -- ConvNet forward pass
    local input = {colorImg:cuda(),depthImg:cuda()}
    local output = model:forward(input):float()

    -- Save suction predictions to HDF5 file
    local suctionPredPath = paths.concat(tmpDataDir,'suction-prediction-output.'..binId..'.0.hdf5')
    local suctionPredFile = hdf5.open(suctionPredPath, 'w')
    suctionPredFile:write('output', output)
    suctionPredFile:close()






    -- Load and pre-process color image
    local filename = paths.concat(tmpDataDir,'passive-vision-input.'..binId..'.1.color.png')
    local colorImg = image.load(filename)
    for i=1,3 do
        colorImg[i]:add(-colorMean[i])
        colorImg[i]:div(colorStd[i])
    end
    colorImg = colorImg:reshape(1,3,imH,imW)

    -- Load and pre-process depth image
    filename = paths.concat(tmpDataDir,'passive-vision-input.'..binId..'.1.depth.png')
    local depth = image.load(filename)
    depth = depth*65536/10000
    depth = depth:clamp(0.0,1.2) -- Depth range of Intel RealSense SR300
    local depthImg = depth:cat(depth,1):cat(depth,1)
    for c=1,3 do
        depthImg[c]:add(-colorMean[c])
        depthImg[c]:div(colorStd[c])
    end
    depthImg = depthImg:reshape(1,3,imH,imW)

    -- ConvNet forward pass
    local input = {colorImg:cuda(),depthImg:cuda()}
    local output = model:forward(input):float()

    -- Save suction predictions to HDF5 file
    suctionPredPath = paths.concat(tmpDataDir,'suction-prediction-output.'..binId..'.1.hdf5')
    suctionPredFile = hdf5.open(suctionPredPath, 'w')
    suctionPredFile:write('output', output)
    suctionPredFile:close()

    -- Send a reply back to the TCP client
    client:send("Done.")

    -- print('[TORCH]: Done.')
    l,e = client:receive()
    -- print(e)
    -- print('toc:'..l)
    binId = tonumber(string.sub(l,1,1))
end
print(e)
