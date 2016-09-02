local nn = require 'nn'
require 'dpnn'
require 'classic.torch' -- Enables serialisation

local Body = classic.class('Body')

-- Architecture based on "Learning Hand-Eye Coordination for Robotic Grasping with Deep Learning and Large-Scale Data Collection"

local histLen = 4
local numFilters = 64
local motorInputs = 6
local batchSize = 32
local size = 60

local data = torch.FloatTensor(batchSize, histLen, 4, size, size):uniform() -- Minibatch
data[{{}, {}, {4}, {}, {}}]:zero() -- Zero motor inputs
data[{{}, {}, {4}, {1}, {1, motorInputs}}] = 2 -- 3 motor inputs

function Body:_init(opts)
  opts = opts or {}
end

function Body:createBody()
	local imageNet = nn.Sequential()
	imageNet:add(nn.Narrow(3, 1, 3)) -- Extract 1st 3 (RGB) channels
	imageNet:add(nn.View(histLen * 3, size, size):setNumInputDims(4)) -- Concatenate in time
	imageNet:add(nn.SpatialConvolution(histLen * 3, numFilters, 5, 5, 2, 2, 1, 1))
	imageNet:add(nn.SpatialBatchNormalization(numFilters))
	imageNet:add(nn.ReLU(true))
	imageNet:add(nn.SpatialConvolution(numFilters, numFilters, 5, 5, 2, 2, 1, 1))
	imageNet:add(nn.SpatialBatchNormalization(numFilters))
	imageNet:add(nn.ReLU(true))
	imageNet:add(nn.SpatialConvolution(numFilters, numFilters, 5, 5, 2, 2, 1, 1))
	imageNet:add(nn.SpatialBatchNormalization(numFilters))
	imageNet:add(nn.ReLU(true))
	
	local convOutputSizes = imageNet:forward(data):size() -- Calculate spatial output size

	local motorNet = nn.Sequential()
	motorNet:add(nn.Narrow(3, 4, 1)) -- Extract 4th channel
	motorNet:add(nn.View(histLen, size, size):setNumInputDims(4)) -- Concatenate in time
	motorNet:add(nn.Narrow(3, 1, 1)) -- Extract motor inputs row
	motorNet:add(nn.Narrow(4, 1, motorInputs)) -- Extract motor inputs
	motorNet:add(nn.View(histLen * motorInputs):setNumInputDims(3)) -- Unroll
	motorNet:add(nn.Linear(histLen * motorInputs, numFilters)) -- Expand to number of convolutional feature maps
	motorNet:add(nn.ReLU(true))
	motorNet:add(nn.Replicate(convOutputSizes[3], 2, 1)) -- Replicate over height (begins spatial tiling)
	motorNet:add(nn.Replicate(convOutputSizes[4], 3, 2)) -- Replicate over width (completes spatial tiling)

	local branches = nn.ConcatTable() -- Apply each module to input
	branches:add(imageNet)
	branches:add(motorNet)

	local net = nn.Sequential()
	net:add(nn.View(-1, histLen, 4, size, size)) -- Always pass through as batch (not using setNumInputDims)
	net:add(branches)
	net:add(nn.JoinTable(1, 3))
	net:add(nn.SpatialConvolution(2 * numFilters, numFilters, 3, 3))
	net:add(nn.SpatialBatchNormalization(numFilters))
	net:add(nn.SpatialConvolution(numFilters, numFilters, 3, 3))
	net:add(nn.SpatialBatchNormalization(numFilters))


	local branches = nn.ConcatTable() -- Apply each module to input
	branches:add(imageNet)
	branches:add(motorNet)

	local net = nn.Sequential()
	net:add(nn.View(-1, histLen, 4, size, size)) -- Always pass through as batch (not using setNumInputDims)
	net:add(branches)
	net:add(nn.JoinTable(1, 3))
	net:add(nn.SpatialConvolution(2 * numFilters, numFilters, 3, 3))
	net:add(nn.SpatialBatchNormalization(numFilters))
	net:add(nn.SpatialConvolution(numFilters, numFilters, 3, 3))
	net:add(nn.SpatialBatchNormalization(numFilters))
	
	return net
end

return Body




