local nn = require 'nn'
require 'dpnn'
require 'classic.torch' -- Enables serialisation
local Body = classic.class('Body')
function Body:_init(opts)
  opts = opts or {}
end

function Body:createBody()
  local imageNet = nn.Sequential()
	imageNet:add(nn.Narrow(2, 1, 3)) -- Extract 1st 3 channels
	imageNet:add(nn.SpatialConvolution(3, 4, 3, 3))
	imageNet:add(nn.ReLU(true))
	imageNet:add(nn.SpatialConvolution(4, 2, 2, 2))
	imageNet:add(nn.ReLU(true))
	imageNet:add(nn.View(18):setNumInputDims(3)) -- Unroll into vector

	local motorNet = nn.Sequential()
	motorNet:add(nn.Narrow(2, 4, 1)) -- Extract 4th channel
	motorNet:add(nn.Narrow(3, 1, 1)) -- Extract motor inputs row
	motorNet:add(nn.Narrow(4, 1, 1)) -- Extract 3 motor inputs
	motorNet:add(nn.View(1):setNumInputDims(1)) -- Unroll into vector (narrow preserves dimensionality)
	
	--[[
	motorNet:add(nn.Squeeze()) -- Remove singleton dimension
	motorNet:add(nn.Squeeze()) -- Remove singleton dimension
	--]]
	
	local branches = nn.ConcatTable() -- Apply each module to input
	branches:add(imageNet)
	branches:add(motorNet)

	local net = nn.Sequential()
	net:add(nn.View(-1, 4, 6, 6)) -- Always pass through as batch
	net:add(branches)

	net:add(nn.JoinTable(1, 1))
	net:add(nn.Squeeze()) -- Squashes real or fake batches with n = 1
  return net
end

return Body
