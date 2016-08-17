
ros = require 'ros'
ros.init('DQN_comms')
local classic = require 'classic'
local Baxter, super = classic.class('Baxter', Env)
local Baxter = {};

--Globals required to extract info from message callbacks
resp_ready = false

function connect_cb(name, topic)
  print("subscriber connected: " .. name .. " (topic: '" .. topic .. "')")
end


function disconnect_cb(name, topic)
  print("subscriber diconnected: " .. name .. " (topic: '" .. topic .. "')")
end


function Baxter:_init(opts)
	opts = opts or {}

	--setup state variables
	self.img_size = 60
	self.screen = torch.ByteTensor(3,self.img_size,self.img_size):zero()
	self.data = torch.ByteTensor(10800,1):zero()
	self.raw_msg = self.data
	self.task = false

	--self.data = torch.ByteTensor(1920000,1):zero()
	--setup ros node and spinner (processes queued send and receive topics)
	self.spinner = ros.AsyncSpinner()
	self.spinner:start()
	self.nodehandle = ros.NodeHandle()
	
	--Message Formats
	self.string_spec = ros.MsgSpec('std_msgs/String')
	self.bool_spec = ros.MsgSpec('std_msgs/Bool')
	self.image_spec = ros.MsgSpec('sensor_msgs/Image')
	self.log_spec = ros.MsgSpec('rosgraph_msgs/Log')
	self.clock_spec = ros.MsgSpec('rosgraph_msgs/Clock')


	self.publisher = self.nodehandle:advertise("chatter",
	self.string_spec, 100, false, connect_cb, disconnect_cb)
	ros.spinOnce()

	--create subscriber
	self.timeout = 20.0
	--Need to edit string in following line!
	self.img_subscriber = self.nodehandle:subscribe("baxter_view", self.image_spec, 		100, 		{ 'udp', 'tcp' }, { tcp_nodelay = true })
	
	self.task_subscriber = self.nodehandle:subscribe("task_state", self.bool_spec, 		100, 		{ 'udp', 'tcp' }, { tcp_nodelay = true })
	
	--Received message callback
	self.img_subscriber:registerCallback(function(msg, header)
		--get state image from msg.data, and messages from msg.header.frame_id
		--this format allows iamge and string data to be sent in single message
		local frame_id = msg.header.frame_id
		print(frame_id)
		-- get raw message data
		
		self.raw_msg = msg.data
		--self.msgToImg()
		--Split reward frame id and terminal status (arrive in comma separated list)
		self.task= tonumber(frame_id)	
		resp_ready = true

	end)
	
	ros.spinOnce()

	resp_ready = false
end

function Baxter:sendMessage(message)
	--Wait for a subscriber
	local t = os.clock()
	while self.publisher:getNumSubscribers() == 0 do
		if (os.clock() - t) < self.timeout then
			io.write(string.format("waiting for subscriber: %.2f \r",(os.clock() -t)))
		else
			io.write("Wait for subscriber timed out \n")
			return false
		end
	end

	-- Send Message
	resp_ready=false
	m = ros.Message(self.string_spec)
	m.data = message
	self.publisher:publish(m)
	return true
end

-- Wait for response message to be recieved - sending message is blocking function
function Baxter:waitForResponse(message)
	local t = os.clock()
	local tries = 1
	while not resp_ready do 
		if (os.clock() - t) < self.timeout then
			sys.sleep(0.1)
			ros.spinOnce()
		else
			if tries > 3 then
				print("Exceeded number of attempts \n")
				self:_close()
				return false
			else
				print("Robot response timed out. Resending " .. message .. "\n")
				self.sendMessage(message)
				ros.spinOnce()
				t = os.clock()
				tries = tries + 1
			end
		end
	end
	return true
end

function Baxter:msgToImg()
		-- Sort message data - pixel values come through in order r[1], g[1], b[1], r[2], b[2], g[2], .. etc
	for i = 1, 10800 do
		if i%3==1 then
			self.data[(i+2)/3] = self.raw_msg[i]
		elseif i%3==2 then
			self.data[3600 + (i+2)/3] = self.raw_msg[i]
		else
			self.data[7200 + (i+2)/3] = self.raw_msg[i]
		end
	end

	self.screen = torch.reshape(self.data,3,self.img_size,self.img_size)
end

	
function Baxter:_close()
	self.subscriber:shutdown()
	ros.shutdown()
end

return Baxter
