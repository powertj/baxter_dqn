require 'image'
local demo = require "Baxter"

demo:_init()

function sleep(n)
  os.execute("sleep " .. tonumber(n))
end


while true do
	sleep(1)
	ros.spinOnce()
	demo:msgToImg()
	-- first three channels of image contain rgb information
	-- 4th channel contains motor angle information
	imgd = demo.screen[{{1,3},{},{}}]
	image.display(imgd)
	-- wrist motor position (normalised to 255)
	print(demo.screen[4][1][1])
	print(demo.task)
	if (demo.task == 1) then
		print("Task Completed")
		demo:sendMessage("reset")
		demo:waitForResponse("reset")
		end
	print("Ready for command")
	local cmd = io.read()
	if (cmd == e) then
		demo:_close()
		break
	end
	demo:sendMessage(cmd)
	demo:waitForResponse(cmd)
	
end



