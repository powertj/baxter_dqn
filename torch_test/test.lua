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
	image.display(demo.screen)
	print(demo.task)
	if (demo.task == 1) then
		print("Task Completed")
		demo:sendMessage("reset")
		demo:waitForResponse("reset")
		end
	print("Ready for command: r: right turn, l: left turn, p: pickup object")
	local cmd = io.read()
	if (cmd == e) then
		demo:_close()
		break
	end
	demo:sendMessage(cmd)
	demo:waitForResponse(cmd)
	
end



