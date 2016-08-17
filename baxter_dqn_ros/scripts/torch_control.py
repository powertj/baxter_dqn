#!/usr/bin/env python

# Joint controller - subscribes to messages from torch-ros to control joints
import roslib
import argparse
import math
import random
import rospy
import rospkg
import sys
import time
import cv2
from cv_bridge import CvBridge

from sensor_msgs.msg import Image
from gazebo_msgs.msg import ModelStates

from gazebo_msgs.srv import (
SpawnModel,
DeleteModel
)

from std_msgs.msg import (
UInt16,
String,
)

from geometry_msgs.msg import (
Pose,
Point,
Quaternion,
)

from IK import move_vertical

import baxter_interface
from baxter_interface import CHECK_VERSION

def load_gazebo_models():
	# Get Models' Path
	table_pose=Pose(position=Point(x=0.8, y=1.0, z=0.0))
	table_reference_frame="world"
	
    # Randomise object type and orientation
	object_angle= random.uniform(0,math.pi)
	object_type = random.randint(0,2)
    
	object_pose=Pose(position=Point(x=0.625, y=0.7975, z=0.8),orientation = \
   						Quaternion(x=0.0,y=0.0,z=math.sin(object_angle/2),w= \
   						math.cos(object_angle/2)))
	object_reference_frame="world"
	
	model_path = rospkg.RosPack().get_path('baxter_dqn_ros')+"/models/"
	
	# Load Table SDF
	table_xml = ''
	with open (model_path + "cafe_table/model.sdf", "r") as table_file:
		table_xml=table_file.read().replace('\n', '')
		
	# Load object URDF
	object_xml = ''
	if object_type == 0:
		with open (model_path + "block/model.urdf", "r") as object_file:
			object_xml=object_file.read().replace('\n', '')
	elif object_type == 1:
		with open (model_path + "sphere/model.urdf", "r") as object_file:
			object_xml=object_file.read().replace('\n', '')
	else:
		with open (model_path + "cylinder/model.urdf", "r") as object_file:
			object_xml=object_file.read().replace('\n', '')

	# Spawn Table SDF
	rospy.wait_for_service('/gazebo/spawn_sdf_model')
	try:
		spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
		resp_sdf = spawn_sdf("cafe_table", table_xml, "/",
				         table_pose, table_reference_frame)
	except rospy.ServiceException, e:
		rospy.logerr("Spawn SDF service call failed: {0}".format(e))
		
	# Spawn object URDF
	rospy.wait_for_service('/gazebo/spawn_urdf_model')
	try:
		spawn_urdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
		resp_urdf = spawn_urdf("object", object_xml, "/",
				           object_pose, object_reference_frame)
	except rospy.ServiceException, e:
		rospy.logerr("Spawn URDF service call failed: {0}".format(e))


def delete_gazebo_models():
	# This will be called on ROS Exit, deleting Gazebo models
	# Do not wait for the Gazebo Delete Model service, since
	# Gazebo should already be running. If the service is not
	# available since Gazebo has been killed, it is fine to error out
	try:
		delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
		resp_delete = delete_model("cafe_table")
		resp_delete = delete_model("object")
	except rospy.ServiceException, e:
		rospy.loginfo("Delete Model service call failed: {0}".format(e))


class BaxterManipulator(object):

	def __init__(self):
		"""
		BaxterManipulator control from torch - Initialisation
		"""
		# Publishers
		self._pub_rate = rospy.Publisher('robot/joint_state_publish_rate', UInt16, queue_size=10)
		self.image_pub = rospy.Publisher("baxter_view",Image,queue_size=4)
		
		#Link with baxter interface
		self._left_arm = baxter_interface.limb.Limb("left")
		self._right_arm = baxter_interface.limb.Limb("right")
		self._left_joint_names = self._left_arm.joint_names()
		self.grip_left = baxter_interface.Gripper('left', CHECK_VERSION)

		print("Getting robot state... ")
		self._rs = baxter_interface.RobotEnable(CHECK_VERSION)
		self._init_state = self._rs.state().enabled
		print("Enabling robot... ")
		self._rs.enable()
		
		# Control parameters
		self._rate = 500.0  # Hz
		self.bridge = CvBridge()
		# set joint state publishing to 500Hz
		self._pub_rate.publish(self._rate)
		self._left_arm.set_joint_position_speed(0.3)
		

	def _reset(self):
		self.grip_left.open()
		
		# Initial positions - right arm manually set to give good view over object - can be tweaked
		self._right_positions = dict(zip(self._right_arm.joint_names(),
                          [math.pi/3.0, -0.55, math.pi/4.0, math.pi/8.0 + 0.75,
                           0.0, 1.26 - math.pi/4.0, 0.0]))
		self._left_positions = dict(zip(self._left_arm.joint_names(),
                          [0.0, -0.55, 0.0, 0.75, 0.0, math.pi/2.0 - 0.2, 0.0]))
                         
		self._right_arm.move_to_joint_positions(self._right_positions)
		self._left_arm.move_to_joint_positions(self._left_positions)
		
		# Task completion flag
		self._task_complete = 0
		
		
	def _reset_control_modes(self):
		rate = rospy.Rate(self._rate)
		for _ in xrange(100):
			if rospy.is_shutdown():
				return False
			self._left_arm.exit_control_mode()
			self._pub_rate.publish(100)  # 100Hz default joint state rate
			rate.sleep()
		return True

	def clean_shutdown(self):
		print("\nExiting example...")
		#return to normal
		self._left_arm.move_to_neutral()
		self._reset_control_modes()
		if not self._init_state:
			print("Disabling robot...")
			self._rs.disable()
		return True

	''' Callback functions '''
	
	# Recieve action command from torch
	def action_callback(self,data):
		self.cmd = data.data
		self.action()
		
	#Recieve image from gazebo - resizes to 60 by 60
	def img_callback(self,data):
  		cv_image = self.bridge.imgmsg_to_cv2(data, "rgb8")
		cv_image = cv2.resize(cv_image, (60, 60))
		self.img_msg = self.bridge.cv2_to_imgmsg(cv_image, "rgb8")
		self.img_msg.header.frame_id = str(self._task_complete)
  	
  	# Recieve object pose information - used to determine whether task has been completed		
	def object_pose_callback(self,data):
		if 'object' in data.name:
			index = data.name.index('object')
			self.object_position_x = data.pose[index].position.x
			self.object_position_y = data.pose[index].position.y
			self.object_position_z = data.pose[index].position.z

	def listener(self):
		rospy.Subscriber('chatter', String, self.action_callback)
		rospy.Subscriber("/gazebo/model_states", ModelStates, self.object_pose_callback)
		rospy.Subscriber("/cameras/right_hand_camera/image",
							Image,self.img_callback)
		rospy.spin()

	def action(self):
		if self.cmd == 'r':
			self._left_positions["left_w2"] += 0.2
			self._pub_rate.publish(self._rate)
			self._left_arm.move_to_joint_positions(self._left_positions)
		elif self.cmd == 'l':
			self._left_positions["left_w2"] -= 0.2
			self._pub_rate.publish(self._rate)
			self._left_arm.move_to_joint_positions(self._left_positions)
			# Pickup object by lowering by predetermined amount to grasp and lift object - 
			# move_vertical function is found in IK.py file
		elif self.cmd == 'p':
			self._left_positions = move_vertical(self._left_positions,'d')
			self._pub_rate.publish(self._rate)
			self._left_arm.move_to_joint_positions(self._left_positions)
			time.sleep(0.2)
			self.grip_left.close()
			time.sleep(1.0)
			self._left_positions = move_vertical(self._left_positions,'u')
			self._pub_rate.publish(self._rate)
			self._left_arm.move_to_joint_positions(self._left_positions)
			
			# Get effector endpoint pose - this is compared with the object pose
			# to determine if the task has been succesfully completed or not
			# z-axis of end affector adjusted by 1 due to difference in frame of reference
			self.end_x = self._left_arm.endpoint_pose()["position"].x
			self.end_y = self._left_arm.endpoint_pose()["position"].y
			self.end_z = 1.0+self._left_arm.endpoint_pose()["position"].z
			# Comparison - threshold values are arbitrary, can be tweaked.
			if (abs(self.end_x - self.object_position_x) < 0.05) & (abs(self.end_y - \
					self.object_position_y) < 0.05)  & (abs(self.end_z - \
 					self.object_position_z) < 0.1):
				self._task_complete = 1
				# Places task complete flag in header of image 
				self.img_msg.header.frame_id = str(self._task_complete)
		# Reset command - waits to be told by torch to reset to ensure torch has recieved terminal status
		elif self.cmd == 'reset':
			print("resetting")
			delete_gazebo_models()
			load_gazebo_models()
			self._reset()
		# Publish image with terminal in header
		self.image_pub.publish(self.img_msg)
		
		
		
def main():

	arg_fmt = argparse.RawDescriptionHelpFormatter
	parser = argparse.ArgumentParser(formatter_class=arg_fmt,
		                     description=main.__doc__)
	parser.parse_args(rospy.myargv()[1:])

	print("Initializing node... ")
	rospy.init_node("baxter_dqn_ros")
	baxter_manipulator = BaxterManipulator()
	# Load Gazebo Models via Spawning Services
	# Note that the models reference is the /world frame
	# and the IK operates with respect to the /base frame
	baxter_manipulator._reset()
	load_gazebo_models()

	
	baxter_manipulator.listener()
	# Remove models from the scene on shutdown
	rospy.on_shutdown(delete_gazebo_models())	
	rospy.on_shutdown(baxter_manipulator.clean_shutdown())
if __name__ == '__main__':
	main()
