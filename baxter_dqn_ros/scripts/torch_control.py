#!/usr/bin/env python

# Joint controller - subscribes to messages from torch-ros to control joints
import roslib
import argparse
import rospy
import rospkg
import sys

from gazebo_msgs.srv import (
SpawnModel,
DeleteModel
)

from geometry_msgs.msg import (
Pose,
Point,
Quaternion,
)


from baxter_robot_class import BaxterManipulator

def load_gazebo_models():
	# Get Models' Path
	table_pose = Pose(position=Point(x=0.8, y=1.0, z=0.0))
	table_reference_frame = "world"
	object_reference_frame = "world"
	
	model_path = rospkg.RosPack().get_path('baxter_dqn_ros')+"/models/"
	
	# Load Table SDF
	table_xml = ''
	with open (model_path + "cafe_table/model.sdf", "r") as table_file:
		table_xml=table_file.read().replace('\n', '')
		
	# Load objects URDF
	xml = {}
	pose = {}
	for i in xrange(1,10):
		xml["object" + str(i)]  = ''
		pose["object" + str(i)] = Pose(position=Point(x=-3.0+i*0.1, y=0.0, z= 0.0))
	
	# Red models
	with open (model_path + "block_r/model.urdf", "r") as object1_file:
			xml["object1"]=object1_file.read().replace('\n', '')
	with open (model_path + "sphere_r/model.urdf", "r") as object2_file:
			xml["object2"]=object2_file.read().replace('\n', '')
	with open (model_path + "cylinder_r/model.urdf", "r") as object3_file:
			xml["object3"]=object3_file.read().replace('\n', '')
	# Green models		
	with open (model_path + "block_g/model.urdf", "r") as object4_file:
			xml["object4"]=object4_file.read().replace('\n', '')
	with open (model_path + "sphere_g/model.urdf", "r") as object5_file:
			xml["object5"]=object5_file.read().replace('\n', '')
	with open (model_path + "cylinder_g/model.urdf", "r") as object6_file:
			xml["object6"]=object6_file.read().replace('\n', '')
	# Blue models		
	with open (model_path + "block_b/model.urdf", "r") as object7_file:
			xml["object7"]=object7_file.read().replace('\n', '')
	with open (model_path + "sphere_b/model.urdf", "r") as object8_file:
			xml["object8"]=object8_file.read().replace('\n', '')
	with open (model_path + "cylinder_b/model.urdf", "r") as object9_file:
			xml["object9"]=object9_file.read().replace('\n', '')

	# Spawn Table SDF
	rospy.wait_for_service('/gazebo/spawn_sdf_model')
	try:
		spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
		resp_sdf = spawn_sdf("cafe_table", table_xml, "/",
				         table_pose, table_reference_frame)
	except rospy.ServiceException, e:
		rospy.logerr("Spawn SDF service call failed: {0}".format(e))
		
	# Spawn object URDF
	for i in xrange(1,10):
		rospy.wait_for_service('/gazebo/spawn_urdf_model')
		try:
			spawn_urdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
			resp_urdf = spawn_urdf("object"+str(i), xml["object"+str(i)], "/",
						         pose["object"+str(i)], object_reference_frame)
						         
		except rospy.ServiceException, e:
			rospy.logerr("Spawn URDF service call failed: {0}".format(e))
		
def delete_gazebo_models():
	# This will be called on ROS Exit, deleting Gazebo models
	try:
		delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
		resp_delete = delete_model("cafe_table")
		resp_delete = delete_model("object")
	except rospy.ServiceException, e:
		rospy.loginfo("Delete Model service call failed: {0}".format(e))

		
def main():

	print("Initializing node... ")
	rospy.init_node("baxter_dqn_ros")
	baxter_manipulator = BaxterManipulator()
	# Load Gazebo Models via Spawning Services
	# Note that the models reference is the /world frame
	baxter_manipulator._reset()
	load_gazebo_models()

	baxter_manipulator.listener()
	# Remove models from the scene on shutdown
	rospy.on_shutdown(delete_gazebo_models())	
	rospy.on_shutdown(baxter_manipulator.clean_shutdown())
	
if __name__ == '__main__':
	main()
