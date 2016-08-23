# baxter_dqn

## Requirements
- ROS indigo w gazebo2
- baxter simulator installation
- Torch7
- torch-ros

## Installation
Place baxter_dqn_ros package in ros workspace alongside baxter simulator installation
While in workspace, rebuild by running
```
source ./devel./setup.bash
catkin_make
catkin_make install
```
## Use
Launch baxter_gazebo with 
```
./baxter.sh sim
roslaunch baxter_gazebo baxter_world.launch
```
Once loaded, in a new terminal run
```
source ./devel/setup.bash
rosrun baxter_dqn_ros torch_control.py
```
torch_control subscribes to commands from Torch, and passes a resized 60x60 RGB image and terminal status back in return. 

Either a sphere, cylinder or box is spawned at a random orientation at start and reset.

The baxter robot currently has three available actions - a right and left rotation at the wrist, and an attempt to pick up an object. 

An attempt to pickup the object results in termination, as unsuccessful attempts often throw the object out of reach. The success of the task is gauged by checking that the pose of the object is approxiamtely the pose of the end-effector at the end of the pickup action. At termination the environment is reset.

Torch7 environment for dqn is found in BaxterEnv.lua, this has not yet been tested.

BaxterEnv.lua subscribes to image topic from ROS, with terminal status sent in header of image message. 

## Known issues
The origin of the objects is not always at their centre. This results in some variation in pose with random orientation. This means that it is sometimes not possible to pick up the object - this should be solved when increasing number of actions.
