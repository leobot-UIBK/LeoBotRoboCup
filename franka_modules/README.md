# Franka Modules
https://roboticsbackend.com/ros-import-python-module-from-another-package/


This package collects different python modules to provide movement and grasp functions for the Franka Emika Panda manipulator.

Movement of the joints as well as of the end effector in carthesian space will be provided by the moveIT moveit_commander functions. 
The Grasping will be done directly by using the franka_lib library. Therefor some ros actions are called.

Two modules are included:

* panda_move_modules.py
* panda_grip_modules.py

One can use this modules to work with the Franka Emika Panda manipulator in every ROS-python-node.
To do so please import the certain module.
> `from franka_modules.panda_move_modules import say_it_works1`






## panda_move_modules
This module includes:

    def panda_init()
    def addBoxEnvironment(name,pose,dimension):
    def addPlaneEnvironment(name,pose=["world", 0.0, 0.0, 0.0, 0, 0, 0],normal=[0,0,1]):
    def addCylinderEnvironment(name,pose,height,radius):
    def planning_joint_goal(move_group,setJointGoal):
    def planning_cartesian_goal(move_group,cartesianGoal):
    def planning_cartesian_path(move_group,cartesianPath):  
    def wait_for_state_update(objectName,scene,box_is_known=False,box_is_attached=False,timeout=4):
    def nextexecution():



## panda_grip_modules
This module includes:  

    def pandagripper_homing_client):
    def pandagripper_grasp_client(width,force,speed=None,epsilon_inner=0.005,epsilon_outer=0.005):
    def pandagripper_stop_client():
    def pandagripper_move_client(width, speed = None):


