# measurement package
author: Peter Manzl

This package contains different functionalities to do measurements. 

#getImageServer.launch 
Provides a service to save the current image and depth image available at the topic wchich are arguments of the launch file. Foilename and directory are arguments as well. 


# MeasurementMaster.launch
Starts the TimedVelServer and MeasurementMaster node. The TimedVelServer takes a trajectory, consisting of [sx, sy, phi, vx_max, vy_max, dphi_max, ax_max, ay_max, ddphi_max] and publishes the needed velocities to follow a fully synchronious PTP-trajectory as a geometry_msgs/twist to the topic /leobot_base/cmd_vel. It is published currently with 1kHz, this time could be changed in the sourcecode in src/TimedVelService (T_ms = 1, cycle time for the publisher in ms). The angular acceleration from the robots wheel are read from the parameter server, overwritten and reset at the end to provide a smooth acceleration. 
The script for the MeasurementMaster.py reads datafiles from path/datafile, both given as arguments to the launchfile. It takes as input the index of the measurement i, the data [sx, sy, phi, vx_max, vy_max, dphi_max, ax_max, ay_max, ddphi_max] and a flag if this trajectory is for a new measurement. It activates and deactivates the vicon controller with the boolean flag /vicon/leobot/leobot_control_flag to drive the robot back to the zero position using vicon (see package viconposition, launchfile is vicon_targetandcontrol.launch). It publishes to /meassurements/flag the flags 1 for start of measuremen t and 2 for end of trajectory. 

#frankaTrainingMove.py
This script uses the franka_modules.panda_move_modules to move the franka emika panda arm around. It was prior to the RoboCup competition 2021 to generate image data in combination to the getImageServer. There the arm moves from a starting position downwards in with different angles and distance to the scene to vary the image data. It is thought of not beeing an effective way to train a neuronal network because the objects are always in the same combination of distances to each other.

 