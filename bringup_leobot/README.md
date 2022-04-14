# How to run the simulation 

## Necesarry installations
For the simulation the TEB local planner must be downloaded and built. The best way to download this planner is via these commands
    
	sudo apt-get install ros-melodic-teb-local-planner
	rosdep install teb_local_planner
    
## How to start the simulation:
	1. run roscore
	2. open CoppeliaSim
	3. drag the Arena_v6.ttt file from the coppelia folder into the CoppeliaSim environment
	4. start a new Terminal window
	5. To start the simulation run the following command:
		$ roslaunch bringup_leobot bringup_sim.launch
	7. now you can set a goal in Rviz with the '2D Nav Goal' button

# starting the Robot 
call the bringup_leobot.launch file to start all modules of the robot. May need to be called twice if the arm was not initialized prior to this. 

# Starting the visualization
To view the most relevant simulations (e.g. for the competition to see what is going on on the robot without a screen) the bringup_gui.launch file can be used. It starts rviz with a predefined configuration in the config/rviz_gui.rviz file that shows the Robot in the map with the current goal, Laserscan data and more. 

# Starting, initializing and shutting the Franka hardware down
To interface the franka in ROS the hardware needs to be started. This can be either down over the webinterface, running on the frankas-IP or directly with the python scripts. 
With the shutdown_franka.launch the franka at the predefined IP adress gets shut down on the hardware level. In the bringup_franka.launch the hardware gets automatically initialized (may be necessary to be run twice as it takes a while to initialized the robots axes and the bringup_franka shows an error when the hardware is not initialized). 
The lock_franka and shutdown_franka respectively sends an request to the webinterface using the cookie which must be saved in the directory config/franka_auth.txt, more information about how to obtain the cookie can be found in the wiki of this git. The hardware can when setup right be unlocked via 
rosrun bringup_leobot lock_franka.py IPAdress false config/frankaAuth.txt
If problems occure contact Peter Manzl. 


