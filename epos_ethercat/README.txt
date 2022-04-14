# epos_ethercat package
This package is used to operate the epos motorcontrollers which drive LeoBots wheels. 
In order to operate the motors the following prerequisites need to be fullfuilled: 

- set the Node numbers of the motors
- set up the PDO mapping in the OD of the motors (see the Wiki and the Firmware specification of the controllers) 


**If the node does not start with a cryptic error on workcounter, restart the power of the platform!**

The main file to operate is the leobot_base.launch file. There the etherCAT interface is specified (standard: enp1s0f1, only needs to be changed when the motors are connected to an other physical jack). In OPMode 3 the motors velocities are controlled and the robot follows the velocity of the geometry_msg/twist sent to the topic /leobot_base/cmd_vel. For that the Jacobian is used to calculate the forward kinematics for the required wheels angular velocity for the desired robots task space velocity (vx, vy, dphi). 
The Node hase several parallel threads running, one for the control of the motors running with 1kHz and a high realtime priority, one with 1kHz and a lower priority to publish in the ROS system. The callback runs with 1kHz and one thread runs with 1Hz to monitor the voltage and temperature of each motors. The motors are numbered as in the wiki with front left as 1, front right as 2, back left as 3 and back right as 4. The used topics are: 
## Publishers: 
- /leobot_base/MotorStatus: leobot_base_msg/base_status, contains the current position, velocity, torque, statuscode and errorcode
- /leobot_base/MotorMonitor: contains the Temperature, Voltage, the cummulated errors and cycle violations of the control thread
- /odom: the odometry as nav_msgs/odometry. There the current wheelvelocity is used to integrate the position of the robot in the rectangle rule

## subscriber
- /leobot_base/cmd_vel: geometry_twist, reads commanded the velocities

## services
- /parameters/TriggerUpdate: std_srvs/Trigger, triggers updating parameters for the package from the parameterserver

## parameters
The following parameters are defined for base movement in the leobot_base namespace: 

- lx/ly: the  distance between the wheels in x-direction (front-back) and y-direction (left-right)
- wheelradius the radius of the mecanumwheel
- t_timeout: if no cmd_vel command is sent for a timeperiod of t_timeout the robot is stopped
- lpanda: the distance of the panda to the geometric center of the platform
- max_vel_[x,y,phi]: the arbitrary defined maximum velocities in task space
- gearbox_ratio_belt: the gearbox ratio of the belt (there is also a planetary drive in the motor itself with a ratio of 33 that is considered by the control thread
- dir_wheels: because of the mounting (left/right) some directions of the wheels change, compare to the figure
- max_wheel[vel, acc]: the max angular velocity and acceleration of the Mecanumwheels. This is limited by the motor hardware = 10000rpm on the motor side (10000 2pi/(30*1.63) ~ 19.3) 

In OPMode 1 the position of each wheel can be controlled independantly (using a profile with the set standard velocity/acceleration) and in OPMode 0x0A (=10) the torque of each wheel is controlled. 