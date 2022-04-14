#RoboCup@Work Tyrolics  

This is the main repository of the RoboCup@Work, Team Tyrolics from the University of Innsbruck.  
For in detail information explore the [Wiki](https://git.uibk.ac.at/csav5085/robocupatwork/-/wikis/home).  

## Installation



## Startup
All components needed to start the robot are started from the bringup_leobot package. The argument as_simulation is set to startup the simulation in Coppeliasim. For that [Coppeliasim](https://www.coppeliarobotics.com/) Version 4.1.0 needs to be installed.  

## Transformations
The coordinate frames of the robot are arranged as a tree using the package **tf2**.  
![image](/leobot_config/figures/TF_tree.png)

