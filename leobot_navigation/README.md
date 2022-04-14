# leobot_navigation
Author: Felix Schoeffthaler <felix.schoeffthaler@student.uibk.ac.at>

## Build


## Run
- The navigation is started from the bringup_leobot/launch/bringup_navigationstack.launch.
- This launch file is also loaded from the bringup_leobot.launch in the same folder.

## config
### move_base
- The parameters for the global and local planners as well as the overall parameters for the move_base node are defined here.

### costmap
- Here the parameters for the global costmap, local costmap and their common parameters are defined.
- In addition to that the parameters for the costmap converter are defined here.

### coppelia
- Here the files for CoppeliaSim environment are located.