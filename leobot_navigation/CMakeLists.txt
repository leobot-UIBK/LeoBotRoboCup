cmake_minimum_required(VERSION 3.0.2)
project(leobot_navigation)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

###########
## Build ##
###########

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)