cmake_minimum_required(VERSION 2.8.3)
project(leobot_safety)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
	#  roscpp
	rospy
	std_msgs
	roslaunch
)

roslaunch_add_file_check(launch)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS scripts/leobot_safety_node.py
	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
#add_executable(leobot_safety_node src/leobot_safety_node.cpp)

#install(DIRECTORY include/${PROJECT_NAME}/
#DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#FILES_MATCHING PATTERN "*.h"
#)
#install(TARGETS leobot_safety_node leobot_safety_node
#	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)
#install(FILES
#	   launch/leobot_safety.launch
#          launch/leobot_modbus.launch
#	   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#	)
#target_link_libraries(leobot_safety_node
#	${catkin_LIBRARIES}
#)
