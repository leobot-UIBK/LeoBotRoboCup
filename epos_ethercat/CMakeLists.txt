cmake_minimum_required(VERSION 2.8.3)
project(epos_ethercat)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        soem
        message_generation
        geometry_msgs
	    leobot_base_msg
	    std_srvs
        tf2_ros
    )

# add_subdirectory(external)

set(CMAKE_CXX_FLAGS "-Wall -std=c++0x -fPIC")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -ggdb")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")

SET(GCC_NEWDTAGS_LINK_FLAGS "-Wl,--disable-new-dtags")
# Wl, passes options to linker, needed here to access .so files in the executable
SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${GCC_NEWDTAGS_LINK_FLAGS}")


# adding librarys:
set(HEADER_LIST src/EPOS4_class.h src/circular_buffer.h)
add_library(motorInfo STATIC src/EPOS4_class.cpp ${HEADER_LIST})
# add_library(circular_buffer STATIC ${HEADER_LIST})

target_link_libraries(motorInfo PUBLIC  ${catkin_LIBRARIES}) # catkin_devel/devel/lib/libsoem.a) # soem is a library in the library motorInfo; Public
# target_include_directories(motorInfo PUBLIC external)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to

##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)
## Generate messages in the 'msg' folder
add_message_files(
        FILES
        # motor_status.msg
        # base_status.msg
        # motor_monitoring.msg
        # motor_wheels.msg
        #   Message2.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
        DEPENDENCIES
        geometry_msgs
        std_msgs
	leobot_base_msg
)
catkin_package(
    CATKIN_DEPENDS message_runtime
)



## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )


################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES epos_ethercat
#  CATKIN_DEPENDS roscpp rospy std_msgs ethercat_grant
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/epos_ethercat.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(Eth_Slaveinfo src/Eth_Slaveinfo.cpp)
add_executable(Eth_SimpleTest src/Eth_simple_Test.cpp)
add_executable(Eth_red_test src/Eth_red_test.cpp)
add_executable(Eth_PDOtest src/Eth_PDOtest.cpp)
add_executable(Eth_EPOS4 src/Eth_EPOS4_main.cpp)
add_executable(Eth_BaseMovement src/Eth_BaseMovement.cpp)
add_executable(RosTest src/test_ros.cpp)

add_executable(Eth_Class_testing src/test_MotorInfo_class.cpp)
add_executable(Sine_publisher src/sine_publisher.cpp)
add_executable(Joy_publisher src/joy_publisher.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )
target_link_libraries(Eth_Slaveinfo ${catkin_LIBRARIES})
target_link_libraries(Eth_SimpleTest ${catkin_LIBRARIES})
target_link_libraries(Eth_red_test ${catkin_LIBRARIES})
target_link_libraries(Eth_PDOtest ${catkin_LIBRARIES})
# target_link_libraries(Eth_EPOS4 soem)
target_link_libraries(Eth_Class_testing motorInfo ${catkin_LIBRARIES})

target_link_libraries(Eth_EPOS4 PRIVATE motorInfo ${catkin_LIBRARIES})
target_link_libraries(Eth_EPOS4 PUBLIC ${catkin_LIBRARIES})


target_link_libraries(Eth_BaseMovement PRIVATE motorInfo ${catkin_LIBRARIES})
target_link_libraries(Eth_BaseMovement PUBLIC ${catkin_LIBRARIES})

target_link_libraries(Sine_publisher ${catkin_LIBRARIES})
target_link_libraries(Joy_publisher ${catkin_LIBRARIES})

target_link_libraries(RosTest PRIVATE motorInfo ${catkin_LIBRARIES})
target_link_libraries(RosTest PUBLIC ${catkin_LIBRARIES})

#############
## Install ##
#############


# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_epos_ethercat.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
