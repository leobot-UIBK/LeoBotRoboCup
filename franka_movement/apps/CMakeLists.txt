## Declare a C++ executable
add_executable(franka_node FrankaNode.cpp)
add_executable(planing_scene_node PlaningSceneNode.cpp)
add_executable(bag_publisher_maintain_time BagPublisherMaintainTime.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(franka_node franka_movement ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${EIGEN3_LIBRARIES})
target_link_libraries(planing_scene_node franka_movement ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${EIGEN3_LIBRARIES})
target_link_libraries(bag_publisher_maintain_time ${catkin_LIBRARIES} ${Boost_LIBRARIES})

## Add cmake target dependencies of the executable
#add_dependencies(franka_node FrankaJoints)
#add_dependencies( franka_node ${PROJECT_NAME}_generate_messages_cpp )
