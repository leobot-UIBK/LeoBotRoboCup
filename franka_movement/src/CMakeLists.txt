# --- add headers for IDE support ---
set(HEADER_LIST "${franka_movement_SOURCE_DIR}/include/franka_movement/")

# --- add libraries ---
add_library(franka_movement
        Arm/Arm.cpp
        Arm/franka_move_joints.cpp
        Arm/franka_move_to_pose.cpp
        Arm/franka_move_to_named.cpp
        Arm/hand.cpp

        PlaningScene/PlaningScene.cpp
        PlaningScene/add_object.cpp
        PlaningScene/PlanningSceneUtils.cpp
                ${HEADER_LIST})

# We need this directory, and users of our library will need it too
target_include_directories(franka_movement PUBLIC ../include)

target_link_libraries(franka_movement PUBLIC
                        Franka::Franka
                        ${catkin_LIBRARIES}
                        ${Boost_LIBRARIES}
                        ${EIGEN3_LIBRARIES})

        # This depends on (header only) boost
#target_link_libraries(quaternionDmp PRIVATE Boost::boost)

# All users of this library will need at least C++11
target_compile_features(franka_movement PUBLIC cxx_std_11)

# IDEs should put the headers in a nice place
source_group(TREE "${PROJECT_SOURCE_DIR}/include" PREFIX "Header Files" FILES ${HEADER_LIST})