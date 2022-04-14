#include "franka_movement/Arm.hpp"

// ROS
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>


// moveit
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <geometry_msgs/Pose.h>


using namespace franka_move;

Arm::Arm(ros::NodeHandle &n, const std::string &planningGroup, const bool &visualize, const std::string &end_effector_link) :
move_group(planningGroup), visualize(visualize),
visual_tools("panda_link0", "visualization_marker_array"), tfListener(tfBuffer){    // TODO check overhead of tfListener (listening all the time may be too much)
    // init moveit
    if(!end_effector_link.empty()) {
        move_group.setEndEffectorLink(end_effector_link);
    }
    // the max vel is already set in some config files
    //move_group.setMaxVelocityScalingFactor(0.1);        // only use 10% of the arm speed for safety
    joint_model_group = move_group.getCurrentState()->getJointModelGroup(planningGroup);

    // todo define franka path constraints for straighter motion pats
    /*moveit_msgs::PositionConstraint pcm;
    pcm.link_name = "panda_link7";
    pcm.header.frame_id = "panda_link0";
    //ocm.orientation.w = 1.0;
    pcm.target_point_offset.x = 0.1;
    pcm.target_point_offset.y = 0.1;
    pcm.target_point_offset.z = 0.1;
    pcm.weight = 0.8;

    moveit_msgs::Constraints test_constraints;
    test_constraints.position_constraints.push_back(pcm);
    move_group.setPathConstraints(test_constraints);
    move_group.setPlanningTime(10.0); */

    // init DEBUG visualization
    ROS_INFO("Initialize visualisation");
    visual_tools.deleteAllMarkers();
    visual_tools.loadRemoteControl();
}


void Arm::visualize_plan(const moveit::planning_interface::MoveGroupInterface::Plan &plan) {
        ROS_INFO("Visualize plan");
        visual_tools.deleteAllMarkers();
        visual_tools.publishTrajectoryLine(plan.trajectory_, joint_model_group);
        visual_tools.trigger();
    if(visualize) {
        visual_tools.prompt("Press 'next' in the RvizVisualToolsGui execute the motion");
    }
}


bool Arm::move_arm_by_move_group(){
    assert(is_moving);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    // TODO no error, if joint values are greater then the panda joint safety bounds

    ROS_INFO("Motion plan %s", success ? "SUCCESSED" : "FAILED");
    if(success){
        visualize_plan(plan);
        move_group.execute(plan);      // todo handle move errors
    }

    return success;
}

bool Arm::set_is_moving(){
    if(!is_moving){
        is_moving = true;
        return true;
    }
    ROS_ERROR("Arm is currently moving. Move request denied!");
    return false;
}

void Arm::release_is_moving(){
    assert(is_moving);  // is moving should be always true at this function call
    is_moving = false;
}



/** Defining our specific spin() function. */
void Arm::spin() {
    ros::Rate loop_rate(50);
    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }
}
