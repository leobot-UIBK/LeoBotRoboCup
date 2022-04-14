#include "franka_movement/Arm.hpp"

#include <ros/ros.h>
#include "franka_movement_msgs/FrankaPose.h"
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>




using namespace franka_move;

bool Arm::move_to_pose_callback(franka_movement_msgs::FrankaPose::Request &req,
                                franka_movement_msgs::FrankaPose::Response &res){
    // coordinate frame conversion using tf2
    geometry_msgs::PoseStamped target_pose;
    try{
        target_pose = tfBuffer.transform<geometry_msgs::PoseStamped>(req.poseStamped,move_group.getPlanningFrame(),
                                                       ros::Duration(0.1));
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("%s",ex.what());
        res.success = false;
        return false;
    }

    // set locks
    if(!Arm::set_is_moving()){
        res.success = false;
        return true;
    }

    // set new pose
    move_group.setPoseTarget(target_pose.pose);

    // move
    bool success = Arm::move_arm_by_move_group();

    if(success){
        // print new pose
        auto p = target_pose.pose.position;
        auto q = target_pose.pose.orientation;
        ROS_INFO("End-effector moved to: x:%f y:%f z:%f | w:%f a:%f b:%f c:%f", p.x, p.y, p.z, q.w, q.x, q.y, q.z);
    }

    res.success = success;

    Arm::release_is_moving();

    return true;
}