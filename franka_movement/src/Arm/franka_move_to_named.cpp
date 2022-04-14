#include "franka_movement/Arm.hpp"

#include <ros/ros.h>
#include "franka_movement_msgs/FrankaNamed.h"
#include "franka_movement_msgs/FrankaHome.h"

using namespace franka_move;


bool Arm::move_to_named_position(const std::string &name){
    if(!Arm::set_is_moving()){
        return false;
    }

    move_group.setNamedTarget(name);
    bool success = Arm::move_arm_by_move_group();

    if(success){
        ROS_INFO("Moved to %s", name.c_str());
    }

    Arm::release_is_moving();
    return success;
}


bool Arm::move_to_home_callback(franka_movement_msgs::FrankaHome::Request &req,
                           franka_movement_msgs::FrankaHome::Response &res){
    res.success = Arm::move_to_named_position("ready");

    return true;
}

bool Arm::move_to_named_callback(franka_movement_msgs::FrankaNamed::Request &req,
                            franka_movement_msgs::FrankaNamed::Response &res){
    res.success = Arm::move_to_named_position(req.pose_name);

    return true;
}