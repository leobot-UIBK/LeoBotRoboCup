#include "franka_movement/Arm.hpp"

#include <ros/ros.h>
#include "franka_movement_msgs/FrankaJoints.h"

using namespace franka_move;

bool Arm::move_joints_to(const std::vector<double> &joint_group_positions){
    if(joint_group_positions.size() != 7){
        ROS_ERROR("Wrong amount of joints. Move request denied!");
        return false;
    }

    if(!Arm::set_is_moving()){
        return false;
    }

    move_group.setJointValueTarget(joint_group_positions);

    // move
    bool success = Arm::move_arm_by_move_group();

    // print new position
    if(success) {
        ROS_INFO("Joints moved to: ");
        std::copy(joint_group_positions.begin(), joint_group_positions.end(),
                  std::ostream_iterator<float>(std::cout, " "));
    }

    Arm::release_is_moving();
    return success;
}

bool Arm::move_joints_callback(franka_movement_msgs::FrankaJoints::Request &req,
                               franka_movement_msgs::FrankaJoints::Response &res) {
    std::vector<double> joint_group_positions =
            {req.joint_1, req.joint_2, req.joint_3, req.joint_4, req.joint_5, req.joint_6, req.joint_7};

    res.success = move_joints_to(joint_group_positions);

    return true;
}

bool Arm::move_joints_relative_callback(franka_movement_msgs::FrankaJoints::Request &req,
                                   franka_movement_msgs::FrankaJoints::Response &res){
    std::vector<double> joint_group_positions =
            {req.joint_1, req.joint_2, req.joint_3, req.joint_4, req.joint_5, req.joint_6, req.joint_7};

    // RobotState is the object that contains all the current position/velocity/acceleration data.
    moveit::core::RobotStatePtr current_state = move_group.getCurrentState();

    // get the current set of joint values for the group.
    std::vector<double> current_joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group, current_joint_group_positions);

    for(int i = 0; i < joint_group_positions.size(); i++){
        joint_group_positions[i] += current_joint_group_positions[i];
    }

    res.success = move_joints_to(joint_group_positions);

    return true;
}
