// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_example_controllers/joint_position_ros_example_controller.h>

#include <cmath>

#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

namespace franka_example_controllers {

bool JointPositionRosExampleController::init(hardware_interface::RobotHW* robot_hardware,
                                          ros::NodeHandle& node_handle) {
  position_joint_interface_ = robot_hardware->get<hardware_interface::PositionJointInterface>();
  if (position_joint_interface_ == nullptr) {
    ROS_ERROR(
        "JointPositionRosExampleController: Error getting position joint interface from hardware!");
    return false;
  }
  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names)) {
    ROS_ERROR("JointPositionRosExampleController: Could not parse joint names");
  }
  if (joint_names.size() != 7) {
    ROS_ERROR_STREAM("JointPositionRosExampleController: Wrong number of joint names, got "
                     << joint_names.size() << " instead of 7 names!");
    return false;
  }
  position_joint_handles_.resize(7);
  for (size_t i = 0; i < 7; ++i) {
    try {
      position_joint_handles_[i] = position_joint_interface_->getHandle(joint_names[i]);
    } catch (const hardware_interface::HardwareInterfaceException& e) {
      ROS_ERROR_STREAM(
          "JointPositionRosExampleController: Exception getting joint handles: " << e.what());
      return false;
    }
  }

  // TODO subscribe topic
    ros::NodeHandle n;
    sub_pose_ = n.subscribe("joint_pose", 1, &JointPositionRosExampleController::desiredJointPoseCallback, this);


    return true;
}

void JointPositionRosExampleController::starting(const ros::Time& /* time */) {
  for (size_t i = 0; i < position_joint_handles_.size(); ++i) {
    initial_pose_[i] = position_joint_handles_[i].getPosition();
  }
  desired_pose_ = initial_pose_;
  elapsed_time_ = ros::Duration(0.0);

    std::cout << std::endl << std::endl;

    for(int i=0; i<initial_pose_.size(); ++i) {
        std::cout << initial_pose_[i] << "\t";
    }

    std::cout << std::endl << std::endl;
}

void JointPositionRosExampleController::update(const ros::Time& /*time*/,
                                            const ros::Duration& period) {
    elapsed_time_ += period;

    // send joint poses to robot
    for(int i = 0; i < position_joint_handles_.size(); i++) {
        position_joint_handles_[i].setCommand(desired_pose_[i]);
    }
}

void JointPositionRosExampleController::desiredJointPoseCallback(const sensor_msgs::JointState &desired_joint_pose) {
    // in desired_joint_pose the first 7 joints should be the arm joints (and the last one the gripper)
    // so this call is ok
    for(int i = 0; i < desired_pose_.size(); i++){
        desired_pose_[i] = desired_joint_pose.position[i];
    }
}

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::JointPositionRosExampleController,
                       controller_interface::ControllerBase)
