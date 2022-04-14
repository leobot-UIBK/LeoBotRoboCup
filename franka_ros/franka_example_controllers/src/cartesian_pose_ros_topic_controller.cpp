// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_example_controllers/cartesian_pose_ros_topic_controller.h>

#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include <controller_interface/controller_base.h>
#include <franka_hw/franka_cartesian_command_interface.h>
#include <hardware_interface/hardware_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace franka_example_controllers {

bool CartesianPoseRosTopicController::init(hardware_interface::RobotHW* robot_hardware,
                                          ros::NodeHandle& node_handle) {
  cartesian_pose_interface_ = robot_hardware->get<franka_hw::FrankaPoseCartesianInterface>();
  if (cartesian_pose_interface_ == nullptr) {
    ROS_ERROR(
        "CartesianPoseRosTopicController: Could not get Cartesian Pose "
        "interface from hardware");
    return false;
  }

  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR("CartesianPoseRosTopicController: Could not get parameter arm_id");
    return false;
  }

  try {
    cartesian_pose_handle_ = std::make_unique<franka_hw::FrankaCartesianPoseHandle>(
        cartesian_pose_interface_->getHandle(arm_id + "_robot"));
  } catch (const hardware_interface::HardwareInterfaceException& e) {
    ROS_ERROR_STREAM(
        "CartesianPoseRosTopicController: Exception getting Cartesian handle: " << e.what());
    return false;
  }

  auto state_interface = robot_hardware->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR("CartesianPoseRosTopicController: Could not get state interface from hardware");
    return false;
  }

  ros::NodeHandle n;
  sub_pose_ = n.subscribe("equilibrium_pose", 1, &CartesianPoseRosTopicController::desiredPoseCallback, this);

  return true;
}

void CartesianPoseRosTopicController::starting(const ros::Time& /* time */) {
  initial_pose_ = cartesian_pose_handle_->getRobotState().O_T_EE_d;
  desired_pose_ = initial_pose_;
  elapsed_time_ = ros::Duration(0.0);

  std::cout << std::endl << std::endl;

  for(int i=0; i<16; ++i)
    std::cout << initial_pose_[i] << "\t";

  std::cout << std::endl << std::endl;

  //ros::spin();

}

void CartesianPoseRosTopicController::desiredPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& desired_pose) {
  // Covert Quaternion message to tf2::Quaternion
  tf2::Quaternion quat_tf;
  tf2::fromMsg(desired_pose->pose.orientation, quat_tf);
  // Convert tf2::Qiaterion to Matrix3x3 (rotation matrix)
  tf2::Matrix3x3 rotation_matrix;
  rotation_matrix.setRotation(quat_tf);
  // Update desired pose
  // Position
  desired_pose_[12] = desired_pose->pose.position.x;
  desired_pose_[13] = desired_pose->pose.position.y;
  desired_pose_[14] = desired_pose->pose.position.z;
  desired_pose_[15] = 1.0;
  // Orientation
  tf2::Vector3 col_ = rotation_matrix.getColumn(0);
  desired_pose_[0] = col_[0]; desired_pose_[1] = col_[1]; desired_pose_[2] = col_[2]; desired_pose_[3] = 0.0;
  col_ = rotation_matrix.getColumn(1);
  desired_pose_[4] = col_[0]; desired_pose_[5] = col_[1]; desired_pose_[6] = col_[2]; desired_pose_[7] = 0.0;
  col_ = rotation_matrix.getColumn(2);
  desired_pose_[8] = col_[0]; desired_pose_[9] = col_[1]; desired_pose_[10] = col_[2]; desired_pose_[11] = 0.0;
}

void CartesianPoseRosTopicController::update( const ros::Time& /* time */,
                                              const ros::Duration& period ) {


  //std::cout << period << std::endl;

  // Update robot pose
  cartesian_pose_handle_->setCommand(desired_pose_);
}

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::CartesianPoseRosTopicController,
                       controller_interface::ControllerBase)
