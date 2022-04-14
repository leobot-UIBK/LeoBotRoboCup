#include <ros/ros.h>

#include "franka_movement/Arm.hpp"
#include <iostream>
#include <boost/lexical_cast.hpp>

#define NODE_NAME "franka_move_node"


int main(int argc, char *argv[]) {
    // TODO node handle namespace shouldn't need to be setted
    ros::init(argc, argv, NODE_NAME);
    ros::NodeHandle n(NODE_NAME);
    ros::AsyncSpinner spinner(2);       // todo why do I need more then one thread??
    spinner.start();

    bool visualize = false;
    if(argc == 2){
        try {
            visualize = boost::lexical_cast<bool>(argv[1]);
        }
        catch (boost::bad_lexical_cast const &e)	// bad input
        {
            ROS_WARN("Bad input argument. Using default values.");
        }
    }

    franka_move::Arm arm(n, "panda_arm", visualize, "panda_gripper_center");

    // set callbacks
    // move_joints
    ros::ServiceServer move_joints_service = n.advertiseService("move_joints", &franka_move::Arm::move_joints_callback, &arm);
    ros::ServiceServer move_joints_relative_service = n.advertiseService("move_joints_relative", &franka_move::Arm::move_joints_relative_callback, &arm);

    // move pose
    ros::ServiceServer move_to_pose_service = n.advertiseService("move_to_pose", &franka_move::Arm::move_to_pose_callback, &arm);

    // move named
    ros::ServiceServer move_to_named_service = n.advertiseService("move_to_named", &franka_move::Arm::move_to_named_callback, &arm);
    ros::ServiceServer move_to_home_service = n.advertiseService("move_to_home", &franka_move::Arm::move_to_home_callback, &arm);

    // pick
    ros::ServiceServer basic_pick_service = n.advertiseService("basic_pick", &franka_move::Arm::basic_pick_callback, &arm);
    ros::ServiceServer pick_service = n.advertiseService("pick", &franka_move::Arm::pick_callback, &arm);

    // place
    ros::ServiceServer place_service = n.advertiseService("place", &franka_move::Arm::place_callback, &arm);
    ros::ServiceServer basic_place_service = n.advertiseService("basic_place", &franka_move::Arm::basic_place_callback, &arm);


    ROS_INFO("Franka node is running ...");

    arm.spin();

    ros::waitForShutdown();
}