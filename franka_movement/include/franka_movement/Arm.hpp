#ifndef ROBOCUPATWORK_ARM_HPP
#define ROBOCUPATWORK_ARM_HPP

// ROS
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <std_srvs/Trigger.h>

// ROS messages
#include "franka_movement_msgs/FrankaJoints.h"
#include "franka_movement_msgs/FrankaPose.h"
#include "franka_movement_msgs/FrankaNamed.h"
#include "franka_movement_msgs/FrankaHome.h"
#include "franka_movement_msgs/FrankaBasicPick.h"
#include "franka_movement_msgs/FrankaPick.h"
#include "franka_movement_msgs/FrankaPlace.h"
#include "franka_movement_msgs/FrankaBasicPlace.h"

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

// moveit
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

namespace franka_move {
    namespace rvt = rviz_visual_tools;

 /**
 * Class to represent the robot arm.
 * Service request handler will be here.
 */
    class Arm {
    public:
        /**
         * The arm can be initialized with a node handle and the name of a robot description.
         * @param planningGroup The name of the planning group of this instance (eg. panda_arm).
         * @param visualize True if you want to visualize and accept every motion in rviz before execution.
         * @param end_effector_link The name of the end_effector_link (virtual links are also possible) (eg. panda_gripper_center).
         *                          If an empty string is set the link wont be changed.
         */
        Arm(ros::NodeHandle &n, const std::string &planningGroup, const bool &visualize, const std::string &end_effector_link="");

/**
 * Spins the arm.
 * @todo is this function really necessary
 */
        void spin();

        // *** service callbacks ***
        /**
         * The callback function of the /move_joints service.
         * This function passes the input params to the move_joints_to function.
         * @see move_joints_to
         */
        bool move_joints_callback(franka_movement_msgs::FrankaJoints::Request &req,
                                  franka_movement_msgs::FrankaJoints::Response &res);

        /**
         * The callback function of the /move_joints_relative service.
         * If possible the joints of the franka emika will be moved by the given joint value relative to the current position.
         * The movement will be done in the move_joints_to function.
         * @see move_joints_to
         */
        bool move_joints_relative_callback(franka_movement_msgs::FrankaJoints::Request &req,
                                           franka_movement_msgs::FrankaJoints::Response &res);

        /**
         * The callback function of the /move_to_pose service.
         * If possible the end-effector will be moved to the given pose.
         * Therefore the coordinate frame of the given pose will be transformed to the base frame of the robot arm.
         * So make sure the frame_id of the geometry_msgs/PoseStamped is set to the frame of the client.
         * @todo tf2 conversion
         */
        bool move_to_pose_callback(franka_movement_msgs::FrankaPose::Request &req,
                                   franka_movement_msgs::FrankaPose::Response &res);

        /**
         * The callback function of the /move_to_named service.
         * Moves the franka to a named position.
         */
        bool move_to_named_callback(franka_movement_msgs::FrankaNamed::Request &req,
                                   franka_movement_msgs::FrankaNamed::Response &res);

        /**
         * The callback function of the /move_to_home service.
         * Moves the franka to the position named home.
         */
        bool move_to_home_callback(franka_movement_msgs::FrankaHome::Request &req,
                                    franka_movement_msgs::FrankaHome::Response &res);

        /**
         * Take a look at the franka_movement README for more information.
         */
        bool basic_pick_callback(franka_movement_msgs::FrankaBasicPick::Request &req,
                        franka_movement_msgs::FrankaBasicPick::Response &res);
        /**
         * Take a look at the franka_movement README for more information.
         */
        bool pick_callback(franka_movement_msgs::FrankaPick::Request &req,
                                franka_movement_msgs::FrankaPick::Response &res);

        /**
         * Take a look at the franka_movement README for more information.
         */
        bool place_callback(franka_movement_msgs::FrankaPlace::Request &req,
                            franka_movement_msgs::FrankaPlace::Response &res);

        /**
         * Take a look at the franka_movement README for more information.
         */
        bool basic_place_callback(franka_movement_msgs::FrankaBasicPlace::Request &req,
                                       franka_movement_msgs::FrankaBasicPlace::Response &res);
    private:
        // *** private fields ***
        // arm
        moveit::planning_interface::MoveGroupInterface move_group;
        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
        const robot_state::JointModelGroup* joint_model_group;
        moveit_visual_tools::MoveItVisualTools visual_tools;

        // tf2 Buffer and Listener
        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener;

        bool is_moving = false;

        bool visualize;


        // *** private functions ***

        void visualize_plan(const moveit::planning_interface::MoveGroupInterface::Plan &plan);

        /**
         * If the given position is reachable and the arm isn't currently moving
         * this function moves the arm joints to the given joint positions.
         * @param joint_group_positions 7 joint positions. One for every joint of the franka emika.
         *                              The first item in the array is the joint next to the robot base.
         * @return True if the move was successful. False otherwise.
         */
        bool move_joints_to(const std::vector<double> &joint_group_positions);

        /**
         * Sets the is moving lock.
         * @return True if the lock has been set successfully. False otherwise.
         */
        bool set_is_moving();

        /**
         * Releases the moving lock.
         */
        void release_is_moving();

        /**
         * If possible the arm will be moved by the status of the move_group.
         * @return True if the movement was successfull. False otherwise.
         */
        bool move_arm_by_move_group();

        /**
         * Moves the franka to a named position.
         * @return True if the movement was successful. False otherwise.
         */
        bool move_to_named_position(const std::string &name);

        /**
         * Performs a basic pick to the given object.
         * A basic pick is a very generalized version of a pick and is approaching the object simply from the top.
         * The object needs to be added to the planing scene.
         * After this function the object will be attached to the end-effector.
         * @param obj_id The planning frame id of the object to pick.
         * @return True if the object could be picked successfully, false otherwise.
         */
        int basic_pick(std::string &obj_id, std::string &supported_surface, bool constrain_grasp_width);

        void closedGripper(trajectory_msgs::JointTrajectory& posture, float obj_width);
        /**
         * Updates the given posture so that the gripper is open.
         * If a object width the gripper will open exactly to the object width.
         * Otherwise it opens at max.
         */
        void openGripper(trajectory_msgs::JointTrajectory& posture, float obj_width = 0.035);

        /**
         * Take a look at the franka_movement README for more information.
         */
        moveit::planning_interface::MoveItErrorCode pick(moveit_msgs::Grasp &grasp, std::string &obj_id);

        /**
         * Take a look at the franka_movement README for more information.
         */
        int basicPlace(std::string &obj_id, geometry_msgs::PoseStamped &place_pose, std::string &supported_surface, bool constrain_grasp_width);

        /**
         * Take a look at the franka_movement README for more information.
         */
        moveit::planning_interface::MoveItErrorCode place(moveit_msgs::PlaceLocation &place_location, std::string &obj_id);

        /**
         * This function takes the pose of an object and returns a quaternion that is a representation of that pose
         * but rotated in such a way that the ee would approach this object always from above.
         * So this function uses the fact that all the objects are symmetric.
         * @param obj_pose The pose of the object.
         * @return The inserted pose rotated so that the gripper would approach it from above.
         */
        tf2::Quaternion shortest_grasp_rotation(const geometry_msgs::Pose &obj_pose, bool from_stack);

    };
}


#endif //ROBOCUPATWORK_ARM_HPP
