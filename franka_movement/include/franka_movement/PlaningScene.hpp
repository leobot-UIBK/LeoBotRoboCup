#ifndef FRANKA_MOVEMENT_PLANINGSCENE_HPP
#define FRANKA_MOVEMENT_PLANINGSCENE_HPP

// ROS core / libs
#include <ros/ros.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <tf2_ros/transform_listener.h>

// ROS messages
#include "franka_movement_msgs/GetCollisionObjectById.h"
#include "franka_movement_msgs/CollisionObject.h"
#include "franka_movement_msgs/AttachedCollisionObject.h"
#include "franka_movement_msgs/AddCollisionObjectById.h"

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


namespace franka_move {
    class PlaningScene{
    public:
        PlaningScene(const bool &visualize);

        // *** Service Callbacks ***
        /**
         * Take a look at the franka_movement README for more information.
         */
        bool collision_object_callback(franka_movement_msgs::CollisionObject::Request &req,
                                           franka_movement_msgs::CollisionObject::Response &res);

        /**
         * Take a look at the franka_movement README for more information.
         */
        bool attached_collision_object_callback(franka_movement_msgs::AttachedCollisionObject::Request &req,
                                          franka_movement_msgs::AttachedCollisionObject::Response &res);

        /**
         * Take a look at the franka_movement README for more information.
         */
        bool get_collision_object_by_id_callback(franka_movement_msgs::GetCollisionObjectById::Request &req,
                                        franka_movement_msgs::GetCollisionObjectById::Response &res);

        /**
         * Take a look at the franka_movement README for more information.
         */
        bool add_collision_object_by_id_callback(franka_movement_msgs::AddCollisionObjectById::Request &,
                                                               franka_movement_msgs::AddCollisionObjectById::Response &);

    private:
        // *** private fields ***
        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
        moveit_visual_tools::MoveItVisualTools visual_tools;
        // tf2 Buffer and Listener
        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener;
        size_t num_objects_with_id = 0;

        bool add_collision_object(moveit_msgs::CollisionObject &object);
        bool move_collision_object(moveit_msgs::CollisionObject &object);
        bool attach_existing_object(moveit_msgs::AttachedCollisionObject &object);
        void add_attached_object(moveit_msgs::AttachedCollisionObject &object);
        void detach_existing_object(moveit_msgs::AttachedCollisionObject &object);
        bool remove_collision_object(moveit_msgs::CollisionObject &object);

        bool has_id(moveit_msgs::CollisionObject &object);
        void add_base_box();

        bool is_existing_object(std::string obj_id);
        bool get_object_by_id(std::string obj_id, moveit_msgs::CollisionObject &obj);
        bool transform_pose_to_frame(geometry_msgs::Pose &, const std::string &, const std::string &);
        void test_area(moveit_msgs::CollisionObject &object);
        bool generate_collision_object_id(std::string &id, const geometry_msgs::PoseStamped &poseStamped);
    };
}

#endif //FRANKA_MOVEMENT_PLANINGSCENE_HPP
