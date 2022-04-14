//
// Created by paddy on 18.05.21.
//

#ifndef FRANKA_MOVEMENT_PLANNINGSCENEUTILS_H
#define FRANKA_MOVEMENT_PLANNINGSCENEUTILS_H

// ROS
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>

// messages
#include <moveit_msgs/CollisionObject.h>
#include <geometry_msgs/Pose.h>

// etc
#include <string>

namespace franka_move {
    class PlanningSceneUtils {
    public:
        /**
         * Returns the collision object with the given planning scene ID transformed to the given tf2 frame.
         * @param obj_id The planning scene ID of the desired object.
         * @param target_frame The tf2 frame the resulting collision object should be transformed to.
         * @param obj The resulting collision object.
         * @param tf_buffer The pre initialized tf2 buffer.
         * @return False if an error occurred, True otherwise.
         */
        static bool get_collision_object_in_frame(const std::string &obj_id, const std::string &target_frame,
                                                  moveit_msgs::CollisionObject &obj, const tf2_ros::Buffer &tf_buffer);

        /**
         * Loads the params corresponding to the given object ID from the ROS parameter server.
         * Note that the object ID is not the planning scene ID but the general name of the object (eg. Bearing_Box).
         * Check the franka_movement README for further explanations of the ROS parameters.
         * @param id The ID of the object linked to the parameters that should be loaded.
         * @return Returns true if all parameters could be loaded. False otherwise.
         */
        static bool get_obj_params(const std::string &id, std::vector<float> &grasp_point, float &grasp_width,
                                   std::vector<float> &bounding_box){
            return get_obj_params_querry(id, &grasp_point, &grasp_width, &bounding_box);
        }

        /**
         * Loads the params corresponding to the given object ID from the ROS parameter server.
         * Note that the object ID is not the planning scene ID but the general name of the object (eg. Bearing_Box).
         * Check the franka_movement README for further explanations of the ROS parameters.
         * @param id The ID of the object linked to the parameters that should be loaded.
         * @return Returns true if all parameters could be loaded. False otherwise.
         */
        static bool get_obj_params(const std::string &id, std::vector<float> &grasp_point, float &grasp_width){
            return get_obj_params_querry(id, &grasp_point, &grasp_width, nullptr);
        }

        /**
         * Loads the params corresponding to the given object ID from the ROS parameter server.
         * Note that the object ID is not the planning scene ID but the general name of the object (eg. Bearing_Box).
         * Check the franka_movement README for further explanations of the ROS parameters.
         * @param id The ID of the object linked to the parameters that should be loaded.
         * @return Returns true if all parameters could be loaded. False otherwise.
         */
        static bool get_obj_params(const std::string &id, std::vector<float> &bounding_box){
            return get_obj_params_querry(id, nullptr, nullptr, &bounding_box);
        }

        /**
         * Splits a given planning scene ID into the name and the number of the object.
         * The # separator will be lost during this process.
         * @param id The planning scene ID.
         * @param name The name of the object.
         * @param number The number of the object.
         * @return Returns True if name and number could be set correctly.
         * Otherwise False and the returned name will be the input ID and the number -1.
         */
        static bool get_name_and_number_from_id(const std::string &id, std::string &name, int &number);

        /**
         * Decides if two objects within the planning scene are the same based on the following criteria.
         * The same planning scene ID (eg. Bearing_Box#3 don't get confused with the name of the object).
         * Both objects have the same name and are close to each other.
         * For this the given threshold is used as a bounding box radius.
         * @param id0 Planning scene ID of the first object.
         * @param pose0 Pose of the first object in the planning scene.
         * @param id1 Planning scene ID of the second object.
         * @param pose1 Pose of the second object in the planning scene.
         * @param threshold A threshold in m used as radius to determine if two objects have the same location.
         * @return True if both objects are the same based on these criteria, otherwise False.
         */
        static bool is_same_object(const std::string &id0, const geometry_msgs::Pose &pose0,
                                   const std::string &id1, const geometry_msgs::Pose &pose1, float threshold=0.05);
    private:
        static bool get_obj_params_querry(const std::string &id, std::vector<float> *grasp_point, float *grasp_width,
                                                std::vector<float> *bounding_box);
    };
}


#endif //FRANKA_MOVEMENT_PLANNINGSCENEUTILS_H
