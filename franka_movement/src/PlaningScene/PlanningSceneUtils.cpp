#include "franka_movement/PlanningSceneUtils.hpp"

// messages
#include "franka_movement_msgs/GetCollisionObjectById.h"
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// ROS
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

// etc.
#include <eigen_conversions/eigen_msg.h>

using namespace franka_move;

bool PlanningSceneUtils::get_collision_object_in_frame(const std::string &obj_id, const std::string &target_frame,
                                                       moveit_msgs::CollisionObject &obj, const tf2_ros::Buffer &tf_buffer)
{
    // *** get obj from planning scene
    ros::NodeHandle n("~");
    ros::ServiceClient client = n.serviceClient<franka_movement_msgs::GetCollisionObjectById>("/planing_scene_node/get_collision_object_by_id");
    franka_movement_msgs::GetCollisionObjectById obj_by_id;
    obj_by_id.request.object_id = obj_id;
    if(!client.call(obj_by_id)){
        ROS_ERROR("Failed to load object with id %s from planning frame!", obj_id.c_str());
        return false;
    }
    obj = obj_by_id.response.collision_object;

    // *** transform object to desired frame
    geometry_msgs::PoseStamped obj_pose_not_transformed;
    obj_pose_not_transformed.header.frame_id = obj.header.frame_id;
    obj_pose_not_transformed.pose = obj.primitive_poses[0];
    geometry_msgs::PoseStamped obj_pose_transformed;
    try{
        obj_pose_transformed = tf_buffer.transform<geometry_msgs::PoseStamped>(obj_pose_not_transformed, target_frame,
                                                                              ros::Duration(0.1));
    }
    catch (tf2::TransformException &ex) {
        ROS_ERROR("Failed during tf transformation: %s",ex.what());
        return false;
    }
    obj.primitive_poses[0] = obj_pose_transformed.pose;

    n.shutdown();

    return true;
}


bool PlanningSceneUtils::get_obj_params_querry(const std::string &id, std::vector<float> *grasp_point, float *grasp_width,
                                        std::vector<float> *bounding_box)
{
    std::stringstream fmt;

    if(grasp_point) {
        fmt << "/objects_manipulation/" << id << "/grasp_point";
        if (!ros::param::get(fmt.str(), *grasp_point)) {
            ROS_ERROR("No param with namespace %s", fmt.str().c_str());
            return false;
        }
    }

    if(grasp_width) {
        fmt.str(std::string());
        fmt << "/objects_manipulation/" << id << "/grasp_width";
        if (!ros::param::get(fmt.str(), *grasp_width)) {
            ROS_ERROR("No param with namespace %s", fmt.str().c_str());
            return false;
        }
    }

    if(bounding_box) {
        fmt.str(std::string());
        fmt << "/objects_manipulation/" << id << "/boundingbox";
        if (!ros::param::get(fmt.str(), *bounding_box)) {
            ROS_ERROR("No param with namespace %s", fmt.str().c_str());
            return false;
        }
    }

    return true;
}

bool PlanningSceneUtils::get_name_and_number_from_id(const std::string &id, std::string &name, int &number)
{
    int pos = -1;
    std::string delimiter = "#";
    std::string id_tmp = id;

    int pos_tmp;
    while ((pos_tmp = id_tmp.find(delimiter)) != std::string::npos) {
        pos += pos_tmp + delimiter.length();
        id_tmp.erase(0, pos_tmp + delimiter.length());
    }

    if(pos < 0){
        // no delimiter found
        number = -1;
        name = id;
        return false;
    }

    name = id.substr(0, pos);
    number = stoi(id_tmp);      // last part of the id is the number

    return true;
}

bool PlanningSceneUtils::is_same_object(const std::string &id0, const geometry_msgs::Pose &pose0, const std::string &id1,
                                        const geometry_msgs::Pose &pose1, float threshold)
{
    // check ids
    std::string name0;
    int number0;
    PlanningSceneUtils::get_name_and_number_from_id(id0, name0, number0);

    std::string name1;
    int number1;
    PlanningSceneUtils::get_name_and_number_from_id(id1, name1, number1);

    if(name0.compare(name1) != 0){
        return false;
    }
    if(number0 == number1){
        // if both names an numbers are the same the objects are considered to be the same
        return true;
    }

    // if this is correct check if the amount of the result is below the threshold
    Eigen::Affine3d e_pose0;
    tf::poseMsgToEigen(pose0, e_pose0);

    Eigen::Affine3d e_pose1;
    tf::poseMsgToEigen(pose1, e_pose1);
    if((e_pose1.translation() - e_pose0.translation()).norm() > threshold) {
        return false;
    }

    return true;
}
