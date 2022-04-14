#include "franka_movement/Arm.hpp"

#include "franka_movement_msgs/GetCollisionObjectById.h"
#include "franka_movement_msgs/PickAndPlace.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <math.h>
#include <geometry_msgs/Vector3.h>
#include <tf2/transform_datatypes.h>

//#include <control_msgs/GripperCommandAction.h>
#include <control_msgs/GripperCommandAction.h>
#include <control_msgs/GripperCommandGoal.h>

#include <franka/gripper.h>
#include <iostream>
#include "franka_movement/PlanningSceneUtils.hpp"

using namespace franka_move;

// *** Callbacks ***

bool Arm::basic_pick_callback(franka_movement_msgs::FrankaBasicPick::Request &req,
                              franka_movement_msgs::FrankaBasicPick::Response &res)
{
    bool constrain_grasp_width = false;
    if(req.constrain_grasp_width != NULL){
        constrain_grasp_width = req.constrain_grasp_width;
        std::cout << "setting width" << std::endl;
    }
    std::cout << "grasp width: " << constrain_grasp_width << std::endl;

    res.error_code = basic_pick(req.object_id, req.supported_surface, constrain_grasp_width);
    return true;
}

bool Arm::pick_callback(franka_movement_msgs::FrankaPick::Request &req,
                        franka_movement_msgs::FrankaPick::Response &res)
{
    moveit_msgs::Grasp grasp = req.grasp;
    // Setting posture of eef before grasp
    openGripper(grasp.pre_grasp_posture);

    // Setting posture of eef during grasp
    //closedGripper(grasp.grasp_posture);

    res.error_code = pick(grasp, req.object_id);

    return true;
}

bool Arm::place_callback(franka_movement_msgs::FrankaPlace::Request &req,
                        franka_movement_msgs::FrankaPlace::Response &res)
{
    moveit_msgs::PlaceLocation placeLocation = req.place_location;

    // Open the Gripper after the place
    openGripper(placeLocation.post_place_posture);

    res.error_code = place(placeLocation, req.object_id);

    return true;
}

bool Arm::basic_place_callback(franka_movement_msgs::FrankaBasicPlace::Request &req,
                        franka_movement_msgs::FrankaBasicPlace::Response &res)
{
    bool constrain_grasp_width = false;
    if(req.constrain_grasp_width != NULL){
        constrain_grasp_width = req.constrain_grasp_width;
        std::cout << "setting width" << std::endl;
    }
    std::cout << "grasp width: " << constrain_grasp_width << std::endl;

    res.error_code = basicPlace(req.object_id, req.place_pose, req.supported_surface, constrain_grasp_width);

    return true;
}


// *** private functions ***

moveit::planning_interface::MoveItErrorCode Arm::pick(moveit_msgs::Grasp &grasp, std::string &obj_id)
{
    std::vector<moveit_msgs::Grasp> grasps;
    grasps.push_back(grasp);

    moveit::planning_interface::MoveItErrorCode result = (move_group.pick(obj_id, grasps));

    ROS_INFO("Motion plan %s", result == moveit::planning_interface::MoveItErrorCode::SUCCESS ? "SUCCESSED" : "FAILED");
    return result;
}

moveit::planning_interface::MoveItErrorCode Arm::place(moveit_msgs::PlaceLocation &place_location, std::string &obj_id)
{
    std::vector<moveit_msgs::PlaceLocation> places;
    places.push_back(place_location);

    moveit::planning_interface::MoveItErrorCode result = move_group.place(obj_id, places);

    ROS_INFO("Motion plan %s", result == moveit::planning_interface::MoveItErrorCode::SUCCESS ? "SUCCESSED" : "FAILED");
    return result;
}


void print_matrix3x3(const tf2::Matrix3x3 &mat)
{
    for(int i = 0; i < 3; i++){
        for(int k = 0; k < 3; k++){
            std::cout << mat.getRow(i)[k] << "\t";
        }
        std::cout << std::endl;
    }
}

tf2::Quaternion Arm::shortest_grasp_rotation(const geometry_msgs::Pose &obj_pose, bool from_stack)
{
    // *** object orientation matrix
    tf2::Quaternion quat_obj;
    tf2::fromMsg(obj_pose.orientation, quat_obj);
    tf2::Matrix3x3 mat_obj(quat_obj);


    // *** axis flips
    // flip z (over x) if it points upwards (we need to grasp from above -> z needs to point downwards)
    if(mat_obj.getColumn(2).z() > 0.0){
        tf2::Matrix3x3 flip_z_over_x;
        flip_z_over_x.setRPY(-M_PI, 0.0, 0.0);
        mat_obj = mat_obj * flip_z_over_x ;
    }


    // flip x (over z) if it points to the robot base
    if(from_stack) {
        if (mat_obj.getColumn(0).x() > 0.0) {
            tf2::Matrix3x3 flip_x_over_z;
            flip_x_over_z.setRPY(0.0, 0.0, -M_PI);
            mat_obj = mat_obj * flip_x_over_z;
        }
    }else{
        if (mat_obj.getColumn(0).x() < 0.0) {
            tf2::Matrix3x3 flip_x_over_z;
            flip_x_over_z.setRPY(0.0, 0.0, -M_PI);
            mat_obj = mat_obj * flip_x_over_z;
        }
    }


    tf2::Quaternion result;
    mat_obj.getRotation(result);
    return result.normalize();
}

int call_pick_place_service(franka_movement_msgs::PickAndPlace &pick_place_srv)
{
    ros::NodeHandle n("~");
    std::string service_name = "/pick_place/pick_place_service";
    ros::ServiceClient client = n.serviceClient<franka_movement_msgs::PickAndPlace>(service_name);
    client.waitForExistence();
    if(!client.call(pick_place_srv)){
        ROS_ERROR("Failed to call service %s.", service_name.c_str());
        return -1;
    }

    n.shutdown();
    return pick_place_srv.response.error_code;
}

int Arm::basic_pick(std::string &obj_id, std::string &supported_surface, bool constrain_grasp_width)
{
    // *** PARAMS
    std::string planning_frame = "panda_link0";     // the planning frame for this grasp. Make sure to use a z-axis allinged to the world z-axis
    float grasp_translation = 0.2;
    float gripper_threshold = 0.02;
    float max_gripper_width = 0.07;

    moveit_msgs::CollisionObject obj;
    PlanningSceneUtils::get_collision_object_in_frame(obj_id, planning_frame, obj, tfBuffer);

    // *** prepare pick
    franka_movement_msgs::PickAndPlace pick;
    pick.request.pick = true;
    pick.request.distance = grasp_translation;
    pick.request.stack_position = supported_surface;
    pick.request.object_id = obj_id;

    // *** GRASP ORIENTATION
    // define grasp pose
    geometry_msgs::Pose obj_pose = obj.primitive_poses[0];
    // get best grasp angle
    // if x is smaller than 0 we flip the gripper the other direction
    tf2::Quaternion orientation = shortest_grasp_rotation(obj_pose, obj_pose.position.x < 0.0);        // without gripper orientation bias

    // the third column of the obj's orientation matrix is the normalized z axis in the parent frame
    tf2::Matrix3x3 obj_rotation_matrix(orientation);
    geometry_msgs::Vector3 grasp_direction;
    grasp_direction.x = obj_rotation_matrix.getColumn(2).x();
    grasp_direction.y = obj_rotation_matrix.getColumn(2).y();
    grasp_direction.z = obj_rotation_matrix.getColumn(2).z();

    pick.request.pose_stamped.header.frame_id = planning_frame;
    // orient relative to obj so that the gripper frame is equal to the obj frame
    pick.request.pose_stamped.pose.position = obj.primitive_poses[0].position;
    // the grasp pose needs to be shifted by the objects orientation
    pick.request.pose_stamped.pose.position.x = pick.request.pose_stamped.pose.position.x - grasp_translation * obj_rotation_matrix.getColumn(2).x();
    pick.request.pose_stamped.pose.position.y = pick.request.pose_stamped.pose.position.y - grasp_translation * obj_rotation_matrix.getColumn(2).y();
    pick.request.pose_stamped.pose.position.z = pick.request.pose_stamped.pose.position.z - grasp_translation * obj_rotation_matrix.getColumn(2).z();
    tf2::convert(orientation, pick.request.pose_stamped.pose.orientation);

    // set pre/post gripper width
    // if the flag is set use the obj width + a threshold as grasp width
    if(constrain_grasp_width){
        float width = obj.primitives[0].dimensions[1] + gripper_threshold;
        pick.request.pre_gripper_width = width <= max_gripper_width ? width : max_gripper_width;
    }else {
        pick.request.pre_gripper_width = max_gripper_width;
    }

    // post gripper width
    // todo grasp point not implemented yet (the vision don't guarantees the orientation of the obj axes so this can't be implemented yet)
    std::vector<float> grasp_point;
    float grasp_width;
    std::string name;
    int number;
    PlanningSceneUtils::get_name_and_number_from_id(obj_id, name, number);
    if(!PlanningSceneUtils::get_obj_params(name, grasp_point, grasp_width)){
        grasp_width = obj.primitives[0].dimensions[1];
    }
    pick.request.post_gripper_width = grasp_width;

    // call service
    return call_pick_place_service(pick);



    //moveit_msgs::Grasp grasp;
    //grasp.grasp_pose.header.frame_id = planning_frame;

    //// *** GRASP ORIENTATION
    //// define grasp pose
    //geometry_msgs::Pose obj_pose = obj.primitive_poses[0];
    //// get best grasp angle
    //tf2::Quaternion orientation_obj = shortest_grasp_rotation(obj_pose);        // without gripper orientation bias

    //// this bias is because of a strange implementation of moveits pick function
    //tf2::Quaternion orientation_bias;
    ////orientation_bias.setRPY(0.0, 0.0, M_PI / 4.0);
    //orientation_bias.setRPY(0.0, 0.0, 0.0);
    //tf2::Quaternion orientation = orientation_obj * orientation_bias;       // rotate the bias by the orientation of the obj
    //orientation.normalize();

    //// the third column of the obj's orientation matrix is the normalized z axis in the parent frame
    //tf2::Matrix3x3 obj_rotation_matrix(orientation_obj);
    //geometry_msgs::Vector3 grasp_direction;
    //grasp_direction.x = obj_rotation_matrix.getColumn(2).x();
    //grasp_direction.y = obj_rotation_matrix.getColumn(2).y();
    //grasp_direction.z = obj_rotation_matrix.getColumn(2).z();

    //// orient relative to obj so that the gripper frame is equal to the obj frame
    //grasp.grasp_pose.pose.position = obj.primitive_poses[0].position;
    //// the grasp pose needs to be shifted by the objects orientation
    //grasp.grasp_pose.pose.position.x = grasp.grasp_pose.pose.position.x - grasp_translation * obj_rotation_matrix.getColumn(2).x();
    //grasp.grasp_pose.pose.position.y = grasp.grasp_pose.pose.position.y - grasp_translation * obj_rotation_matrix.getColumn(2).y();
    //grasp.grasp_pose.pose.position.z = grasp.grasp_pose.pose.position.z - grasp_translation * obj_rotation_matrix.getColumn(2).z();
    //tf2::convert(orientation, grasp.grasp_pose.pose.orientation);


    //// Setting pre-grasp approach
    //// ++++++++++++++++++++++++++
    ///* Defined with respect to frame_id */
    //grasp.pre_grasp_approach.direction.header.frame_id = planning_frame;
    //grasp.pre_grasp_approach.direction.vector.x = grasp_direction.x;
    //grasp.pre_grasp_approach.direction.vector.y = grasp_direction.y;
    //grasp.pre_grasp_approach.direction.vector.z = grasp_direction.z;
    //grasp.pre_grasp_approach.min_distance = grasp_translation - (obj.primitives[0].dimensions[2] / 2.0);      // the half of the hight of the obj -> the distance to the middle point
    //grasp.pre_grasp_approach.desired_distance = grasp_translation;


    //// Setting post-grasp retreat
    //// ++++++++++++++++++++++++++
    ///* Defined with respect to frame_id */
    //grasp.post_grasp_retreat.direction.header.frame_id = planning_frame;
    //grasp.post_grasp_retreat.direction.vector.x = -grasp_direction.x;
    //grasp.post_grasp_retreat.direction.vector.y = -grasp_direction.y;
    //grasp.post_grasp_retreat.direction.vector.z = -grasp_direction.z;
    //grasp.post_grasp_retreat.min_distance = grasp_translation - (obj.primitives[0].dimensions[2] / 2.0);      // the half of the hight of the obj -> the distance to the middle point
    //grasp.post_grasp_retreat.desired_distance = grasp_translation;

    //// Setting posture of eef before grasp
    //// if the flag is set use the obj width + a threshold as grasp width
    //if(constrain_grasp_width){
    //    openGripper(grasp.pre_grasp_posture, obj.primitives[0].dimensions[1] / 2.0 + 0.01);
    //}else {
    //    openGripper(grasp.pre_grasp_posture);
    //}
    //// Setting posture of eef during grasp
    //// todo grasp point not implemented yet (the vision don't guarantees the orientation of the obj axes so this can't be implemented yet)
    //std::vector<float> grasp_point;
    //float grasp_width;
    //std::string name;
    //int number;
    //PlanningSceneUtils::get_name_and_number_from_id(obj_id, name, number);
    //if(!PlanningSceneUtils::get_obj_params(name, grasp_point, grasp_width)){
    //    grasp_width = obj.primitives[0].dimensions[1];
    //}

    //closedGripper(grasp.grasp_posture, grasp_width);        // the grippers finger span needs to be equal to the objects y dimension

    //// Set support surface as table1.
    //move_group.setSupportSurfaceName(supported_surface);
    //// Call pick to pick up the object using the grasps given
    //return pick(grasp, obj_id);
}

int Arm::basicPlace(std::string &obj_id, geometry_msgs::PoseStamped &place_pose, std::string &supported_surface, bool constrain_grasp_width)
{
    std::string planning_frame = "panda_link0";     // the planning frame for this grasp. Make sure to use a z-axis allinged to the world z-axis
    float place_translation = 0.2;
    float gripper_threshold = 0.02;
    float max_gripper_width = 0.08;

    // transform place_pose to planning frame
    try{
        place_pose = tfBuffer.transform<geometry_msgs::PoseStamped>(place_pose, planning_frame, ros::Duration(0.1));
    }
    catch (tf2::TransformException &ex) {
        ROS_ERROR("Failed during tf transformation: %s",ex.what());
        return false;
    }

    moveit_msgs::CollisionObject obj;
    PlanningSceneUtils::get_collision_object_in_frame(obj_id, planning_frame, obj, tfBuffer);

    // *** prepare place
    franka_movement_msgs::PickAndPlace place;
    place.request.pick = false;
    place.request.distance = place_translation;
    place.request.stack_position = supported_surface;
    place.request.object_id = obj_id;

    // *** place position
    place.request.pose_stamped.header.frame_id = planning_frame;
    place.request.pose_stamped.pose = place_pose.pose;

    std::cout << place.request.pose_stamped.pose.orientation.x << place.request.pose_stamped.pose.orientation.y
    << place.request.pose_stamped.pose.orientation.z << place.request.pose_stamped.pose.orientation.w << std::endl;

    // the third column of the obj's orientation matrix is the normalized z axis in the parent frame
    tf2::Quaternion orientation_q;
    tf2::fromMsg(place_pose.pose.orientation, orientation_q);
    tf2::Matrix3x3 obj_rotation_matrix(orientation_q);
    geometry_msgs::Vector3 place_direction;
    place_direction.x = obj_rotation_matrix.getColumn(2).x();
    place_direction.y = obj_rotation_matrix.getColumn(2).y();
    place_direction.z = obj_rotation_matrix.getColumn(2).z();

    // the grasp pose needs to be shifted by the objects orientation
    place.request.pose_stamped.pose.position.x = place.request.pose_stamped.pose.position.x - place_translation * obj_rotation_matrix.getColumn(2).x();
    place.request.pose_stamped.pose.position.y = place.request.pose_stamped.pose.position.y - place_translation * obj_rotation_matrix.getColumn(2).y();
    place.request.pose_stamped.pose.position.z = place.request.pose_stamped.pose.position.z - place_translation * obj_rotation_matrix.getColumn(2).z();

    // *** gripper width
    // if the flag is set use the obj width + a threshold as grasp width
    if(constrain_grasp_width){
        float width = obj.primitives[0].dimensions[1] + gripper_threshold;
        place.request.post_gripper_width = width <= max_gripper_width ? width : max_gripper_width;
    }else {
        place.request.post_gripper_width = max_gripper_width;
    }

    // call service
    return call_pick_place_service(place);


    //moveit_msgs::PlaceLocation place_location;

    //// place_pose
    //place_location.place_pose.header.frame_id = planning_frame;

    //// no orientation bias needed because moveits place function uses the right orientation of the EE
    //place_location.place_pose.pose.orientation = place_pose.pose.orientation;


    //// Setting pre-place approach
    //// ++++++++++++++++++++++++++
    ///* Defined with respect to frame_id */
    //place_location.pre_place_approach.direction.header.frame_id = planning_frame;
    //// TODO right now it is assumend that the place location corresponds with the middle point of the object
    //place_location.place_pose.pose.position = place_pose.pose.position;

    //// place_pose as rotation matrix -> get z-axis in relation to the planning frame
    //tf2::Quaternion place_ori_q(place_pose.pose.orientation.x, place_pose.pose.orientation.y,
    //                            place_pose.pose.orientation.z, place_pose.pose.orientation.w);
    //tf2::Matrix3x3 place_ori_matrix(place_ori_q);
    //// the third column of the place_pose orientation matrix is the normalized z axis in the parent frame
    //place_location.pre_place_approach.direction.vector.x = -place_ori_matrix.getColumn(2).x();
    //place_location.pre_place_approach.direction.vector.y = -place_ori_matrix.getColumn(2).y();
    //place_location.pre_place_approach.direction.vector.z = -place_ori_matrix.getColumn(2).z();
    //place_location.pre_place_approach.min_distance = place_translation - 0.02;      // TODO remove this magic number!!
    //place_location.pre_place_approach.desired_distance = place_translation;

    //// Setting post-place approach
    //// ++++++++++++++++++++++++++
    ///* Defined with respect to frame_id */
    //place_location.post_place_retreat.direction.header.frame_id = planning_frame;
    //// the third column of the place_pose orientation matrix is the normalized z axis in the parent frame
    //place_location.post_place_retreat.direction.vector.x = place_ori_matrix.getColumn(2).x();
    //place_location.post_place_retreat.direction.vector.y = place_ori_matrix.getColumn(2).y();
    //place_location.post_place_retreat.direction.vector.z = place_ori_matrix.getColumn(2).z();
    //place_location.post_place_retreat.min_distance = place_translation - 0.02;      // TODO remove this magic number!!
    //place_location.post_place_retreat.desired_distance = place_translation;

    //// if the flag is set use the obj width + a threshold as grasp width
    //if(constrain_grasp_width){
    //    openGripper(place_location.post_place_posture, obj.primitives[0].dimensions[1] / 2.0 + 0.01);   // todo get object
    //}else {
    //    openGripper(place_location.post_place_posture);
    //}

    //move_group.setSupportSurfaceName(supported_surface);

    //return place(place_location, obj_id);
}

void Arm::openGripper(trajectory_msgs::JointTrajectory &posture, float obj_width)
{
    /* Add both finger joints of panda robot. */
    posture.joint_names.resize(2);
    posture.joint_names[0] = "panda_finger_joint1";
    posture.joint_names[1] = "panda_finger_joint2";

    /* Set them as open, wide enough for the object to fit. */
    posture.points.resize(1);
    posture.points[0].positions.resize(2);
    posture.points[0].positions[0] = obj_width;
    posture.points[0].positions[1] = obj_width;
    posture.points[0].time_from_start = ros::Duration(0.5);
}

void Arm::closedGripper(trajectory_msgs::JointTrajectory& posture, float obj_width)
{
    /* Add both finger joints of panda robot. */
    posture.joint_names.resize(2);
    posture.joint_names[0] = "panda_finger_joint1";
    posture.joint_names[1] = "panda_finger_joint2";

    /* Set them as closed. */
    posture.points.resize(1);
    posture.points[0].positions.resize(2);
    posture.points[0].positions[0] = obj_width / 2.0;
    posture.points[0].positions[1] = obj_width / 2.0;
    posture.points[0].time_from_start = ros::Duration(0.5);
}

