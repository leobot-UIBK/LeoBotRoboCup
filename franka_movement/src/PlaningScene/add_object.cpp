#include "franka_movement/PlaningScene.hpp"

#include "franka_movement_msgs/CollisionObject.h"
#include "franka_movement_msgs/AttachedCollisionObject.h"

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <boost/foreach.hpp>
#include "franka_movement/PlanningSceneUtils.hpp"

using namespace franka_move;


// ------------------------------------------------------------------------- Callbacks

void PlaningScene::test_area(moveit_msgs::CollisionObject &object)
{
    std::map<std::string, moveit_msgs::CollisionObject> obj_map = planning_scene_interface.getObjects();

    std::pair<std::string, moveit_msgs::CollisionObject> obj_pair;
    BOOST_FOREACH(obj_pair, obj_map) {
        PlanningSceneUtils::is_same_object(obj_pair.second.id, obj_pair.second.primitive_poses[0], object.id,
                                           object.primitive_poses[0]);
    }

}

bool PlaningScene::collision_object_callback(franka_movement_msgs::CollisionObject::Request &req,
                                                 franka_movement_msgs::CollisionObject::Response &res){
    std::string target_frame_id = "world";

    moveit_msgs::CollisionObject object = req.collision_object;

    // transform object to world frame.
    for(int i = 0; i < object.primitive_poses.size(); i++){
        if(!transform_pose_to_frame(object.primitive_poses[i], object.header.frame_id, target_frame_id)){
            return false;
        }
    }
    object.header.frame_id = target_frame_id;

    if(!has_id(object)){
        ROS_ERROR("No object id set. Aborting!");
        return false;
    }

    switch(req.operation){
        case franka_movement_msgs::CollisionObject::Request::ADD:
            if(is_existing_object(object.id)){
                if(!move_collision_object(object)){
                    ROS_ERROR("Object with id %s not moved.", object.id.c_str());
                    return false;
                }
            }else{
                if(!add_collision_object(object)){
                    ROS_ERROR("Object with id %s not added.", object.id.c_str());
                    return false;
                }
            }
            break;
        case franka_movement_msgs::CollisionObject::Request::REMOVE:
            if(!PlaningScene::remove_collision_object(object)){
                return false;
            }
            break;
        default:
            ROS_ERROR("Please set an operation. Possible operations are ADD, REMOVE\n");
            return false;
    }

    return true;
}

bool PlaningScene::attached_collision_object_callback(franka_movement_msgs::AttachedCollisionObject::Request &req,
                                  franka_movement_msgs::AttachedCollisionObject::Response &res){
    moveit_msgs::AttachedCollisionObject attached_obj = req.attached_collision_object;
    if(!has_id(attached_obj.object)){
        ROS_ERROR("No object id set. Aborting!");
        return false;
    }

    std::string target_frame_id = "world";

    for(int i = 0; i < attached_obj.object.primitive_poses.size(); i++){
        if(!transform_pose_to_frame(attached_obj.object.primitive_poses[i], attached_obj.object.header.frame_id, target_frame_id)){
            return false;
        }
    }
    attached_obj.object.header.frame_id = target_frame_id;

    switch (req.operation) {
        case franka_movement_msgs::AttachedCollisionObject::Request::ADD:
            if(is_existing_object(attached_obj.object.id)){
                if(!attach_existing_object(attached_obj)){
                    return false;
                }
            }else{
                add_attached_object(attached_obj);
            }
            break;
        case franka_movement_msgs::AttachedCollisionObject::Request::REMOVE:
            detach_existing_object(attached_obj);
            if(!remove_collision_object(attached_obj.object)){
                return false;
            }
            break;
        case franka_movement_msgs::AttachedCollisionObject::Request::DETACH:
            detach_existing_object(attached_obj);
            break;
        default:
            ROS_ERROR("Please set an operation. Possible operations are ADD, REMOVE, DETACH\n");
            return false;
    }

    return true;
}

bool PlaningScene::get_collision_object_by_id_callback(franka_movement_msgs::GetCollisionObjectById::Request &req,
                                                       franka_movement_msgs::GetCollisionObjectById::Response &res)
{
    moveit_msgs::CollisionObject obj;
    if(!get_object_by_id(req.object_id, obj)){
        return false;
    }
    res.collision_object = obj;

    return true;
}

bool PlaningScene::add_collision_object_by_id_callback(franka_movement_msgs::AddCollisionObjectById::Request &req,
                                                       franka_movement_msgs::AddCollisionObjectById::Response &res)
{
    std::string id = req.id;

    std::vector<float> bounding_box;
    if(!PlanningSceneUtils::get_obj_params(req.id, bounding_box)){
        ROS_ERROR("Object with id %s not added!", req.id.c_str());
        res.success = false;
        return false;
    }

    // check if object already exists and get id
    if(!generate_collision_object_id(id, req.poseStamped)){
        ROS_INFO("Object already in scene with id %s. Updating location.", id.c_str());
    }

    // build new object
    moveit_msgs::CollisionObject new_object;
    new_object.id = id;
    // note that the object will be transformed and added to the world frame later on
    new_object.header.frame_id = req.poseStamped.header.frame_id;

    shape_msgs::SolidPrimitive primitive;
    primitive.type = shape_msgs::SolidPrimitive::BOX;
    for(float box : bounding_box){
        primitive.dimensions.push_back(box);
    }

    new_object.primitives.push_back(primitive);
    new_object.primitive_poses.push_back(req.poseStamped.pose);

    // add by calling collision_object_callback
    ros::NodeHandle n("~");
    ros::ServiceClient client = n.serviceClient<franka_movement_msgs::CollisionObject>("collision_object");
    franka_movement_msgs::CollisionObject srv;
    srv.request.operation = srv.request.ADD;
    srv.request.collision_object = new_object;
    if (!client.call(srv)){
        res.success = false;
        return false;
    }
    res.planning_scene_id = id;
    res.success = true;

    return true;
}

// ------------------------------------------------------------------------- Private functions

bool PlaningScene::is_existing_object(std::string obj_id){
    std::vector<std::string> ids = planning_scene_interface.getKnownObjectNames();
    for(const std::string& id : ids){
        if(id == obj_id){
            return true;
        }
    }
    return false;
}

bool PlaningScene::add_collision_object(moveit_msgs::CollisionObject &object){
    // if id exits move object
    object.operation = moveit_msgs::CollisionObject::ADD;

    // add/move object to scene
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.push_back(object);

    if(!planning_scene_interface.applyCollisionObjects(collision_objects)){
        return false;
    }

    ROS_INFO("Object %s added/moved", object.id.c_str());
    return true;
}

bool PlaningScene::move_collision_object(moveit_msgs::CollisionObject &object){
    // moving as removing and adding
    // TODO consider to do this with the MOVE flag of collision objects
    std::vector<std::string> object_ids;
    object_ids.push_back(object.id);
    planning_scene_interface.removeCollisionObjects(object_ids);

    if(!add_collision_object(object)){
        return false;
    }

    return true;
}


bool PlaningScene::attach_existing_object(moveit_msgs::AttachedCollisionObject &object) {
    // remove old object
    if(!remove_collision_object(object.object)){
        ROS_ERROR("Attaching failed!");
        return false;
    }

    // and add new attached object
    add_attached_object(object);
    return true;
}

void PlaningScene::add_attached_object(moveit_msgs::AttachedCollisionObject &object){
    object.object.operation = moveit_msgs::CollisionObject::ADD;

    // add/move object to scene
    planning_scene_interface.applyAttachedCollisionObject(object);

    ROS_INFO("Object %s attached", object.object.id.c_str());
}

void PlaningScene::detach_existing_object(moveit_msgs::AttachedCollisionObject &object){
    object.object.operation = moveit_msgs::CollisionObject::REMOVE;
    planning_scene_interface.applyAttachedCollisionObject(object);

    add_collision_object(object.object);
}


bool PlaningScene::get_object_by_id(std::string obj_id, moveit_msgs::CollisionObject &obj)
{
    std::vector<std::string> obj_ids;
    obj_ids.push_back(obj_id);

    std::map<std::string, moveit_msgs::CollisionObject> obj_map = planning_scene_interface.getObjects(obj_ids);
    if(obj_map.empty()){
       std::map<std::string, moveit_msgs::AttachedCollisionObject> att_obj_map = planning_scene_interface.getAttachedObjects(obj_ids);
       if(att_obj_map.empty()){
           ROS_ERROR("No Collision object with name %s", obj_id.c_str());
           return false;
       }else{
           obj = att_obj_map.at(obj_id).object;
       }
    } else {
        obj = obj_map.at(obj_id);
    }
    return true;
}

// removeObject
bool PlaningScene::remove_collision_object(moveit_msgs::CollisionObject &object){
    if(!is_existing_object(object.id)) {
        ROS_ERROR("No object with id %s found to remove!", object.id.c_str());
        return false;
    }
    object.operation = moveit_msgs::CollisionObject::REMOVE;
    planning_scene_interface.applyCollisionObject(object);
    return true;
}

bool PlaningScene::has_id(moveit_msgs::CollisionObject &object)
{
    if (object.id.empty()) {
        return false;
    } else {
        return true;
    }
}

bool PlaningScene::transform_pose_to_frame(geometry_msgs::Pose &pose, const std::string &source_frame_id, const std::string &target_frame_id)
{
    // transform input to world frame
    if(target_frame_id.compare(source_frame_id) == 0){
        return true;
    }
    try{
        geometry_msgs::TransformStamped transform;
        transform = tfBuffer.lookupTransform(target_frame_id, source_frame_id, ros::Time(0.0), ros::Duration(1.0));

        tf2::doTransform(pose, pose, transform);
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("%s",ex.what());
        return false;
    }
    return true;
}

bool PlaningScene::generate_collision_object_id(std::string &id, const geometry_msgs::PoseStamped &poseStamped)
{
    // pose needs to be in world frame
    std::string target_frame_id = "world";

    geometry_msgs::Pose pose = poseStamped.pose;
    if(!transform_pose_to_frame(pose, poseStamped.header.frame_id, target_frame_id)){
        return false;
    }

    // loop over all objects
    std::map<std::string, moveit_msgs::CollisionObject> obj_map = planning_scene_interface.getObjects();

    std::pair<std::string, moveit_msgs::CollisionObject> obj_pair;
    BOOST_FOREACH(obj_pair, obj_map) {
        // both poses need to be in the same frame
        geometry_msgs::Pose pose_other = obj_pair.second.primitive_poses[0];
        if(!transform_pose_to_frame(pose_other, obj_pair.second.header.frame_id, target_frame_id)){
            return false;
        }

        if(PlanningSceneUtils::is_same_object(obj_pair.second.id, pose_other, id, pose)){
            id = obj_pair.second.id;
            return false;
        }
    }
    // set new unique id
    id = id + "#" + std::to_string(num_objects_with_id++);

    return true;
}
