#include "franka_movement/PlaningScene.hpp"

#include <moveit/planning_scene_interface/planning_scene_interface.h>

using namespace franka_move;


PlaningScene::PlaningScene(const bool &visualize) : visual_tools("panda_link0", "visualization_marker_array"),
                                                    tfListener(tfBuffer)
{
    // init DEBUG visualization
    ROS_INFO_NAMED("DEBUG", "Initialize visualisation");
    visual_tools.deleteAllMarkers();
    visual_tools.loadRemoteControl();

    PlaningScene::add_base_box();
}

void PlaningScene::add_base_box()
{
    // add BaseBox
    moveit_msgs::CollisionObject base_box;
    base_box.header.frame_id = "panda_link0";
    base_box.id = "base_box";
    base_box.operation = moveit_msgs::CollisionObject::ADD;

    // shape and pose
    shape_msgs::SolidPrimitive base_box_primitive;
    base_box_primitive.type = shape_msgs::SolidPrimitive::BOX;
    base_box_primitive.dimensions = {0.705, 0.504, 0.2612};        // TODO use accurate values

    geometry_msgs::Pose base_box_pose;
    base_box_pose.orientation.w = 1.0;

    base_box_pose.position.x = -0.211;
    base_box_pose.position.y = 0.0;
    base_box_pose.position.z = 0.0794;

    base_box.primitives.push_back(base_box_primitive);
    base_box.primitive_poses.push_back(base_box_pose);

    moveit_msgs::AttachedCollisionObject base_box_attached;
    base_box_attached.object = base_box;
    base_box_attached.link_name = "panda_link0";
    base_box_attached.touch_links = {"panda_link0", "panda_link1"};        // allow collisions with the link1

    add_attached_object(base_box_attached);
}

