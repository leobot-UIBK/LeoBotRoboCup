#include <ros/ros.h>

#include <boost/lexical_cast.hpp>

#include "franka_movement/PlaningScene.hpp"

#define NODE_NAME "planing_scene_node"

int main(int argc, char *argv[]){
    // TODO node handle namespace shouldn't need to be setted
    ros::init(argc, argv, NODE_NAME);
    ros::NodeHandle n(NODE_NAME);
    ros::AsyncSpinner spinner(2);
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

    // get PlaningScene
    franka_move::PlaningScene planingScene(visualize);

    // set callbacks
    ros::ServiceServer collision_object_service =
            n.advertiseService("collision_object",
                               &franka_move::PlaningScene::collision_object_callback, &planingScene);

    ros::ServiceServer attached_collision_object_service =
            n.advertiseService("attached_collision_object",
                               &franka_move::PlaningScene::attached_collision_object_callback, &planingScene);

    ros::ServiceServer get_collision_object_by_id_service =
            n.advertiseService("get_collision_object_by_id",
                               &franka_move::PlaningScene::get_collision_object_by_id_callback, &planingScene);

    ros::ServiceServer add_collision_object_by_id_service =
            n.advertiseService("add_collision_object_by_id",
                               &franka_move::PlaningScene::add_collision_object_by_id_callback, &planingScene);


    ROS_INFO("Planing Scene node is running ...");

    ros::waitForShutdown();
}