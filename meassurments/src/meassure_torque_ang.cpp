#include "ros/ros.h"
#include "std_msgs/String.h"
#include <ros/package.h>
#include <fstream>
#include <iostream>
#include <ctime>

std::ofstream file_meassurements;

void data_callback(const std_msgs::String::ConstPtr& msg) // callback data: write into file
{
    //ROS_INFO("I heard: [%s]", msg->data.c_str());
    file_meassurements << msg->data.c_str() << std::endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "publisher_torque");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/fake_data", 1000, data_callback);
    const std::string pkg_name ="meassurments";
    std::string path = ros::package::getPath(pkg_name) +  "/Data/";
    std::cout << "Path Meassurement: " <<  path << std::endl;

    // encode meassurement file with the time
    time_t rawtime;
    struct tm * timeinfo;
    char buffer [80];
    time (&rawtime);
    timeinfo = localtime (&rawtime);
    strftime (buffer,80,"torque_ang_%Y%m%d_%H_%M_%S.txt",timeinfo);
    path += buffer;
    std::cout << path << std::endl;

    file_meassurements.open(path);
    if (file_meassurements.is_open()) {
        ROS_INFO("started Meassurement");
    }


    ros::spin();
    return 0;
}
