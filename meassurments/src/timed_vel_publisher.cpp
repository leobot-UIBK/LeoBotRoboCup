#include <geometry_msgs/TwistStamped.h>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"


ros::Time operator +(const ros::Time t1, const ros::Time t2)
{
    ros::Time t_ans;
    t_ans.sec = t1.sec + t2.sec;
    if (t1.nsec + t2.nsec > 1e9){
       t_ans.nsec = t1.nsec + t2.nsec - 1e9;
       t_ans.sec = t_ans.sec + 1;
    }else {
       t_ans.nsec = t1.nsec + t2.nsec;
    }
    return t_ans;
}


int main(int argc, char **argv) {

    ros::init(argc, argv, "cmdvel_publisher_timed");
    ros::NodeHandle n;

    if (argc > 4) {
        float x = atof(argv[1]);
        float y = atof(argv[2]);
        float z = atof(argv[3]);
        float t = atof(argv[4]);
        // Advertize the publisher on the topic you like
        ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/leobot_base/cmd_vel", 10,true);
        // std::string msg = "Sleep for" + std::to_string(t) + "seconds";
        ROS_INFO("Publishing for [%f] seconds.", t);
        while (ros::ok()) {

            geometry_msgs::Twist myTwistMsg;
            // Here you build your twist message
            myTwistMsg.linear.x = x;
            myTwistMsg.linear.y = y;
            myTwistMsg.angular.z = z;

            ros::Time beginTime = ros::Time::now();

            int i = 1;
            /* myTwistMsg.header.seq = i;
            myTwistMsg.header.stamp = ros::Time::now();
            myTwistMsg.header.frame_id = "/base_link"; */
            pub.publish(myTwistMsg);
            ros::Time endTime = ros::Time::now() + ros::Time(t);
            ROS_INFO("Start Moving.");
            while (ros::Time::now() < endTime) {
               /* myTwistMsg.header.seq = i++;
               myTwistMsg.header.stamp = ros::Time::now();*/
               pub.publish(myTwistMsg);
                ros::Duration(0.1).sleep();
            }
            /* myTwistMsg.header.seq = i++;
            myTwistMsg.header.stamp = ros::Time::now(); */
            myTwistMsg.linear.x = 0;
            myTwistMsg.linear.y = 0;
            myTwistMsg.angular.z = 0;
            pub.publish(myTwistMsg);
            ROS_INFO("finished");
            n.shutdown();
            return 0;
        }
    } else{
        ROS_INFO("use of timed cmdvel publisher: \n");
        ROS_INFO("rosrun cmdvel_publisher_timed x y z t \n");
    }
}

