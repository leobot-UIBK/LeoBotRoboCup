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
    int steps = 4;
    float vel = 0.5, vel_ang = 0.75;
    float t_act = 0;
    float t_rot = (M_PI/4)/vel_ang*2;
    float t_trans = 1.5;
    float factor_y = 2.2; 

    // for a square:
    /* 
    std::vector<float> vx {vel, 0, -vel, 0};
    std::vector<float> vy {0, vel, 0, -vel};
    std::vector<float> w {0, 0, 0, 0};
    std::vector<float> t {0, t_trans, t_trans*factor_y, t_trans, t_trans*factor_y};
*/ 
    std::vector<float> vx{vel,  0,      vel,      0,      vel,    0,      vel,      0};
    std::vector<float> vy{0,    0,      0,    0,      0,      0,      0,    0};
    std::vector<float> w{0,     vel_ang, 0,     vel_ang,0,      vel_ang,0,      vel_ang};
    std::vector<float> t{0, t_trans,  t_rot,  t_trans*t_trans,      t_rot,  t_trans,      t_rot,  t_trans*t_trans,      t_rot}; 
    float t_sum = 0;
    std::vector<float> t_acc;

    // to ensure stand still even if a message is not transmitted
    vx.push_back(0);
    vy.push_back(0);
    w.push_back(0);
    t.push_back(0.5);

    printf("planned movement:\n");
    for (int k = 0; k< t.size(); k++){
        t_sum += t[k];
        t_acc.push_back(t_sum);
        if (k > 0){
            printf("step %d \t t %f to %f \t x %f \t y %f \t w %f\n", k, t_acc[k-1], t_acc[k], vx[k-1], vy[k-1], w[k-1]);
        }
    }

    // Advertize the publisher on the topic you like
    ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/leobot_base/cmd_vel", 1,true);
    ROS_INFO("Publishing for [%f] seconds.", t_sum);
    while (ros::ok()) {
         geometry_msgs::Twist myTwistMsg;

        myTwistMsg.linear.x = vx[0];
        myTwistMsg.linear.y = vy[0];
        myTwistMsg.angular.z = w[0];
        ros::Time beginTime = ros::Time::now();
        int i = 1;
        // create header
        pub.publish(myTwistMsg);
        ROS_INFO("Start Moving.");
        int j = 0;
        while(t_act < t_sum) {
            if (t_act > t_acc[j] || t_act == t_acc[0]) {
                myTwistMsg.linear.x = vx[j];
                myTwistMsg.linear.y = vy[j];
                myTwistMsg.angular.z = w[j];
                ROS_INFO("t is %f, x is %f, y is %f, w is %f", t_act, vx[j], vy[j], w[j]);
                j++;
            }

            /* myTwistMsg.header.seq = i++;
            myTwistMsg.header.frame_id = "/base_link";
            myTwistMsg.header.stamp = ros::Time::now(); */

            pub.publish(myTwistMsg);
            ros::Duration(0.05).sleep();
            t_act += 0.05;
            }
            ROS_INFO("finished");
            n.shutdown();
            return 0;
        }
}

