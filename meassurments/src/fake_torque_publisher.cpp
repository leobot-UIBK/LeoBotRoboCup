
#include "ros/ros.h"

#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include <math.h>
#include <sstream>
#include <stdint.h>
#include <time.h>

double create_data(int t){
    return (1 + (0.01*(7.5 - (rand()%15))));
}

double create_phi_data(int t){
     double val =(1/(2*0.01))*1 *0.001*t * 0.001*t;
    return val + 0.4*(0.2 - rand() % 10000)/1000;
}


int main(int argc, char **argv)
{
    int t_fin = 1000;
    /**
     * The ros::init() function needs to see argc and argv so that it can perform
     * any ROS arguments and name remapping that were provided at the command line.
     * For programmatic remappings you can use a different version of init() which takes
     * remappings directly, but for most command-line programs, passing argc and argv is
     * the easiest way to do it.  The third argument to init() is the name of the node.
     *
     * You must call one of the versions of ros::init() before using any other
     * part of the ROS system.
     */
    ros::init(argc, argv, "fake_data_node");

    /**
     * NodeHandle is the main access point to communications with the ROS system.
     * The first NodeHandle constructed will fully initialize this node, and the last
     * NodeHandle destructed will close down the node.
     */
    ros::NodeHandle n;

    /**
     * The advertise() function is how you tell ROS that you want to
     * publish on a given topic name. This invokes a call to the ROS
     * master node, which keeps a registry of who is publishing and who
     * is subscribing. After this advertise() call is made, the master
     * node will notify anyone who is trying to subscribe to this topic name,
     * and they will in turn negotiate a peer-to-peer connection with this
     * node.  advertise() returns a Publisher object which allows you to
     * publish messages on that topic through a call to publish().  Once
     * all copies of the returned Publisher object are destroyed, the topic
     * will be automatically unadvertised.
     *
     * The second parameter to advertise() is the size of the message queue
     * used for publishing messages.  If messages are published more quickly
     * than we can send them, the number here specifies how many messages to
     * buffer up before throwing some away.
     */
    ros::Publisher fake_data = n.advertise<std_msgs::String>("fake_data", 1000);
    ros::Publisher activate_controller = n.advertise<std_msgs::Float64>("/velocity_controller/command", 1000);
    ros::Rate loop_rate(1000);
    std_msgs::Float64 msg_velocity;
//     std::string filename;

    auto t0 = ros::WallTime::now();
    auto t_now = t0;
    int long t = 0;
    bool flag_start = 0;
    ROS_INFO("start of meassurement");
    while (ros::ok())
    {
        if (flag_start == 0){
            msg_velocity.data = 1.0;
            activate_controller.publish(msg_velocity);
        }
        t_now = ros::WallTime::now();
        uint long t_sec = (t_now.sec - t0.sec);
        int long t_nsec = int32_t(t_now.nsec - t0.nsec)/1000000;
        t = (t_sec * 1000 + t_nsec);
        std_msgs::String msg;

        std::stringstream ss;
        double tau = create_data(t);
        double phi = create_phi_data(t);
        ss << t << "; " <<  tau << "; " << phi;
        msg.data = ss.str();
        /**
         * The publish() function is how you send messages. The parameter
         * is the message object. The type of this object must agree with the type
         * given as a template parameter to the advertise<>() call, as was done
         * in the constructor above.
         */
        fake_data.publish(msg);
        if (t>=t_fin){
            msg_velocity.data = 0.0;
            activate_controller.publish(msg_velocity);
            ROS_INFO("end of meassurement");
            break;
        }

        ros::spinOnce();

        loop_rate.sleep();

   }
   // file.close();
    return 0;
}
