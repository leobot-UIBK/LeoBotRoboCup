
#include <ros/ros.h>
#include <stdio.h>
#include <geometry_msgs/Twist.h>

#include <sensor_msgs/Joy.h>


class TeleopLeoBot
{
public:
    TeleopLeoBot();

private:
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

    ros::NodeHandle nh_;

    int linear_x, linear_y, angular; // angular_p, angular_m;
    double l_scale_x, l_scale_y, a_scale_;
    ros::Publisher vel_pub_;
    ros::Subscriber joy_sub_;

};

// base constructor:
TeleopLeoBot::TeleopLeoBot():
        linear_x(1),
        linear_y(2),
        angular(3)
{
    // read parameters from ROS parameter server
    nh_.param("axis_linear_x", linear_x, linear_x);
    nh_.param("axis_linear_y", linear_y, linear_y);
    nh_.param("axis_rotational_z", angular, angular);
    nh_.param("scale_linear_x", l_scale_x, l_scale_x);
    nh_.param("scale_linear_y", l_scale_y, l_scale_y);
    nh_.param("scale_angular", a_scale_, a_scale_);
    vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/leobot_base//cmd_vel", 1);
    joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopLeoBot::joyCallback, this);

}

void TeleopLeoBot::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    geometry_msgs::Twist twist;
    twist.angular.z = a_scale_*(joy->axes[angular]); // joy->buttons[angular_p] - joy->buttons[angular_m]);
    twist.linear.x = l_scale_x*joy->axes[linear_x];
    twist.linear.y = l_scale_y*joy->axes[linear_y];
    vel_pub_.publish(twist);
}


int main(int argc, char **argv){
    ros::init(argc, argv, "joy_publisher");
    TeleopLeoBot teleop_leobot;

    ros::spin();
    return EXIT_SUCCESS;
}
