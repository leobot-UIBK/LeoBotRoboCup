//
// Created by robo on 5/12/20.
//
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "EPOS4_class.h"


bool operator <(const timespec& lhs, const timespec& rhs)
{
    if (lhs.tv_sec == rhs.tv_sec)
        return lhs.tv_nsec < rhs.tv_nsec;
    else
        return lhs.tv_sec < rhs.tv_sec;
}

static void periodic_task_init(struct period_info *pinfo, float T_ms)
{
    pinfo->period_ns = 1000000*T_ms; // timeperiod is 1ms * Period time in ms
    pinfo->t_off = 0;
    clock_gettime(CLOCK_MONOTONIC, &(pinfo->next_period));
}

static void inc_period(struct period_info *pinfo)
{
    pinfo->next_period.tv_nsec += pinfo->period_ns + pinfo->t_off;
    while (pinfo->next_period.tv_nsec >= 1000000000) {
        /* timespec nsec overflow */
        pinfo->next_period.tv_sec++;
        pinfo->next_period.tv_nsec -= 1000000000;
    }
}

static void wait_rest_of_period(struct period_info *pinfo, int *ret)
{
    inc_period(pinfo); // increment by period time

    timespec t_act;
    clock_gettime(CLOCK_MONOTONIC, &t_act);
    if ((pinfo->next_period) < t_act) {
        *ret = 10;
        return;
        // cycle violation!
    }
    /* no signal wakes */
    *ret=0;
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &pinfo->next_period, nullptr);
}


int main(int argc, char **argv){
    ros::init(argc, argv, "sine_vel_publisher");
    ros::NodeHandle node_handle;
    ros::Publisher pub = node_handle.advertise<geometry_msgs::Twist>("/leobot_base/cmd_vel", 10);
    geometry_msgs::Twist tar_vel;

    struct period_info pinfo_pub;
    periodic_task_init(&pinfo_pub, 1);
    uint i;
    int ret = 0;

    while (ros::ok()) {
        switch(ret){
            case 0:
                tar_vel.linear.x = (float) 0.25 * sin((M_PI * i) / 2000);
                pub.publish(tar_vel);
                break;

            case 10: break; // cycle violation: do not send data
            default: break;
        }
        i++;
        wait_rest_of_period(&pinfo_pub, &ret);
    }

return EXIT_SUCCESS;
}