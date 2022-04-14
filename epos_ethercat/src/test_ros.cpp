

#include <ros/ros.h>
#include "std_msgs/String.h"
#include "std_msgs/String.h"
#include "leobot_base_msg/motor_monitoring.h"

bool operator <(const timespec& lhs, const timespec& rhs)
{
    if (lhs.tv_sec == rhs.tv_sec)
        return lhs.tv_nsec < rhs.tv_nsec;
    else
        return lhs.tv_sec < rhs.tv_sec;
}

int main(int argc, char *argv[]){
    timespec a, b;
    clock_gettime(CLOCK_MONOTONIC, &(b));
    clock_gettime(CLOCK_MONOTONIC, &(a));

    if (a<b){
        printf("a > b");
    } else if (b < a){
        printf("b > a");
    }
    ros::init(argc, argv, "listener");
    leobot_base_msg::motor_monitoring msg;
    msg.Temperature.push_back(1);
    msg.Temperature[0] = 2;

    ros::NodeHandle n("leobot_base");
    std::vector<int> dir_w;
    n.getParam("dir_wheels", dir_w);
    printf("\ndir w read: %d ", dir_w[0]);

    std::vector<int32_t> dv;
    std::vector<int32_t> acc;
   dv.push_back(1); dv.push_back(7); dv.push_back(12); dv.push_back(-3);

    auto max_it = std::max_element(std::begin(dv), std::end(dv));
    printf("max it: %d", *(max_it));

    short test= 0b10010;


    std::vector<std::vector<int>> test_2Dvec;
    std::vector<int> test_1Dvec {1,2};
    test_2Dvec.push_back(test_1Dvec);
    test_2Dvec.push_back({{3, 4}});
    printf("test binary: %d and 2Dvec %d", test, test_2Dvec[0][1]);
    return EXIT_SUCCESS;
}
