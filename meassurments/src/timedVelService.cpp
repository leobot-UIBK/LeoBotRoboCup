#include <geometry_msgs/TwistStamped.h>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "measurement_msgs/measurement_flag.h"
#include "measurement_msgs/timedVel.h"
#include "std_srvs/Trigger.h"

struct period_info {
    struct timespec next_period;
    long period_ns;
    long t_off;
};

static void inc_period(struct period_info *pinfo)
{
    pinfo->next_period.tv_nsec += pinfo->period_ns + pinfo->t_off;
    while (pinfo->next_period.tv_nsec >= 1000000000) {
        /* timespec nsec overflow */
        pinfo->next_period.tv_sec++;
        pinfo->next_period.tv_nsec -= 1000000000;
    }
}

// signum function
template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}


static void periodic_task_init(struct period_info *pinfo, float T_ms)
{
    pinfo->period_ns = 1000000*T_ms; // timeperiod is 1ms * Period time in ms
    pinfo->t_off = 0;
    clock_gettime(CLOCK_MONOTONIC, &(pinfo->next_period));
}
bool operator <(const timespec& lhs, const timespec& rhs)
{
    if (lhs.tv_sec == rhs.tv_sec)
        return lhs.tv_nsec < rhs.tv_nsec;
    else
        return lhs.tv_sec < rhs.tv_sec;
}

static void wait_rest_of_period(struct period_info *pinfo, int *ret)
{
    inc_period(pinfo); // increment by period time

    timespec t_act;
    clock_gettime(CLOCK_MONOTONIC, &t_act);
    if ((pinfo->next_period) < t_act) {
        *ret = 10;
        return;
        // cycle violation! --> the next action may be skipped to catch up
    }
    /* no signal wakes implemented --> remaining in nullptr...*/
    *ret=0;
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &pinfo->next_period, nullptr);
}
void calculate_times(float &s, float &v, float &a, float &ta, float &tv, float &te){
    if (v != 0 & a != 0) {
        ta = v / a;
        te = s/v + ta;
        tv = te - ta;
        if (te < 2*ta){ // max velocity is never reached
            ta = sqrt((2/a) * s/2); // time to reach half the value
            te = 2*ta;
            tv = ta;
        }
    } else if (s == 0){
        ta = 0;
        tv = 0;
        te = 0;
    }
 //printf("ta: %f \t te %f \t tv %f\n", ta, te, tv); // for debugging
    return;
}

float calculate_velocity(float v_max, float a_max, float t, float ta, float tv, float te){
    float v = 0;
    if (t <= ta){
        v = t * a_max;
    }
    else if (t > ta & t < tv){
        v = v_max;
    }
    else if (t >= tv & t <= te){
        v = a_max * (te - t);
    }
    return v;
}

// scale acceleration and velocity of the to align all times (acceleration etc.)
void synchronize_velacc(std::vector<float> s, std::vector<float> &v_max, std::vector<float> &a_max, std::vector<float> &ta, std::vector<float> &tv, std::vector<float> &te){
    int i_maxTime = std::max_element(te.begin(),te.end()) - te.begin();
    int i_maxTimeacc = std::max_element(ta.begin(),ta.end()) - ta.begin();
    if (i_maxTime != i_maxTimeacc){
        te[i_maxTime] = te[i_maxTime] - 2*ta[i_maxTime] + 2*ta[i_maxTimeacc];
        tv[i_maxTime] = tv[i_maxTime] - ta[i_maxTime] + ta[i_maxTimeacc];
        printf("Warning! case i_max(ta) != i_max(te) not implemented yet! ");
    }
    for (int i = 0; i<3; i++){
        v_max[i] = (s[i]/(te[i_maxTime] - ta[i_maxTimeacc]));
        a_max[i] = v_max[i]/ta[i_maxTimeacc];
    }
    for (int i = 0; i<3; i++){
        calculate_times(s[i], v_max[i], a_max[i], ta[i], tv[i], te[i]);
        if (te[i] == 0 | isnan(te[i])){
        ta[i] = ta[i_maxTime];
        tv[i] = tv[i_maxTime];
        te[i] = te[i_maxTime];
        }
    }
    for (int i = 0; i<3; i++){
        calculate_times(s[i], v_max[i], a_max[i], ta[i], tv[i], te[i]);
    }
    if ( std::adjacent_find(te.begin(), te.end(), std::not_equal_to<>() ) == te.end()){
        printf("te not equal!\n");
    }

        return;
}

// Service Function
/* obtains the distance and max velocities/accelerations for all directions from the service request and plans a
 * trajectory with a ramp profile in the velocities (linear velocities, constant acceleration). The disctance is fixed,
 * while the velocities/accelerations given are the maximal velocities/accelerations and are changed to obtain a
 * fully synchronous interpolation.   */
bool PublishVelocities(measurement_msgs::timedVel::Request & req,
                      measurement_msgs::timedVel::Response & res,
                      ros::Publisher & pubVel, ros::NodeHandle n)
{
    // read parameters for movement from service request and parameter server
    int index = req.data[0];
    float sx, sy, phiz, vx_max, vy_max, w_max, ax_max, ay_max, dw_max, vx, vy, w;
    sx = req.data[1];
    sy = req.data[2];
    phiz = req.data[3];
    vx_max = abs(req.data[4]);
    vy_max = abs(req.data[5]);
    w_max = abs(req.data[6]);
    ax_max = abs(req.data[7]);
    ay_max = abs(req.data[8]);
    dw_max = abs(req.data[9]);
    // ToDo: add sign check for s to plan in positive domain and flip sign accordingly (otherwise v must also be negative)
    std::vector<int> sign_traj{sgn(sx),sgn(sy),sgn(phiz)};
    sx = abs(sx); sy = abs(sy); phiz = abs(phiz);
    // printf("Movement i= %d, s = [%f %f %f], v = [%f, %f, %f], a=[%f, %f, %f]\n", index, sx, sy, phiz, vx_max, vy_max, w_max, ax_max, ay_max, dw_max);
    float dw_WheelDef = 0, w_WheelDef=0, rWheel = 0, ly_robot = 0, lx_robot = 0;  // default parameters on the parameter server to restore after service

    n.getParam("/leobot_base/max_wheelvel", w_WheelDef); // obtain the max angular acceleration from the parameter server
    n.getParam("/leobot_base/max_wheelacc", dw_WheelDef); // obtain the max angular acceleration from the parameter server
    n.getParam("/leobot_base/wheelradius", rWheel); // obtain the max angular acceleration from the parameter server
    n.getParam("/leobot_base/lx", lx_robot); // obtain robot parameters from the parameter server
    n.getParam("/leobot_base/ly", ly_robot);

    // plan smooth trajectory with constant acceleration
    ROS_INFO("TRAJECTORY started");
    geometry_msgs::Twist velMsg;
    if(sx == 0){
        vx_max = 0;
        ax_max = 0;
    }
    if(sy == 0){
        vy_max = 0;
        ay_max = 0;
    }
    if(phiz == 0){
        w_max = 0;
        dw_max = 0;
    }
    float w_needed = (1/rWheel) * (abs(vx_max) + abs(vy_max) + abs(w_max)*(lx_robot+ly_robot));
    float dw_needed = (1/rWheel) * (abs(ax_max) + abs(ay_max) + abs(dw_max)*(lx_robot+ly_robot));
    if (w_needed > w_WheelDef){
        float vel_scale = w_WheelDef / w_needed;
        printf("Velocity scaled by factor %f.\n", vel_scale);
        vx_max = vx_max * vel_scale; // scaling of velocities
        vy_max = vy_max * vel_scale;
        w_max = w_max * vel_scale;

    }
    if(dw_needed > dw_WheelDef){
        float acc_scale = dw_WheelDef / dw_needed;
        printf("Acceleration scaled by factor %f.\n", acc_scale);
        ax_max = ax_max * acc_scale; // scaling of velocities
        ay_max = ay_max * acc_scale;
        dw_max = dw_max * acc_scale;
    }
    std::vector<float> ta(3, 0); std::vector<float> tv(3,0); std::vector<float> te(3,0);
    std::vector<float> s {sx,sy,phiz}; std::vector<float> v_max {vx_max,vy_max,w_max}; std::vector<float> a_max {ax_max,ay_max,dw_max};
    std::vector<float> v_control(3,0);


    for (int i = 0; i<3; i++){
        if(s[i] != 0 & (v_max[i] == 0 | a_max[i] == 0)) {
            res.flagSuccess = 0;
            ROS_ERROR("Error! Trajectory could not be planned! ");
            return 0;
        }
        calculate_times(s[i], v_max[i], a_max[i], ta[i], tv[i], te[i]);
    }
    /* calculate_times(sx, vx_max, ax_max, ta[0], tv[0], te[0]);
    calculate_times(sy, vy_max, ay_max, ta[1], tv[1], te[1]);
    calculate_times(phiz, w_max, dw_max, ta[2], tv[2], te[2]); */
    // slow axes to reach the end at the same time (synchronious interpolation)
    synchronize_velacc(s, v_max, a_max, ta, tv, te);

    // initialize timespecs for loop
    timespec t_start;
    timespec t_act;
    int T_ms = 1; // the cycle time for the publisher
    struct period_info pInfo;
    periodic_task_init(&pInfo, T_ms);
    int i_maxTime = std::max_element(te.begin(),te.end()) - te.begin();

    int ret = 0;
    float t = 0.0;
    for (int i = 0; i<3; i++){
        printf("i = %d, ta = %f, tv = %f, te = %f, v = %f, a = %f\n", i, ta[i], tv[i], te[i], v_max[i], a_max[i]);
    }
    // ToDo: update new wheelvelocities and acceleration onto parameterserver, restore the initial configuration at the end.
    float w_WheelUsed = (1/rWheel) * (abs(v_max[0]) + abs(v_max[1]) + abs(v_max[2])*(lx_robot+ly_robot));
    float dw_WheelUsed = (1/rWheel) * (abs(a_max[0]) + abs(a_max[1]) + abs(a_max[2])*(lx_robot+ly_robot));
    n.setParam("/leobot_base/max_wheelvel", w_WheelUsed); // obtain the max angular acceleration from the parameter server
    n.setParam("/leobot_base/max_wheelacc", dw_WheelUsed); // obtain the max angular acceleration from the parameter server
    // call service of the base to ensure update of parameters
     ros::ServiceClient client = n.serviceClient<std_srvs::Trigger>("/leobot_base/parameters/TriggerUpdate");
    std_srvs::Trigger srv;
    client.call(srv);
    ROS_INFO("The trajectory will take %f seconds from which %f seconds are acceleration.  \n", te[i_maxTime], ta[i_maxTime]);
    clock_gettime(CLOCK_MONOTONIC, &t_start);
    clock_gettime(CLOCK_MONOTONIC, &t_act);

    pubVel.publish(velMsg);
    while(t <= te[i_maxTime] && ros::ok()){ // all entrys of te are equal after scaling
        clock_gettime(CLOCK_MONOTONIC, &t_act);
        t = float(t_act.tv_sec - t_start.tv_sec) + float(t_act.tv_nsec - t_start.tv_nsec)*1e-9;
        if (ret == 0) {
            for (int i = 0; i < 3; i++) {
                v_control[i] = sign_traj[i] * calculate_velocity(v_max[i], a_max[i], t, ta[i], tv[i],
                                                                 te[i]); // calculate velocities for new timestep
            }
            velMsg.linear.x = v_control[0];
            velMsg.linear.y = v_control[1];
            velMsg.angular.z = v_control[2];
            // t += T_ms * 1e-3; // add period in milli seconds
            pubVel.publish(velMsg); // publish calculated velocities for control node
            ros::spinOnce();
        }
        else if(ret == 10){
            // cycle violation
        }
        // todo: check if and how many cycles were missed!
        wait_rest_of_period(&pInfo, &ret); // wait for continuation
   }
    if(not(ros::ok())){
        res.flagSuccess = 0;
    }
    else{
        res.flagSuccess = 1;
    }
    res.time = t;
    velMsg.linear.x = 0;
    velMsg.linear.y = 0;
    velMsg.angular.z = 0;
    pubVel.publish(velMsg);
    printf("Trajectory finished.\n");
    n.setParam("/leobot_base/max_wheelvel", w_WheelDef); // obtain the max angular acceleration from the parameter server
    n.setParam("/leobot_base/max_wheelacc", dw_WheelDef); // obtain the max angular acceleration from the parameter server

    client.call(srv);
    return 1;
}


int main(int argc, char **argv) {

    ros::init(argc, argv, "TrajectoryServer");
    ros::NodeHandle n;
    ros::Publisher pubVel = n.advertise<geometry_msgs::Twist>("/leobot_base/cmd_vel", 100, true);
    ros::ServiceServer service = n.advertiseService<measurement_msgs::timedVel::Request, measurement_msgs::timedVel::Response>
            ("/leobot_base/TrajectorySrv", boost::bind(PublishVelocities,_1,_2, pubVel, n));
    // ros::Publisher pubVel = tempNH.advertise<geometry_msgs::Twist>("/leobot_base/cmd_vel", 100, true);
    ROS_INFO("Trajectory Server is ready for movement.");
    ros::spin();
}

