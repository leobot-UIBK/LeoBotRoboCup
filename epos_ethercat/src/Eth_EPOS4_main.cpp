//
// Created by robo on 4/20/20.
//

#include "EPOS4_class.h"
#include <limits.h>
#include <sched.h>
#include <signal.h>

#include "circular_buffer.h"

// ROS specific headers:
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include "geometry_msgs/Twist.h"
#include "leobot_base_msg/motor_status.h"
#include "leobot_base_msg/motor_monitoring.h"
#include "leobot_base_msg/motor_wheels.h"

#define EC_TIMEOUTMON 500

// Operation modes defined by the EPOS4-Firmware
#define OP_profile_pos 1
#define OP_profile_vel 3
#define OP_cyclic_torque 10


// #define cycletime

char IOmap[4096];
int expectedWKC;
boolean needlf;
volatile int wkc;
boolean inOP;
uint8 currentgroup = 0;

int64 toff, gl_delta; // for sync of DC

// checking if motor is functional
OSAL_THREAD_HANDLE thread1;

// timespec for status thread
timespec t_start_stat;
timespec t_act_stat;
pthread_t tid_status;

// timespec for control thread
timespec t_start_ctl;
timespec t_act_ctl;
pthread_t tid_ctl;

int err_cnt = 0;
int OPMode;

FILE *pFile;
pthread_mutex_t lock;

timespec t_1;
timespec t_2;
int64 t_avg, t_max, dt;

boolean flag_debug = true;
pthread_t tid_fileIO;
circular_buffer<uint64_t> c_buff(256);

ros::Subscriber subMotor;
ros::Publisher pubMotor;
pthread_t tid_pub;


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
    /* no signal wakes implemented --> remaining in nullptr...*/
    *ret=0;
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &pinfo->next_period, nullptr);
}

void cleanup_stat(void* pMotorInfo)
{
    // nothing that needs to be cleaned up here.
    printf("cleanup stat called.\n");
}

void cleanup_fileIO(void * data){
    while(!c_buff.empty()){
        fprintf(pFile, "%lld \t 90\n", c_buff.get());
    }
    printf("cleanup fileIO \n");
}

void cleanup_pub(void* data){
    printf("cleanup Publisher.\n");
    pubMotor.shutdown();
}

void cleanup_ctl(void* pMotorInfo)
{
    inOP = false;
    MotorInfo* EPOS4 = (MotorInfo*) pMotorInfo;


    switch(OPMode){ // mapping of PDO-Data for control is depending of OP-Mode:
        case OP_profile_pos:
            // nothing to do here
            break;
        case OP_profile_vel:
            // stop motor when control thread is stopped
            EPOS4->pPDO_out_vel->controlword = 0x000F;
            EPOS4->pPDO_out_vel->tar_vel = 0;
            break;
        case OP_cyclic_torque:
            EPOS4->pPDO_out_torque->controlword = 0x000F;
            EPOS4->pPDO_out_torque->tar_torque = 0; // EPOS4->tar_torque;
            break;
    }
    ec_send_processdata();
    wkc = ec_receive_processdata(200); // timeout value: small to avoid cycle violations?

    printf("cleanup ctl called.\n");



    osal_usleep(50000); // wait a short period so the motor can stop to prevent too many quick stops!
    EPOS4->poweroff();
}

void stat_func(void *pMotorInfo)
{
    MotorInfo *EPOS4 = (MotorInfo*) pMotorInfo;
    int size = 2;
    uint16_t buff;
    int wkc_stat = 0;
    wkc_stat = ec_SDOread(EPOS4->slave_number, 0x3201, 0x01, FALSE, &size, &buff, EC_TIMEOUTTXM);
    EPOS4->Temp = buff;
    wkc_stat = ec_SDOread(EPOS4->slave_number, 0x2200, 0x01, FALSE, &size, &buff, EC_TIMEOUTTXM);
    EPOS4->Voltage = buff;
    wkc_stat = ec_SDOread(EPOS4->slave_number, 0x1C33, 0x0B, FALSE, &size, &buff, EC_TIMEOUTTXM);
    EPOS4->SM_events_missed = buff;
    uint32_t buff2;
    size = 4;
    wkc_stat = ec_SDOread(EPOS4->slave_number, 0x1C33, 0x02, FALSE, &size, &buff2, EC_TIMEOUTTXM);
    EPOS4->CycleTime = buff2;
}

// thread function for cyclic Motor check (voltage, Temperature, etc.)
void *pth_check_status(void *pMotorInfo)
{
    pthread_cleanup_push(cleanup_stat, nullptr);
    auto *EPOS4 = (MotorInfo*) pMotorInfo;
    struct period_info pinfo_stat;
    periodic_task_init(&pinfo_stat, 1000);
    ros::NodeHandle *nH = (ros::NodeHandle*) EPOS4->nodehandle;
    ros::Publisher pubStat =  nH->advertise<leobot_base_msg::motor_monitoring>("/MotorMonitor", 10);
    leobot_base_msg::motor_monitoring msg;
    msg.Temperature.push_back(0);
    msg.Voltage.push_back(0);
    clock_gettime(CLOCK_MONOTONIC, &(t_start_stat));
    // printf("time init stat \n");
    int ret = 0;
    while (EPOS4->isrunning == 1) {

        stat_func(pMotorInfo);
        clock_gettime(CLOCK_MONOTONIC, &(t_act_stat));


         printf("Status t %ld, T %d, V %d \n", t_act_stat.tv_sec - t_start_stat.tv_sec, EPOS4->Temp, EPOS4->Voltage);
         printf("Cycle time %lu, SM events missed %u\n", EPOS4->CycleTime, EPOS4->SM_events_missed);
         fprintf(pFile, "%ld\t 80 \n", (t_act_stat.tv_sec - t_start_ctl.tv_sec)*1000000000 + (t_act_stat.tv_nsec - t_start_ctl.tv_nsec));
        /* now in ctl function!
        * if((1 << 3) & EPOS4->pPDO_in->statusword) {
          // printf("fault!\n");
          uint16_t errorcode;
          int size = 2;
          ec_SDOread(EPOS4->slave_number, 0x603F, 0x00, FALSE, &size, &errorcode, EC_TIMEOUTTXM);
          EPOS4->print_errors(errorcode);
          fprintf(pFile, "%ld\t 99 \n", (t_act_stat.tv_sec - t_start_ctl.tv_sec)*1000000000 + (t_act_stat.tv_nsec - t_start_ctl.tv_nsec));
       } */
        msg.Voltage[0] = EPOS4->Voltage/10;
        msg.Temperature[0] = EPOS4->Temp/10;
       pubStat.publish(msg);
        wait_rest_of_period(&pinfo_stat, &ret);
    }
    pthread_cleanup_pop(1);
}


uint64 i_callback = 0;
// ROS callback function for receiving data;
void ros_callback_vel(const geometry_msgs::Twist::ConstPtr& msg, MotorInfo *pEPOS4, RosParam *pleobot_param){
/* in the top view: x-axis goes to the front, y to the left and rotation around z according to the right-hand-rule
  * wi is the speed of the i-th wheel; they are numberd the following way:
  *
  *          x^
  *     //1   |   2\\
  *           |
  *      y <--|
  *
  *     \\3     4//
  *
*/
float w1, w2, w3, w4;
    w1 = (1/pleobot_param->r_wheel)*msg->linear.x - (1/pleobot_param->r_wheel) * msg->linear.y  - pleobot_param->factor_rotation*msg->angular.z;
    w2 = (1/pleobot_param->r_wheel)*msg->linear.x + (1/pleobot_param->r_wheel) * msg->linear.y  + pleobot_param->factor_rotation*msg->angular.z;
    w3 = (1/pleobot_param->r_wheel)*msg->linear.x - (1/pleobot_param->r_wheel) * msg->linear.y  + pleobot_param->factor_rotation*msg->angular.z;
    w4 = (1/pleobot_param->r_wheel)*msg->linear.x + (1/pleobot_param->r_wheel) * msg->linear.y  - pleobot_param->factor_rotation*msg->angular.z;
    pEPOS4->set_tar_vel(w1);
    pEPOS4->tar_vel = (int) (60* w1*pleobot_param->gear_ratio_belt * pEPOS4->gearbox_ratio/(2*M_PI)); // calculate vel in rpm/ if (v_i > 10000) {
    i_callback++;
    if(flag_debug && (i_callback % 1000 == 0)) {
        ROS_INFO("x %f y %f phi %f", msg->linear.x, msg->linear.y, msg->angular.z);
        ROS_INFO("w1 %f w2 %f w3 %f w4 %f", w1, w2, w3, w4);
        ROS_INFO("PDO tar: %d, act %d", pEPOS4->tar_vel, pEPOS4->pPDO_in->vel_act);
    }
    // printf("warning: max speed of the motor was exceeded!");
}

void ros_callback_pos(const leobot_base_msg::motor_wheels::ConstPtr& msg, MotorInfo *pEPOS4, RosParam *pleobot_param){
    pEPOS4->set_tar_pos(msg->wheel_1); // calculate vel in rpm/ if (v_i > 10000) {
    i_callback++;
    if(flag_debug && (i_callback % 1000 == 0)) {
        ROS_INFO("p1 %f p2 %f p3 %f p4 %f", msg->wheel_1, msg->wheel_2, msg->wheel_3, msg->wheel_4);
        ROS_INFO("PDO tar: %d, act %d", pEPOS4->tar_pos, pEPOS4->pPDO_in->pos_act);
    }
}

void ros_callback_torque(const leobot_base_msg::motor_wheels::ConstPtr& msg, MotorInfo *pEPOS4, RosParam *pleobot_param){
    pEPOS4->set_tar_torque(msg->wheel_1);
    i_callback++;
    if(flag_debug && (i_callback % 1000 == 0)) {
        ROS_INFO("t1 %f t2 %f t3 %f t4 %f", msg->wheel_1, msg->wheel_2, msg->wheel_3, msg->wheel_4);
        ROS_INFO("PDO tar: %d, act %d", pEPOS4->tar_torque, pEPOS4->pPDO_in->torq_act);
    }
}
/* PI calculation to get linux time synced to DC time */
void ec_sync(int64 reftime, int64 cycletime , int64 *offsettime)
{
    static int64 integral = 0;
    int64 delta;
    /* set linux sync point 50us later than DC sync, just as example */
    delta = (reftime - 50000) % cycletime;
    if(delta> (cycletime / 2)) { delta= delta - cycletime; }
    if(delta>0){ integral++; }
    if(delta<0){ integral--; }
    *offsettime = -(delta / 100) - (integral / 20);
    gl_delta = delta;
}

void handler_int(int num){


    pthread_cancel(tid_ctl);
    pthread_cancel(tid_status);
    pthread_cancel(tid_fileIO);
    pthread_cancel(tid_pub);

    subMotor.shutdown();
    ros::shutdown;
    fclose(pFile);
    ec_close();
    write(STDOUT_FILENO, "\n Shutdown.\n\n", 35);

    printf("cycle violations: %d\n", err_cnt);
    printf("average cycle time of ctl function: %lld us, max %lld us\n",  t_avg/1000, t_max/1000);
    munlockall();
    exit(1);
}


uint64_t t_stp;
// control function for the Motor
void ctl_func(void *pMotorInfo, int i)
{
    auto *EPOS4 = (MotorInfo*) pMotorInfo;
    // clock_gettime(CLOCK_MONOTONIC, &(t_act_ctl));
    // EPOS4->pPDO_out->tar_vel = (int) 1000*sin((float) M_PI * i / 1000);
    switch(OPMode){ // mapping of PDO-Data for control is depending of OP-Mode:
        case OP_profile_pos:
            // Start new movement to tar_pos if either tar_pos is not the same as specified or if the target is
            // // reached but an error greater than 100 increments is present.
            if(EPOS4->pPDO_out_pos->tar_pos != EPOS4->tar_pos ||
                 ((1 << 10) & EPOS4->pPDO_in->statusword) && (abs(EPOS4->pPDO_in->pos_act - EPOS4->pPDO_out_pos->tar_pos)) > 100)
            {
                EPOS4->pPDO_out_pos->controlword = 0x003F;
            }else{
                EPOS4->pPDO_out_pos->controlword = 0x000F;
            }
            EPOS4->pPDO_out_pos->profile_vel = 1000;
            EPOS4->pPDO_out_pos->tar_pos = EPOS4->tar_pos;

            break;
        case OP_profile_vel:
            EPOS4->pPDO_out_vel->controlword = 0x000F;
            EPOS4->pPDO_out_vel->tar_vel = EPOS4->tar_vel;
            break;
        case OP_cyclic_torque:
            EPOS4->pPDO_out_torque->controlword = 0x000F;
            EPOS4->pPDO_out_torque->tar_torque = EPOS4->tar_torque; // EPOS4->tar_torque;
            break;
        default:
            printf("Mapping for OP-Mode not possible. \n");
            exit(-1);
    }
    // ToDo: map acceleration to PDO and use calculated acceleration for every wheel
    ec_send_processdata();
    wkc = ec_receive_processdata(200); // timeout value: small to avoid cycle violations?

    clock_gettime(CLOCK_MONOTONIC, &t_2);
    t_stp = ((uint64)(t_1.tv_sec - t_start_ctl.tv_sec))*1000000000 + (t_1.tv_nsec - t_start_ctl.tv_nsec);
    c_buff.put(t_stp);
    dt = ((uint64)(t_2.tv_sec - t_1.tv_sec))*1000000000 + (t_2.tv_nsec - t_1.tv_nsec);
    t_avg = ((t_avg*i) + dt)/(i+1); // be aware of overflow and rounding error!
    if (dt > t_max){
        t_max = dt;
    }
    /*
    if (wkc >= expectedWKC) {
        if ((1 << 3) & EPOS4->pPDO_in->statusword){
            err_cnt ++;
            if ((err_cnt < 10) || (err_cnt % 1000 == 0) )
            {
                printf("err 0x%04.4x\n", EPOS4->pPDO_in->errorcode);
                // diagnosis message: subindes 0x06...0x0A
                uint64_t buff;
                int size = 8;
               //ec_SDOread(1, 0x10F3, 0x06, FALSE, &size, &buff, EC_TIMEOUTRXM);
               // printf("diag: %lu\n", buff);
                // EPOS4->pPDO_out->controlword = 0x80;
            }
        }
    }*/
    // EPOS4->add_timestamp(t_act_ctl, t_start_ctl,90);
    // fprintf(pFile, "%ld\t 90\n", (t_act_ctl.tv_sec - t_start_ctl.tv_sec)*1000000000 + (t_act_ctl.tv_nsec - t_start_ctl.tv_nsec));
}


// thread function for cyclic Motor control
void *pth_ctl_func (void *pMotorInfo)
{
    pthread_cleanup_push(cleanup_ctl, pMotorInfo);

    MotorInfo *EPOS4 = (MotorInfo*) pMotorInfo;
    //  pthread_mutex_init(&lock, nullptr);
    struct period_info pinfo_ctl;
    EPOS4->isrunning = 1;
    uint64 i = 0, t_stp;
    int ret=0;
    float dt_ctl = 1;
    clock_gettime(CLOCK_MONOTONIC, &(t_start_ctl));
    periodic_task_init(&pinfo_ctl, dt_ctl);
    t_avg = 0;
    t_max = 0;
    float vx_w, vy_w, omega;
    float vx, vy, x, y, theta;
    while (1){
        clock_gettime(CLOCK_MONOTONIC, &t_1);
        switch(ret){
            case 0:
                ctl_func(pMotorInfo, i);
                i++; break;
            case 10: // skip if cycle violation
                err_cnt ++;
                printf("cv!\t");
                break;
        }
        // calculate odeometry from the actual wheel speed of all wheels; the frame on the robot shall be called "base_link"
        /*
        vx_w =  vx*cos(theta) + vy*sin(theta);
        vy_w = -vx*sin(theta) + vy*cos(theta);
        x = x + vx_w*dt_ctl/1000;
        y = y + vy_w*dt_ctl/1000;
        theta = theta + omega*dt_ctl/1000;
	// use tf2 to publish; tutorial at 
	// --> http://wiki.ros.org/tf2/Tutorials/Writing%20a%20tf2%20broadcaster%20%28C%2B%2B%29
    // http://wiki.ros.org/navigation/Tutorials/RobotSetup/Odom*/



        // if (ec_slave[0].hasdc)
        //{
            /* ToDo: as in SOEM issue 42, see if clock is working! */
            /* calulate toff to get linux time and DC synced */
            // ec_sync(ec_DCtime, 2000000, &pinfo_ctl.t_off);
            // fprintf(pFile, "%ld\t 50 \n", pinfo_ctl.t_off);
            // fprintf(pFile, "%ld \t 40 \n", ec_DCtime);
            // fprintf(pFile, "%d \t 30  \n", EPOS4->pPDO_in->pos_act);
            // fprintf(pFile, "%lld \t 40\n", gl_delta);
        // }
        /* calculate next cycle start */
        //add_timespec(&pinfo_ctl, cycletime + toff);
        /* wait to cycle start */

        wait_rest_of_period(&pinfo_ctl, &ret);

    }
    EPOS4->isrunning = 0;
    pthread_cleanup_pop(1);
}

// thread function to write from cyclic buffer into a file
void *pth_fileIO(void *data)
{
    pthread_cleanup_push(cleanup_fileIO, NULL);
    struct period_info pinfo_fileIO;
    periodic_task_init(&pinfo_fileIO, 40);
    int ret = 0;
    while (1) {
        while(!c_buff.empty()) {
            fprintf(pFile, "%llu \t 90\n", c_buff.get());
        }
        wait_rest_of_period(&pinfo_fileIO, &ret);
    }
    pthread_cleanup_pop(1);
}

void *pth_pub(void *pMotorInfo){

    pthread_cleanup_push(cleanup_pub, pMotorInfo);
    MotorInfo *EPOS4 = (MotorInfo*) pMotorInfo;
    ros::NodeHandle *nH = (ros::NodeHandle*) EPOS4->nodehandle;
    pubMotor =  nH->advertise<leobot_base_msg::motor_status>("/MotorStatus", 10);
    struct period_info pinfo_pub;
    periodic_task_init(&pinfo_pub, 1);
    int ret;
    leobot_base_msg::motor_status msg;

   while(EPOS4->isrunning) {
       msg.act_pos = EPOS4->get_position();
       msg.act_vel = EPOS4->get_velocity();
       msg.statusword = (double) EPOS4->pPDO_in->statusword;
       msg.act_torque = EPOS4->get_torque();
       msg.errorcode = (double) EPOS4->pPDO_in->errorcode;
       pubMotor.publish(msg);
       wait_rest_of_period(&pinfo_pub, &ret);
   }
    pthread_cleanup_pop(1);
}


// main function, loops with subscriber!
    void EPOS_Main(char *ifname)
{
    MotorInfo EPOS4;

    int i, j, oloop, iloop, chk;
    needlf = FALSE;
    inOP = FALSE;
    signal(SIGINT, handler_int);
    signal(SIGTERM, handler_int);

    // printf("starting EPOS4: \n");

    /* initialise SOEM, bind socket to ifname */
    if (ec_init(ifname))
    {
        printf("ec_init on %s succeeded.\n",ifname);
        /* find and auto-config slaves */



        if ( ec_config_init(FALSE) > 0 )
        {
            printf("%d slaves found and configured.\n",ec_slavecount);

            { // scope do encapsulate buffer variable
                // input PDOs (from masters perspective)
                printf("setting PDO mapping for requested mode. \n");
                uint8_t buff = 0;
                ec_SDOwrite(1, 0x1A00, 0x00, FALSE, 1, &buff, EC_TIMEOUTRXM);
                uint32_t buff2 = 0x60410010;    // Statusword
                ec_SDOwrite(1, 0x1A00, 0x01, FALSE, 4, &buff2, EC_TIMEOUTRXM);
                buff2 = 0x60640020;     // Position actual value
                ec_SDOwrite(1, 0x1A00, 0x02, FALSE, 4, &buff2, EC_TIMEOUTRXM);
                buff2 = 0x606C0020;     // velocity actual value
                ec_SDOwrite(1, 0x1A00, 0x03, FALSE, 4, &buff2, EC_TIMEOUTRXM);
                buff2 = 0x60770010;     // Torque actual value
                ec_SDOwrite(1, 0x1A00, 0x04, FALSE, 4, &buff2, EC_TIMEOUTRXM);
                buff2 = 0x603F0010;     // Errorcode
                ec_SDOwrite(1, 0x1A00, 0x05, FALSE, 4, &buff2, EC_TIMEOUTRXM);
                buff = 5;
                ec_SDOwrite(1, 0x1A00, 0x00, FALSE, 1, &buff, EC_TIMEOUTRXM);
                // output PDOs (from masters Perspective)
                buff = 0;
                ec_SDOwrite(1, 0x1600, 0x00, FALSE, 1, &buff, EC_TIMEOUTRXM);
                int num_extra = 0;
                buff2 = 0x60400010; // controlword
                ec_SDOwrite(1, 0x1600, 0x01, FALSE, 4, &buff2, EC_TIMEOUTRXM);
                switch(OPMode){ // mapping of PDO-Data for control is depending of OP-Mode:
                    case OP_profile_pos:
                        {
                            int size = 4;
                            ec_SDOread(1, 0x6064, 0x00, FALSE, &size, &(EPOS4.tar_pos), EC_TIMEOUTRXM); // read actual position and set target position to initial position!
                        }
                        buff2 = 0x60810020; // profile velocity
                        ec_SDOwrite(1, 0x1600, 0x03, FALSE,  4, &buff2, EC_TIMEOUTRXM);
                        num_extra++;
                        buff2 = 0x60830020; // profile acceleration
                        ec_SDOwrite(1, 0x1600, 0x04, FALSE,  4, &buff2, EC_TIMEOUTRXM);
                        num_extra++;
                        buff2 = 0x60840020; // profile decceleration
                        ec_SDOwrite(1, 0x1600, 0x05, FALSE,  4, &buff2, EC_TIMEOUTRXM);
                        num_extra++;
                        buff2 = 0x607A0020; // targent position (in increments), subindex 0, 0x20 = 32 bit integer
                        break;
                    case OP_profile_vel:
                        buff2 = 0x60830020;
                        ec_SDOwrite(1, 0x1600, 0x03, FALSE,  4, &buff2, EC_TIMEOUTRXM);
                        num_extra++;
                        buff2 = 0x60840020;
                        ec_SDOwrite(1, 0x1600, 0x04, FALSE,  4, &buff2, EC_TIMEOUTRXM);
                        num_extra++;
                        buff2 = 0x60FF0020; // target velocity (rpm)
                        break;
                    case OP_cyclic_torque:
                        buff2 = 0x60710010; // target torque
                        break;
                    default:
                        printf("Mapping for OP-Mode not possible. \n");
                        exit(-1);
                }
                ec_SDOwrite(1, 0x1600, 0x02, FALSE, 4, &buff2, EC_TIMEOUTRXM);
                buff = 2 + num_extra;
                ec_SDOwrite(1, 0x1600, 0x00, FALSE, 1, &buff, EC_TIMEOUTRXM);
            }
            printf("PDOs mapped\n");
            ec_config_map(&IOmap);
            ec_configdc();

            printf("Slaves mapped, state to SAFE_OP.\n");

            /* wait for all slaves to reach SAFE_OP state */
            ec_statecheck(0, EC_STATE_SAFE_OP,  EC_TIMEOUTSTATE * 4);
            EPOS4.isonline = true;
            osal_usleep(1000);


            oloop = ec_slave[0].Obytes;
            if ((oloop == 0) && (ec_slave[0].Obits > 0)) oloop = 1;
            // if (oloop > 8) oloop = 8;
            ec_slave[0].Ibytes = 12; // to match PDO
            iloop = ec_slave[0].Ibytes;
            if ((iloop == 0) && (ec_slave[0].Ibits > 0)) iloop = 1;
            EPOS4.slave_number = 1;
            EPOS4.startup_OD();

            // printf("segments : %d : %d %d %d %d\n",ec_group[0].nsegments ,ec_group[0].IOsegment[0],ec_group[0].IOsegment[1],ec_group[0].IOsegment[2],ec_group[0].IOsegment[3]);
            printf("Request operational state for all slaves\n");
            expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
            printf("Calculated workcounter %d\n", expectedWKC);
            ec_slave[0].state = EC_STATE_OPERATIONAL;
            /* send one valid process data to make outputs in slaves happy*/
            ec_send_processdata();
            ec_receive_processdata(EC_TIMEOUTRET);
            /* request OP state for all slaves */
            ec_writestate(0);
            chk = 200;

            // create nodehandle for ros node
            ros::NodeHandle n;
            EPOS4.nodehandle = &n;
            RosParam leobot_param;
            // no buffer --> only the most recent message is used
            switch(OPMode){
                case OP_profile_pos:
                    subMotor = n.subscribe<leobot_base_msg::motor_wheels>("/leobot_base/cmd_pos", 1, boost::bind(ros_callback_pos, _1, &EPOS4, &leobot_param));
                    break;
                case OP_profile_vel:
                    subMotor = n.subscribe<geometry_msgs::Twist>("/leobot_base/cmd_vel", 1, boost::bind(ros_callback_vel, _1, &EPOS4, &leobot_param));
                    break;
                case OP_cyclic_torque:
                    subMotor = n.subscribe<leobot_base_msg::motor_wheels>("/leobot_base/cmd_torque", 1, boost::bind(ros_callback_torque, _1, &EPOS4, &leobot_param));
                    break;
                default:
                    printf("something went wront! OP Mode not recognized!\n");
                    break;
            }

            ROS_INFO("Motor Node Online\n");
            int num;
            n.getParam("/leobot_base/flag_debug", num);
            if(num!=0){
                flag_debug = true;
                ROS_INFO("debugging is active.");
            }

            /* wait for all slaves to reach OP state */
            do
            {
                ec_send_processdata();
                ec_receive_processdata(EC_TIMEOUTRET);
                ec_statecheck(0, EC_STATE_OPERATIONAL, 50000);
            }
            while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));

            if (ec_slave[0].state == EC_STATE_OPERATIONAL )
            {
                printf("Operational state reached for all slaves.\n");
                inOP = TRUE;

                // assigning PDO data to struct
                EPOS4.pPDO_in = (struct PDO_in *)(ec_slave[1].inputs);
                switch(OPMode){ // mapping of PDO-Data for control is depending of OP-Mode:
                    case OP_profile_pos:
                        EPOS4.pPDO_out_pos = (struct PDO_out_pos*)(ec_slave[1].outputs);
                        EPOS4.pPDO_out_pos->controlword = 0x07; // Switch on
                        // write the at startup read actual position into the target position PDO, otherwise robot
                        // moves at startup into zero-position if an other Operationmode was already used before without
                        // resetting the motor ;
                        EPOS4.pPDO_out_pos->tar_pos = EPOS4.tar_pos;

                        break;
                    case OP_profile_vel:
                        EPOS4.pPDO_out_vel = (struct PDO_out_vel *)(ec_slave[1].outputs);
                        EPOS4.pPDO_out_vel->controlword = 0x07;
                        EPOS4.pPDO_out_vel->profile_acc = 10000;
                        EPOS4.pPDO_out_vel->profile_dec = 10000;
                        break;
                    case OP_cyclic_torque:
                        EPOS4.pPDO_out_torque = (struct PDO_out_torque *)(ec_slave[1].outputs);
                        EPOS4.pPDO_out_torque->controlword = 0x07;
                        break;
                    default:
                        printf("something went wrong!");
                        break;
                }
                ec_send_processdata();
                wkc = ec_receive_processdata(EC_TIMEOUTRET);
                EPOS4.print_status();
                if(1 << 6 & EPOS4.pPDO_in->statusword){
                    // switch on disabled

                }
                // read the parameter from the ROS parameter server
                n.getParam("/leobot_base/wheelradius", leobot_param.r_wheel);
                n.getParam("/leobot_base/max_vel_x", leobot_param.max_vel_x);
                n.getParam("/leobot_base/max_vel_y", leobot_param.max_vel_y);
                n.getParam("/leobot_base/max_vel_phi", leobot_param.max_vel_phi);
                n.getParam("/leobot_base/ly", leobot_param.ly);
                n.getParam("/leobot_base/lx", leobot_param.lx);
                n.getParam("/leobot_base/gearbox_ratio_belt", leobot_param.gear_ratio_belt);
                leobot_param.alpha = atan(leobot_param.lx/leobot_param.ly);
                leobot_param.factor_rotation = (leobot_param.ly + leobot_param.lx)/leobot_param.r_wheel;
                EPOS4.gear_ratio_belt = leobot_param.gear_ratio_belt;


                // setting Operation mode
                uint16 ctl_word;
                int size;
                ctl_word = 0x06;
                wkc = ec_SDOwrite(1, 0x6040, 0x00, FALSE, 2 , &ctl_word, EC_TIMEOUTRET*5);
                ctl_word = 0x0F;
                wkc = ec_SDOwrite(1, 0x6040, 0x00, FALSE, 2 , &ctl_word, EC_TIMEOUTRET*5);
                wkc = ec_SDOread(1, 0x6040, 0x00, FALSE, &size, & ctl_word, EC_TIMEOUTRET*5);

                uint8_t buff = OPMode; // Profile Velocity Mode
                size = 1;
                wkc = ec_SDOwrite(1, 0x6060, 0x00, FALSE, 1,  &buff, EC_TIMEOUTRXM);

                size = 4;
                { // check software position limits; if both are 0 --> disabled
                    uint32 buff;
                    wkc = ec_SDOread(1, 0x607D, 0x01, FALSE, &size, &buff, EC_TIMEOUTRXM);
                    printf("software limit position (upper): %d\n", buff);
                    wkc = ec_SDOread(1, 0x607D, 0x02, FALSE, &size, &buff, EC_TIMEOUTRXM);
                    printf("software limit position (lower): %d\n", buff);
                    wkc = ec_SDOread(1, 0x6081, 0x00, FALSE, &size, &buff, EC_TIMEOUTRXM);
                    printf("Profile velocity: %d\n", buff);

                }

                pFile = fopen("/tmp/epos_timestamps.txt", "w+");

                // prepare RT-threads
                int ret_ctl, ret_stat;
                struct sched_param param_ctl;
                struct sched_param param_stat;
                pthread_attr_t attr_ctl;
                pthread_attr_t attr_stat;

                /* Lock memory to prevent latency because of memory swapping to hard drive*/
                if(mlockall(MCL_CURRENT|MCL_FUTURE) == -1) {
                    printf("mlockall failed: %m\n");
                    exit(-2);
                }

                /* Initialize pthread attributes (default values) */
                ret_ctl = pthread_attr_init(&attr_ctl);
                ret_stat = pthread_attr_init(&attr_stat);
                if (ret_ctl || ret_stat) {
                    printf("init pthread attributes failed\n");
                    return ;
                }

                /* Set a specific stack size  */
                 ret_ctl = pthread_attr_setstacksize(&attr_ctl, PTHREAD_STACK_MIN);
                ret_stat = pthread_attr_setstacksize(&attr_stat, PTHREAD_STACK_MIN);
                if (ret_ctl || ret_stat) {
                    printf("pthread setstacksize failed\n");
                    exit(-2);
                }

                /* Set scheduler policy and priority of pthread */
                ret_ctl = pthread_attr_setschedpolicy(&attr_ctl, SCHED_FIFO);
                ret_stat = pthread_attr_setschedpolicy(&attr_stat, SCHED_FIFO);
                if (ret_ctl || ret_stat) {
                    printf("pthread setschedpolicy failed\n");
                    exit(-2);
                }

                param_ctl.sched_priority = 90;
                param_stat.sched_priority = 80;
                ret_ctl = pthread_attr_setschedparam(&attr_ctl, &param_ctl);
                ret_stat = pthread_attr_setschedparam(&attr_stat, &param_stat);
                if (ret_ctl || ret_stat) {
                    printf("pthread setschedparam failed\n");
                    exit(-2);
                }
                /* Use scheduling parameters of attr */
                ret_ctl = pthread_attr_setinheritsched(&attr_ctl, PTHREAD_EXPLICIT_SCHED);
                ret_stat = pthread_attr_setinheritsched(&attr_stat, PTHREAD_EXPLICIT_SCHED);
                if (ret_ctl || ret_stat) {
                    printf("pthread setinheritsched failed\n");
                    exit(-2);
                }

                // set affinity of processes to a seperate CPU; not implemented yet
                // int sched_setaffinity(pid_t pid, size_t cpusetsize, const cpu_set_t *mask);

                /* Create a pthread with specified attributes */
                ret_ctl = pthread_create(&tid_ctl, &attr_ctl, &pth_ctl_func, &EPOS4);
                ret_stat = pthread_create(&tid_status, &attr_stat, &pth_check_status, &EPOS4);
                pthread_create(&tid_pub, NULL, &pth_pub, &EPOS4);
                int ret_fileIO = 0;
                if (flag_debug){
                    ret_fileIO = pthread_create(&tid_fileIO, NULL, &pth_fileIO, NULL);
                }
                if (ret_ctl || ret_stat || ret_fileIO) {
                    printf("create pthread failed\n");
                    exit(-2);
                }

                struct period_info pinfo_main;
                periodic_task_init(&pinfo_main, 1);
                int ret;
                while(EPOS4.isrunning){
                    ros::spinOnce();
                    wait_rest_of_period(&pinfo_main, &ret);
                }

                //pthread_join(tid_status, nullptr);
                //pthread_join(tid_ctl, nullptr);
                fclose(pFile);

                // printf("\n Motor shut down, %d cycle violations, avg %f\n", Cyc_Violations, clk_avg*1000/CLOCKS_PER_SEC);
                //ec_send_processdata();
                //wkc = ec_receive_processdata(EC_TIMEOUTRET);
                EPOS4.poweroff();
            }
            else
            {
                printf("Not all slaves reached operational state.\n");
                ec_readstate();
                for(i = 1; i<=ec_slavecount ; i++)
                {
                    if(ec_slave[i].state != EC_STATE_OPERATIONAL)
                    {
                        printf("Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s\n",
                               i, ec_slave[i].state, ec_slave[i].ALstatuscode, ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
                    }
                }
            }
            printf("\nRequest init state for all slaves\n");
            ec_slave[0].state = EC_STATE_INIT;
            /* request INIT state for all slaves */
            ec_writestate(0);
        }
        else
        {
            printf("No slaves found!\n");
            printf("Check if all slaves are connected and the interface is specified correctly.\n");
        }
        printf("End, close socket\n");
        /* stop SOEM, close socket */
        ec_close();
    }
    else
    {
        printf("No socket connection on %s\nExcecute as root\n",ifname);
    }
}

// ecatcheck is running in a seperat thread in the Operating System Abstraction Layer currently unused
OSAL_THREAD_FUNC ecatcheck( void *ptr )
{
    int slave;
    (void)ptr;                  /* Not used */

    while(true)
    {
        if( inOP && ((wkc < expectedWKC) || ec_group[currentgroup].docheckstate))
        {
            if (needlf)
            {
                needlf = FALSE;
                printf("\n");
            }
            /* one ore more slaves are not responding */
            ec_group[currentgroup].docheckstate = FALSE;
            ec_readstate();
            for (slave = 1; slave <= ec_slavecount; slave++)
            {
                if ((ec_slave[slave].group == currentgroup) && (ec_slave[slave].state != EC_STATE_OPERATIONAL))
                {
                    ec_group[currentgroup].docheckstate = TRUE;
                    if (ec_slave[slave].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR))
                    {
                        printf("ERROR : slave %d is in SAFE_OP + ERROR, attempting ack.\n", slave);
                        ec_slave[slave].state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
                        ec_writestate(slave);
                    }
                    else if(ec_slave[slave].state == EC_STATE_SAFE_OP)
                    {
                        printf("WARNING : slave %d is in SAFE_OP, change to OPERATIONAL.\n", slave);
                        ec_slave[slave].state = EC_STATE_OPERATIONAL;
                        ec_writestate(slave);
                    }
                    else if(ec_slave[slave].state > EC_STATE_NONE)
                    {
                        if (ec_reconfig_slave(slave, EC_TIMEOUTMON))
                        {
                            ec_slave[slave].islost = FALSE;
                            printf("MESSAGE : slave %d reconfigured\n",slave);
                        }
                    }
                    else if(!ec_slave[slave].islost)
                    {
                        /* re-check state */
                        ec_statecheck(slave, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
                        if (ec_slave[slave].state == EC_STATE_NONE)
                        {
                            ec_slave[slave].islost = TRUE;
                            printf("ERROR : slave %d lost\n",slave);
                        }
                    }
                }
                if (ec_slave[slave].islost)
                {
                    if(ec_slave[slave].state == EC_STATE_NONE)
                    {
                        if (ec_recover_slave(slave, EC_TIMEOUTMON))
                        {
                            ec_slave[slave].islost = FALSE;
                            printf("MESSAGE : slave %d recovered\n",slave);
                        }
                    }
                    else
                    {
                        ec_slave[slave].islost = FALSE;
                        printf("MESSAGE : slave %d found\n",slave);
                    }
                }
            }
            if(!ec_group[currentgroup].docheckstate)
                printf("OK : all slaves resumed OPERATIONAL.\n");
        }
        // nanosleep( );
        osal_usleep(10000);
    }
}

int main(int argc, char *argv[], char * envp[])
{
    printf("Communication with MotorController EPOS 4 \n"
           "\t based on example code of SOEM (Simple Open EtherCAT Master)\n \n");
 // debugging error: roslib.so not found //
/* register uid_t uid;
    uid = geteuid ();
    printf("uid: %d\n", uid);
    for (int i = 0; envp[i] != NULL; i++)
    {

        printf("\n%s", envp[i]);

    } */
    // init ros environment without sigint handler --> costum one is implemented to end threads properly
    ros::init(argc, argv, "Motor", ros::init_options::NoSigintHandler);
/*
    pthread_t thread_ID_main;
    thread_ID_main = pthread_self();
    printf("Thread ID main: %d \n", thread_ID_main);
    pid_t process_id;
    pid_t p_process_id;

    process_id = getpid();
    p_process_id = getppid();
    printf("The process id: %d\n",process_id);
    printf("The process id of parent function: %d\n \n",p_process_id); */
    if (pthread_mutex_init(&lock, nullptr)!=0){
        printf(" \n mutex init failed! \n");
        return EXIT_FAILURE;
    }

    if (argc > 2)
    {
        /* create thread to handle slave error handling in OP */
        // pthread_create( &thread1, nullptr, (void *) &ecatcheck, (void*) &ctime);
        // osal_thread_create(&thread1, 128000, (void *) &ecatcheck, (void*) &ctime);
        OPMode = atoi(argv[2]);
        printf("requested OPMode %d: ", OPMode);
        switch(OPMode){
            case OP_profile_pos:
                printf("Profile Position Mode (PPM)\n");
                break;
            case OP_profile_vel:
                printf("Profile Velocity Mode (PVM)\n");
                break;
            case OP_cyclic_torque:
                printf("Cyclic Torque Mode (CST)\n");
                break;
            default:
                printf("OPMode is unknown.\n");
                ros::shutdown();
                return 0;
        }
        EPOS_Main(argv[1]);
    }
    else
    {
        printf("Usage: Eth_EPOS4 ifname1 OPMode\nifname = enp1s0f1 for example.\nOPModes:\n1 \t(Profile Position Mode)\n3 \t(Profile Velocity Mode)\n10 \t(Cyclic torque)\n");
    }
    printf("End program\n");

    return (0);
}



