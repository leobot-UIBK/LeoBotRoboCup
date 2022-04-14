 /* +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  *
  * Base_movement LeoBot
  * Version 1.0
  *
  * Details: Implementation of the movement of the Leobot on hardware level including the
  *          CanOpen over EtherCAT communication using the Simple Open EtherCAT Master
  *          (SOEM). The structure is based on the SOEM examples, the utilized slaves
  *          are maxon motorcontrollers "EPOS4". This is embedded into the Robot Operating
  *          System ROS. The CanOpen over EtherCAT protocol is used. Two threads handle
  *          the communication (status and control), two handle the ROS interface
  *          (publisher and subscriber).
  *
  * Author:  Peter Manzl
  *          This code is based on the example simple_test.c (c)Arthur Ketels 2010 - 2011,
  *          included in the Simple Open EtherCAT Master (SOEM).
  *
  * Date created:       2020-04-21
  * Use under GPLv2 licence permitted without any express or implied warranty. Do NOT use
  * this application without detailed knowledge of the hardware and electronics.
  * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */

#include "EPOS4_class.h"
#include <limits.h>
#include <sched.h>
#include <signal.h>

#include "circular_buffer.h"

// ROS specific headers:
#include <ros/ros.h>
// #include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include "geometry_msgs/Twist.h"
#include "leobot_base_msg/base_status.h"
#include "leobot_base_msg/motor_monitoring.h"
#include "leobot_base_msg/motor_wheels.h"
#include "std_srvs/Trigger.h"
#include <ros/transport_hints.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>


 #define EC_TIMEOUTMON 500

// Operation modes defined by the EPOS4-Firmware
#define OP_profile_pos 1
#define OP_profile_vel 3
#define OP_cyclic_torque 10

// i don't care bits are disabled by the IDC-N mask and are here set to 0
#define IDC1 0x6F               // 0110 1111
#define SwitchOnDisabled 0x40   // x10x 0000
#define ReadyToSwitchOn 0x21    // x01x 0001
#define SwitchedOn 0x23         // x01x 0011
#define OPEnabled 0x27          // x01x 0111
#define QuickStop 0x07          // x00x 0111
#define FaultReaction 0x0F      // x00x 1111
#define Fault 0x08              // x00x 1000

#define NSEC_PER_SEC 1000000000

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
pthread_t tid_ctl;
float dt_ctl = 1;

int err_cnt = 0, cycle_vl = 0, to_cnt = 0;
int OPMode;

FILE *pFile;

timespec t_1;
timespec t_2;
int64 t_avg, t_max, dt;

bool flag_debug;
bool flag_filter;
pthread_t tid_fileIO;
circular_buffer<uint64_t> c_buff_t(256);
circular_buffer<uint16> c_buff_i(256);

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
        // cycle violation! --> the next action may be skipped to catch up
    }
    /* no signal wakes implemented --> remaining in nullptr...*/
    *ret=0;
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &pinfo->next_period, nullptr);
}

void cleanup_stat(void* pbase)
{
    // nothing that needs to be cleaned up here.
    printf("cleanup stat called.\n");
}

void cleanup_fileIO(void * data){
    while(!c_buff_t.empty() && !c_buff_i.empty()){
        fprintf(pFile, "%llu \t , %u\n", c_buff_t.get(), c_buff_i.get());
    }
    printf("cleanup fileIO \n");
}

void cleanup_pub(void* data){
    printf("cleanup Publisher.\n");
    pubMotor.shutdown();
}

void cleanup_ctl(void* pbase)
{
    inOP = false;
    base* leobot = (base*) pbase;

for(int i_slave=0; i_slave < 4; i_slave++){
        switch(OPMode){ // mapping of PDO-Data for control is depending of OP-Mode:
            case OP_profile_pos:
                // nothing to do here
                break;
            case OP_profile_vel:
                // stop motor when control thread is stopped
                leobot->M[i_slave].pPDO_out_vel->controlword = 0x000F;
                leobot->M[i_slave].pPDO_out_vel->tar_vel = 0;
                break;
            case OP_cyclic_torque:
                leobot->M[i_slave].pPDO_out_torque->controlword = 0x000F;
                leobot->M[i_slave].pPDO_out_torque->tar_torque = 0; // EPOS4->tar_torque;
                break;
        }
}

    ec_send_processdata(); // send target values to stop the motors.
    wkc = ec_receive_processdata(400);

    printf("cleanup ctl called.\n");

    /* for(int i_slave=0; i_slave<4; i_slave++){
        leobot->M[i_slave].poweroff();
    }
    osal_usleep(5000); // wait a short period so the motor can stop!
*/
}

void stat_func(void *pbase)
{
    base *leobot = (base*) pbase;
    int size, wkc_stat;
    uint16_t buff;
    size=2;
    for(int i=0; i<4; i++) {
        wkc_stat = ec_SDOread(leobot->M[i].slave_number, 0x3201, 0x01, FALSE, &size, &buff, EC_TIMEOUTTXM);
        // read temperature
        leobot->M[i].Temp = buff;
        // read voltage
        wkc_stat = ec_SDOread(leobot->M[i].slave_number, 0x2200, 0x01, FALSE, &size, &buff, EC_TIMEOUTTXM);
        leobot->M[i].Voltage = buff;
        // read synchronisation events missed
        wkc_stat = ec_SDOread(leobot->M[i].slave_number, 0x1C33, 0x0B, FALSE, &size, &buff, EC_TIMEOUTTXM);
        leobot->M[i].SM_events_missed = buff;
        uint32_t buff2;
        size = 4;
        wkc_stat = ec_SDOread(leobot->M[i].slave_number, 0x1C33, 0x02, FALSE, &size, &buff2, EC_TIMEOUTTXM);
        leobot->M[i].CycleTime = buff2;
    }
}

// thread function for cyclic Motor check (voltage, Temperature, etc.)
void *pth_check_status(void *pbase)
{
    pthread_cleanup_push(cleanup_stat, nullptr);
    auto *leobot = (base*) pbase;
    struct period_info pinfo_stat;
    periodic_task_init(&pinfo_stat, 1000);
    // get Nodehandle from base pointer and advertise Topic
    ros::NodeHandle *nH = (ros::NodeHandle*) leobot->nodehandle;
    ros::Publisher pubStat =  nH->advertise<leobot_base_msg::motor_monitoring>("/leobot_base/MotorMonitor", 10);
    leobot_base_msg::motor_monitoring msg;
    msg.Voltage.resize(4); // init 4 voltage and temperature values
    msg.Temperature.resize(4);
    clock_gettime(CLOCK_MONOTONIC, &(t_start_stat));
    int ret = 0;
    while (1) {
        stat_func(pbase);
        clock_gettime(CLOCK_MONOTONIC, &(t_act_stat));

        if (flag_debug) {
            printf("Status t %ld\t", t_act_stat.tv_sec - t_start_stat.tv_sec);
            for (int i=0; i<4; i++){
                printf("T%d %d, V%d %d \t", leobot->M[i].NodeID, leobot->M[i].Temp, leobot->M[i].NodeID,
                       leobot->M[i].Voltage);
            }
           printf("\n");
        }
        for (int i = 0; i < 4; i++) {
            msg.Voltage[i] = (float)leobot->M[i].Voltage / 10;
            msg.Temperature[i] = (float) leobot->M[i].Temp / 10;
        }
        msg.err_cnt = err_cnt;
        msg.cycl_vl = cycle_vl;
        msg.to_cnt = to_cnt;
        if (flag_debug){
            c_buff_t.put((t_act_stat.tv_sec - t_start_ctl.tv_sec)*1000000000 + (t_act_stat.tv_nsec - t_start_ctl.tv_nsec));
            c_buff_i.put(4);
            // fprintf(pFile, "%ld\t 80 \n", (t_act_stat.tv_sec - t_start_ctl.tv_sec)*1000000000 + (t_act_stat.tv_nsec - t_start_ctl.tv_nsec));
        }
        pubStat.publish(msg);
        wait_rest_of_period(&pinfo_stat, &ret);
    }
    pthread_cleanup_pop(1);
}


    uint64 i_callback = 0;
    // ROS callback function for receiving data;
    void ros_callback_vel(const geometry_msgs::Twist::ConstPtr& msg, base *pleobot, RosParam *pleobot_param){
    /* in the bottom view: x-axis goes to the front, y to the left and rotation around z according to the right-hand-rule
      * the directons of the rolls are switched when viewing from the top.
      * wi is the speed of the i-th wheel; they are numbered the following way:
      *
      *          x^
      *     //1   |   2\\
      *           |
      *      y <--|
      *
      *     \\3     4//
      *
    */
        pleobot->setVel(msg->linear.x, msg->linear.y, msg->angular.z);
        i_callback++;
        pleobot_param->i_timeout = 0;

        if(flag_debug && (i_callback % 100 == 0)) {
            ROS_INFO("x %f y %f phi %f", msg->linear.x, msg->linear.y, msg->angular.z);
            ROS_INFO("w1 %f rad/s, w2 %f rad/s, w3 %f rad/s, w4 %f rad/s", pleobot->w[0], pleobot->w[1],
                         pleobot->w[2], pleobot->w[3]);
            for(int i=0; i <4; i++){
                printf("PDO Node %d PDO tar_vel: %d, act %d\n", i, pleobot->M[i].pPDO_out_vel->tar_vel, pleobot->M[i].pPDO_in->vel_act);
            }
        }
        if(flag_debug){
            timespec t_act_sub;
            clock_gettime(CLOCK_MONOTONIC, &t_act_sub);
            // fprintf(pFile, "%ld\t 60 \n", (t_act_sub.tv_sec - t_start_ctl.tv_sec)*1000000000 + (t_act_sub.tv_nsec - t_start_ctl.tv_nsec));
            c_buff_t.put((t_act_sub.tv_sec - t_start_ctl.tv_sec)*1000000000 + (t_act_sub.tv_nsec - t_start_ctl.tv_nsec));
            c_buff_i.put(4);
         }
    }

    void ros_callback_pos(const leobot_base_msg::motor_wheels::ConstPtr& msg, base *pleobot, RosParam *pleobot_param){
        pleobot->M[0].set_tar_pos(msg->wheel_1*pleobot_param->dir_w[0]);
        pleobot->M[1].set_tar_pos(msg->wheel_2*pleobot_param->dir_w[1]);
        pleobot->M[2].set_tar_pos(msg->wheel_3*pleobot_param->dir_w[2]);
        pleobot->M[3].set_tar_pos(msg->wheel_4*pleobot_param->dir_w[3]);

        i_callback++;
        pleobot_param->i_timeout = 0;
        /* if(flag_debug) {
            ROS_INFO("reset callback timeout"); // delete info
        }*/
        if(flag_debug && (i_callback % 100 == 0)) {
            ROS_INFO("p1 %f p2 %f p3 %f p4 %f", msg->wheel_1, msg->wheel_2, msg->wheel_3, msg->wheel_4);
            for(int i=0; i<4; i++){
                ROS_INFO("Node %d: PDO tar: %d, act %d", i, pleobot->M[i].tar_pos, pleobot->M[i].pPDO_in->pos_act);
            }
        }

    }

    void ros_callback_torque(const leobot_base_msg::motor_wheels::ConstPtr& msg, base *pleobot, RosParam *pleobot_param){
        pleobot->M[0].set_tar_torque(msg->wheel_1*pleobot_param->dir_w[0]);
        pleobot->M[1].set_tar_torque(msg->wheel_2*pleobot_param->dir_w[1]);
        pleobot->M[2].set_tar_torque(msg->wheel_3*pleobot_param->dir_w[2]);
        pleobot->M[3].set_tar_torque(msg->wheel_4*pleobot_param->dir_w[3]);

        i_callback++;
        if(flag_debug && (i_callback % 100 == 0)) {
            ROS_INFO("t1 %f t2 %f t3 %f t4 %f", msg->wheel_1, msg->wheel_2, msg->wheel_3, msg->wheel_4);
            for(int i=0; i<4; i++){
                ROS_INFO("Node %d: PDO tar: %d, act %d", i, pleobot->M[i].tar_torque, pleobot->M[i].pPDO_in->torq_act);
            }
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

        osal_usleep(1500000); // wait a short time to continue to give the motors enough time to stop.
        subMotor.shutdown();
        ros::shutdown;
        if(flag_debug){
            fclose(pFile);
        }
        write(STDOUT_FILENO, "\n Shutdown.\n\n", 35);

        printf("cycle violations: %d; timeouts: %d\n", cycle_vl, to_cnt);
        printf("average cycle time of ctl function: %lld us, max %lld us\n",  t_avg/1000, t_max/1000);
        munlockall();
        ec_close();
        exit(1);
    }


    // control function for the Motor
    void ctl_func(void *pbase, int i) {
        auto *leobot = (base *) pbase;
        uint64_t t_stp;
        int wkc_pdo;
        bool flag_changed = false;

    /* in the state vector the state of each motor in the motor statemachine is saved.
     * see also the Firmware specification of the EPOS4 Motorcontroller.
     * discription          | value of PDO_in->Statusword
     * switch on disabled   |   x10x 0000
     * ready to switch on   |   x01x 0001
     * switched on          |   x01x 0011
     * OP enabled           |   x01x 0111
     * Quick Stop           |   x00x 0111
     * Fault reaction       |   x00x 1111
     * Fault                |   x00x 1000
     */

        bool flag_fault = 0;
        std::vector<bool> fault_motor(4);
        std::vector<uint16_t> ctl_word(4,0);
        for (int i_motor = 0; i_motor < 4; i_motor++) {
            // printf("Motor %d, ", i_motor);
            if ((leobot->M[i_motor].pPDO_in->statusword & IDC1) == SwitchOnDisabled) {
                ctl_word[i_motor] = 0x06;
            } else if ((leobot->M[i_motor].pPDO_in->statusword & IDC1) == ReadyToSwitchOn) {
                ctl_word[i_motor] = 0x0F;
            } else if ((leobot->M[i_motor].pPDO_in->statusword & IDC1) == SwitchedOn) {
                ctl_word[i_motor] = 0x0F;
            } else if ((leobot->M[i_motor].pPDO_in->statusword & IDC1) == OPEnabled) {
                std::vector<int32_t> dv(4);
                std::vector<uint32_t> a(4);
                for (int i_motor = 0; i_motor < 4; i_motor++) {
                    if (OPMode == OP_profile_vel &&
                        leobot->M[i_motor].pPDO_out_vel->tar_vel != leobot->M[i_motor].tar_vel) {
                        flag_changed = true;
                    }
                }
                if (flag_changed) {
                    for (int i_motor = 0; i_motor < 4; i_motor++) {
                        dv[i_motor] = abs(leobot->M[i_motor].pPDO_in->vel_act - leobot->M[i_motor].tar_vel);
                    }
                    auto max_it = std::max_element(std::begin(dv), std::end(dv)); // returns pointer to max element!
                    for (int i_motor = 0; i_motor < 4; i_motor++) {
                        a[i_motor] = abs(((int) dv[i_motor] * leobot->M[i_motor].calc_acc_maxon(leobot->param->max_wheelacc)) /
                                         ((int) *max_it));
                        if ((a[i_motor] == 0) || a[i_motor] > 10000) {
                            a[i_motor] = 10000;
                        }
                        leobot->M[i_motor].pPDO_out_vel->profile_acc = a[i_motor];
                        leobot->M[i_motor].pPDO_out_vel->profile_dec = a[i_motor];
                    }
                }
                for (int i_motor = 0; i_motor < 4; i_motor++) {
                    switch (OPMode) { // mapping of PDO-Data for control is depending of OP-Mode:
                        case OP_profile_pos:
                            // Start new movement to tar_pos if either tar_pos is not the same as specified or if the target is
                            // reached but an error greater than 100 increments is present.
                            if (leobot->M[i_motor].pPDO_out_pos->tar_pos != leobot->M[i_motor].tar_pos ||
                                ((1 << 10) & leobot->M[i_motor].pPDO_in->statusword) &&
                                (abs(leobot->M[i_motor].pPDO_in->pos_act - leobot->M[i_motor].pPDO_out_pos->tar_pos)) >
                                100) {
                                leobot->M[i_motor].pPDO_out_pos->controlword = 0x003F;
                            } else {
                                leobot->M[i_motor].pPDO_out_pos->controlword = 0x000F;
                            }
                            leobot->M[i_motor].pPDO_out_pos->profile_vel = 1000;
                            leobot->M[i_motor].pPDO_out_pos->tar_pos = leobot->M[i_motor].tar_pos;

                            break;
                        case OP_profile_vel:
                            leobot->M[i_motor].pPDO_out_vel->controlword = 0x000F;
                            leobot->M[i_motor].pPDO_out_vel->tar_vel = leobot->M[i_motor].tar_vel;
                            break;
                        case OP_cyclic_torque:
                            leobot->M[i_motor].pPDO_out_torque->controlword = 0x000F;
                            leobot->M[i_motor].pPDO_out_torque->tar_torque = leobot->M[i_motor].tar_torque; // EPOS4->tar_torque;
                            break;
                        default:
                            printf("Mapping for OP-Mode not possible. \n");
                            exit(-1);
                    }
                }
                ctl_word[i_motor] = 0x0F;
            } else if ((leobot->M[i_motor].pPDO_in->statusword & IDC1) == QuickStop) {
                ctl_word[i_motor] = 0x0F;
            } else if ((leobot->M[i_motor].pPDO_in->statusword & IDC1) == FaultReaction) {
                ctl_word[i_motor] = 0x80;
            } else if ((leobot->M[i_motor].pPDO_in->statusword & IDC1) == Fault) {
                ctl_word[i_motor] = 0x80;
            } else {
                ROS_INFO("State unknown!\n");
            }
        }

        // prevent moving of a wheel if not all wheels are operational!
        for (int i_motor = 0; i_motor < 4; i_motor++) {
            if (1 << 3 & leobot->M[i_motor].pPDO_in->statusword) { // fault bit is set!
                flag_fault = true;
                fault_motor[i_motor] = true;
                err_cnt++;
            }
        }

        // write the ctl_word (dependent from the statemachine) into the PDO-data and prevent movement if not all motors are operational!
        for (int i_motor = 0; i_motor < 4; i_motor++){
            switch (OPMode) {
                case OP_profile_pos:
                    leobot->M[i_motor].pPDO_out_pos->controlword = ctl_word[i_motor];
                    if(flag_fault == true){
                       leobot->M[i_motor].pPDO_out_pos->tar_pos = leobot->M[i_motor].pPDO_in->pos_act;
                    }
                    break;
                case OP_profile_vel:
                    leobot->M[i_motor].pPDO_out_vel->controlword = ctl_word[i_motor];
                    if(flag_fault == true){
                        leobot->M[i_motor].pPDO_out_vel->tar_vel = 0;
                    }
                    break;
                case OP_cyclic_torque:
                    leobot->M[i_motor].pPDO_out_torque->controlword = ctl_word[i_motor];
                    if(flag_fault == true){
                        leobot->M[i_motor].pPDO_out_torque->tar_torque = 0;
                    }
                    break;
            }
        }

        clock_gettime(CLOCK_MONOTONIC, &t_1);
        ec_send_processdata();
        wkc_pdo = ec_receive_processdata(400); // timeout value: small to avoid cycle violations, but big enough to avoid timeouts

        if (wkc_pdo < 0){
            to_cnt++;
           if (flag_debug == true){ 
                printf("to!");
           }
        }
        clock_gettime(CLOCK_MONOTONIC, &t_2);
        t_stp = ((uint64)(t_1.tv_sec - t_start_ctl.tv_sec))*1000000000 + (t_1.tv_nsec - t_start_ctl.tv_nsec);
        if (flag_debug) {
            c_buff_t.put(t_stp); //
            c_buff_i.put(1);
        }
        dt = ((uint64)(t_2.tv_sec - t_1.tv_sec))*1000000000 + (((int64) t_2.tv_nsec) - t_1.tv_nsec);
        t_avg = ((t_avg*i) + dt)/(i+1); // be aware of overflow and rounding error!
        if (dt > t_max){
            t_max = dt;
        }
    }

    /* add ns to timespec */
    void add_timespec(struct timespec *ts, int64 addtime)
    {
        int64 sec, nsec;

        nsec = addtime % NSEC_PER_SEC;
        sec = (addtime - nsec) / NSEC_PER_SEC;
        ts->tv_sec += sec;
        ts->tv_nsec += nsec;
        if ( ts->tv_nsec > NSEC_PER_SEC )
        {
            nsec = ts->tv_nsec % NSEC_PER_SEC;
            ts->tv_sec += (ts->tv_nsec - nsec) / NSEC_PER_SEC;
            ts->tv_nsec = nsec;
        }
    }

    // thread function for cyclic Motor control
    void *pth_ctl_func (void *pbase)
    {
        pthread_cleanup_push(cleanup_ctl, pbase);

            base *leobot = (base*) pbase;
            //  pthread_mutex_init(&lock, nullptr);
            struct period_info pinfo_ctl;
            leobot->isrunning = 1;
            uint64 i = 0, t_stp;
            int ret=0;
            clock_gettime(CLOCK_MONOTONIC, &(t_start_ctl));
            periodic_task_init(&pinfo_ctl, dt_ctl);
            t_avg = 0;
            t_max = 0;


            while (1){
                switch(ret){
                    case 0:
                        //clock_gettime(CLOCK_MONOTONIC, &t_1);
                        ctl_func(pbase, i);
                        i++; break;
                    case 10: // skip if cycle violation
                        cycle_vl ++;
                        if (flag_debug == true){
                            printf("cv!\t");
                        }
                        break;
                }
                leobot->param->i_timeout++;

                switch(OPMode){
                    case OP_profile_pos:
                        // nothing to do here
                        break;
                    case OP_profile_vel:
                        if(leobot->M[0].tar_vel != 0 || leobot->M[1].tar_vel != 0 ||  leobot->M[2].tar_vel != 0|| leobot->M[3].tar_vel !=0){
                            if(leobot->param->i_timeout > leobot->param->n_timeout){
                                ROS_INFO("Safety Stop! Timeout");
                                printf("Command took t = %f, max permitted is t= %f\n", leobot->param->i_timeout*1e-3, leobot->param->n_timeout*1e-3);
                                leobot->param->i_timeout = 0;
                                leobot->setVel(0,0,0);
                            }
                        }
                        break;
                    case OP_cyclic_torque:
                        if(leobot->M[0].tar_torque != 0 || leobot->M[1].tar_torque != 0 ||  leobot->M[2].tar_torque != 0|| leobot->M[3].tar_torque !=0){
                            if(leobot->param->i_timeout > leobot->param->n_timeout){
                                ROS_INFO("Safety Stop! Timeout");
                                printf("Command took t = %f, max permitted is t= %f\n", leobot->param->i_timeout*1e-3, leobot->param->n_timeout*1e-3);
                                leobot->param->i_timeout = 0;
                                for(int i_motor=0; i_motor<4; i_motor++){
                                    leobot->M[0].tar_torque = 0;
                                }
                            }
                        }
                        break;
                }

                wait_rest_of_period(&pinfo_ctl, &ret);
            }
            leobot->isrunning = 0;
        pthread_cleanup_pop(1);
    }

    // thread function to write from cyclic buffer into a file
    void *pth_fileIO(void *data)
    {
        pthread_cleanup_push(cleanup_fileIO, nullptr);
            struct period_info pinfo_fileIO;
            periodic_task_init(&pinfo_fileIO, 40);
            int ret = 0;
            while (1) {
                while(!c_buff_t.empty() && !c_buff_i.empty()) {
                    fprintf(pFile, "%llu \t %u\n", c_buff_t.get(), c_buff_i.get());
                }
                wait_rest_of_period(&pinfo_fileIO, &ret);
            }
        pthread_cleanup_pop(1);
    }

    void *pth_pub(void *pbase)
    {

        pthread_cleanup_push(cleanup_pub, pbase);
        base *leobot = (base*) pbase;
        ros::NodeHandle *nH = (ros::NodeHandle*) leobot->nodehandle;
        pubMotor =  nH->advertise<leobot_base_msg::base_status>("/leobot_base/MotorStatus", 10);
        ros::Publisher pubOdo = nH->advertise<nav_msgs::Odometry>("odom", 1);
        static tf2_ros::TransformBroadcaster tfBroadcaster; // broadcasts the transformations odom->base_link
        geometry_msgs::TransformStamped tfOdom;
        geometry_msgs::TransformStamped tfPanda;

        // initialize the transformations with zeros
        tfOdom.transform.translation.x = .0;
        tfOdom.transform.translation.y = .0;
        tfOdom.transform.translation.z = .0;

        tfOdom.transform.rotation.x = .0;
        tfOdom.transform.rotation.y = .0;
        tfOdom.transform.rotation.z = .0;
        tfOdom.transform.rotation.w = 1.0;
        tfPanda = tfOdom;


        tf2::Quaternion myQuat;
        tfOdom.header.frame_id = "odom";
        tfOdom.child_frame_id = "base_link";
        tfOdom.transform.translation.z = 0.0;
        struct period_info pinfo_pub;
        periodic_task_init(&pinfo_pub, 1);
        int ret = 0;
        leobot_base_msg::base_status msg;
        uint64 i_loop = 1;
        float vx_w, vy_w, omega, w1, w2, w3, w4;
        float vx, vy, x = 0, y = 0, theta = 0;
        leobot->odo_pose.pose.pose.position.z = 0.0;  // height of the base is constant as it can not fly.
        leobot->odo_pose.header.frame_id = "odom";
        leobot->odo_pose.child_frame_id = "base_link";
        tfOdom.transform.rotation.w = 0;
        float lpanda;
        nH->getParam("lpanda", lpanda);

        while(leobot->isrunning) {
           // calculate odeometry from the actual wheel speed of all wheels; the frame on the robot shall be called "base_link"/
           w1 = leobot->M[0].get_velocity() * leobot->param->dir_w[0];
           w2 = leobot->M[1].get_velocity() * leobot->param->dir_w[1];
           w3 = leobot->M[2].get_velocity() * leobot->param->dir_w[2];
           w4 = leobot->M[3].get_velocity() * leobot->param->dir_w[3];

            // forward kinematics for O-configuration of the wheels regarding the top view
           vx = (leobot->param->r_wheel / 4) * (w1 + w2 + w3 + w4);
           vy = (leobot->param->r_wheel / 4) * ((-1)*w1 + w2 + w3 - w4);
           omega = (leobot->param->r_wheel / 4) * (1 / (leobot->param->lx + leobot->param->ly)) * (
                   (-1) * w1 + w2 - w3 + w4);

           // transform from base frame into world frame/odom
           vx_w = vx * cos(theta) - vy * sin(theta);
           vy_w = vx * sin(theta) + vy * cos(theta);
           // theta = theta + omega * dt_ctl / 1000;
           x += vx_w * dt_ctl / 1000;
           y += vy_w * dt_ctl / 1000;
           theta += omega * dt_ctl / 1000;
           myQuat.setRPY(0,0,theta);

           leobot->odo_pose.pose.pose.position.x = x;
           leobot->odo_pose.pose.pose.position.y = y;
           theta += omega * dt_ctl / 1000;
           // write quaternions into message
           leobot->odo_pose.pose.pose.orientation.x = myQuat.x();
           leobot->odo_pose.pose.pose.orientation.y =myQuat.y();
           leobot->odo_pose.pose.pose.orientation.z =myQuat.z();
           leobot->odo_pose.pose.pose.orientation.w = myQuat.w();

           // write remaining data into header and twist data from odometry topic
           leobot->odo_pose.header.seq = i_loop;
           leobot->odo_pose.header.stamp = ros::Time::now();
           leobot->odo_pose.twist.twist.linear.x = vx_w;
           leobot->odo_pose.twist.twist.linear.y = vy_w;
           leobot->odo_pose.twist.twist.angular.z = omega;

            // create data for transformation broadcaster
           tfOdom.transform.translation.x = x;
           tfOdom.transform.translation.y = y;
           tfOdom.transform.rotation.x = myQuat.x();
           tfOdom.transform.rotation.y = myQuat.y();
           tfOdom.transform.rotation.z = myQuat.z();
           tfOdom.transform.rotation.w = myQuat.w();
           tfOdom.header.stamp = ros::Time::now();
           tfOdom.header.seq = i_loop;

           for (int i = 0; i < 4; i++) {
               msg.act_pos[i] = leobot->M[i].get_position() * leobot->param->dir_w[i];
               msg.act_vel[i] = leobot->M[i].get_velocity() * leobot->param->dir_w[i];
               msg.statusword[i] = leobot->M[i].pPDO_in->statusword;
               msg.act_torque[i] = leobot->M[i].get_torque() * leobot->param->dir_w[i];
               msg.errorcode[i] = leobot->M[i].pPDO_in->errorcode;
           }
           if(ret == 0){
                pubMotor.publish(msg);
                pubOdo.publish(leobot->odo_pose);
                tfPanda = tfOdom;
                tfPanda.transform.translation.x += lpanda * cos(theta);
                tfPanda.transform.translation.y += lpanda * sin(theta);
                tfPanda.child_frame_id = "panda_base";
                tfPanda.transform.translation.z = 0.0512;
                tfBroadcaster.sendTransform(tfPanda);
                tfBroadcaster.sendTransform(tfOdom);
           }
           i_loop++;
           if (flag_debug) {
               timespec t_act_pub;
               clock_gettime(CLOCK_MONOTONIC, &t_act_pub);
               c_buff_t.put((t_act_pub.tv_sec - t_start_ctl.tv_sec) * 1000000000 + (t_act_pub.tv_nsec - t_start_ctl.tv_nsec));
               c_buff_i.put(3);
           }
           wait_rest_of_period(&pinfo_pub, &ret); //
       }
        pthread_cleanup_pop(1);
    }

    void set_controlword(void* pbase, int ctlword)
    {
        base *leobot = (base*) pbase;
        for (int i=0; i<4; i++){
            switch(OPMode)
            {
                case OP_profile_pos:
                    leobot->M[i].pPDO_out_pos->controlword = ctlword;
                    break;
                case OP_profile_vel:
                    leobot->M[i].pPDO_out_vel->controlword = ctlword;
                    break;

                case OP_cyclic_torque:
                    leobot->M[i].pPDO_out_torque->controlword = ctlword;
                    break;
            }
        }
    }


    bool refreshParam(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res,
                       base *pleobot, RosParam *pleobot_param)
    {
        // n.getParam("max_vel_phi", leobot_param.max_vel_phi);

        ros::NodeHandle *nH = (ros::NodeHandle*) pleobot->nodehandle;
        nH->getParam("t_timeout", pleobot->param->t_timeout);
        nH->getParam("t_timeout", pleobot->param->max_vel_x);
        nH->getParam("t_timeout", pleobot->param->max_vel_y);
        nH->getParam("t_timeout", pleobot->param->max_vel_phi);
        nH->getParam("t_timeout", pleobot->param->max_wheelvel);
        nH->getParam("t_timeout", pleobot->param->max_wheelacc);
        res.success = true;
        res.message = "leobot_base read updated parameters. ";
        return 0;
    }

    // main function, loops with subscriber!
    void EPOS_Main(char *ifname)
    {
        base leobot;
        int chk = 200;
        needlf = FALSE;
        inOP = FALSE;
        signal(SIGINT, handler_int);
        signal(SIGTERM, handler_int);
        /* initialise SOEM, bind socket to ifname */
        if (ec_init(ifname))
        {
            printf("ec_init on %s succeeded.\n",ifname);
            /* find and auto-config slaves */
            if ( ec_config_init(FALSE) > 0 )
            // when ec_config_init finishes, all states are requested to state PRE_OP
            {
                printf("%d slaves found and configured.\n",ec_slavecount);
                if (ec_slavecount != 4){
                    printf("Aborting: not all 4 slaves required for the base movement were found. \n");
                    exit(-1);
                }
                // create nodehandle for ros node
                ros::NodeHandle n("leobot_base");
                leobot.nodehandle = &n;
                RosParam leobot_param;
                leobot.param = &leobot_param;

                ROS_INFO("Motor Node Online\n");

                n.getParam("flag_debug", flag_debug);
                n.getParam("flag_filter", flag_filter);

                printf("read from parameter server.\n");
                // read the parameter from the ROS parameter server
                n.getParam("wheelradius", leobot_param.r_wheel);
                n.getParam("max_vel_x", leobot_param.max_vel_x);
                n.getParam("max_vel_y", leobot_param.max_vel_y);
                n.getParam("max_vel_phi", leobot_param.max_vel_phi);
                n.getParam("max_wheelvel", leobot_param.max_wheelvel);
                n.getParam("max_wheelacc", leobot_param.max_wheelacc);
                n.getParam("ly", leobot_param.ly);
                n.getParam("lx", leobot_param.lx);
                n.getParam("gearbox_ratio_belt", leobot_param.gear_ratio_belt);

                leobot_param.alpha = atan(leobot_param.lx / leobot_param.ly);
                leobot_param.factor_rotation = (leobot_param.ly + leobot_param.lx) / leobot_param.r_wheel; // for O-configuration
                // leobot_param.factor_rotation =  (leobot_param.lx + leobot_param.ly) / leobot_param.r_wheel; // for X-configuration

                // read timeout from parameters
                n.getParam("t_timeout", leobot_param.t_timeout);
                leobot_param.n_timeout = leobot_param.t_timeout * 1000;
                leobot_param.i_timeout = 0;

                for (int i_node = 0; i_node < 4; i_node++) {
                    leobot.M[i_node].gear_ratio_belt = leobot_param.gear_ratio_belt;
                }
                n.getParam("dir_wheels", leobot_param.dir_w);
                // Mapping of PDO: permitted only  in PRE-OP mode!
                { // scope do encapsulate buffer variable
                    uint8_t buff;
                    uint32_t buff2, buff3;

                    for (int slavenumber=1; slavenumber <=4; slavenumber++) {
                        printf("setting PDO mapping for requested mode. \n");
                        // input PDOs (from masters perspective)
                        buff=0;
                        ec_SDOwrite(slavenumber, 0x1A00, 0x00, FALSE, 1, &buff, EC_TIMEOUTRXM);
                        buff2 = 0x60410010;     // Statusword
                        ec_SDOwrite(slavenumber, 0x1A00, 0x01, FALSE, 4, &buff2, EC_TIMEOUTRXM);
                        buff2 = 0x60640020;     // Position actual value
                        ec_SDOwrite(slavenumber, 0x1A00, 0x02, FALSE, 4, &buff2, EC_TIMEOUTRXM);
                        if(flag_filter){
                            buff2 = 0x30D30120;     // velocity actual value avaraged
                        }else{
                            buff2 = 0x606C0020;     // velocity actual value
                        }
                        ec_SDOwrite(slavenumber, 0x1A00, 0x03, FALSE, 4, &buff2, EC_TIMEOUTRXM);
                        if(flag_filter){
                            buff2 = 0x30D20110;     // Torque actual value avaraged
                        }else{
                            buff2 = 0x60770010;     // Torque actual value
                        }
                        ec_SDOwrite(slavenumber, 0x1A00, 0x04, FALSE, 4, &buff2, EC_TIMEOUTRXM);
                        buff2 = 0x603F0010;     // Errorcode
                        ec_SDOwrite(slavenumber, 0x1A00, 0x05, FALSE, 4, &buff2, EC_TIMEOUTRXM);
                        buff = 5;
                        ec_SDOwrite(slavenumber, 0x1A00, 0x00, FALSE, 1, &buff, EC_TIMEOUTRXM);
                        // output PDOs (from masters Perspective)
                        buff = 0;
                        ec_SDOwrite(slavenumber, 0x1600, 0x00, FALSE, 1, &buff, EC_TIMEOUTRXM);
                        int num_extra = 0;
                        buff2 = 0x60400010; // controlword
                        ec_SDOwrite(slavenumber, 0x1600, 0x01, FALSE, 4, &buff2, EC_TIMEOUTRXM);
                        switch (OPMode) { // mapping of PDO-Data for control is depending of OP-Mode:
                            case OP_profile_pos: {
                                int size = 4;
                                ec_SDOread(slavenumber, 0x6064, 0x00, FALSE, &size, &buff3,
                                           EC_TIMEOUTRXM); // read actual position to set target position to initial position!
                            }
                                buff2 = 0x60810020; // profile velocity
                                ec_SDOwrite(slavenumber, 0x1600, 0x03, FALSE, 4, &buff2, EC_TIMEOUTRXM);
                                num_extra++;
                                buff2 = 0x60830020; // profile acceleration
                                ec_SDOwrite(slavenumber, 0x1600, 0x04, FALSE, 4, &buff2, EC_TIMEOUTRXM);
                                num_extra++;
                                buff2 = 0x60840020; // profile decceleration
                                ec_SDOwrite(slavenumber, 0x1600, 0x05, FALSE, 4, &buff2, EC_TIMEOUTRXM);
                                num_extra++;
                                buff2 = 0x607A0020; // target position (in increments), subindex 0, 0x20 = 32 bit integer
                                break;
                            case OP_profile_vel:
                                buff2 = 0x60830020; // profile acceleration
                                ec_SDOwrite(slavenumber, 0x1600, 0x03, FALSE, 4, &buff2, EC_TIMEOUTRXM);
                                num_extra++;
                                buff2 = 0x60840020; // profile deceleration
                                ec_SDOwrite(slavenumber, 0x1600, 0x04, FALSE, 4, &buff2, EC_TIMEOUTRXM);
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
                        // write OP-mode specific
                        ec_SDOwrite(slavenumber, 0x1600, 0x02, FALSE, 4, &buff2, EC_TIMEOUTRXM);
                        buff = 2 + num_extra; // number of mapped PDO etrys
                        ec_SDOwrite(slavenumber, 0x1600, 0x00, FALSE, 1, &buff, EC_TIMEOUTRXM);
                        int size = 1;
                        // Node-ID; may not match the slave-ID! --> important for identification of physical wheels.
                        // the Node-ID is set with the DIP-switch on the slave hardware.
                        ec_SDOread(slavenumber, 0x2000, 0x00, FALSE, &size, &buff, EC_TIMEOUTRXM);

                        if (buff <=4){
                            leobot.M[buff-1].slave_number = slavenumber;
                            leobot.M[buff-1].NodeID = buff;
                            leobot.M[buff-1].isonline = true;
                            if (OPMode == OP_profile_pos){
                                leobot.M[buff-1].tar_pos = buff3; // prevent movement at start!
                            }
                            leobot.M[buff-1].startup_OD();
                            printf("PDOs Slave %d | Node-ID %d mapped\n", leobot.M[buff-1].slave_number,
                                   leobot.M[buff-1].NodeID);
                        }
                    }
                }
                if (flag_filter){
                    printf("using already filtered torque and velocity values\n");
                }
                ec_config_map(&IOmap);
                ec_configdc();
                printf("Slaves mapped, state to SAFE_OP.\n");

                /* wait for all slaves to reach SAFE_OP state */
                ec_statecheck(0, EC_STATE_SAFE_OP,  EC_TIMEOUTSTATE * 4);
                osal_usleep(4000);


                // no buffer --> only the most recent message is used
                switch(OPMode){
                    case OP_profile_pos:
                        subMotor = n.subscribe<leobot_base_msg::motor_wheels>("cmd_pos", 1, boost::bind(ros_callback_pos, _1, &leobot, &leobot_param));
                        break;
                    case OP_profile_vel:
                        // const ros::TransportHints & transport_hints = ros::TransportHints().tcpNoDelay();
                        subMotor = n.subscribe<geometry_msgs::Twist>("cmd_vel", 1, boost::bind(ros_callback_vel, _1, &leobot, &leobot_param),
                                ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay());
                        break;
                    case OP_cyclic_torque:
                        subMotor = n.subscribe<leobot_base_msg::motor_wheels>("cmd_torque", 1, boost::bind(ros_callback_torque, _1, &leobot, &leobot_param));
                        break;
                    default:
                        printf("something went wront! OP Mode not recognized!\n");
                        break;
                }



                // prepare RT-threads
                printf("setting scheduler options!\n");
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
                if (ret_ctl || ret_stat){
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
                param_stat.sched_priority = 30;
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

                // printf("segments : %d : %d %d %d %d\n",ec_group[0].nsegments ,ec_group[0].IOsegment[0],ec_group[0].IOsegment[1],ec_group[0].IOsegment[2],ec_group[0].IOsegment[3]);
                printf("Request operational state for all slaves\n");
                expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
                printf("Calculated workcounter %d\n", expectedWKC);
                ec_slave[0].state = EC_STATE_OPERATIONAL;
                /* send one valid process data to make outputs in slaves happy*/
                ec_send_processdata();
                ec_receive_processdata(EC_TIMEOUTRET);
                osal_usleep(5000);
            /* request OP state for all slaves */
            ec_writestate(0);
            /* wait for all slaves to reach OP state */
            do
            {
                ec_send_processdata();
                ec_receive_processdata(EC_TIMEOUTRET);
                ec_statecheck(0, EC_STATE_OPERATIONAL, 50000);
            }
            while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));

            if (ec_slave[0].state == EC_STATE_OPERATIONAL ) {
                printf("Operational state reached for all slaves.\n");
                inOP = TRUE;
                // assigning PDO data to struct
                for (int i_slave = 0; i_slave < 4; i_slave++) {
                    leobot.M[i_slave].pPDO_in = (struct PDO_in *) (ec_slave[leobot.M[i_slave].slave_number].inputs);
                    switch (OPMode) { // mapping of PDO-Data for control is depending of OP-Mode:
                        case OP_profile_pos:
                            leobot.M[i_slave].pPDO_out_pos = (struct PDO_out_pos *) (ec_slave[leobot.M[i_slave].slave_number].outputs);
                            leobot.M[i_slave].pPDO_out_pos->controlword = 0x07; // Switch on
                            // write the at startup read actual position into the target position PDO, otherwise robot
                            // moves at startup into zero-position if an other Operationmode was already used before without
                            // resetting the motor ;
                            leobot.M[i_slave].pPDO_out_pos->tar_pos = leobot.M[i_slave].tar_pos;
                            break;
                        case OP_profile_vel:
                            leobot.M[i_slave].pPDO_out_vel = (struct PDO_out_vel *) (ec_slave[leobot.M[i_slave].slave_number].outputs);
                            leobot.M[i_slave].pPDO_out_vel->controlword = 0x07;
                            leobot.M[i_slave].pPDO_out_vel->profile_acc = 10000;
                            leobot.M[i_slave].pPDO_out_vel->profile_dec = 10000;

                            break;
                        case OP_cyclic_torque:
                            leobot.M[i_slave].pPDO_out_torque = (struct PDO_out_torque *) (ec_slave[leobot.M[i_slave].slave_number].outputs);
                            leobot.M[i_slave].pPDO_out_torque->controlword = 0x07;
                            break;
                        default:
                            printf("something went wrong!");
                            break;
                    }
                }
                ec_send_processdata();
                wkc = ec_receive_processdata(EC_TIMEOUTRET);
                osal_usleep(1000);

                // setting Operation mode
                printf("set OP Mode %d\n", OPMode);
                uint8_t buff;
                for(int i_slave=1; i_slave <=4; i_slave++){ // set the OP mode for all slaves
                    buff = OPMode; // OP Mode as specified
                    wkc = ec_SDOwrite(i_slave, 0x6060, 0x00, FALSE, 1, &buff, EC_TIMEOUTRXM);
                    leobot.M[i_slave].OPMode = OPMode;
                }
                if(flag_debug){
                    pFile = fopen("/tmp/epos_timestamps.txt", "w+");
                }

                // set affinity of processes to a seperate CPU; not implemented as it seems not to be needed.
                // int sched_setaffinity(pid_t pid, size_t cpusetsize, const cpu_set_t *mask);

                /* Create a pthread with specified attributes */
                int ret_pub = pthread_create(&tid_pub, nullptr, &pth_pub, &leobot);
                int ret_fileIO = 0;
                if (flag_debug){
                    ret_fileIO = pthread_create(&tid_fileIO, nullptr, &pth_fileIO, nullptr);
                }
                ret_ctl = pthread_create(&tid_ctl, &attr_ctl, &pth_ctl_func, &leobot);
                ret_stat =  pthread_create(&tid_status, &attr_stat, &pth_check_status, &leobot);
                if (ret_ctl || ret_stat || ret_fileIO || ret_pub) {
                    printf("create pthread failed\n");
                    exit(-2);
                }
                printf("Successfully created threads\n");
                struct period_info pinfo_main;
                periodic_task_init(&pinfo_main, 1);
                int ret_subSpin;
                printf("Subscriber spinning. \n");

                ros::ServiceServer service = n.advertiseService<std_srvs::Trigger::Request,std_srvs::Trigger::Response>
                        ("/parameters/TriggerUpdate", boost::bind(refreshParam,_1,_2, &leobot, &leobot_param)); //, &leobot, &leobot_param));
                while(ros::ok()){
                    ros::spinOnce();
                    wait_rest_of_period(&pinfo_main, &ret_subSpin);
                }
            }
            else
            {
                printf("Not all slaves reached operational state.\n");
                ec_readstate();
                for(int i = 1; i<=ec_slavecount ; i++)
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

// ecatcheck is running in a seperat thread in the Operating System Abstraction Layer; currently unused
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
    printf("Movement of the Base: Communication with MotorController EPOS 4 \n"
           "\t based on example code of SOEM (Simple Open EtherCAT Master)\n \n");

    /*    
    std::vector<std::string> NodeNames;
    ros::master::getNodes(NodeNames);
    for (int j = 0; j < NodeNames.size() ; j++) {
        std::cout << NodeNames[j] << std::endl;
        if (NodeNames[j].compare("/leobot_base_movement")){
            ROS_INFO("Leobot_base is already running and not started again. ");
            exit(0);
        }
    } */

// init ros environment without sigint handler --> costum one is implemented to end threads properly
    ros::init(argc, argv, "leobot_movement", ros::init_options::NoSigintHandler);


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
        printf("Usage: Eth_BaseMovement ifname1 OPMode\nifname = enp1s0f1 for example.\nOPModes:\n1 \t(Profile Position Mode)\n3 \t(Profile Velocity Mode)\n10 \t(Cyclic torque)\n");
    }
    printf("End program\n");

    return (0);
}



