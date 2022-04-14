  /* +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  *
  * EPOS4 header
  * Version 1.0
  *
  * Details:   Implementation of the classes base, MotorInfo and others to enable the
  *            movement of the base of 'LeoBot' on the hardware level. The
  *            implementation is specific to the EPOS4 motorcontrollers from maxon.
  *            The class includes the reading of various parameters in the Object
  *            dictionary, process data objects are to be mapped in the application
  *            and inverse kinematics are implemented for velocity mode.
  *
  * Author:             Peter Manzl
  * Date created:       2020-04-20
  *
  * Use under BSD licence permitted without any express or implied warranty. Do NOT use
  * this application without detailed knowledge of the hardware and electronics.
  * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */

#include <stdio.h>
#include <string.h>
#include <string>
#include <inttypes.h>
#include "ethercat.h"
#include <stdint.h>
#include <unistd.h>
#include <sys/types.h>
#include <pthread.h>
#include <time.h>
#include <math.h>
#include <vector>
#include <sys/mman.h>

#include <nav_msgs/Odometry.h>

#ifndef PACKAGES_MOTOR_ETH_EPOS4_MAIN_H
#define PACKAGES_MOTOR_ETH_EPOS4_MAIN_H

// structs for IN and Output with PDOs (Process data objects).
// the packed attribute is needed to avoid padding bytes as the hardware uses none.

// for different Operation Modes different PDO configuartions are used!
struct PDO_out_vel {
    uint16_t controlword;
    int32_t tar_vel;
    uint32_t profile_acc;
    uint32_t profile_dec;
}__attribute__((packed));

struct PDO_out_pos {
   uint16_t controlword;
   int32_t tar_pos;
   uint32_t profile_vel;
   uint32_t profile_acc;
   uint32_t profile_dec;
}__attribute__((packed));

struct PDO_out_torque {
    uint16_t controlword;
    int16_t tar_torque;
}__attribute__((packed));

struct PDO_in {
    uint16_t statusword;
    int32_t pos_act;
    int32_t vel_act;
    int16_t torq_act;
    uint16_t errorcode;
}__attribute__((packed));


struct OD_entry{
    std::string Name;
    uint Index;
    uint SubIndex;
    uint size;
    std::string Typename;
};

// Object Dictionary; the Indexes/subindexes and Nomes of the Objects which need to be accessed are stored here.
// they are hardcoded in Version 1.0 but should be parsed from the .ESI file in later versions.
class cOD{
    public:
        std::vector<OD_entry> entry;
        void print_OD();

        cOD(){ // std constructor
            entry.clear(); // empty vector
        }
};

struct period_info {
    struct timespec next_period;
    long period_ns;
    long t_off;
};


// parameters read from the parameter server, used to calculate the wheel kinematics
struct RosParam{
    float r_wheel;
    float ly;
    float lx;
    float max_vel_x;    // monitor max velocity of wheels!
    float max_vel_y;
    float max_vel_phi;
    float max_acc_motor; // in maxon units: 10000 rpm/s is standard;
    float alpha;
    float l;
    // the factor for coupling the wheel velocities to the rotational velocity of the base around itself
    float factor_rotation;
    float gear_ratio_belt;
    int mode;
    std::vector<int> dir_w;
    float max_wheelvel;
    float max_wheelacc;
    int32_t i_timeout;
    int32_t n_timeout;
    float t_timeout;
};

// class for the Motor; contains pointer to "Process data Objects" as well as other  needed parameters.
// also provedes getter-Methods which calculates
class MotorInfo
{
public:
    boolean isonline;
    volatile boolean isrunning;
    int slave_number;
    uint8 NodeID;
    int SI_unit_velocity;   // act_vel to rad/s
    int SI_units_acceleration; // act_acc to rad/s2
    int Temp;               // in 0.1 degree C
    int Voltage;            // in 0.1V
    uint16_t SM_events_missed;
    uint32_t CycleTime;
    int ObjectID;           //
    int SerialNumbercomplete;
    float gearbox_ratio;
    float gear_ratio_belt;
    int increments;
    float unit_vel;
    float unit_acc;
    int32 tar_pos;
    int32 tar_vel;
    int16 tar_torque;
    uint OPMode;
    uint CycleViolations;
    void *nodehandle;
    uint ratedTorque;
    cOD OD;

    volatile PDO_out_pos* pPDO_out_pos; // pointer onto ouput Bits of the by SOEM provided values
    volatile PDO_out_vel* pPDO_out_vel; // pointer onto ouput Bits of the by SOEM provided values
    volatile PDO_out_torque* pPDO_out_torque; // pointer onto ouput Bits of the by SOEM provided values
    volatile PDO_in* pPDO_in;

    // Methods:
    float get_position();
    float get_velocity();
    float get_torque();
    float get_temp();
    float get_voltage();
    void print_status();
    void print_errors(uint16_t error);
    void init_OD_indices();
    void init_std_OD();
    void startup_OD();
    void reset_fault();
    void poweroff();
    void startup_motor();
    void set_tar_pos(float pos_rad);     // from rad(ROS) --> inkrements(Motor)
    void set_tar_vel(float vel_rads);
    void set_tar_torque(float torque_Nm);  // from Nm(ROS)  --> 1/1000 of rated torque
    int32 calc_acc_maxon(float acc_rads2);  // from Nm(ROS)  --> 1/1000 of rated torque

// standard constructor:
    MotorInfo()
        {
        isonline = 0;
        isrunning = 0;
        slave_number = 0;
        SI_unit_velocity = 0;
        Temp = 0;
        Voltage = 0;
        SM_events_missed = 0;
        ObjectID = 0;
        SerialNumbercomplete = 0;
        gearbox_ratio = 33;
        gear_ratio_belt = 1;
        increments = 16384;
        OPMode = 1;
        unit_vel = 1;
        tar_vel = 0;
        tar_pos = 0;
        tar_torque = 0;
        pPDO_out_vel = nullptr; // pointer onto ouput Bits of the by SOEM provided values
        pPDO_out_pos = nullptr;
        pPDO_out_torque = nullptr;
        pPDO_in = nullptr;
    }
private:



};

// base class to bundle integrate the 4 motors driving the robot
class base
{
public:
    MotorInfo M[4];
    void *nodehandle;
    boolean isrunning;
    std::vector<float> vel;
    std::vector<float> w;
    nav_msgs::Odometry odo_pose; // for Odometry
    RosParam *param;

    void setVel(float vx, float vy, float w);

    base(){ // constructor
        vel = {0,0,0};
        w = {0,0,0,0};
    }
};

#endif //PACKAGES_MOTOR_ETH_EPOS4_MAIN_H

