  /* +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  *
  * EPOS4 sourcefile
  * Version 1.0
  *
  * Details:    Implementation of the classes base, MotorInfo and others to enable the
  *            movement of the base of 'LeoBot' on the hardware level. The
  *            implementation is specific to the EPOS4 motorcontrollers from maxon.
  *             The class includes the reading of various parameters in the Object
  *            dictionary, process data objects are to be mapped in the application
  *            and inverse kinematics are implemented for velocity mode.
  *
  * Author:             Peter Manzl
  * Date created:       2020-04-20
  *
  * Use under BSD licence permitted without any express or implied warranty. Do NOT use
  * this application without detailed knowledge of the hardware and electronics.
  * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */

#include "EPOS4_class.h"


  template <typename T> int sgn(T val) {
      return (T(0) < val) - (val < T(0));
  }
// output for Object Dictionary
void cOD::print_OD(){
    printf("\n Entries in the OD: \n");
    for (int i=0; i< this->entry.size(); i++){
        printf("\t Index 0x%04.4x, Subindex 0x%02.2x: size %d, type %s \t \%s \n", this->entry[i].Index, this->entry[i].SubIndex,
                this->entry[i].size, this->entry[i].Typename.c_str(), this->entry[i].Name.c_str());
    }
}

// calculates Position in rad from sensor increments
float MotorInfo::get_position(){
    return (2*M_PI* (float) this->pPDO_in->pos_act)/(this->increments*this->gearbox_ratio*this->gear_ratio_belt);
}

// rpm into rad/s
float MotorInfo::get_velocity(){
    return ((float) 2*M_PI*this->pPDO_in->vel_act)*this->unit_vel / (60*this->gearbox_ratio*this->gear_ratio_belt);
}

float MotorInfo::get_torque(){
    return ((float) this->pPDO_in->torq_act)/1000 *(this->ratedTorque)/1000000 *
                                                    (this->gearbox_ratio*this->gear_ratio_belt);
    // ratedTorque is in uNm --> factor of of 1e6 in between to have Nm as output. (SI-units!)
    // torque_act is in 1/1000 of the rated torque, the gear needs to be considered also
}

int32 MotorInfo::calc_acc_maxon(float acc_rads2){
    int32 acc_maxon = (acc_rads2*this->unit_acc * 60 * this->gearbox_ratio * this->gear_ratio_belt )/ (2*M_PI);
    return acc_maxon;
}

float MotorInfo::get_temp(){
    return ((float)this->Temp)/10.0;
}

float MotorInfo::get_voltage(){
    return ((float)this->Voltage)/10.0;
}

void MotorInfo::print_status(){
    if((this->pPDO_in == nullptr)){
       printf("No Statusword or Processdata mapped.\n");
       return;
    }
    int statusword = this->pPDO_in->statusword;
    printf("Node %d: statusword: 0x%04.4x | ", this->NodeID, statusword);
        if (1 & statusword) {
            printf("ready to switch on; ");
        }
        if ((1 << 1) & statusword) {
            printf("Switched on; ");
        }
        if ((1 << 2) & statusword) {
            printf("Operation enabled; ");

        }
        if ((1 << 3) & statusword) {
            printf("Fault; ");
            // if fault: read error code in Index / Subindex: 0x603F / 0x00
            // all error codes are described in the EPOS 4 firmware specification p. 237 ff
           //  wkc = ec_SDOread(slave, 0x603F, 0x00, FALSE, &size, &statusword, EC_TIMEOUTRXM*2);
            // printf("\nerrorcode: 0x%4.4x ", statusword);
            /*if (errorcode == 0x8180){
                printf("--> EtherCAT Communication error");
            }*/
        }
        printf("\n");
        if ((1 << 4) & statusword) {
            printf("Voltage enabled | ");
        }
        if ((1 << 5) & statusword) {
            printf("Quick Stop | ");
        }
        if ((1 << 6) & statusword) {
            printf("Switch on disabled | ");
        }
        if ((1 << 7) & statusword) {
            printf("Warning present | ");
        }
        if ((1 << 8) & statusword) {
            printf("reserved(0) | ");
        }
        if ((1 << 9) & statusword) {
            printf("Remote | ");
        }
        if ((1 << 10) & statusword) {
            if (this->OPMode==3){ // PVM: Profile Velocity Mode
                printf("Target reached | ");
            }else{
                printf("Operation Mode specific | ");
            }
            //printf("Operationmode specific; ");
        }
        if ((1 << 11) & statusword) {
            if (this->OPMode==3){
                printf("Speed is limited; ");
            }else{

            }
        }
        if ((1 << 12) & statusword) {
            printf("Operationmode specific; ");
            if (this->OPMode==3){
                printf("Speed");
            }else{

            }
        }
        if ((1 << 13) & statusword) {
            if (this->OPMode==3){
                printf("not used in OPMode 3 | ");
            }else{

            }
        }
        if ((1 << 14) & statusword) {
            printf("reserved(0) | ");
        }
        if ((1 << 15) & statusword) {
            printf("Position referenced to home position | ");

    }
        printf("\n");
} // end of method

void MotorInfo::print_errors(uint16_t error){
    printf("Node-ID %d: errorcode 0x%04.4x \t", this->NodeID, error);
    // watchdog, CAN and sign of life (from extension) are disabled
    switch(error){
        case 0x1000: printf("Generic error\n"); break;
        case 0x1080 ... 0x1083: printf("Generic initialization error\n"); break;
        case 0x1090: printf("Firmware incompability error\n"); break;
        case 0x2310: printf("Overcurrent error\n"); break;
        case 0x2320: printf("Power stage protection error\n"); break;
        case 0x3210: printf("Overvoltage error\n"); break;
        case 0x3220: printf("Undervoltage error\n"); break;
        case 0x4380: printf("Thermal motor overload error\n"); break;
        case 0x5113: printf("Logic supply voltage too low error\n"); break;
        case 0x5280: printf("Hardware defect error \n"); break;
        case 0x5281: printf("Hardware incompability error\n"); break;
        case 0x5480 ... 0x5483: printf("Hardware error\n"); break;
        case 0x6320: printf("Software param->ter error \n \t corrupt param->ter \n"); break;
        case 0x7320: printf("Position sensor error\n"); break;
        case 0x8180: printf("EtherCAT communication error\n"); break;
        case 0x8181: printf("EtherCAT initialization error\n"); break;
        case 0x8182: printf("EtherCAT Rx queue overflow\n"); break;
        case 0x8183: printf("EtherCAT communication error (internal)\n"); break;
        case 0x8250: printf("RPDO timeout\n"); break;
        case 0x8280: printf("EtherCAT PDO communication error\n"); break;
        default: printf("see EPOS4 Firmware Specification for further information. \n"); break;
    }

}

void MotorInfo::init_OD_indices(){
    printf("\nreading OD indices from ESI file not implemented yet!\n ");

}

void MotorInfo::init_std_OD(){
    this->OD.entry.clear();
    this->OD.entry.push_back(OD_entry{"Node-ID", 0x2000, 0x00, sizeof(uint8), "uint8"});
    this->OD.entry.push_back(OD_entry{"Serial Number complete", 0x2100, 0x01, sizeof(uint64), "uint64"});
    this->OD.entry.push_back(OD_entry{"Power supply Voltage", 0x2200, 0x01, sizeof(uint16), "uint16"});
    this->OD.entry.push_back(OD_entry{"Gear reduction numerator", 0x3003, 0x01, sizeof(uint32), "uint32"});
    this->OD.entry.push_back(OD_entry{"Gear reduction denumerator", 0x3003, 0x02, sizeof(uint32), "uint32"});
    this->OD.entry.push_back(OD_entry{"Temperature",0x3201,0x01, sizeof(int16), "int16"});
    this->OD.entry.push_back(OD_entry{"Statusword", 0x6041, 0x0000, sizeof(uint16), "uint16"});
    this->OD.entry.push_back(OD_entry{"Error Code", 0x603F, 0x00, sizeof(uint16),"uint16"});
    this->OD.entry.push_back(OD_entry{"OP-Mode", 0x6060, 0x00, sizeof(int8), "int8"});
    this->OD.entry.push_back(OD_entry{"SI Units velocity", 0x60A9, 0x00, sizeof(uint32), "uint32"});
    this->OD.entry.push_back(OD_entry{"Motor Type", 0x6402, 0x00, sizeof(uint32), "uint32"}); // index 10
    // 11:
    this->OD.entry.push_back(OD_entry{"Main Sensor resolution", 0x3000, 0x05, sizeof(uint32), "uint32"});
    this->OD.entry.push_back(OD_entry{"Error Code", 0x603F, 0x00, sizeof(uint16), "uint16"});
    this->OD.entry.push_back(OD_entry{"Motor rated torque", 0x6076, 0x00, sizeof(uint32), "uint32"});
    this->OD.entry.push_back(OD_entry{"SI Units acceleration", 0x60AA, 0x00, sizeof(uint32), "uint32"});
    this->OD.entry.push_back(OD_entry{"Sync Manager 2 Synchronization type", 0x1C32, 0x01, sizeof(uint16), "uint16"});
    this->OD.entry.push_back(OD_entry{"Sync Manager 2 Cycle time", 0x1C32, 0x02, sizeof(uint32), "uint32"});
    this->OD.entry.push_back(OD_entry{"Sync Manager 2 types supported", 0x1C32, 0x02, sizeof(uint32), "uint32"});
    this->OD.entry.push_back(OD_entry{"Sync Manager 2 Min Cycle time [ns]", 0x1C32, 0x05, sizeof(uint32), "uint32"});
    this->OD.entry.push_back(OD_entry{"Sync Manager 2 Calc and copy time [ns]", 0x1C32, 0x06, sizeof(uint32), "uint32"});
    this->OD.entry.push_back(OD_entry{"Sync Manager 2 Delay time [ns]", 0x1C32, 0x09, sizeof(uint32), "uint32"});
    // 21:
    this->OD.entry.push_back(OD_entry{"Sync Manager 2 SM-event missed [ns]", 0x1C32, 0x0B, sizeof(uint16), "uint16"});
    this->OD.entry.push_back(OD_entry{"Abort Connection option Code", 0x6007, 0x00, sizeof(int16), "int16"});
}


void MotorInfo::startup_OD(){
    if (this->OD.entry.empty()){
        printf("No OD specified, inserting standard Indices. \n");
        init_std_OD();
    }
    if (!(this->isonline)){
        printf("Controller is not online, can not read OD. \n");
        return;
    }
    printf("reading OD of slave: %d\n", this->slave_number);
    printf("there are %d entrys",3); //  this->OD.entry.empty());
        int wkc, size;
        std::vector<uint64> data;
        for (int i=0; i < this->OD.entry.size(); i++){
        size = this->OD.entry[i].size;
        uint64 buff = 0;
       wkc = ec_SDOread(this->slave_number, this->OD.entry[i].Index, this->OD.entry[i].SubIndex, FALSE, &size, &buff, EC_TIMEOUTRXM*2);
        if (wkc == 0) {
            printf("reading of SDO %s failed. \n", this->OD.entry[i].Name.c_str());
        } else{
            printf("SDO %s is %lld.\n", this->OD.entry[i].Name.c_str(), buff);
          if ((this->OD.entry[i].Index == 0x6402) && (buff != 10)){
              printf("Error; read Motor Type is not BLDC.\n");
          } else if ((this->OD.entry[i].Index == 0x6041) && ((1 << 3) & buff)){
              printf("Statusword Fault bit is set.\n");
              wkc = ec_SDOread(this->slave_number, 0x603F, 0x00, FALSE, &size, &buff, EC_TIMEOUTRXM*2);
              printf("Errorcode: 0x%04.4x\n", buff);
              this->reset_fault();
          }
        }
        data.push_back(buff);
    }
    // OD entries read and in vector data
    this->SerialNumbercomplete = (uint64) data[1];
    this->Temp = (int16) data[5];
    this->Voltage = (uint16) data[2];
  //  this->ObjectID = 0;
    this->gearbox_ratio = ((float)data[3])/data[4];
    this->OPMode = data[8];
    this->SI_unit_velocity = data[9];
    this->increments = data[11];
    switch(data[9]){
        case 0x00B44700: this->unit_vel = 1; break;
        case 0xFFB44700: this->unit_vel = 0.1; break;
        case 0xFEB44700: this->unit_vel = 0.01; break;
        case 0xFDB44700: this->unit_vel = 0.001; break;
        case 0xFCB44700: this->unit_vel = 0.0001; break;
        case 0xFBB44700: this->unit_vel = 0.00001; break;
        case 0xFAB44700: this->unit_vel = 0.000001; break;
        default: printf("unit velocity is no standard unit and could not be read! \n");
    }
    printf("unit vel is %f rpm. \n", this->unit_vel);
    switch(data[14]) {
        case 0x00C00300: this->unit_acc = 1; break;
        default: printf("unit acceleration is no standard unit and could not be read! \n");
    }
    printf("unit acc is %f rpm/s. \n", this->unit_acc);

    if (data[10]!= 10){
        printf("Motor is not BLDC? \n");
    }
    this->ratedTorque = data[13];
    this->SM_events_missed = data[21];
} // end of method


void MotorInfo::reset_fault() {
    printf("\nreset fault. \n");
        uint16_t ctl_word = 0x8F;
        ec_SDOwrite(this->slave_number, 0x6040, 0x00, FALSE, 2 , &ctl_word, EC_TIMEOUTRET*5);

}

// power off by setting controlword to voltage disabled
void MotorInfo::poweroff() {
    int size = 2;
    this->isonline = 0; // cyclic thread is killed by cleanup handler, not object
    uint16_t ctl_word = 0x06;
    ec_SDOwrite(this->slave_number, 0x6040, 0x00, FALSE, 2, &ctl_word, EC_TIMEOUTRET*5);
}

// startup of Motor, currently unused
void MotorInfo::startup_motor(){


}

// from rad(ROS) --> inkrements(Motor)
void MotorInfo::set_tar_pos(float pos_rad){
    this->tar_pos = (int)(this->increments*pos_rad*this->gearbox_ratio * this->gear_ratio_belt/(2*M_PI));
}

void MotorInfo::set_tar_vel(float vel_rads){ // converts from rad/s into rpm considering the gear ratio
    this->tar_vel = (int) (60* vel_rads * this->gear_ratio_belt * this->gearbox_ratio/(2*M_PI));
}
// from Nm(ROS)  --> 1/1000 of rated torque
void MotorInfo::set_tar_torque(float torque_Nm){
    this->tar_torque = (int) (((1000000000.0*torque_Nm/(this->ratedTorque))/(this->gearbox_ratio*this->gear_ratio_belt)));
}

void base::setVel(float vx, float vy, float w){
    if (abs(vx) > this->param->max_vel_x) {
        vx = this->param->max_vel_x * sgn(vx);
    }
    if (abs(vy) > this->param->max_vel_y){
        vy = this->param->max_vel_y*sgn(vy);
    }

    if(abs(w) > this->param->max_vel_phi){
        w = this->param->max_vel_phi*sgn(w);
    }

    this->vel = {vx, vy, w};
    // inverse kinematics for the X-arrangement of the Mecanum wheels (viewing the bottom rolls)
    /* this->w[0] = (1/this->param->r_wheel)*vx + (1/this->param->r_wheel) * vy  - this->param->factor_rotation*w;
    this->w[1] = (1/this->param->r_wheel)*vx - (1/this->param->r_wheel) * vy  + this->param->factor_rotation*w;
    this->w[2] = (1/this->param->r_wheel)*vx - (1/this->param->r_wheel) * vy  - this->param->factor_rotation*w;
    this->w[3] = (1/this->param->r_wheel)*vx + (1/this->param->r_wheel) * vy  + this->param->factor_rotation*w; */

    // the following is the inverse kinematics for the O-arrangement of the Mecanum wheels (when viewing the bottom rolls)
    this->w[0] = (1/this->param->r_wheel)*vx - (1/this->param->r_wheel) * vy  - this->param->factor_rotation*w;
    this->w[1] = (1/this->param->r_wheel)*vx + (1/this->param->r_wheel) * vy  + this->param->factor_rotation*w;
    this->w[2] = (1/this->param->r_wheel)*vx + (1/this->param->r_wheel) * vy  - this->param->factor_rotation*w;
    this->w[3] = (1/this->param->r_wheel)*vx - (1/this->param->r_wheel) * vy  + this->param->factor_rotation*w;

    // implement max wheelvelocity; if surpassed project all veloicites back to allowed values to
    // maintain the ratio of vx, vy and w
    float max_wvel = 0;
    for (int i_motor = 0; i_motor < 4; i_motor ++){
        if (abs(max_wvel) < abs(this->w[i_motor])){
            max_wvel = abs(this->w[i_motor]);
        }
    }
    if (max_wvel > this->param->max_wheelvel){ // max angular velocity surpasses the stored parameter
        for(int i_motor=0; i_motor < 4; i_motor++){
            this->w[i_motor] = this->w[i_motor] * (this->param->max_wheelvel/max_wvel);
        }
    }

    for (int i=0; i< 4; i++){
        this->M[i].set_tar_vel(this->param->dir_w[i]* this->w[i]);
    }
}



