uint16[4] statusword # described in EPOS4-Firmware specification
uint16[4] errorcode # must be 0 --> described as above. 
float32[4] act_pos # in rad; 0 is starting position as it is not homed. 
float32[4] act_vel # in rad/s
float32[4] act_torque
