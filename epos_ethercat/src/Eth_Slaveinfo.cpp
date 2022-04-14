/** \file
 * \brief Example code for Simple Open EtherCAT master
 *
 * Usage : slaveinfo [ifname] [-sdo] [-map]
 * Ifname is NIC interface, f.e. eth0.
 * Optional -sdo to display CoE object dictionary.
 * Optional -map to display slave PDO mapping
 *
 * This shows the configured slave data.
 *
 * (c)Arthur Ketels 2010 - 2011
 */

#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <stdlib.h>
#include "ethercat.h"


#include <ros/ros.h>

char IOmap[4096];
ec_ODlistt ODlist;
ec_OElistt OElist;
boolean printSDO =  FALSE; //FALSE;
boolean printMAP = TRUE; // FALSE
char usdo[128];
char hstr[1024];

char* dtype2string(uint16 dtype)
{
    switch(dtype)
    {
        case ECT_BOOLEAN:
            sprintf(hstr, "BOOLEAN");
            break;
        case ECT_INTEGER8:
            sprintf(hstr, "INTEGER8");
            break;
        case ECT_INTEGER16:
            sprintf(hstr, "INTEGER16");
            break;
        case ECT_INTEGER32:
            sprintf(hstr, "INTEGER32");
            break;
        case ECT_INTEGER24:
            sprintf(hstr, "INTEGER24");
            break;
        case ECT_INTEGER64:
            sprintf(hstr, "INTEGER64");
            break;
        case ECT_UNSIGNED8:
            sprintf(hstr, "UNSIGNED8");
            break;
        case ECT_UNSIGNED16:
            sprintf(hstr, "UNSIGNED16");
            break;
        case ECT_UNSIGNED32:
            sprintf(hstr, "UNSIGNED32");
            break;
        case ECT_UNSIGNED24:
            sprintf(hstr, "UNSIGNED24");
            break;
        case ECT_UNSIGNED64:
            sprintf(hstr, "UNSIGNED64");
            break;
        case ECT_REAL32:
            sprintf(hstr, "REAL32");
            break;
        case ECT_REAL64:
            sprintf(hstr, "REAL64");
            break;
        case ECT_BIT1:
            sprintf(hstr, "BIT1");
            break;
        case ECT_BIT2:
            sprintf(hstr, "BIT2");
            break;
        case ECT_BIT3:
            sprintf(hstr, "BIT3");
            break;
        case ECT_BIT4:
            sprintf(hstr, "BIT4");
            break;
        case ECT_BIT5:
            sprintf(hstr, "BIT5");
            break;
        case ECT_BIT6:
            sprintf(hstr, "BIT6");
            break;
        case ECT_BIT7:
            sprintf(hstr, "BIT7");
            break;
        case ECT_BIT8:
            sprintf(hstr, "BIT8");
            break;
        case ECT_VISIBLE_STRING:
            sprintf(hstr, "VISIBLE_STRING");
            break;
        case ECT_OCTET_STRING:
            sprintf(hstr, "OCTET_STRING");
            break;
        default:
            sprintf(hstr, "Type 0x%4.4X", dtype);
    }
    return hstr;
}


struct PDO_out {
    uint16_t controlword;
    int32_t vel_tar;
};
struct PDO_in {
    uint16_t statusword;
    int32_t pos_act;
    int32_t vel_act;
    int16_t torq_act;
};

void printbits(int x)
{
    for(int i=sizeof(x)<<3; i; i--)
        putchar('0'+((x>>(i-1))&1));
}

void read_statusword(int slave){
// prints statusword as explained in the Firmware Specification of the EPOS4
    int wkc;
    uint16_t statusword;
    uint8_t OP_Mode;
    int size = 1;
    wkc = ec_SDOread(slave, 0x6060, 0x00, FALSE, &size, &OP_Mode, EC_TIMEOUTRXM*2);
    size = 2;

    wkc = ec_SDOread(slave, 0x6041, 0x00, FALSE, &size, &statusword, EC_TIMEOUTRXM*3);
    ec_receive_processdata(EC_TIMEOUTRET);

    ec_send_processdata();
    struct PDO_in *slave_i;
    slave_i = (struct PDO_in *)(ec_slave[1].inputs);
    ec_receive_processdata(EC_TIMEOUTRXM *10);
    printf("Read PDO: Statusword 0x%x | Position actual %d | velocity actual %d |  torque actual %d \n", slave_i->statusword,
            slave_i->pos_act, slave_i->vel_act, slave_i->torq_act);
    printf("statusword PDO: 0x%x , statusword SDO 0x%x \n", slave_i->statusword, statusword);

    // statusword = slave_i->statusword;
    if (wkc > 0) {
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
            wkc = ec_SDOread(slave, 0x603F, 0x00, FALSE, &size, &statusword, EC_TIMEOUTRXM*2);
            printf("\nerrorcode: 0x%4.4x ", statusword);
            if (statusword == 0x8180){
                printf("--> EtherCAT Communication error");
            }
        }
        printf("\n");
        if ((1 << 4) & statusword) {
            printf("Voltage enabled; ");
        }
        if ((1 << 5) & statusword) {
            printf("Quick Stop; ");
        }
        if ((1 << 6) & statusword) {
            printf("Switch on disabled; ");
        }
        if ((1 << 7) & statusword) {
            printf("Warning present; ");
        }
        if ((1 << 8) & statusword) {
            printf("reserved(0); ");
        }
        if ((1 << 9) & statusword) {
            printf("Remote; ");
        }
        if ((1 << 10) & statusword) {
            if (OP_Mode==3){ // PVM: Profile Velocity Mode
                printf("Target reached; ");
            }else{
                printf("Operation Mode specific");
            }
            //printf("Operationmode specific; ");
        }
        if ((1 << 11) & statusword) {
            if (OP_Mode==3){
                printf("Speed is limited; ");
            }else{

            }
        }
        if ((1 << 12) & statusword) {
            printf("Operationmode specific; ");
            if (OP_Mode==3){
                printf("Speed");
            }else{

            }
        }
        if ((1 << 13) & statusword) {
            if (OP_Mode==3){
                printf("not used");
            }else{
            }
        }
        if ((1 << 14) & statusword) {
                printf("reserved(0)");
        }
        if ((1 << 15) & statusword) {
            printf("Position referenced to home position; ");
        }
    }
    printf("\n");
}

void reset_fault(int slave){
    int size = 2;
    uint16_t ctl_word = 0x80;
    ec_SDOwrite(slave, 0x6040, 0x00, FALSE, 2 , &ctl_word, EC_TIMEOUTRET*5);
}

int read_temp(int slave) {
    // in 1/10 °C
    int wkc, temp;
    int size = 2;
    wkc = ec_SDOread(slave, 0x3201, 0x01, FALSE, &size, &temp, EC_TIMEOUTRXM);
    if (wkc > 0) {
        printf("Temperature: %2.1f °C \n", float32(temp)/10);
    }
    else{
        printf("Temperature not accessable!");
        temp = -1;

    }
    return temp;
}

bool setSpeed_SDO(int slave, int32_t speed_tar){
    uint16_t ctl_word = 0x0F;
    int wkc;
    wkc = ec_SDOwrite(slave, 0x60FF, 0x00, FALSE, 4,  &speed_tar, EC_TIMEOUTRET*2);
    if (wkc > 0){
        // printf(" target velocity is %d rpm \n", speed_tar);
    }
    wkc = ec_SDOwrite(slave, 0x6040, 0x00, FALSE, 2 , &ctl_word, EC_TIMEOUTRET*2);
    read_statusword(slave);
}

int read_supply(int slave){
    int wkc, volt;
    int size = 2;
    wkc = ec_SDOread(slave, 0x2200, 0x01, FALSE, &size, &volt, EC_TIMEOUTRXM);
    if (wkc > 0) {
        printf("Supply Voltage: %2.1f V \n", float32(volt)/10);
    }
    else{
        printf("Supply Voltage not Available!");
        volt = -1;

    }
    return volt;
}

void set_PDOs_velocity(uint16_t slave){
    // reset node --> resets also PDOs assigned!
    // the RxPDO1 and TxPDO1 mapping is used:

    int wkc; // size in bytes
    uint16_t ctl_word = 0x0F;
    wkc = ec_SDOwrite(slave, 0x6040, 0x00, FALSE, 2 , &ctl_word, EC_TIMEOUTRET*5);

    // set number of mapped objects to zero, change the entrys of subindexes 1 to 12, then set the number of mapped objects
    uint8_t mapped_obj_num = 0;
    ec_SDOwrite(slave, 0x1600, 0x00, FALSE, 1 , &mapped_obj_num, EC_TIMEOUTRET*2);
    uint32_t mapped_obj2 = 0x60FF0020; // index 0x60FF, subindex 0x00 is tar velocity, size to write is 0x20 (int32 is 0x20 bit long)
    ec_SDOwrite(slave, 0x1600, 0x02, FALSE, 4, &mapped_obj2, EC_TIMEOUTRET*2);
    mapped_obj_num = 2;
    ec_SDOwrite(slave, 0x1600, 0x00, FALSE, 1 , &mapped_obj_num, EC_TIMEOUTRET*2);

    // map Tx-Objects:
    mapped_obj_num = 0;
    ec_SDOwrite(slave, 0x1A00, 0x00, FALSE, 1 , &mapped_obj_num, EC_TIMEOUTRET*2);
    mapped_obj2 = 0x60640020;
    uint32_t mapped_obj3 = 0x606c0020;
    uint32_t mapped_obj4 = 0x60770010;
    ec_SDOwrite(slave, 0x1A00, 0x02, FALSE, 4 , &mapped_obj2, EC_TIMEOUTRET*2);
    ec_SDOwrite(slave, 0x1A00, 0x03, FALSE, 4 , &mapped_obj3, EC_TIMEOUTRET*2);
    ec_SDOwrite(slave, 0x1A00, 0x04, FALSE, 4 , &mapped_obj4, EC_TIMEOUTRET*2);
    mapped_obj_num = 4;
    ec_SDOwrite(slave, 0x1A00, 0x00, FALSE, 1 , &mapped_obj_num, EC_TIMEOUTRET*2);
}

int set_enable(int slave){


}

void set_speed(uint16 slave, int32_t speed){
    struct PDO_out *slave_out;
    slave_out = (struct PDO_out *)(ec_slave[slave].outputs);
    // target = (struct TorqueOut *)(ec_slave[1].outputs);
    slave_out->vel_tar = speed;
    slave_out->controlword = 0x0F;

    ec_send_processdata();
    // printf("target velocity of slave %d set to %d | ", slave, slave_out->vel_tar);
    int wkc, size=4, read_speed;
    // wkc = ec_SDOread(slave, 0x60FF, 0x00, FALSE, &size, &read_speed, EC_TIMEOUTRET*5);
    // printf("read velocity: %d \n", read_speed);
    return;
}

// controlword:
/* Bitweise codiert:
 * Bit 0: switched on
 * Bit 1: enable voltage
 * Bit 2: Quick stop
 * Bit 3: enable Operation
 * Switch on and enable Operation: Bit 0xxx 1111
 * Fault reset: 1xxx xxxx
 * stat
 */

char* SDO2string(uint16 slave, uint16 index, uint8 subidx, uint16 dtype)
{
    int l = sizeof(usdo) - 1, i;
    uint8 *u8;
    int8 *i8;
    uint16 *u16;
    int16 *i16;
    uint32 *u32;
    int32 *i32;
    uint64 *u64;
    int64 *i64;
    float *sr;
    double *dr;
    char es[32];

    memset(&usdo, 0, 128);
    ec_SDOread(slave, index, subidx, FALSE, &l, &usdo, EC_TIMEOUTRXM);
    if (EcatError)
    {
        return ec_elist2string();
    }
    else
    {
        switch(dtype)
        {
            case ECT_BOOLEAN:
                u8 = (uint8*) &usdo[0];
                if (*u8) sprintf(hstr, "TRUE");
                else sprintf(hstr, "FALSE");
                break;
            case ECT_INTEGER8:
                i8 = (int8*) &usdo[0];
                sprintf(hstr, "0x%2.2x %d", *i8, *i8);
                break;
            case ECT_INTEGER16:
                i16 = (int16*) &usdo[0];
                sprintf(hstr, "0x%4.4x %d", *i16, *i16);
                break;
            case ECT_INTEGER32:
            case ECT_INTEGER24:
                i32 = (int32*) &usdo[0];
                sprintf(hstr, "0x%8.8x %d", *i32, *i32);
                break;
            case ECT_INTEGER64:
                i64 = (int64*) &usdo[0];
                sprintf(hstr, "0x%16.16"PRIx64" %"PRId64, *i64, *i64);
                break;
            case ECT_UNSIGNED8:
                u8 = (uint8*) &usdo[0];
                sprintf(hstr, "0x%2.2x %u", *u8, *u8);
                break;
            case ECT_UNSIGNED16:
                u16 = (uint16*) &usdo[0];
                sprintf(hstr, "0x%4.4x %u", *u16, *u16);
                break;
            case ECT_UNSIGNED32:
            case ECT_UNSIGNED24:
                u32 = (uint32*) &usdo[0];
                sprintf(hstr, "0x%8.8x %u", *u32, *u32);
                break;
            case ECT_UNSIGNED64:
                u64 = (uint64*) &usdo[0];
                sprintf(hstr, "0x%16.16"PRIx64" %"PRIu64, *u64, *u64);
                break;
            case ECT_REAL32:
                sr = (float*) &usdo[0];
                sprintf(hstr, "%f", *sr);
                break;
            case ECT_REAL64:
                dr = (double*) &usdo[0];
                sprintf(hstr, "%f", *dr);
                break;
            case ECT_BIT1:
            case ECT_BIT2:
            case ECT_BIT3:
            case ECT_BIT4:
            case ECT_BIT5:
            case ECT_BIT6:
            case ECT_BIT7:
            case ECT_BIT8:
                u8 = (uint8*) &usdo[0];
                sprintf(hstr, "0x%x", *u8);
                break;
            case ECT_VISIBLE_STRING:
                strcpy(hstr, usdo);
                break;
            case ECT_OCTET_STRING:
                hstr[0] = 0x00;
                for (i = 0 ; i < l ; i++)
                {
                    sprintf(es, "0x%2.2x ", usdo[i]);
                    strcat( hstr, es);
                }
                break;
            default:
                sprintf(hstr, "Unknown type");
        }
        return hstr;
    }
}

/** Read PDO assign structure */
int si_PDOassign(uint16 slave, uint16 PDOassign, int mapoffset, int bitoffset)
{
    uint16 idxloop, nidx, subidxloop, rdat, idx, subidx;
    uint8 subcnt;
    int wkc, bsize = 0, rdl;
    int32 rdat2;
    uint8 bitlen, obj_subidx;
    uint16 obj_idx;
    int abs_offset, abs_bit;

    rdl = sizeof(rdat); rdat = 0;
    /* read PDO assign subindex 0 ( = number of PDO's) */
    // printf("PDOassign: %X\n ", PDOassign);
    wkc = ec_SDOread(slave, PDOassign, 0x00, FALSE, &rdl, &rdat, EC_TIMEOUTRXM);
    rdat = etohs(rdat); // why??
    // printf(" PDOassign Index: %x (EPOS: Sync manager PDO assignment), rdat: %d \n", PDOassign, rdat); // debugging
    /* positive result from slave ? */
    if ((wkc > 0) && (rdat > 0))
    {
        /* number of available sub indexes */
        nidx = rdat;
        bsize = 0;
        /* read all PDO's */
        for (idxloop = 1; idxloop <= nidx; idxloop++) {
            rdl = sizeof(rdat); rdat = 0;
            /* read PDO assign */
            wkc = ec_SDOread(slave, PDOassign, (uint8)idxloop, FALSE, &rdl, &rdat, EC_TIMEOUTRXM);
            // printf("reading PDOs: Index %x subindex %x, value %x", PDOassign, (uint8) idxloop, rdat); // rdat contains the index of the mapping of the PDO RxPDO1 and TxPDO1
            /* result is index of PDO */
            idx = etohs(rdat);
            if (idx > 0)
            {
                rdl = sizeof(subcnt); subcnt = 0;
                /* read number of subindexes of PDO */
                wkc = ec_SDOread(slave,idx, 0x00, FALSE, &rdl, &subcnt, EC_TIMEOUTRXM);
                subidx = subcnt;
                /* for each subindex */
                for (subidxloop = 1; subidxloop <= subidx; subidxloop++)
                {
                    rdl = sizeof(rdat2); rdat2 = 0;
                    /* read SDO that is mapped in PDO */
                    wkc = ec_SDOread(slave, idx, (uint8)subidxloop, FALSE, &rdl, &rdat2, EC_TIMEOUTRXM);
                    rdat2 = etohl(rdat2);
                    /* extract bitlength of SDO */
                    bitlen = LO_BYTE(rdat2);
                    bsize += bitlen;
                    obj_idx = (uint16)(rdat2 >> 16);
                    obj_subidx = (uint8)((rdat2 >> 8) & 0x000000ff);
                    abs_offset = mapoffset + (bitoffset / 8);
                    abs_bit = bitoffset % 8;
                    ODlist.Slave = slave;
                    ODlist.Index[0] = obj_idx;
                    OElist.Entries = 0;
                    wkc = 0;
                    /* read object entry from dictionary if not a filler (0x0000:0x00) */
                    if(obj_idx || obj_subidx)
                        wkc = ec_readOEsingle(0, obj_subidx, &ODlist, &OElist);
                        printf("  [0x%4.4X.%1d] 0x%4.4X:0x%2.2X 0x%2.2X", abs_offset, abs_bit, obj_idx, obj_subidx, bitlen);
                    if((wkc > 0) && OElist.Entries)
                    {
                        printf(" %-12s %s\n", dtype2string(OElist.DataType[obj_subidx]), OElist.Name[obj_subidx]);
                    }
                    else
                        printf("\n");
                    bitoffset += bitlen;
                };
            };
        };
    };
    /* return total found bitlength (PDO) */
    return bsize;
}

int si_map_sdo(int slave)
{
    int wkc, rdl;
    int retVal = 0;
    uint8 nSM, iSM, tSM;
    int Tsize, outputs_bo, inputs_bo;
    uint8 SMt_bug_add;
    // the
    printf("PDO mapping according to CoE :\n");
    SMt_bug_add = 0;
    outputs_bo = 0;
    inputs_bo = 0;
    rdl = sizeof(nSM); nSM = 0;



    /* read SyncManager Communication Type object count */
    // rdl:  Size in bytes of parameter buffer, returns bytes read from SDO
    // nSM: Pointer to parameter buffer
    wkc = ec_SDOread(slave, ECT_SDO_SMCOMMTYPE, 0x00, FALSE, &rdl, &nSM, EC_TIMEOUTRXM);
    /* positive result from slave ? */
    printf("rdl: %d, nSM: %d \n", rdl, nSM);
    if ((wkc > 0) && (nSM > 2))
    {
        /* make nSM equal to number of defined SM */
        nSM--;
        /* limit to maximum number of SM defined, if true the slave can't be configured */
        if (nSM > EC_MAXSM)
            nSM = EC_MAXSM;
        /* iterate for every SM type defined */
        for (iSM = 2 ; iSM <= nSM ; iSM++)
        {
            rdl = sizeof(tSM); tSM = 0;
            /* read SyncManager Communication Type */
            wkc = ec_SDOread(slave, ECT_SDO_SMCOMMTYPE, iSM + 1, FALSE, &rdl, &tSM, EC_TIMEOUTRXM);
            printf("ECT_SDO_SMCommtype: %x, subindex %d\n", ECT_SDO_SMCOMMTYPE, iSM);

            ec_slave[0].state = EC_STATE_OPERATIONAL;
            ec_send_processdata();
            ec_receive_processdata(EC_TIMEOUTRET);
            ec_writestate(0);
            osal_usleep(10000);



            if (wkc > 0)
            {
                if((iSM == 2) && (tSM == 2)) // SM2 has type 2 == mailbox out, this is a bug in the slave!
                {
                    SMt_bug_add = 1; // try to correct, this works if the types are 0 1 2 3 and should be 1 2 3 4
                    printf("Activated SM type workaround, possible incorrect mapping.\n");
                }
                if(tSM)
                    tSM += SMt_bug_add; // only add if SMt > 0

                if (tSM == 3) // outputs
                {
                    /* read the assign RXPDO */
                    printf("  SM%1d outputs\n     addr b   index: sub bitl data_type    name\n", iSM);
                    Tsize = si_PDOassign(slave, ECT_SDO_PDOASSIGN + iSM, (int)(ec_slave[slave].outputs - (uint8 *)&IOmap[0]), outputs_bo );
                    outputs_bo += Tsize;
                }
                if (tSM == 4) // inputs
                {
                    /* read the assign TXPDO */
                    printf("  SM%1d inputs\n     addr b   index: sub bitl data_type    name\n", iSM);
                    Tsize = si_PDOassign(slave, ECT_SDO_PDOASSIGN + iSM, (int)(ec_slave[slave].inputs - (uint8 *)&IOmap[0]), inputs_bo );
                    inputs_bo += Tsize;

                    int buff;
                    int size = 4;
                    // ec_send_processdata();
                    // wkc = ec_receive_processdata(EC_TIMEOUTRET);
                    // wkc = ec_TxPDO(slave, 0x00010, &200size, &buff, EC_TIMEOUTRXM);
                    //printf("wkc is %d", wkc);
                    //printf(" read %d byte from PDO 1: %x \n", size, buff);

                    // debugging
                    wkc = ec_SDOread(slave, 0x6041, 0x00, FALSE, &size, &buff, EC_TIMEOUTRXM*10);
                    printf("wkc is %d", wkc);
                    printf(" read %d byte from PDO 1: 0b", size);
                    printbits(buff);
                    printf("\n");
                    read_statusword(slave);
                    read_temp(slave);
                    read_supply(slave);
                    // printf(" size: %d\n ", size);
                    uint16_t ctl_word = 0x00;
                    size = 2;
                    wkc = ec_SDOread(slave, 0x6040, 0x00, FALSE, &size, &ctl_word, EC_TIMEOUTRET*5);
                    if (wkc > 0){
                        printf("wkc %d, read index 0x6040 (Controlword): 0x%4.4x\n", wkc, ctl_word);
                    }
                    ctl_word = 0x06;
                    wkc = ec_SDOwrite(slave, 0x6040, 0x00, FALSE, 2 , &ctl_word, EC_TIMEOUTRET*5);
                    ctl_word = 0x0F;
                    wkc = ec_SDOwrite(slave, 0x6040, 0x00, FALSE, 2 , &ctl_word, EC_TIMEOUTRET*5);
                    wkc = ec_SDOread(slave, 0x6040, 0x00, FALSE, &size, & ctl_word, EC_TIMEOUTRET*5);
                    if (wkc > 0){
                        printf("wkc %d, read index 0x6040 (Controlword, afterwards): 0x%4.4x \n", wkc, ctl_word);
                    }
                    read_statusword(slave);

                    uint8 NodeID; size=1;
                    wkc = ec_SDOread(slave, 0x6040, 0x00, FALSE, &size, &NodeID, EC_TIMEOUTRET*5);
                    printf("Node ID: %d", NodeID);
                    /*
                    uint8_t OP_mode = 3;
                    size = 1;
                    wkc = ec_SDOwrite(slave, 0x6060, 0x00, FALSE, 1,  &OP_mode, EC_TIMEOUTRXM);
                    wkc = ec_SDOread(slave, 0x6060, 0x00, FALSE, &size, &OP_mode, EC_TIMEOUTRET*5);
                    if (wkc > 0){
                        printf("wkc %d, read index 0x6060 - OP-Mode: %d \n", wkc, OP_mode);
                    }
                    osal_usleep(100000); */
                    /*
                    printf("now, start the motor: \n");
                    int32_t tar_velocity = 1000;
                    int32 act_pos;
                    int32 act_vel;
                    setSpeed_SDO(slave, tar_velocity);


                    osal_usleep(1000000);

                    struct PDO_in *slave_i;
                    slave_i = (struct PDO_in *)(ec_slave[slave].inputs);
                    ec_receive_processdata(EC_TIMEOUTRET*10); */
                    /*
                    for(uint32_t i =0; i<1000; i++){
                        tar_velocity = i*2;
                        // set_speed(slave,tar_velocity);
                        // setSpeed_SDO(slave, tar_velocity);
                        // ec_send_processdata();
                        if(i % 50 == 0) {
                            uint16 stat = 0;
                            size = 4; //  4;
                            // wkc = ec_SDOread(slave, 0x6041, 0x00, FALSE, &size, &stat, EC_TIMEOUTRET *10);
                            ec_receive_processdata(EC_TIMEOUTRXM);

                            printf("Statusword PDO: 0x%x | Statusword SDO: 0x%x \n", slave_i->statusword, stat);

                            // wkc = ec_SDOread(slave, 0x60FF, 0x00, FALSE, &size, &buff, EC_TIMEOUTRET *10);
                            // wkc = ec_SDOread(slave, 0x6064, 0x00, FALSE, &size, &act_pos, EC_TIMEOUTRET*10);
                            // wkc = ec_SDOread(slave, 0x606C, 0x00, FALSE, &size, &act_vel, EC_TIMEOUTRET*10); // 0x30D3 - actual averaged; x606C
                            printf("pos %5.5d \t| vel %d \t | read_act_pos %5.5f \t | read_act_vel %5.5d \t | setspeed: %5.5d \t| read_speed %.5d  \t \n", ((slave_i->pos_act)) , slave_i->vel_act, float32(act_pos)*(1 + .0/16384.0), act_vel, tar_velocity, buff);

                            // *float32 (1+ .0/(16384))
                        }
                        osal_usleep(5000);
                    } */
                    // set_speed(slave, 0);
                    // osal_usleep(5000000);
/*
                   // for DC-clock sync:
                   size=4;
                   wkc = ec_SDOread(slave, 0x0910, 0x00, FALSE, &size, &buff, EC_TIMEOUTRET *10);
                    printf("Index 0x0910: %d \t ", buff);
                    wkc = ec_SDOread(slave, 0x0990, 0x00, FALSE, &size, &buff, EC_TIMEOUTRET *10);
                    printf("Index 0x0990: %d \t ", buff);
                    wkc = ec_SDOread(slave, 0x09A0, 0x00, FALSE, &size, &buff, EC_TIMEOUTRET *10);
                    printf("Index 0x09A0: %d \n ", buff);
                    wkc = ec_SDOread(slave, 0x0981, 0x00, FALSE, &size, &buff, EC_TIMEOUTRET *10);
                    printf("Index 0x0981: %d \n ", buff);
                    // stop motor:
                    // tar_velocity = 0;
                    // setSpeed_SDO(slave, tar_velocity);

                    ctl_word = 0x07;
                     wkc = ec_SDOwrite(slave, 0x6040, 0x00, FALSE, 2 , &ctl_word, EC_TIMEOUTRET*2);
                    osal_usleep(10000);
                    read_statusword(slave);
                    reset_fault(slave);
                    read_statusword(slave); */
                }
            }
        }
    }
    /* found some I/O bits ? */
    if ((outputs_bo > 0) || (inputs_bo > 0))
        retVal = 1;
    return retVal;
}




int si_siiPDO(uint16 slave, uint8 t, int mapoffset, int bitoffset)
{
    uint16 a , w, c, e, er, Size;
    uint8 eectl;
    uint16 obj_idx;
    uint8 obj_subidx;
    uint8 obj_name;
    uint8 obj_datatype;
    uint8 bitlen;
    int totalsize;
    ec_eepromPDOt eepPDO;
    ec_eepromPDOt *PDO;
    int abs_offset, abs_bit;
    char str_name[EC_MAXNAME + 1];

    eectl = ec_slave[slave].eep_pdi;
    Size = 0;
    totalsize = 0;
    PDO = &eepPDO;
    PDO->nPDO = 0;
    PDO->Length = 0;
    PDO->Index[1] = 0;
    for (c = 0 ; c < EC_MAXSM ; c++) PDO->SMbitsize[c] = 0;
    if (t > 1)
        t = 1;
    PDO->Startpos = ec_siifind(slave, ECT_SII_PDO + t);
    if (PDO->Startpos > 0)
    {
        a = PDO->Startpos;
        w = ec_siigetbyte(slave, a++);
        w += (ec_siigetbyte(slave, a++) << 8);
        PDO->Length = w;
        c = 1;
        /* traverse through all PDOs */
        do
        {
            PDO->nPDO++;
            PDO->Index[PDO->nPDO] = ec_siigetbyte(slave, a++);
            PDO->Index[PDO->nPDO] += (ec_siigetbyte(slave, a++) << 8);
            PDO->BitSize[PDO->nPDO] = 0;
            c++;
            /* number of entries in PDO */
            e = ec_siigetbyte(slave, a++);
            PDO->SyncM[PDO->nPDO] = ec_siigetbyte(slave, a++);
            a++;
            obj_name = ec_siigetbyte(slave, a++);
            a += 2;
            c += 2;
            if (PDO->SyncM[PDO->nPDO] < EC_MAXSM) /* active and in range SM? */
            {
                str_name[0] = 0;
                if(obj_name)
                    ec_siistring(str_name, slave, obj_name);
                if (t)
                    printf("  SM%1d RXPDO 0x%4.4X %s\n", PDO->SyncM[PDO->nPDO], PDO->Index[PDO->nPDO], str_name);
                else
                    printf("  SM%1d TXPDO 0x%4.4X %s\n", PDO->SyncM[PDO->nPDO], PDO->Index[PDO->nPDO], str_name);
                printf("     addr b   index: sub bitl data_type    name\n");
                /* read all entries defined in PDO */
                for (er = 1; er <= e; er++)
                {
                    c += 4;
                    obj_idx = ec_siigetbyte(slave, a++);
                    obj_idx += (ec_siigetbyte(slave, a++) << 8);
                    obj_subidx = ec_siigetbyte(slave, a++);
                    obj_name = ec_siigetbyte(slave, a++);
                    obj_datatype = ec_siigetbyte(slave, a++);
                    bitlen = ec_siigetbyte(slave, a++);
                    abs_offset = mapoffset + (bitoffset / 8);
                    abs_bit = bitoffset % 8;

                    PDO->BitSize[PDO->nPDO] += bitlen;
                    a += 2;

                    /* skip entry if filler (0x0000:0x00) */
                    if(obj_idx || obj_subidx)
                    {
                        str_name[0] = 0;
                        if(obj_name)
                            ec_siistring(str_name, slave, obj_name);

                        printf("  [0x%4.4X.%1d] 0x%4.4X:0x%2.2X 0x%2.2X", abs_offset, abs_bit, obj_idx, obj_subidx, bitlen);
                        printf(" %-12s %s\n", dtype2string(obj_datatype), str_name);
                    }
                    bitoffset += bitlen;
                    totalsize += bitlen;
                }
                PDO->SMbitsize[ PDO->SyncM[PDO->nPDO] ] += PDO->BitSize[PDO->nPDO];
                Size += PDO->BitSize[PDO->nPDO];
                c++;
            }
            else /* PDO deactivated because SM is 0xff or > EC_MAXSM */
            {
                c += 4 * e;
                a += 8 * e;
                c++;
            }
            if (PDO->nPDO >= (EC_MAXEEPDO - 1)) c = PDO->Length; /* limit number of PDO entries in buffer */
        }
        while (c < PDO->Length);
    }
    if (eectl) ec_eeprom2pdi(slave); /* if eeprom control was previously pdi then restore */
    return totalsize;
}

// --------------------------------  -------------------------------- //
int si_map_sii(int slave)
{
    int retVal = 0;
    int Tsize, outputs_bo, inputs_bo;

    printf("PDO mapping according to SII :\n");

    outputs_bo = 0;
    inputs_bo = 0;
    /* read the assign RXPDOs */
    Tsize = si_siiPDO(slave, 1, (int)(ec_slave[slave].outputs - (uint8*)&IOmap), outputs_bo );
    outputs_bo += Tsize;
    /* read the assign TXPDOs */
    Tsize = si_siiPDO(slave, 0, (int)(ec_slave[slave].inputs - (uint8*)&IOmap), inputs_bo );
    inputs_bo += Tsize;
    /* found some I/O bits ? */
    if ((outputs_bo > 0) || (inputs_bo > 0))
        retVal = 1;
    return retVal;
}

void si_sdo(int cnt)
{
    int i, j;

    ODlist.Entries = 0;
    memset(&ODlist, 0, sizeof(ODlist));

    //int test = ec_readODlist(cnt, &ODlist);
    // printf("test: %d, cnt: %d, ODList Entries: %d \n", test, cnt, ODlist.Entries);

    if( ec_readODlist(cnt, &ODlist))
    {
        printf(" CoE Object Description found, %d entries.\n",ODlist.Entries);
        for( i = 0 ; i < ODlist.Entries ; i++)
        {
            ec_readODdescription(i, &ODlist);
            while(EcatError) printf("%s", ec_elist2string());
            printf(" Index: %4.4x Datatype: %4.4x Objectcode: %2.2x Name: %s\n",
                   ODlist.Index[i], ODlist.DataType[i], ODlist.ObjectCode[i], ODlist.Name[i]);
            memset(&OElist, 0, sizeof(OElist));
            ec_readOE(i, &ODlist, &OElist);
            while(EcatError) printf("%s", ec_elist2string());
            for( j = 0 ; j < ODlist.MaxSub[i]+1 ; j++)
            {
                if ((OElist.DataType[j] > 0) && (OElist.BitLength[j] > 0))
                {
                    printf("  Sub: %2.2x Datatype: %4.4x Bitlength: %4.4x Obj.access: %4.4x Name: %s\n",
                           j, OElist.DataType[j], OElist.BitLength[j], OElist.ObjAccess[j], OElist.Name[j]);
                    if ((OElist.ObjAccess[j] & 0x0007))
                    {
                        printf("          Value :%s\n", SDO2string(cnt, ODlist.Index[i], j, OElist.DataType[j]));
                    }
                }
            }
        }
    }
    else
    {
        while(EcatError) printf("%s", ec_elist2string());
    }
}

void slaveinfo(char *ifname)
{
    int cnt, i, j, nSM;
    uint16 ssigen;
    int expectedWKC;

    printf("Starting slaveinfo\n");

    /* initialise SOEM, bind socket to ifname */
    if (ec_init(ifname))
    {
        printf("ec_init on %s succeeded.\n",ifname);
        /* find and auto-config slaves */
        if ( ec_config(FALSE, &IOmap) > 0 )
        {
            // ec_config_init()
            ec_configdc();
            ec_dcsync0(1, TRUE, 1000000, 0);

            // ec_slave[1].blockLRW = 1; // for PDO setting? --> as in git issue from SOEM. seems to make no difference?
            while(EcatError) printf("%s", ec_elist2string());
            printf("%d slaves found and configured.\n",ec_slavecount);
            expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
            printf("Calculated workcounter %d\n", expectedWKC);
            /* wait for all slaves to reach SAFE_OP state */
            ec_statecheck(0, EC_STATE_SAFE_OP,  EC_TIMEOUTSTATE * 3);
            if (ec_slave[0].state != EC_STATE_SAFE_OP )
            {
                printf("Not all slaves reached safe operational state.\n");
                ec_readstate();
                for(i = 1; i<=ec_slavecount ; i++)
                {
                    if(ec_slave[i].state != EC_STATE_SAFE_OP)
                    {
                        printf("Slave %d State=%2x StatusCode=%4x : %s\n",
                               i, ec_slave[i].state, ec_slave[i].ALstatuscode, ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
                    }
                }
            }


            ec_readstate();
            for( cnt = 1 ; cnt <= ec_slavecount ; cnt++) // interation over all found slaves
            {
                printf("\nSlave:%d\n Name:%s\n Output size: %dbits\n Input size: %dbits\n State: %d\n Delay: %d[ns]\n Has DC: %d\n",
                       cnt, ec_slave[cnt].name, ec_slave[cnt].Obits, ec_slave[cnt].Ibits,
                       ec_slave[cnt].state, ec_slave[cnt].pdelay, ec_slave[cnt].hasdc);
                if (ec_slave[cnt].hasdc) printf(" DCParentport:%d\n", ec_slave[cnt].parentport);
                printf(" Activeports:%d.%d.%d.%d\n", (ec_slave[cnt].activeports & 0x01) > 0 ,
                       (ec_slave[cnt].activeports & 0x02) > 0 ,
                       (ec_slave[cnt].activeports & 0x04) > 0 ,
                       (ec_slave[cnt].activeports & 0x08) > 0 );
                printf(" Configured address: %4.4x\n", ec_slave[cnt].configadr);
                printf(" Man: %8.8x ID: %8.8x Rev: %8.8x\n", (int)ec_slave[cnt].eep_man, (int)ec_slave[cnt].eep_id, (int)ec_slave[cnt].eep_rev);

                // full serial Number:
                long serial_full;
                int size_ser_full, size = 1;
                uint8_t NodeID;
                ec_SDOread(cnt, 0x2100, 01, FALSE, &size_ser_full, &serial_full, EC_TIMEOUTRXM);
                printf(" Serial Number complete: %lx\n", serial_full);
                ec_SDOread(cnt, 0x2000, 00, FALSE, &size, &NodeID, EC_TIMEOUTRXM);
                printf("Slave %d has the Node ID %d\n", cnt, NodeID);

                for(nSM = 0 ; nSM < EC_MAXSM ; nSM++)
                {
                    if(ec_slave[cnt].SM[nSM].StartAddr > 0)
                        printf(" SM%1d A:%4.4x L:%4d F:%8.8x Type:%d\n",nSM, etohs(ec_slave[cnt].SM[nSM].StartAddr), etohs(ec_slave[cnt].SM[nSM].SMlength),
                               etohl(ec_slave[cnt].SM[nSM].SMflags), ec_slave[cnt].SMtype[nSM]);
                }
                for(j = 0 ; j < ec_slave[cnt].FMMUunused ; j++)
                {
                    printf(" FMMU%1d Ls:%8.8x Ll:%4d Lsb:%d Leb:%d Ps:%4.4x Psb:%d Ty:%2.2x Act:%2.2x\n", j,
                           etohl(ec_slave[cnt].FMMU[j].LogStart), etohs(ec_slave[cnt].FMMU[j].LogLength), ec_slave[cnt].FMMU[j].LogStartbit,
                           ec_slave[cnt].FMMU[j].LogEndbit, etohs(ec_slave[cnt].FMMU[j].PhysStart), ec_slave[cnt].FMMU[j].PhysStartBit,
                           ec_slave[cnt].FMMU[j].FMMUtype, ec_slave[cnt].FMMU[j].FMMUactive);
                }
                printf(" FMMUfunc 0:%d 1:%d 2:%d 3:%d\n",
                       ec_slave[cnt].FMMU0func, ec_slave[cnt].FMMU1func, ec_slave[cnt].FMMU2func, ec_slave[cnt].FMMU3func);
                printf(" MBX length wr: %d rd: %d MBX protocols : %2.2x\n", ec_slave[cnt].mbx_l, ec_slave[cnt].mbx_rl, ec_slave[cnt].mbx_proto);
                ssigen = ec_siifind(cnt, ECT_SII_GENERAL);
                /* SII general section */
                if (ssigen)
                {
                    ec_slave[cnt].CoEdetails = ec_siigetbyte(cnt, ssigen + 0x07);
                    ec_slave[cnt].FoEdetails = ec_siigetbyte(cnt, ssigen + 0x08);
                    ec_slave[cnt].EoEdetails = ec_siigetbyte(cnt, ssigen + 0x09);
                    ec_slave[cnt].SoEdetails = ec_siigetbyte(cnt, ssigen + 0x0a);
                    if((ec_siigetbyte(cnt, ssigen + 0x0d) & 0x02) > 0)
                    {
                        ec_slave[cnt].blockLRW = 1;
                        ec_slave[0].blockLRW++;
                    }
                    ec_slave[cnt].Ebuscurrent = ec_siigetbyte(cnt, ssigen + 0x0e);
                    ec_slave[cnt].Ebuscurrent += ec_siigetbyte(cnt, ssigen + 0x0f) << 8;
                    ec_slave[0].Ebuscurrent += ec_slave[cnt].Ebuscurrent;
                }
                printf(" CoE details: %2.2x FoE details: %2.2x EoE details: %2.2x SoE details: %2.2x\n",
                       ec_slave[cnt].CoEdetails, ec_slave[cnt].FoEdetails, ec_slave[cnt].EoEdetails, ec_slave[cnt].SoEdetails);
                printf(" Ebus current: %d[mA]\n only LRD/LWR:%d\n",
                       ec_slave[cnt].Ebuscurrent, ec_slave[cnt].blockLRW);

                // printf("config PDOs");
                // set_PDOs_velocity(cnt);
                // -------------- --------------- //
                if ((ec_slave[cnt].mbx_proto & ECT_MBXPROT_COE) && printSDO)
                    // printf("Test cnt: %d \n ", cnt);
                     si_sdo(cnt); // EPOS4: error 0004?
                if(printMAP)
                {
                    if (ec_slave[cnt].mbx_proto & ECT_MBXPROT_COE)
                        si_map_sdo(cnt); // for COE
                    else
                        si_map_sii(cnt);
                }
            }
        }
        else
        {
            printf("No slaves found!\n");
        }

        printf("End slaveinfo, close socket\n");
        /* stop SOEM, close socket */
        ec_close();
    }
    else
    {
        printf("No socket connection on %s\nExcecute as root\n",ifname);
    }
}

char ifbuf[1024];


/* tODO:
    -> change OP Mode
    -> set Velocity
    -> get Velocity
    -> Statusword
    -> controlword
    -> read errors/faultmode
    --> read torque
    --> position actual value


    nice:
    -> get Temperature
    -> PDOs
    -> Voltage
    ->
    ->

 */
int motor_enable (int slave)
{

}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "listener");
    ec_adaptert * adapter = NULL;
    printf("SOEM (Simple Open EtherCAT Master)\nSlaveinfo\n");

    if (argc > 1)
    {
        if ((argc > 2) && (strncmp(argv[2], "-sdo", sizeof("-sdo")) == 0)) printSDO = TRUE;
        if ((argc > 2) && (strncmp(argv[2], "-map", sizeof("-map")) == 0)) printMAP = TRUE;
        /* start slaveinfo */
        strcpy(ifbuf, argv[1]);
        slaveinfo(ifbuf);
    }
    else
    {
        printf("Usage: slaveinfo ifname [options]\nifname = eth0 for example\nOptions :\n -sdo : print SDO info\n -map : print mapping\n");

        printf ("Available adapters\n");
        adapter = ec_find_adapters ();
        while (adapter != NULL)
        {
            printf ("Description : %s, Device to use for wpcap: %s\n", adapter->desc,adapter->name);
            adapter = adapter->next;
        }
    }




    printf("End program\n");

    return (0);
}
