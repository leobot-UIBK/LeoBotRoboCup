/** \file
 * \brief Example code for Simple Open EtherCAT master
 *
 * Usage : simple_test [ifname1]
 * ifname is NIC interface, f.e. eth0
 *
 * This is a minimal test for the EPOS4 motorcontroller, based on the simpletest code of SOEM (c)Arthur Ketels 2010 - 2011
 * Use under BSD-license permitted, only use by trained persons, no liability for damage of any kind will be accepted.
 * Peter Manzl, Team Tyrolics, University of Innsbruck
 */

#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "ethercat.h"
#include <stdint.h>
#include <unistd.h>
#include <sys/types.h>
#include <pthread.h>
#include <time.h>

#define EC_TIMEOUTMON 500

char IOmap[4096];
OSAL_THREAD_HANDLE thread1;
int expectedWKC;
boolean needlf;
volatile int wkc;
boolean inOP;
uint8 currentgroup = 0;


struct PDO_out {
    uint16_t controlword;
    int32_t vel_tar;
}__attribute__((packed));

struct PDO_in {
    uint16_t statusword;
    int32_t pos_act;
    int32_t vel_act;
    int16_t torq_act;
}__attribute__((packed));

void simpletest(char *ifname)
{
    /* pid_t process_id;
    pid_t p_process_id;

    process_id = getpid();
    p_process_id = getppid();
    printf("Simpletest \n");
    printf("The process id: %d\n",process_id);
    printf("The process id of parent function: %d\n \n",p_process_id); */

    int i, j, oloop, iloop, chk;
    needlf = FALSE;
    inOP = FALSE;

    printf("Starting simple test\n");

    /* initialise SOEM, bind socket to ifname */
    if (ec_init(ifname))
    {
        printf("ec_init on %s succeeded.\n",ifname);
        /* find and auto-config slaves */


        if ( ec_config_init(FALSE) > 0 )
        {
            printf("%d slaves found and configured.\n",ec_slavecount);

            ec_config_map(&IOmap);

            ec_configdc();

            printf("Slaves mapped, state to SAFE_OP.\n");
            /* wait for all slaves to reach SAFE_OP state */
            ec_statecheck(0, EC_STATE_SAFE_OP,  EC_TIMEOUTSTATE * 4);

            oloop = ec_slave[0].Obytes;
            if ((oloop == 0) && (ec_slave[0].Obits > 0)) oloop = 1;
            if (oloop > 8) oloop = 8;
            ec_slave[0].Ibytes = 12; // added to match PDO
            iloop = ec_slave[0].Ibytes;
            if ((iloop == 0) && (ec_slave[0].Ibits > 0)) iloop = 1;
            // if (iloop > 8) iloop = 8;

            printf("segments : %d : %d %d %d %d\n",ec_group[0].nsegments ,ec_group[0].IOsegment[0],ec_group[0].IOsegment[1],ec_group[0].IOsegment[2],ec_group[0].IOsegment[3]);

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

                volatile PDO_in *slave1_in;
                volatile PDO_out *slave1_out;
                slave1_in = (struct PDO_in *)(ec_slave[1].inputs);
                slave1_out = (struct PDO_out *)(ec_slave[1].outputs);
                uint16 ctl_word;
                int size;
                ctl_word = 0x06;
                wkc = ec_SDOwrite(1, 0x6040, 0x00, FALSE, 2 , &ctl_word, EC_TIMEOUTRET*5);
                ctl_word = 0x0F;
                wkc = ec_SDOwrite(1, 0x6040, 0x00, FALSE, 2 , &ctl_word, EC_TIMEOUTRET*5);
                wkc = ec_SDOread(1, 0x6040, 0x00, FALSE, &size, & ctl_word, EC_TIMEOUTRET*5);

                uint8_t OP_mode = 3;
                size = 1;
                wkc = ec_SDOwrite(1, 0x6060, 0x00, FALSE, 1,  &OP_mode, EC_TIMEOUTRXM);

                printf("size controlword: %d, size vel_tar %d, size PDO_out %d", sizeof(slave1_out->controlword),
                       sizeof(slave1_out->vel_tar), sizeof(PDO_out));


                clock_t clock_loop, clock_act, clock_act2;
                /* cyclic loop */
                clock_loop = clock();
                clock_act = clock_loop;
                float clk_avg = 0;
                int clocks_cycle = CLOCKS_PER_SEC/1000;
                int i = 0, Cyc_Violations = 0, num_cycles = 4000;
                // printf("iloop: %d | opool: %d", iloop, oloop);
                while(i < num_cycles)
                {
                    clock_act = clock();
                    if(clock_act > (clock_loop+clocks_cycle) ){
                        slave1_out->controlword = 0x000F;
                        if (clock_act > (clock_loop+2*clocks_cycle)){
                            printf("\n\n warning: cycle violation!!\n\n");
                            Cyc_Violations ++;
                        }
                        clock_loop +=clocks_cycle;
                        slave1_out->vel_tar = 360;  //i<<8;
                        // printf("ctl_word: 0x%x | vel_tar %ld | ", slave1_out->controlword, slave1_out->vel_tar);
                        ec_send_processdata();
                        wkc = ec_receive_processdata(EC_TIMEOUTRET);
                        // printf("expected wkc: %d | wkc: %d \n", expectedWKC, wkc);
                        printf("PDO_O1 0x%04.4x | PDO_O2 %d | ", slave1_out->controlword, slave1_out->vel_tar);
                        printf("PDO_I1 0x%04.4x | PDO_I2 %f (revolutions) | PDO_I3 %d | PDO_I4 %d \n", slave1_in->statusword,
                               ((float) slave1_in->pos_act)/16384.0, slave1_in->vel_act, slave1_in->torq_act);
                        if (wkc >= expectedWKC) {
                            slave1_in = (struct PDO_in *) (ec_slave[1].inputs);
                            slave1_out = (struct PDO_out *) (ec_slave[1].outputs);
                            //printf("Processdata cycle %4d, WKC %d , O:", i, wkc);
                            //printf(" | oloop : %d", oloop); /// debugging
                            //printf(" | iloop : %d \n", iloop); /// debugging
                            for (j = 0; j < oloop; j++) {
                                printf(" %2.2x", *(ec_slave[0].outputs + j));
                            }


                            printf(" I:");
                            for (j = 0; j < iloop; j++) {
                                printf(" %2.2x", *(ec_slave[0].inputs + j));
                            }
                            printf(" T:%"PRId64"", ec_DCtime);
                            needlf = TRUE;
                        }
                        i++;
                        clk_avg += ((float)(clock() - clock_act))/num_cycles;
                        printf("cycles: %f\n", clk_avg);
                    }
                    //  osal_usleep(50000);

                }
                // clock_end = clock();
                // printf("test took %f seconds", (float)(clock_end-clock_start)/CLOCKS_PER_SEC);
                slave1_out->controlword = 0x0F;
                slave1_out->vel_tar = 0;
                printf("ctl_word: 0x%x | vel_tar %ld | ", slave1_out->controlword, slave1_out->vel_tar);
                ec_send_processdata();
                inOP = FALSE;
                printf("Motor shut down, %d cycle violations, avg %f\n", Cyc_Violations, clk_avg*1000/CLOCKS_PER_SEC);

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
        }
        printf("End simple test, close socket\n");
        /* stop SOEM, close socket */
        ec_close();
    }
    else
    {
        printf("No socket connection on %s\nExcecute as root\n",ifname);
    }
}

// ecatcheck is running in a seperat thread in the Operating System Abstraction Layer
OSAL_THREAD_FUNC ecatcheck( void *ptr )
{
    /* pid_t process_id;
    pid_t p_process_id;

    process_id = getpid();
    p_process_id = getppid();
    printf("OSAL_Thread_Func ecatcheck \n");
    printf("The process id: %d\n",process_id);
    printf("The process id of parent function: %d\n",p_process_id);
    printf("thread pointer: %lld \n \n ",  &thread1);

    pthread_t thread_ID_OSAL;
    thread_ID_OSAL = pthread_self();
    printf("Thread ID OSAL: %d \n", thread_ID_OSAL);
*/
    int slave;
    (void)ptr;                  /* Not used */

    while(1)
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
        osal_usleep(10000);
    }
}

int main(int argc, char *argv[])
{
    printf("SOEM (Simple Open EtherCAT Master)\nSimple test\n");

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

    if (argc > 1)
    {
        /* create thread to handle slave error handling in OP */
//      pthread_create( &thread1, NULL, (void *) &ecatcheck, (void*) &ctime);
        osal_thread_create(&thread1, 128000, (void *) &ecatcheck, (void*) &ctime);
        /* start cyclic part */
        simpletest(argv[1]);

    }
    else
    {
        printf("Usage: simple_test ifname1\nifname = eth0 for example\n");
    }
    printf("End program\n");

    return (0);
}