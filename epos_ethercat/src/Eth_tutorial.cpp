


#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#include "ethercat.h"

#define EC_TIMEOUTMON 500

char IOmap[4096];
OSAL_THREAD_HANDLE thread1;
int expectedWKC;
boolean needlf;
volatile int wkc;
boolean inOP;
uint8 currentgroup = 0;



int main (void) {
    rprintp("SOEM (Simple Open EtherCAT Master)\nSimple test\n");
    task_spawn("simpletest", simpletest, 9, 8192, NULL);
    /* initialise SOEM, bind socket to ifname */
    if (ec_init(ifname) > 0) {

        /* initialise SOEM, bind socket to ifname */
        if (ec_init(ifname) > 0) {

        }
    }

}
