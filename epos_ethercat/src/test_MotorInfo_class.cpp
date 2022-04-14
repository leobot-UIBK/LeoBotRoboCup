//
// Created by robo on 4/21/20.
//

#include "EPOS4_class.h"
#include <signal.h>
#include "circular_buffer.h"
#include <limits.h>

// timespec for status thread
timespec t_start_stat;
timespec t_act_stat;
pthread_t tid_status;
volatile int wkc;
pthread_mutex_t lock;


// timespec for control thread
timespec t_start_ctl;
timespec t_act_ctl;
pthread_t tid_ctl;

FILE *pFile;

timespec t_1;
timespec t_2;
double t_avg = 0;
uint64 t_max, dt;

bool inOP;
int err_cnt;

boolean flag_debug = TRUE;
pthread_t tid_fileIO;
circular_buffer<uint64_t> c_buff(256);

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
        // printf("cv!\t");
        *ret = 10;
        return;
        // cycle violation!
    }
    /* no signal wakes */
    *ret=0;
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &pinfo->next_period, nullptr);
}

void cleanup_fileIO(void* data)
{
    printf("cleanup fileIO thread called.\n");
}

void cleanup_stat(void* pMotorInfo)
{
    // nothing that needs to be cleaned up here.
    printf("cleanup stat called.\n");
}

void cleanup_ctl(void* pMotorInfo)
{
    inOP = false;
    MotorInfo* EPOS4 = (MotorInfo*) pMotorInfo;
    EPOS4->poweroff();
    printf("cleanup ctl called.\n");
    printf("cycle violations: %d\n", err_cnt);
    printf("average cycle time of ctl function: %.0f ns, max %llu ns\n",  t_avg, t_max);

}

void stat_func(void *pMotorInfo)
{
    MotorInfo *EPOS4 = (MotorInfo*) pMotorInfo;
    int size = 2;
    uint16_t buff;
    /*
    int wkc_stat = 0;
    wkc_stat = ec_SDOread(EPOS4->slave_number, 0x3201, 0x01, FALSE, &size, &buff, EC_TIMEOUTTXM);
    EPOS4->Temp = buff;
    wkc_stat = ec_SDOread(EPOS4->slave_number, 0x2200, 0x01, FALSE, &size, &buff, EC_TIMEOUTTXM);
    EPOS4->Voltage = buff; */
}

// thread function for cyclic Motor check (voltage, Temperature, etc.)
void *pth_check_status(void *pMotorInfo)
{
    pthread_cleanup_push(cleanup_stat, nullptr);
        auto *EPOS4 = (MotorInfo*) pMotorInfo;
        struct period_info pinfo_stat;
        periodic_task_init(&pinfo_stat, 1000);

        clock_gettime(CLOCK_MONOTONIC, &(t_start_stat));
        // printf("time init stat \n");
        int ret = 0;
        while (EPOS4->isrunning == 1) {
            stat_func(pMotorInfo);
            clock_gettime(CLOCK_MONOTONIC, &(t_act_stat));
            printf("Status t %ld | T %d | V %d\n", t_act_stat.tv_sec - t_start_stat.tv_sec, EPOS4->Temp, EPOS4->Voltage, EPOS4->SM_events_missed);
            printf("SM events missed %u\n", EPOS4->SM_events_missed);
            if (flag_debug) {
                fprintf(pFile, "%lld\t 80 \n", (t_act_stat.tv_sec - t_start_ctl.tv_sec) * 1000000000 +
                                              (t_act_stat.tv_nsec - t_start_ctl.tv_nsec));
            }
            wait_rest_of_period(&pinfo_stat, &ret);
        }
    pthread_cleanup_pop(1);
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

}

void handler_int(int num){
    pthread_cancel(tid_ctl);
    pthread_cancel(tid_status);
    pthread_cancel(tid_fileIO);
    write(STDOUT_FILENO, "\n Shutdown.\n\n", 35);
    printf("sig %d", num);
    fclose(pFile);
}
// control function for the Motor
void ctl_func(void *pMotorInfo, int i)
{
    MotorInfo *EPOS4 = (MotorInfo*) pMotorInfo;
}


void alarm_hand(int num){
    printf("alrm! \n");
}

// thread function for cyclic Motor control
void *pth_ctl_func (void *pMotorInfo) {
    pthread_cleanup_push(cleanup_ctl, pMotorInfo);
        MotorInfo *EPOS4 = (MotorInfo *) pMotorInfo;
        struct period_info pinfo_ctl;
        EPOS4->isrunning = 1;
        uint64 i = 0, t_stp = 0; // loop variable and timestamp
        int ret = 0;
        clock_gettime(CLOCK_MONOTONIC, &(t_start_ctl));
        sigset_t sigset_ctl;
        periodic_task_init(&pinfo_ctl, 1);
        t_avg = 0;
        t_max = 0;
        while (1) {
            clock_gettime(CLOCK_MONOTONIC, &t_1);
            switch (ret) {
                case 0:
                    // ctl_func(pMotorInfo, i);
                    i++;
                    break;
                case 10: // skip if cycle violation
                    err_cnt++;
                    printf("cv!\n");
                    break;
            }

            t_stp = ((uint64) (t_1.tv_sec - t_start_ctl.tv_sec)) * 1000000000 + (t_1.tv_nsec - t_start_ctl.tv_nsec);
            c_buff.put(t_stp); // get timestamp and write it into circular buffer
            clock_gettime(CLOCK_MONOTONIC, &t_2);
            dt = ((uint64) (t_2.tv_sec - t_1.tv_sec)) * 1000000000 + (t_2.tv_nsec - t_1.tv_nsec);
            t_avg = ((t_avg * i) + dt) / (i + 1); // be aware of overflow and rounding error!
            if (dt > t_max) {
                t_max = dt;
            }
            wait_rest_of_period(&pinfo_ctl, &ret);
        }
    pthread_cleanup_pop(1);
}

// thread function to write from cyclic buffer into a file
void *pth_fileIO(void *data)
{
    pthread_cleanup_push(cleanup_fileIO, nullptr);
        struct period_info pinfo_fileIO;
        periodic_task_init(&pinfo_fileIO, 20);
        int ret = 0;
        while (1) {
            while(!c_buff.empty()) {
                fprintf(pFile, "%lld \t 90\n", c_buff.get());
            }
            wait_rest_of_period(&pinfo_fileIO, &ret);
        }
    pthread_cleanup_pop(1);
}


int main(){

    signal(SIGINT, handler_int);
    signal(SIGTERM, handler_int);

    MotorInfo EPOS4;
    // EPOS4.startup_OD();
    // EPOS4.OD.print_OD();
    // EPOS4.print_status();
    // EPOS4.pPDO_in = (struct PDO_in *)(ec_slave[1].inputs);

    pFile = fopen("/tmp/test.txt", "w+");

    int ret_ctl, ret_stat, ret_fileIO;
    struct sched_param param_ctl;
    struct sched_param param_stat;
    pthread_attr_t attr_ctl;
    pthread_attr_t attr_stat;


    /*
    sigset_t sigset_ctl;
    sigemptyset(&sigset_ctl);
    int sig = 0;
    alarm(1);
    printf("sigwait! \n"); */

    /* Lock memory */
    if(mlockall(MCL_CURRENT|MCL_FUTURE) == -1) {
        printf("mlockall failed: %m\n");
        exit(-2);
    }

    /* Initialize pthread attributes (default values) */
    ret_ctl = pthread_attr_init(&attr_ctl);
    ret_stat = pthread_attr_init(&attr_stat);
    if (ret_ctl || ret_stat) {
        printf("init pthread attributes failed\n");
        return EXIT_FAILURE;
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

    param_ctl.sched_priority = 99;
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

    /* Create a pthread with specified attributes */

    ret_ctl = pthread_create(&tid_ctl, &attr_ctl, &pth_ctl_func, &EPOS4);
    ret_stat = pthread_create(&tid_status, &attr_stat, &pth_check_status, &EPOS4);
    if(flag_debug) ret_fileIO = pthread_create(&tid_fileIO, nullptr, &pth_fileIO, nullptr);
    if (ret_ctl || ret_stat || (ret_fileIO && flag_debug)) {
        printf("create pthread failed\n");
        exit(-2);
    }

    pthread_join(tid_status, nullptr);
    pthread_join(tid_ctl, nullptr);
    pthread_join(tid_fileIO, nullptr);

    fclose(pFile);



}
